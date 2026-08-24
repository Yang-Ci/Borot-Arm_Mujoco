from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus, JointMotorCmd, JointMotorState
from rebotarm_msgs.srv import SetGripper
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


_GRIPPER_COMMAND_OPEN_M = 0.09
_GRIPPER_VISUAL_OPEN_M = 0.057
_GRIPPER_HALF_VISUAL_SCALE = _GRIPPER_VISUAL_OPEN_M / (_GRIPPER_COMMAND_OPEN_M * 2.0)


class FakeReBotArmDriver(Node):
    def __init__(self) -> None:
        super().__init__("fake_rebotarm_driver")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("max_joint_speed", 1.8)
        self.declare_parameter("max_gripper_speed", 0.08)
        self.declare_parameter("gripper_open", 0.09)

        self.namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        self.max_joint_speed = float(self.get_parameter("max_joint_speed").value)
        self.max_gripper_speed = float(self.get_parameter("max_gripper_speed").value)
        self.gripper_open = float(self.get_parameter("gripper_open").value)

        self.positions = {name: 0.0 for name in self.joint_names}
        self.targets = {name: 0.0 for name in self.joint_names}
        self.velocities = {name: 0.0 for name in self.joint_names}
        self.gripper_position = 0.0
        self.gripper_target = 0.0
        self.gripper_velocity = 0.0
        self.enabled = True
        self.state_machine = "IDLE"
        self.gravity_comp_active = False
        self.last_time = self.get_clock().now()

        self.joint_state_pub = self.create_publisher(
            JointState,
            f"/{self.namespace}/joint_states",
            qos_profile_sensor_data,
        )
        self.joint_motor_pubs = {
            name: self.create_publisher(
                JointMotorState,
                f"/{self.namespace}/joints/{name}/state",
                qos_profile_sensor_data,
            )
            for name in self.joint_names
        }
        self.gripper_state_pub = self.create_publisher(
            JointMotorState,
            f"/{self.namespace}/gripper/state",
            qos_profile_sensor_data,
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            ArmStatus,
            f"/{self.namespace}/arm_status",
            latched_qos,
        )

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for name in self.joint_names:
            self.create_subscription(
                JointMotorCmd,
                f"/{self.namespace}/joints/{name}/cmd",
                self._make_joint_cmd_callback(name),
                reliable_qos,
            )
        self.create_subscription(
            JointMotorCmd,
            f"/{self.namespace}/gripper/cmd",
            self._gripper_cmd_callback,
            reliable_qos,
        )

        self.create_service(Trigger, f"/{self.namespace}/enable", self._enable)
        self.create_service(Trigger, f"/{self.namespace}/disable", self._disable)
        self.create_service(Trigger, f"/{self.namespace}/safe_home", self._safe_home)
        self.create_service(
            Trigger,
            f"/{self.namespace}/gravity_compensation/start",
            self._start_gravity_compensation,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/gravity_compensation/stop",
            self._stop_gravity_compensation,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/gravity_compensation/status",
            self._gravity_compensation_status,
        )
        self.create_service(SetGripper, f"/{self.namespace}/gripper/set", self._set_gripper)

        rate = max(float(self.get_parameter("joint_state_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.publish_status()
        self.get_logger().info(
            f"fake reBotArm driver started: namespace=/{self.namespace}, "
            f"joints={self.joint_names}"
        )

    def _make_joint_cmd_callback(self, joint_name: str):
        def _callback(msg: JointMotorCmd) -> None:
            if not self.enabled:
                return
            target = self.targets[joint_name]
            if msg.use_pos:
                target = float(msg.pos)
            elif msg.use_vel:
                target = self.positions[joint_name] + float(msg.vel) * 0.05
            self.targets[joint_name] = target
            self.gravity_comp_active = False
            self.state_machine = "LOWLEVEL_STREAMING"
            self.publish_status()

        return _callback

    def _gripper_cmd_callback(self, msg: JointMotorCmd) -> None:
        if not self.enabled:
            return
        if msg.use_pos:
            self.gripper_target = self._clamp(float(msg.pos), 0.0, self.gripper_open)
        elif msg.use_vel:
            self.gripper_target = self._clamp(
                self.gripper_position + float(msg.vel) * 0.05,
                0.0,
                self.gripper_open,
            )
        self.gravity_comp_active = False
        self.state_machine = "LOWLEVEL_STREAMING"
        self.publish_status()

    def _enable(self, _request, response):
        self.enabled = True
        if not self.gravity_comp_active:
            self.state_machine = "IDLE"
        response.success = True
        response.message = "fake driver enabled"
        self.publish_status()
        return response

    def _disable(self, _request, response):
        self.enabled = False
        self.gravity_comp_active = False
        self.state_machine = "IDLE"
        response.success = True
        response.message = "fake driver disabled"
        self.publish_status()
        return response

    def _safe_home(self, _request, response):
        self.gravity_comp_active = False
        for name in self.joint_names:
            self.targets[name] = 0.0
        self.gripper_target = 0.0
        self.state_machine = "LOWLEVEL_STREAMING"
        response.success = True
        response.message = "fake safe_home accepted"
        self.publish_status()
        return response

    def _set_gripper(self, request, response):
        self.gripper_target = self._clamp(float(request.position), 0.0, self.gripper_open)
        self.gravity_comp_active = False
        self.state_machine = "LOWLEVEL_STREAMING"
        response.success = True
        response.reached_position = float(self.gripper_position)
        self.publish_status()
        return response

    def _start_gravity_compensation(self, _request, response):
        self.enabled = True
        self.gravity_comp_active = True
        for name in self.joint_names:
            self.targets[name] = self.positions[name]
        self.state_machine = "GRAVITY_COMP"
        response.success = True
        response.message = "fake gravity compensation started"
        self.publish_status()
        return response

    def _stop_gravity_compensation(self, _request, response):
        was_active = self.gravity_comp_active
        self.gravity_comp_active = False
        self.state_machine = "IDLE"
        response.success = True
        response.message = (
            "fake gravity compensation stopped"
            if was_active
            else "fake gravity compensation was not active"
        )
        self.publish_status()
        return response

    def _gravity_compensation_status(self, _request, response):
        response.success = bool(self.gravity_comp_active)
        response.message = (
            "fake gravity compensation active"
            if self.gravity_comp_active
            else "fake gravity compensation inactive"
        )
        self.publish_status()
        return response

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 0.001)
        self.last_time = now

        any_motion = False
        for name in self.joint_names:
            before = self.positions[name]
            self.positions[name] = self._step_towards(
                before,
                self.targets[name],
                self.max_joint_speed * dt,
            )
            self.velocities[name] = (self.positions[name] - before) / dt
            any_motion = any_motion or abs(self.targets[name] - self.positions[name]) > 0.002

        before_gripper = self.gripper_position
        self.gripper_position = self._step_towards(
            before_gripper,
            self.gripper_target,
            self.max_gripper_speed * dt,
        )
        self.gripper_velocity = (self.gripper_position - before_gripper) / dt
        any_motion = any_motion or abs(self.gripper_target - self.gripper_position) > 0.001

        if self.enabled and not any_motion and self.state_machine == "LOWLEVEL_STREAMING":
            self.state_machine = "IDLE"
            self.publish_status()

        self._publish_joint_states(now)

    def _publish_joint_states(self, now) -> None:
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.joint_names)
        msg.position = [self.positions[name] for name in self.joint_names]
        msg.velocity = [self.velocities[name] for name in self.joint_names]
        msg.effort = [0.0 for _name in self.joint_names]

        gripper_joint_position = float(self.gripper_position * _GRIPPER_HALF_VISUAL_SCALE)
        gripper_joint_velocity = float(self.gripper_velocity * _GRIPPER_HALF_VISUAL_SCALE)
        msg.name.append("finger_left")
        msg.position.append(gripper_joint_position)
        msg.velocity.append(gripper_joint_velocity)
        msg.effort.append(0.0)
        self.joint_state_pub.publish(msg)

        for name in self.joint_names:
            state = JointMotorState()
            state.header = msg.header
            state.joint_name = name
            state.position = float(self.positions[name])
            state.velocity = float(self.velocities[name])
            state.torque = 0.0
            state.status_code = 0
            self.joint_motor_pubs[name].publish(state)

        gripper_state = JointMotorState()
        gripper_state.header = msg.header
        gripper_state.joint_name = "gripper"
        gripper_state.position = float(self.gripper_position)
        gripper_state.velocity = float(self.gripper_velocity)
        gripper_state.torque = 0.0
        gripper_state.status_code = 0
        self.gripper_state_pub.publish(gripper_state)

    def publish_status(self) -> None:
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = "fake_pos_vel"
        msg.enabled = bool(self.enabled)
        msg.control_loop_active = bool(self.enabled)
        msg.state_machine = self.state_machine
        msg.joint_names = list(self.joint_names)
        msg.per_joint_status_code = [0 for _name in self.joint_names]
        msg.error_codes = []
        self.status_pub.publish(msg)

    @staticmethod
    def _step_towards(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if math.isclose(delta, 0.0, abs_tol=1e-9):
            return target
        return current + FakeReBotArmDriver._clamp(delta, -max_step, max_step)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeReBotArmDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
