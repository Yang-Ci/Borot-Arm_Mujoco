from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState


_GRIPPER_COMMAND_OPEN_M = 0.09
_GRIPPER_VISUAL_OPEN_M = 0.057
_GRIPPER_HALF_VISUAL_SCALE = _GRIPPER_VISUAL_OPEN_M / (_GRIPPER_COMMAND_OPEN_M * 2.0)


class JointStatePublisher:
    def __init__(self, node, hardware, namespace: str, rate_hz: float) -> None:
        self._node = node
        self._hardware = hardware
        self._publisher = node.create_publisher(
            JointState,
            f"/{namespace}/joint_states",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._joint_state_publishers = {
            name: node.create_publisher(
                JointMotorState,
                f"/{namespace}/joints/{name}/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
            for name in hardware.joint_names
        }
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = node.create_publisher(
            ArmStatus,
            f"/{namespace}/arm_status",
            latched_qos,
            callback_group=node.reentrant_group,
        )
        self._gripper_state_publisher = None
        if hardware.has_gripper:
            self._gripper_state_publisher = node.create_publisher(
                JointMotorState,
                f"/{namespace}/gripper/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
        period = 1.0 / max(float(rate_hz), 1.0)
        self._timer = node.create_timer(
            period,
            self.publish,
            callback_group=node.reentrant_group,
        )
        self.publish_status()

    def publish(self) -> None:
        try:
            pos, vel, effort = self._hardware.get_joint_state()
        except Exception as exc:
            self._node.get_logger().warn(f"joint state read failed: {exc}")
            return

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = self._hardware.joint_names
        msg.position = [float(v) for v in pos]
        msg.velocity = [float(v) for v in vel]
        msg.effort = [float(v) for v in effort]

        if self._hardware.has_gripper:
            g_pos_m = self._hardware.gripper_position_m()
            g_vel_m = self._hardware.gripper_velocity_m()
            msg.name = list(msg.name) + ["finger_left"]
            msg.position = list(msg.position) + [float(g_pos_m) * _GRIPPER_HALF_VISUAL_SCALE]
            msg.velocity = list(msg.velocity) + [float(g_vel_m) * _GRIPPER_HALF_VISUAL_SCALE]
            msg.effort = list(msg.effort) + [0.0]
        self._publisher.publish(msg)

        status_codes = self._hardware.get_joint_status_codes()
        for i, name in enumerate(self._hardware.joint_names):
            motor_msg = JointMotorState()
            motor_msg.header = msg.header
            motor_msg.joint_name = name
            motor_msg.position = float(pos[i])
            motor_msg.velocity = float(vel[i])
            motor_msg.torque = float(effort[i])
            motor_msg.status_code = int(status_codes[i])
            self._joint_state_publishers[name].publish(motor_msg)

        if self._gripper_state_publisher is not None:
            _g_pos, _g_vel, g_torque, g_status = self._hardware.get_gripper_state()
            gripper_msg = JointMotorState()
            gripper_msg.header = msg.header
            gripper_msg.joint_name = "gripper"
            gripper_msg.position = float(self._hardware.gripper_position_m())
            gripper_msg.velocity = float(self._hardware.gripper_velocity_m())
            gripper_msg.torque = float(g_torque)
            gripper_msg.status_code = int(g_status)
            self._gripper_state_publisher.publish(gripper_msg)

    def publish_status(self) -> None:
        msg = ArmStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.mode = self._hardware.mode
        msg.enabled = self._hardware.enabled
        msg.control_loop_active = self._hardware.control_loop_active
        msg.state_machine = self._hardware.state_machine
        msg.joint_names = self._hardware.joint_names
        msg.per_joint_status_code = self._hardware.get_joint_status_codes()
        msg.error_codes = self._hardware.error_codes
        self._status_publisher.publish(msg)
