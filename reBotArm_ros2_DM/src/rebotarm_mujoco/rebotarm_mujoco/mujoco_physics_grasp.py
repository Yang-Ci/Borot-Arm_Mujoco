from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
import json
from pathlib import Path
import threading

import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .real2sim_sync import JointMapping, MujocoReal2Sim, _DEFAULT_GRASP_OBJECTS


_CONTROLLED_ROS_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "finger_left")


@dataclass(frozen=True)
class PhysicsJoint:
    ros: str
    mujoco: str
    qpos_addr: int
    dof_addr: int
    scale: float
    offset: float
    kp: float
    kd: float
    force_limit: float
    lower: float | None = None
    upper: float | None = None

    def ros_to_mujoco_position(self, value: float) -> float:
        mapped = float(value) * self.scale + self.offset
        if self.lower is not None:
            mapped = max(mapped, self.lower)
        if self.upper is not None:
            mapped = min(mapped, self.upper)
        return mapped

    def ros_to_mujoco_velocity(self, value: float) -> float:
        return float(value) * self.scale

    def mujoco_to_ros_position(self, value: float) -> float:
        if abs(self.scale) < 1e-9:
            return float(value)
        return (float(value) - self.offset) / self.scale

    def mujoco_to_ros_velocity(self, value: float) -> float:
        if abs(self.scale) < 1e-9:
            return float(value)
        return float(value) / self.scale


class MujocoPhysicsGrasp(Node):
    """Run MuJoCo dynamics for contact/friction-based object grasping."""

    def __init__(self) -> None:
        super().__init__("mujoco_physics_grasp")

        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_map_file", "")
        self.declare_parameter("target_joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("sim_joint_state_topic", "/rebotarm/mujoco/physics_joint_states")
        self.declare_parameter("object_states_topic", "/rebotarm/mujoco/object_states")
        self.declare_parameter("object_names", _DEFAULT_GRASP_OBJECTS)
        self.declare_parameter("open_viewer", True)
        self.declare_parameter("control_hz", 500.0)
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("arm_kp", [140.0, 140.0, 110.0, 55.0, 38.0, 28.0])
        self.declare_parameter("arm_kd", [7.0, 7.0, 5.5, 2.5, 1.8, 1.4])
        self.declare_parameter("arm_torque_limit", 30.0)
        self.declare_parameter("gripper_kp", 1800.0)
        self.declare_parameter("gripper_kd", 18.0)
        self.declare_parameter("gripper_force_limit", 32.0)

        self.model_path = MujocoReal2Sim._resolve_package_path(
            str(self.get_parameter("model_path").value or ""),
            "models",
            "rebotarm_b601_colored.xml",
        )
        joint_map_path = MujocoReal2Sim._resolve_package_path(
            str(self.get_parameter("joint_map_file").value or ""),
            "config",
            "joint_map_kinematic.yaml",
        )
        self.target_topic = str(self.get_parameter("target_joint_state_topic").value)
        self.sim_topic = str(self.get_parameter("sim_joint_state_topic").value)
        self.object_states_topic = str(self.get_parameter("object_states_topic").value)
        self.object_names = [
            name.strip()
            for name in str(self.get_parameter("object_names").value or _DEFAULT_GRASP_OBJECTS).split(",")
            if name.strip()
        ]
        self.open_viewer = MujocoReal2Sim._as_bool(self.get_parameter("open_viewer").value)
        self.control_hz = max(float(self.get_parameter("control_hz").value), 1.0)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.arm_kp = self._read_vector_parameter("arm_kp", 6)
        self.arm_kd = self._read_vector_parameter("arm_kd", 6)
        self.arm_torque_limit = max(float(self.get_parameter("arm_torque_limit").value), 0.0)
        self.gripper_kp = max(float(self.get_parameter("gripper_kp").value), 0.0)
        self.gripper_kd = max(float(self.get_parameter("gripper_kd").value), 0.0)
        self.gripper_force_limit = max(float(self.get_parameter("gripper_force_limit").value), 0.0)

        self.model = MujocoReal2Sim._load_mujoco_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.gravity_data = mujoco.MjData(self.model)
        self.joint_map = MujocoReal2Sim._load_joint_map(joint_map_path)
        self.joints = self._build_physics_joints()
        self.object_qpos_addrs = self._build_object_qpos_addrs()

        self._lock = threading.Lock()
        self._target_q = np.array([self.data.qpos[joint.qpos_addr] for joint in self.joints], dtype=np.float64)
        self._target_qd = np.zeros(len(self.joints), dtype=np.float64)
        self._last_tau = np.zeros(len(self.joints), dtype=np.float64)
        self._viewer = None

        self.sim_pub = self.create_publisher(JointState, self.sim_topic, qos_profile_sensor_data)
        self.object_pub = self.create_publisher(String, self.object_states_topic, 10)
        self.create_subscription(
            JointState,
            self.target_topic,
            self._target_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.control_hz, self._control_timer)
        self.create_timer(1.0 / self.publish_hz, self._publish_timer)

        if self.open_viewer:
            self._open_viewer_window()

        self.get_logger().info(
            "MuJoCo physics grasp ready: "
            f"model={self.model_path}, target={self.target_topic}, "
            f"sim_joint_states={self.sim_topic}, objects={self.object_names}"
        )

    def _read_vector_parameter(self, name: str, length: int) -> np.ndarray:
        raw = self.get_parameter(name).value
        values = list(raw) if isinstance(raw, (list, tuple)) else [float(raw)]
        if not values:
            values = [0.0]
        if len(values) < length:
            values.extend([values[-1]] * (length - len(values)))
        return np.array([float(value) for value in values[:length]], dtype=np.float64)

    def _build_physics_joints(self) -> list[PhysicsJoint]:
        joints: list[PhysicsJoint] = []
        for ros_index, ros_name in enumerate(_CONTROLLED_ROS_JOINTS):
            mappings = self.joint_map.get(ros_name, [JointMapping(ros_name)])
            for mapping in mappings:
                joint_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    mapping.mujoco,
                )
                if joint_id < 0:
                    raise RuntimeError(f"MuJoCo joint {mapping.mujoco!r} was not found")
                joint_type = int(self.model.jnt_type[joint_id])
                if joint_type not in (
                    int(mujoco.mjtJoint.mjJNT_HINGE),
                    int(mujoco.mjtJoint.mjJNT_SLIDE),
                ):
                    raise RuntimeError(f"MuJoCo joint {mapping.mujoco!r} must be hinge or slide")
                is_gripper = ros_name == "finger_left"
                joints.append(
                    PhysicsJoint(
                        ros=ros_name,
                        mujoco=mapping.mujoco,
                        qpos_addr=int(self.model.jnt_qposadr[joint_id]),
                        dof_addr=int(self.model.jnt_dofadr[joint_id]),
                        scale=float(mapping.scale),
                        offset=float(mapping.offset),
                        kp=self.gripper_kp if is_gripper else float(self.arm_kp[ros_index]),
                        kd=self.gripper_kd if is_gripper else float(self.arm_kd[ros_index]),
                        force_limit=self.gripper_force_limit if is_gripper else self.arm_torque_limit,
                        lower=float(self.model.jnt_range[joint_id][0])
                        if self.model.jnt_limited[joint_id]
                        else None,
                        upper=float(self.model.jnt_range[joint_id][1])
                        if self.model.jnt_limited[joint_id]
                        else None,
                    )
                )
        return joints

    def _build_object_qpos_addrs(self) -> dict[str, int]:
        result: dict[str, int] = {}
        for name in self.object_names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id < 0:
                self.get_logger().warn(f"physics object body {name!r} was not found")
                continue
            joint_addr = int(self.model.body_jntadr[body_id])
            joint_count = int(self.model.body_jntnum[body_id])
            for joint_id in range(joint_addr, joint_addr + joint_count):
                if int(self.model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
                    result[name] = int(self.model.jnt_qposadr[joint_id])
                    break
            if name not in result:
                self.get_logger().warn(f"physics object {name!r} has no freejoint")
        return result

    def _open_viewer_window(self) -> None:
        import mujoco.viewer

        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._viewer.sync()

    def _target_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        with self._lock:
            for index, joint in enumerate(self.joints):
                msg_index = name_to_index.get(joint.ros)
                if msg_index is None or msg_index >= len(msg.position):
                    continue
                self._target_q[index] = joint.ros_to_mujoco_position(msg.position[msg_index])
                if msg_index < len(msg.velocity):
                    self._target_qd[index] = joint.ros_to_mujoco_velocity(msg.velocity[msg_index])

    def _control_timer(self) -> None:
        if self._viewer is not None and not self._viewer.is_running():
            self.get_logger().info("MuJoCo physics viewer closed")
            rclpy.shutdown()
            return

        viewer_lock = self._viewer.lock() if self._viewer is not None else nullcontext()
        with viewer_lock:
            with self._lock:
                target_q = self._target_q.copy()
                target_qd = self._target_qd.copy()

            self.data.qfrc_applied[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            for index, joint in enumerate(self.joints):
                q = float(self.data.qpos[joint.qpos_addr])
                qd = float(self.data.qvel[joint.dof_addr])
                tau = float(self.data.qfrc_bias[joint.dof_addr])
                tau += joint.kp * (target_q[index] - q) + joint.kd * (target_qd[index] - qd)
                if joint.force_limit > 0.0:
                    tau = float(np.clip(tau, -joint.force_limit, joint.force_limit))
                self.data.qfrc_applied[joint.dof_addr] = tau
                self._last_tau[index] = tau
            mujoco.mj_step(self.model, self.data)

    def _publish_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(_CONTROLLED_ROS_JOINTS)

        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []
        for ros_name in _CONTROLLED_ROS_JOINTS:
            joint_index = next(index for index, joint in enumerate(self.joints) if joint.ros == ros_name)
            joint = self.joints[joint_index]
            positions.append(joint.mujoco_to_ros_position(self.data.qpos[joint.qpos_addr]))
            velocities.append(joint.mujoco_to_ros_velocity(self.data.qvel[joint.dof_addr]))
            efforts.append(float(self._last_tau[joint_index]))
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self.sim_pub.publish(msg)
        self._publish_object_states(stamp)

        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def _publish_object_states(self, stamp) -> None:
        objects = []
        for name, qpos_addr in self.object_qpos_addrs.items():
            qpos = self.data.qpos[qpos_addr : qpos_addr + 7]
            objects.append(
                {
                    "name": name,
                    "position": [float(value) for value in qpos[:3]],
                    "quat_wxyz": [float(value) for value in qpos[3:7]],
                }
            )
        msg = String()
        msg.data = json.dumps(
            {
                "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
                "objects": objects,
            },
            ensure_ascii=False,
        )
        self.object_pub.publish(msg)

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoPhysicsGrasp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
