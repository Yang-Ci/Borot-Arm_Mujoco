from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
import sys
import threading

import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .real2sim_sync import JointMapping, MujocoReal2Sim


_ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


@dataclass(frozen=True)
class TorqueJoint:
    ros: str
    mujoco: str
    qpos_addr: int
    dof_addr: int
    scale: float = 1.0
    offset: float = 0.0
    lower: float | None = None
    upper: float | None = None

    def ros_to_mujoco_position(self, value: float) -> float:
        mapped = float(value) * self.scale + self.offset
        if self.lower is not None:
            mapped = max(mapped, self.lower)
        if self.upper is not None:
            mapped = min(mapped, self.upper)
        return mapped

    def mujoco_to_ros_position(self, value: float) -> float:
        if abs(self.scale) < 1e-9:
            return float(value)
        return (float(value) - self.offset) / self.scale

    def mujoco_to_ros_velocity(self, value: float) -> float:
        if abs(self.scale) < 1e-9:
            return float(value)
        return float(value) / self.scale

    def mujoco_to_ros_tau(self, value: float) -> float:
        return float(value) * self.scale

    def ros_to_mujoco_tau(self, value: float) -> float:
        if abs(self.scale) < 1e-9:
            return float(value)
        return float(value) / self.scale


class MujocoTorqueControl(Node):
    """MuJoCo-only gravity torque comparison and torque closed-loop simulation."""

    def __init__(self) -> None:
        super().__init__("mujoco_torque_control")

        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_map_file", "")
        self.declare_parameter("target_joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("compare_joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("sim_joint_state_topic", "/rebotarm/mujoco/joint_states")
        self.declare_parameter("mujoco_tau_g_topic", "/rebotarm/mujoco/tau_g")
        self.declare_parameter("sdk_tau_g_topic", "/rebotarm/mujoco/sdk_tau_g")
        self.declare_parameter("tau_g_diff_topic", "/rebotarm/mujoco/tau_g_diff")
        self.declare_parameter("open_viewer", True)
        self.declare_parameter("control_hz", 500.0)
        self.declare_parameter("publish_hz", 60.0)
        self.declare_parameter("compare_log_hz", 2.0)
        self.declare_parameter("sdk_compare_enabled", True)
        self.declare_parameter("torque_limit", 18.0)
        self.declare_parameter("kp", [10.0, 10.0, 8.0, 4.0, 3.0, 2.0])
        self.declare_parameter("kd", [1.2, 1.2, 0.9, 0.35, 0.25, 0.2])

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
        self.compare_topic = str(self.get_parameter("compare_joint_state_topic").value)
        self.sim_topic = str(self.get_parameter("sim_joint_state_topic").value)
        self.mujoco_tau_topic = str(self.get_parameter("mujoco_tau_g_topic").value)
        self.sdk_tau_topic = str(self.get_parameter("sdk_tau_g_topic").value)
        self.diff_topic = str(self.get_parameter("tau_g_diff_topic").value)
        self.open_viewer = self._as_bool(self.get_parameter("open_viewer").value)
        self.control_hz = max(float(self.get_parameter("control_hz").value), 1.0)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.compare_log_hz = max(float(self.get_parameter("compare_log_hz").value), 0.0)
        self.sdk_compare_enabled = self._as_bool(
            self.get_parameter("sdk_compare_enabled").value
        )
        self.torque_limit = max(float(self.get_parameter("torque_limit").value), 0.0)

        self.model = MujocoReal2Sim._load_mujoco_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.gravity_data = mujoco.MjData(self.model)
        self.joint_map = MujocoReal2Sim._load_joint_map(joint_map_path)
        self.joints = self._build_torque_joints()
        self.joint_names = [joint.ros for joint in self.joints]
        self.kp = self._read_vector_parameter("kp", len(self.joints))
        self.kd = self._read_vector_parameter("kd", len(self.joints))

        self._lock = threading.Lock()
        self._target_q = self._read_sim_positions()
        self._target_qd = np.zeros(len(self.joints), dtype=np.float64)
        self._last_tau = np.zeros(len(self.joints), dtype=np.float64)
        self._last_compare_log = 0.0
        self._warned_missing_compare: set[str] = set()
        self._viewer = None
        self._sdk_gravity = self._load_sdk_gravity() if self.sdk_compare_enabled else None

        self.sim_pub = self.create_publisher(
            JointState,
            self.sim_topic,
            qos_profile_sensor_data,
        )
        self.mujoco_tau_pub = self.create_publisher(
            JointState,
            self.mujoco_tau_topic,
            qos_profile_sensor_data,
        )
        self.sdk_tau_pub = self.create_publisher(
            JointState,
            self.sdk_tau_topic,
            qos_profile_sensor_data,
        )
        self.diff_pub = self.create_publisher(
            JointState,
            self.diff_topic,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            JointState,
            self.target_topic,
            self._target_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.compare_topic,
            self._compare_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.control_hz, self._control_timer)
        self.create_timer(1.0 / self.publish_hz, self._publish_timer)

        if self.open_viewer:
            self._open_viewer_window()

        self.get_logger().info(
            "MuJoCo torque control ready: "
            f"model={self.model_path}, target_topic={self.target_topic}, "
            f"compare_topic={self.compare_topic}, joints={self.joint_names}, "
            f"sdk_compare={'on' if self._sdk_gravity is not None else 'off'}"
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _read_vector_parameter(self, name: str, length: int) -> np.ndarray:
        raw = self.get_parameter(name).value
        values = list(raw) if isinstance(raw, (list, tuple)) else [float(raw)]
        if not values:
            values = [0.0]
        if len(values) < length:
            values.extend([values[-1]] * (length - len(values)))
        return np.array([float(value) for value in values[:length]], dtype=np.float64)

    def _build_torque_joints(self) -> list[TorqueJoint]:
        joints: list[TorqueJoint] = []
        for ros_name in _ARM_JOINTS:
            mappings = self.joint_map.get(ros_name, [JointMapping(ros_name)])
            if not mappings:
                raise RuntimeError(f"no MuJoCo mapping configured for {ros_name!r}")
            mapping = mappings[0]
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                mapping.mujoco,
            )
            if joint_id < 0:
                raise RuntimeError(
                    f"MuJoCo joint {mapping.mujoco!r} for ROS joint {ros_name!r} "
                    f"was not found in {self.model_path}"
                )
            joint_type = int(self.model.jnt_type[joint_id])
            if joint_type not in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ):
                raise RuntimeError(
                    f"MuJoCo joint {mapping.mujoco!r} must be hinge or slide "
                    "for torque control"
                )
            joints.append(
                TorqueJoint(
                    ros=ros_name,
                    mujoco=mapping.mujoco,
                    qpos_addr=int(self.model.jnt_qposadr[joint_id]),
                    dof_addr=int(self.model.jnt_dofadr[joint_id]),
                    scale=mapping.scale,
                    offset=mapping.offset,
                    lower=float(self.model.jnt_range[joint_id][0])
                    if self.model.jnt_limited[joint_id]
                    else None,
                    upper=float(self.model.jnt_range[joint_id][1])
                    if self.model.jnt_limited[joint_id]
                    else None,
                )
            )
        return joints

    @staticmethod
    def _workspace_root() -> Path:
        return Path(__file__).resolve().parents[3]

    @classmethod
    def _sdk_candidates(cls) -> list[Path]:
        workspace = cls._workspace_root()
        return [
            workspace / "third_party" / "reBotArm_control_py",
            workspace / "sdk" / "reBotArm_control_py",
            Path.home() / "reBotArm_control_py",
            Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    def _load_sdk_gravity(self):
        for root in self._sdk_candidates():
            if not (root / "reBotArm_control_py").is_dir():
                continue
            root_str = str(root)
            if root_str not in sys.path:
                sys.path.insert(0, root_str)
            try:
                from reBotArm_control_py.dynamics import compute_generalized_gravity

                self.get_logger().info(f"SDK gravity model loaded from {root}")
                return compute_generalized_gravity
            except Exception as exc:
                self.get_logger().warn(f"SDK gravity import failed from {root}: {exc}")
        self.get_logger().warn("SDK gravity compare disabled: reBotArm_control_py not found")
        return None

    def _open_viewer_window(self) -> None:
        import mujoco.viewer

        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._viewer.sync()

    def _target_callback(self, msg: JointState) -> None:
        q, qd = self._joint_state_to_vectors(msg, require_all=False)
        if q is None:
            return
        with self._lock:
            mask = np.isfinite(q)
            self._target_q[mask] = q[mask]
            if qd is not None:
                vel_mask = np.isfinite(qd)
                self._target_qd[vel_mask] = qd[vel_mask]

    def _compare_callback(self, msg: JointState) -> None:
        q, _qd = self._joint_state_to_vectors(msg, require_all=True)
        if q is None:
            return

        tau_mujoco = self._compute_mujoco_tau_g(q)
        stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        self._publish_tau(self.mujoco_tau_pub, stamp, q, tau_mujoco)

        if self._sdk_gravity is None:
            return
        try:
            tau_sdk = np.asarray(self._sdk_gravity(q=q), dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f"SDK tau_g failed: {exc}")
            return
        tau_sdk = tau_sdk[: len(self.joints)]
        if tau_sdk.shape[0] != len(self.joints):
            self.get_logger().warn(
                f"SDK tau_g length {tau_sdk.shape[0]} does not match "
                f"{len(self.joints)} MuJoCo joints"
            )
            return

        diff = tau_mujoco - tau_sdk
        self._publish_tau(self.sdk_tau_pub, stamp, q, tau_sdk)
        self._publish_tau(self.diff_pub, stamp, q, diff)
        self._log_compare(tau_mujoco, tau_sdk, diff)

    def _joint_state_to_vectors(
        self,
        msg: JointState,
        *,
        require_all: bool,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        if not msg.name or not msg.position:
            return None, None

        name_to_index = {name: index for index, name in enumerate(msg.name)}
        q = np.full(len(self.joints), np.nan, dtype=np.float64)
        qd = np.full(len(self.joints), np.nan, dtype=np.float64)
        missing: list[str] = []

        for index, joint in enumerate(self.joints):
            msg_index = name_to_index.get(joint.ros)
            if msg_index is None or msg_index >= len(msg.position):
                missing.append(joint.ros)
                continue
            q[index] = float(msg.position[msg_index])
            if msg_index < len(msg.velocity):
                qd[index] = float(msg.velocity[msg_index])

        if missing and require_all:
            missing_key = ",".join(missing)
            if missing_key not in self._warned_missing_compare:
                self._warned_missing_compare.add(missing_key)
                self.get_logger().warn(
                    f"cannot compare tau_g until joint_states contains: {missing_key}"
                )
            return None, None
        return q, qd

    def _read_sim_positions(self) -> np.ndarray:
        return np.array(
            [
                joint.mujoco_to_ros_position(self.data.qpos[joint.qpos_addr])
                for joint in self.joints
            ],
            dtype=np.float64,
        )

    def _read_sim_velocities(self) -> np.ndarray:
        return np.array(
            [
                joint.mujoco_to_ros_velocity(self.data.qvel[joint.dof_addr])
                for joint in self.joints
            ],
            dtype=np.float64,
        )

    def _write_positions_to_data(self, data: mujoco.MjData, q_ros: np.ndarray) -> None:
        for index, joint in enumerate(self.joints):
            data.qpos[joint.qpos_addr] = joint.ros_to_mujoco_position(q_ros[index])

    def _compute_mujoco_tau_g(self, q_ros: np.ndarray) -> np.ndarray:
        with self._lock:
            self.gravity_data.qpos[:] = self.data.qpos
            self.gravity_data.qvel[:] = 0.0
            self.gravity_data.qacc[:] = 0.0
            self._write_positions_to_data(self.gravity_data, q_ros)
            mujoco.mj_forward(self.model, self.gravity_data)
            return np.array(
                [
                    joint.mujoco_to_ros_tau(self.gravity_data.qfrc_bias[joint.dof_addr])
                    for joint in self.joints
                ],
                dtype=np.float64,
            )

    def _control_timer(self) -> None:
        if self._viewer is not None and not self._viewer.is_running():
            self.get_logger().info("MuJoCo viewer closed")
            rclpy.shutdown()
            return

        viewer_lock = self._viewer.lock() if self._viewer is not None else nullcontext()
        with viewer_lock:
            q = self._read_sim_positions()
            qd = self._read_sim_velocities()
            with self._lock:
                target_q = self._target_q.copy()
                target_qd = self._target_qd.copy()
            tau_g = self._compute_mujoco_tau_g(q)
            tau = tau_g + self.kp * (target_q - q) + self.kd * (target_qd - qd)
            if self.torque_limit > 0.0:
                np.clip(tau, -self.torque_limit, self.torque_limit, out=tau)

            self.data.qfrc_applied[:] = 0.0
            for index, joint in enumerate(self.joints):
                self.data.qfrc_applied[joint.dof_addr] = joint.ros_to_mujoco_tau(tau[index])
            mujoco.mj_step(self.model, self.data)
            self._last_tau = tau

    def _publish_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        q = self._read_sim_positions()
        qd = self._read_sim_velocities()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [float(value) for value in q]
        msg.velocity = [float(value) for value in qd]
        msg.effort = [float(value) for value in self._last_tau]
        self.sim_pub.publish(msg)

        self._publish_tau(self.mujoco_tau_pub, stamp, q, self._compute_mujoco_tau_g(q))
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def _publish_tau(self, publisher, stamp, q: np.ndarray, tau: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [float(value) for value in q]
        msg.velocity = [0.0 for _joint in self.joints]
        msg.effort = [float(value) for value in tau]
        publisher.publish(msg)

    def _log_compare(
        self,
        tau_mujoco: np.ndarray,
        tau_sdk: np.ndarray,
        diff: np.ndarray,
    ) -> None:
        if self.compare_log_hz <= 0.0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        min_period = 1.0 / self.compare_log_hz
        if now - self._last_compare_log < min_period:
            return
        self._last_compare_log = now

        abs_diff = np.abs(diff)
        worst = int(np.argmax(abs_diff))
        self.get_logger().info(
            "tau_g compare: "
            f"max_abs={abs_diff[worst]:.4f} Nm at {self.joint_names[worst]}, "
            f"mujoco={tau_mujoco[worst]:+.4f}, sdk={tau_sdk[worst]:+.4f}, "
            f"rms={float(np.sqrt(np.mean(diff * diff))):.4f}"
        )

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoTorqueControl()
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
