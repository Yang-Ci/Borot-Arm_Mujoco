from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
import mujoco
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import JointMotorCmd
from rebotarm_msgs.srv import MoveToPoseIK
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .real2sim_sync import JointMapping, MujocoReal2Sim


_ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
_DEFAULT_TARGET = np.array([0.34, 0.0, 0.20], dtype=np.float64)


@dataclass(frozen=True)
class SimJoint:
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

    def clamp_ros(self, value: float) -> float:
        mapped = self.ros_to_mujoco_position(value)
        return self.mujoco_to_ros_position(mapped)


class MujocoSimTaskServer(Node):
    """Simulation-only task tools: IK, smooth trajectories, target pose, records."""

    def __init__(self) -> None:
        super().__init__("mujoco_sim_task_server")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_map_file", "")
        self.declare_parameter("joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("target_pose_topic", "/rebotarm/mujoco/target_pose")
        self.declare_parameter("record_hz", 30.0)
        self.declare_parameter("command_hz", 60.0)
        self.declare_parameter("max_joint_speed", 1.4)
        self.declare_parameter("ik_iterations", 360)
        self.declare_parameter("ik_tolerance", 0.004)
        self.declare_parameter("ik_damping", 0.035)
        self.declare_parameter("ik_orientation_weight", 0.75)
        self.declare_parameter("ik_orientation_tolerance", 0.07)
        self.declare_parameter("records_dir", "")

        self.namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.record_hz = max(float(self.get_parameter("record_hz").value), 1.0)
        self.command_hz = max(float(self.get_parameter("command_hz").value), 5.0)
        self.max_joint_speed = max(float(self.get_parameter("max_joint_speed").value), 0.05)
        self.ik_iterations = max(int(self.get_parameter("ik_iterations").value), 1)
        self.ik_tolerance = max(float(self.get_parameter("ik_tolerance").value), 0.001)
        self.ik_damping = max(float(self.get_parameter("ik_damping").value), 0.001)
        self.ik_orientation_weight = max(
            float(self.get_parameter("ik_orientation_weight").value),
            0.0,
        )
        self.ik_orientation_tolerance = max(
            float(self.get_parameter("ik_orientation_tolerance").value),
            0.01,
        )
        records_dir = str(self.get_parameter("records_dir").value or "").strip()
        self.records_dir = Path(records_dir).expanduser() if records_dir else Path.home() / ".ros" / "rebotarm_sim_records"

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
        self.model = MujocoReal2Sim._load_mujoco_model(self.model_path)
        self.ik_data = mujoco.MjData(self.model)
        self.base_qpos = np.array(self.ik_data.qpos, dtype=np.float64)
        self.joint_map = MujocoReal2Sim._load_joint_map(joint_map_path)
        self.joints = self._build_joints()
        self.joint_names = [joint.ros for joint in self.joints]
        self.tcp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        if self.tcp_site_id < 0:
            raise RuntimeError(f"site 'tcp' was not found in {self.model_path}")

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.command_pubs = {
            name: self.create_publisher(
                JointMotorCmd,
                f"/{self.namespace}/joints/{name}/cmd",
                reliable_qos,
            )
            for name in self.joint_names
        }
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            self.target_pose_topic,
            reliable_qos,
        )

        self._lock = threading.RLock()
        self._current_q = np.zeros(len(self.joints), dtype=np.float64)
        self._last_target_pose = _DEFAULT_TARGET.copy()
        self._recording = False
        self._record_started = 0.0
        self._record_last_sample = 0.0
        self._record_samples: list[dict[str, float]] = []
        self._last_saved_record: Path | None = None
        self._replay_cancel = threading.Event()

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_timer(
            1.0 / self.record_hz,
            self._record_timer,
            callback_group=self.callback_group,
        )

        self.create_service(
            MoveToPoseIK,
            f"/{self.namespace}/move_to_pose_ik",
            self._handle_ik,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/start",
            self._start_record,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/stop",
            self._stop_record,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/replay",
            self._replay_record,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/clear",
            self._clear_record,
            callback_group=self.callback_group,
        )

        self._move_to_pose_server = ActionServer(
            self,
            MoveToPose,
            f"/{self.namespace}/move_to_pose",
            execute_callback=self._execute_move_to_pose,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self.callback_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self.namespace}/follow_joint_trajectory",
            execute_callback=self._execute_follow_joint_trajectory,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "MuJoCo sim task server ready: "
            f"namespace=/{self.namespace}, model={self.model_path}, "
            f"joints={self.joint_names}, records={self.records_dir}"
        )

    def _build_joints(self) -> list[SimJoint]:
        joints: list[SimJoint] = []
        for ros_name in _ARM_JOINTS:
            mappings = self.joint_map.get(ros_name, [JointMapping(ros_name)])
            mapping = mappings[0]
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                mapping.mujoco,
            )
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo joint {mapping.mujoco!r} was not found")
            joints.append(
                SimJoint(
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

    def _accept_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        self._replay_cancel.set()
        return CancelResponse.ACCEPT

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        by_name = {name: index for index, name in enumerate(msg.name)}
        updates = self._current_q.copy()
        changed = False
        for index, joint in enumerate(self.joints):
            msg_index = by_name.get(joint.ros)
            if msg_index is None or msg_index >= len(msg.position):
                continue
            updates[index] = joint.clamp_ros(float(msg.position[msg_index]))
            changed = True
        if changed:
            with self._lock:
                self._current_q = updates

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._last_target_pose = np.array(
                [
                    float(msg.pose.position.x),
                    float(msg.pose.position.y),
                    float(msg.pose.position.z),
                ],
                dtype=np.float64,
            )

    def _handle_ik(self, request, response):
        target = self._pose_to_xyz(request.target_pose)
        target_mat = self._pose_to_matrix_if_requested(request.target_pose)
        start_q = self._current_positions()
        solution, error, orientation_error = self._solve_ik(target, start_q, target_mat)
        response.q_solution = [float(value) for value in solution]
        response.success = bool(
            error <= self.ik_tolerance
            and orientation_error <= self._active_orientation_tolerance(target_mat)
        )
        response.message = (
            f"IK success, error={error * 1000:.1f} mm, orient={orientation_error:.3f}"
            if response.success
            else f"IK best effort, error={error * 1000:.1f} mm, orient={orientation_error:.3f}"
        )
        return response

    def _execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()
        target = self._pose_to_xyz(goal.target_pose)
        target_mat = self._pose_to_matrix_if_requested(goal.target_pose)
        start_q = self._current_positions()
        solution, error, orientation_error = self._solve_ik(target, start_q, target_mat)
        if (
            error > self.ik_tolerance
            or orientation_error > self._active_orientation_tolerance(target_mat)
        ):
            goal_handle.abort()
            result.success = False
            result.message = f"IK failed, error={error * 1000:.1f} mm, orient={orientation_error:.3f}"
            result.final_pose = self._pose_from_xyz(self._fk(start_q))
            return result

        duration = max(float(goal.duration), 0.2)
        ok = self._execute_joint_path(
            [start_q, solution],
            [0.0, duration],
            goal_handle=goal_handle,
            feedback_type=MoveToPose,
        )
        final_q = self._current_positions()
        result.final_pose = self._pose_from_xyz(self._fk(final_q))
        if not ok:
            goal_handle.canceled()
            result.success = False
            result.message = "canceled"
            return result

        goal_handle.succeed()
        result.success = True
        result.message = f"move_to_pose complete, IK error={error * 1000:.1f} mm"
        return result

    def _execute_follow_joint_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        if not trajectory.joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        try:
            qs, times = self._trajectory_to_full_path(
                list(trajectory.joint_names),
                list(trajectory.points),
            )
        except Exception as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        ok = self._execute_joint_path(
            qs,
            times,
            goal_handle=goal_handle,
            feedback_type=FollowJointTrajectory,
            trajectory_joint_names=list(trajectory.joint_names),
        )
        if not ok:
            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "canceled"
            return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "follow_joint_trajectory complete"
        return result

    def _trajectory_to_full_path(
        self,
        joint_names: list[str],
        points: list[JointTrajectoryPoint],
    ) -> tuple[list[np.ndarray], list[float]]:
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("joint_names must not contain duplicates")

        known = set(self.joint_names)
        requested = set(joint_names)
        if not requested.issubset(known):
            unknown = ", ".join(sorted(requested - known))
            raise ValueError(f"unknown trajectory joints: {unknown}")

        current = self._current_positions()
        qs = [current.copy()]
        times = [0.0]
        last_time = 0.0
        for point in points:
            if len(point.positions) != len(joint_names):
                raise ValueError("point.positions length must match joint_names")
            next_q = qs[-1].copy()
            for name, value in zip(joint_names, point.positions):
                index = self.joint_names.index(name)
                next_q[index] = self.joints[index].clamp_ros(float(value))
            t = self._ros_time_to_seconds(point.time_from_start)
            if t < last_time - 1e-9:
                raise ValueError("time_from_start must be nondecreasing")
            if t <= 1e-9 and len(qs) == 1:
                qs[0] = next_q
                continue
            qs.append(next_q)
            times.append(max(t, last_time + 1.0 / self.command_hz))
            last_time = times[-1]
        return qs, times

    def _execute_joint_path(
        self,
        qs: list[np.ndarray],
        times: list[float],
        *,
        goal_handle=None,
        feedback_type=None,
        trajectory_joint_names: list[str] | None = None,
    ) -> bool:
        if len(qs) < 2 or len(qs) != len(times):
            return True
        started = time.monotonic()
        period = 1.0 / self.command_hz
        self._replay_cancel.clear()

        for segment in range(1, len(qs)):
            q0 = qs[segment - 1]
            q1 = qs[segment]
            t0 = times[segment - 1]
            t1 = max(times[segment], t0 + period)
            while True:
                if self._replay_cancel.is_set() or (
                    goal_handle is not None and goal_handle.is_cancel_requested
                ):
                    return False
                now = time.monotonic() - started
                ratio = self._clamp((now - t0) / (t1 - t0), 0.0, 1.0)
                eased = ratio * ratio * (3.0 - 2.0 * ratio)
                q = q0 + (q1 - q0) * eased
                self._publish_joint_targets(q)
                self._publish_feedback(
                    goal_handle,
                    feedback_type,
                    q,
                    now,
                    total=max(times[-1], period),
                    trajectory_joint_names=trajectory_joint_names,
                )
                if ratio >= 1.0:
                    break
                time.sleep(period)
        with self._lock:
            self._current_q = qs[-1].copy()
        return True

    def _publish_feedback(
        self,
        goal_handle,
        feedback_type,
        q: np.ndarray,
        elapsed: float,
        *,
        total: float,
        trajectory_joint_names: list[str] | None,
    ) -> None:
        if goal_handle is None or feedback_type is None:
            return
        if feedback_type is MoveToPose:
            feedback = MoveToPose.Feedback()
            feedback.current_pose = self._pose_from_xyz(self._fk(q))
            feedback.progress = self._clamp(elapsed / max(total, 1e-6), 0.0, 1.0)
            feedback.time_elapsed = float(elapsed)
            goal_handle.publish_feedback(feedback)
            return
        if feedback_type is FollowJointTrajectory and trajectory_joint_names:
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(trajectory_joint_names)
            point = JointTrajectoryPoint()
            point.positions = [
                float(q[self.joint_names.index(name)]) for name in trajectory_joint_names
            ]
            point.velocities = [0.0 for _name in trajectory_joint_names]
            feedback.actual = point
            feedback.desired = point
            feedback.error = JointTrajectoryPoint()
            feedback.error.positions = [0.0 for _name in trajectory_joint_names]
            goal_handle.publish_feedback(feedback)

    def _solve_ik(
        self,
        target: np.ndarray,
        start_q: np.ndarray,
        target_mat: np.ndarray | None = None,
    ) -> tuple[np.ndarray, float, float]:
        q = start_q.copy()
        best_q = q.copy()
        pos, mat = self._fk_pose(q)
        best_position_error = float(np.linalg.norm(target - pos))
        best_orientation_error = self._orientation_error_norm(mat, target_mat)
        best_score = self._ik_score(best_position_error, best_orientation_error, target_mat)
        eps = 0.004

        for _ in range(self.ik_iterations):
            pos, mat = self._fk_pose(q)
            error_vec = target - pos
            position_error = float(np.linalg.norm(error_vec))
            orientation_vec = (
                self._orientation_error(mat, target_mat)
                if target_mat is not None and self.ik_orientation_weight > 0.0
                else np.zeros(3, dtype=np.float64)
            )
            orientation_error = float(np.linalg.norm(orientation_vec))
            score = self._ik_score(position_error, orientation_error, target_mat)
            if score < best_score:
                best_score = score
                best_position_error = position_error
                best_orientation_error = orientation_error
                best_q = q.copy()
            if (
                position_error <= self.ik_tolerance
                and orientation_error <= self._active_orientation_tolerance(target_mat)
            ):
                return q, position_error, orientation_error

            jacp = np.zeros((3, self.model.nv), dtype=np.float64)
            jacr = np.zeros((3, self.model.nv), dtype=np.float64)
            mujoco.mj_jacSite(self.model, self.ik_data, jacp, jacr, self.tcp_site_id)
            columns = [joint.dof_addr for joint in self.joints]
            jac = jacp[:, columns]
            solve_error = error_vec
            if target_mat is not None and self.ik_orientation_weight > 0.0:
                solve_error = np.concatenate(
                    [error_vec, self.ik_orientation_weight * orientation_vec]
                )
                jac = np.vstack([jac, self.ik_orientation_weight * jacr[:, columns]])

            a = jac @ jac.T + (self.ik_damping * self.ik_damping) * np.eye(jac.shape[0])
            try:
                delta = jac.T @ np.linalg.solve(a, solve_error)
            except np.linalg.LinAlgError:
                break

            norm = float(np.linalg.norm(delta))
            if norm > 0.10:
                delta *= 0.10 / norm
            for index, joint in enumerate(self.joints):
                q[index] = joint.clamp_ros(q[index] + delta[index])

        return best_q, best_position_error, best_orientation_error

    def _active_orientation_tolerance(self, target_mat: np.ndarray | None) -> float:
        if target_mat is None or self.ik_orientation_weight <= 0.0:
            return float("inf")
        return self.ik_orientation_tolerance

    def _ik_score(
        self,
        position_error: float,
        orientation_error: float,
        target_mat: np.ndarray | None,
    ) -> float:
        if target_mat is None or self.ik_orientation_weight <= 0.0:
            return position_error
        return float(np.hypot(position_error, self.ik_orientation_weight * orientation_error))

    def _fk(self, q: np.ndarray) -> np.ndarray:
        pos, _mat = self._fk_pose(q)
        return pos

    def _fk_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.ik_data.qpos[:] = self.base_qpos
        self.ik_data.qvel[:] = 0.0
        self.ik_data.qacc[:] = 0.0
        for index, joint in enumerate(self.joints):
            self.ik_data.qpos[joint.qpos_addr] = joint.ros_to_mujoco_position(q[index])
        mujoco.mj_forward(self.model, self.ik_data)
        return (
            np.array(self.ik_data.site_xpos[self.tcp_site_id], dtype=np.float64),
            np.array(self.ik_data.site_xmat[self.tcp_site_id], dtype=np.float64).reshape(3, 3),
        )

    def _publish_joint_targets(self, q: np.ndarray) -> None:
        for index, joint in enumerate(self.joints):
            msg = JointMotorCmd()
            msg.mode = 1
            msg.use_pos = True
            msg.use_vel = False
            msg.use_kp = False
            msg.use_kd = False
            msg.use_tau = False
            msg.use_vlim = True
            msg.pos = float(joint.clamp_ros(q[index]))
            msg.vel = 0.0
            msg.kp = 0.0
            msg.kd = 0.0
            msg.tau = 0.0
            msg.vlim = float(self.max_joint_speed)
            msg.stamp = self.get_clock().now().to_msg()
            self.command_pubs[joint.ros].publish(msg)

    def _publish_target_pose(self, target: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose = self._pose_from_xyz(target)
        self.target_pose_pub.publish(msg)
        with self._lock:
            self._last_target_pose = target.copy()

    def _record_timer(self) -> None:
        with self._lock:
            if not self._recording:
                return
            now = time.monotonic()
            if now - self._record_last_sample < 1.0 / self.record_hz:
                return
            self._record_last_sample = now
            q = self._current_q.copy()
            target = self._last_target_pose.copy()
            t = now - self._record_started

        sample = {"t": float(t)}
        for name, value in zip(self.joint_names, q):
            sample[name] = float(value)
        sample["target_x"] = float(target[0])
        sample["target_y"] = float(target[1])
        sample["target_z"] = float(target[2])
        with self._lock:
            self._record_samples.append(sample)

    def _start_record(self, _request, response):
        with self._lock:
            self._record_samples = []
            self._recording = True
            self._record_started = time.monotonic()
            self._record_last_sample = 0.0
        response.success = True
        response.message = "recording started"
        return response

    def _stop_record(self, _request, response):
        with self._lock:
            self._recording = False
            samples = list(self._record_samples)
        if not samples:
            response.success = False
            response.message = "no samples recorded"
            return response

        self.records_dir.mkdir(parents=True, exist_ok=True)
        path = self.records_dir / f"rebotarm_record_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        fields = ["t", *self.joint_names, "target_x", "target_y", "target_z"]
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(samples)
        self._last_saved_record = path
        response.success = True
        response.message = f"saved {len(samples)} samples to {path}"
        return response

    def _replay_record(self, _request, response):
        with self._lock:
            samples = list(self._record_samples)
            self._recording = False
        if len(samples) < 2:
            response.success = False
            response.message = "need at least two recorded samples to replay"
            return response

        self._replay_cancel.set()
        self._replay_cancel = threading.Event()
        thread = threading.Thread(target=self._replay_samples, args=(samples, self._replay_cancel), daemon=True)
        thread.start()
        response.success = True
        response.message = f"replaying {len(samples)} samples"
        return response

    def _clear_record(self, _request, response):
        self._replay_cancel.set()
        with self._lock:
            self._recording = False
            self._record_samples = []
        response.success = True
        response.message = "record cleared"
        return response

    def _replay_samples(self, samples: list[dict[str, float]], cancel_event: threading.Event) -> None:
        qs = [
            np.array([float(sample.get(name, 0.0)) for name in self.joint_names], dtype=np.float64)
            for sample in samples
        ]
        times = [float(sample.get("t", 0.0)) for sample in samples]
        started = time.monotonic()
        period = 1.0 / self.command_hz
        for index, q in enumerate(qs):
            if cancel_event.is_set():
                return
            wait_until = started + max(times[index], 0.0)
            while time.monotonic() < wait_until:
                if cancel_event.is_set():
                    return
                time.sleep(min(period, max(wait_until - time.monotonic(), 0.0)))
            self._publish_joint_targets(q)

    def _current_positions(self) -> np.ndarray:
        with self._lock:
            return self._current_q.copy()

    @staticmethod
    def _pose_to_xyz(pose: Pose) -> np.ndarray:
        return np.array(
            [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            dtype=np.float64,
        )

    @staticmethod
    def _pose_to_matrix_if_requested(pose: Pose) -> np.ndarray | None:
        quat = np.array(
            [
                float(pose.orientation.w),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quat))
        if norm < 1e-9:
            return None
        quat /= norm
        if np.allclose(quat, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-5):
            return None
        mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(mat, quat)
        return mat.reshape(3, 3)

    @staticmethod
    def _orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
        return 0.5 * (
            np.cross(current[:, 0], target[:, 0])
            + np.cross(current[:, 1], target[:, 1])
            + np.cross(current[:, 2], target[:, 2])
        )

    def _orientation_error_norm(
        self,
        current: np.ndarray,
        target: np.ndarray | None,
    ) -> float:
        if target is None or self.ik_orientation_weight <= 0.0:
            return 0.0
        return float(np.linalg.norm(self._orientation_error(current, target)))

    @staticmethod
    def _pose_from_xyz(xyz: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _ros_time_to_seconds(value) -> float:
        return float(value.sec) + float(value.nanosec) * 1e-9

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoSimTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
