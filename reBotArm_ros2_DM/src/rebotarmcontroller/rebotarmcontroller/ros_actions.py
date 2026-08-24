from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose
from trajectory_msgs.msg import JointTrajectoryPoint

from .conversions import pose_to_xyz_rpy

_FOLLOW_TRAJECTORY_START_TOL = 0.10
_FOLLOW_TRAJECTORY_SETTLE_TIMEOUT = 2.0
_FOLLOW_TRAJECTORY_GOAL_TOLERANCE = 0.03


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_move_to_pose,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_follow_joint_trajectory,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group,
        )

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_move_to_pose(self, _goal_handle):
        self._hardware.endpos_ctrl._stop_send.set()
        self._hardware.endpos_ctrl._moving = False
        return CancelResponse.ACCEPT

    def cancel_follow_joint_trajectory(self, _goal_handle):
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            self._hardware.set_state_machine("TRAJ_RUNNING")
            self._node.publish_arm_status()
            self._hardware.ensure_pos_vel_control()
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.endpos_ctrl.move_to_traj(
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
                float(goal.duration),
            )
        except Exception as exc:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.final_pose = self._hardware.current_pose()
            return result

        if not ok:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = "trajectory planning failed"
            result.final_pose = self._hardware.current_pose()
            return result

        start = time.monotonic()
        requested_duration = float(goal.duration)
        feedback = MoveToPose.Feedback()
        while bool(getattr(self._hardware.endpos_ctrl, "_moving", False)):
            if goal_handle.is_cancel_requested:
                self._hardware.endpos_ctrl._stop_send.set()
                self._hardware.endpos_ctrl._moving = False
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                result.final_pose = self._hardware.current_pose()
                return result

            if self._hardware.state_machine != "TRAJ_RUNNING":
                goal_handle.abort()
                result.success = False
                result.message = "preempted"
                result.final_pose = self._hardware.current_pose()
                return result

            feedback.current_pose = self._hardware.current_pose()
            elapsed = float(time.monotonic() - start)
            if requested_duration > 0.0:
                feedback.progress = max(0.0, min(1.0, elapsed / requested_duration))
            else:
                traj = getattr(self._hardware.endpos_ctrl, "_traj", [])
                if traj:
                    idx = float(getattr(self._hardware.endpos_ctrl, "_traj_idx", 0))
                    feedback.progress = max(0.0, min(1.0, idx / float(len(traj))))
                else:
                    feedback.progress = 1.0
            feedback.time_elapsed = elapsed
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        result.success = True
        result.message = "move_to_pose complete"
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory

        if not trajectory.joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        try:
            current = self._current_positions_for(list(trajectory.joint_names))
            sample_times, sample_positions = self._validated_trajectory(
                list(trajectory.joint_names),
                trajectory.points,
                current,
            )
        except Exception as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        trajectory_done = False
        start = time.monotonic()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)

        self._hardware.set_state_machine("TRAJ_RUNNING")
        self._node.publish_arm_status()
        try:
            self._hardware.ensure_pos_vel_control()

            for target_time, target in zip(sample_times[1:], sample_positions[1:]):
                if not self._wait_until_time(goal_handle, start + target_time, result):
                    return result

                self._set_endpos_target(list(trajectory.joint_names), target)

                desired = JointTrajectoryPoint()
                desired.positions = [float(v) for v in target]
                feedback.desired = desired
                feedback.actual = self._actual_point(list(trajectory.joint_names))
                feedback.error = self._error_point(desired, feedback.actual)
                goal_handle.publish_feedback(feedback)
            trajectory_done = True
        except Exception as exc:
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result
        finally:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()

        if not trajectory_done:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory interrupted"
            return result

        self._set_endpos_target(list(trajectory.joint_names), sample_positions[-1])
        ok, max_error = self._wait_until_goal_reached(
            list(trajectory.joint_names),
            sample_positions[-1],
            _FOLLOW_TRAJECTORY_GOAL_TOLERANCE,
        )
        if not ok:
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = (
                "trajectory goal not reached within tolerance "
                f"(max error {max_error:.3f} rad > "
                f"{_FOLLOW_TRAJECTORY_GOAL_TOLERANCE:.3f} rad)"
            )
            return result
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "follow_joint_trajectory complete"
        return result

    def _set_endpos_target(self, joint_names: list[str], positions: np.ndarray) -> None:
        if set(joint_names) != set(self._hardware.joint_names):
            raise ValueError(f"trajectory joints must match {self._hardware.joint_names}")
        by_name = {name: float(pos) for name, pos in zip(joint_names, positions)}
        ordered = np.array(
            [by_name[name] for name in self._hardware.joint_names],
            dtype=np.float64,
        )
        self._hardware.endpos_ctrl._q_target[:] = ordered

    def _current_positions_for(self, joint_names: list[str]) -> np.ndarray:
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("joint_names must not contain duplicates")
        if set(joint_names) != set(self._hardware.joint_names):
            raise ValueError(
                f"trajectory joints must match {self._hardware.joint_names}"
            )
        current, _, _ = self._hardware.get_joint_state()
        by_name = {
            name: float(pos)
            for name, pos in zip(self._hardware.joint_names, current)
        }
        return np.array([by_name[name] for name in joint_names], dtype=np.float64)

    def _validated_trajectory(
        self,
        joint_names: list[str],
        points: list[JointTrajectoryPoint],
        current: np.ndarray,
    ) -> tuple[list[float], list[np.ndarray]]:
        sample_times = [0.0]
        sample_positions = [current.copy()]
        last_time = 0.0
        last_positions = current.copy()

        for index, point in enumerate(points, start=1):
            if len(point.positions) != len(joint_names):
                raise ValueError("point.positions length must match joint_names")

            point_time = float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) * 1e-9
            )
            if point_time < last_time - 1e-9:
                raise ValueError("trajectory time_from_start must be nondecreasing")
            positions = np.array(point.positions, dtype=np.float64)

            if index == 1:
                start_delta = float(np.max(np.abs(positions - current)))
                if start_delta > _FOLLOW_TRAJECTORY_START_TOL:
                    raise ValueError(
                        "first trajectory point is too far from current joint state "
                        f"(max delta {start_delta:.3f} rad)"
                    )
                if point_time <= 1e-9:
                    last_positions = current.copy()
                    continue

            sample_times.append(point_time)
            sample_positions.append(positions)
            last_time = point_time
            last_positions = positions

        return sample_times, sample_positions

    def _wait_until_time(self, goal_handle, target_time: float, result) -> bool:
        while time.monotonic() < target_time:
            if goal_handle.is_cancel_requested:
                self._hardware.hold_current_position()
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "canceled"
                return False
            if self._hardware.state_machine != "TRAJ_RUNNING":
                self._hardware.hold_current_position()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "preempted"
                return False
            time.sleep(0.01)
        return True

    def _wait_until_goal_reached(
        self,
        joint_names: list[str],
        target: np.ndarray,
        goal_tolerance: float,
    ) -> tuple[bool, float]:
        deadline = time.monotonic() + _FOLLOW_TRAJECTORY_SETTLE_TIMEOUT
        max_error = float("inf")
        while time.monotonic() < deadline:
            actual = self._current_positions_for(joint_names)
            max_error = float(np.max(np.abs(actual - target)))
            if max_error <= goal_tolerance:
                return True, max_error
            time.sleep(0.05)
        return False, max_error

    def _actual_point(self, joint_names: list[str] | None = None) -> JointTrajectoryPoint:
        pos, vel, _ = self._hardware.get_joint_state()
        if joint_names is not None:
            by_name = {
                name: i
                for i, name in enumerate(self._hardware.joint_names)
            }
            pos = [pos[by_name[name]] for name in joint_names]
            vel = [vel[by_name[name]] for name in joint_names]
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in pos]
        point.velocities = [float(v) for v in vel]
        return point

    @staticmethod
    def _error_point(desired: JointTrajectoryPoint, actual: JointTrajectoryPoint) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [
            float(a - d) for d, a in zip(desired.positions, actual.positions)
        ]
        if desired.velocities and actual.velocities:
            point.velocities = [
                float(a - d) for d, a in zip(desired.velocities, actual.velocities)
            ]
        return point

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position, goal.max_effort)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.gripper_position_m()
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.position = self._hardware.gripper_position_m()
                result.effort = self._hardware.get_gripper_state()[2]
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos = self._hardware.gripper_position_m()
            effort = self._hardware.get_gripper_state()[2]
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        result.position = self._hardware.gripper_position_m()
        result.effort = self._hardware.get_gripper_state()[2]
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result
