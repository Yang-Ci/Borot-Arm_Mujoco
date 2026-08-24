#!/usr/bin/env python3
"""Joint-space move_to demo for the reBotArm ROS controller."""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_NAMESPACE = "rebotarm"


def _duration_msg(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def _smoothstep(ratio: float) -> float:
    ratio = max(0.0, min(1.0, ratio))
    return 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5


class DemoMoveTo(Node):
    def __init__(
        self,
        target_positions: list[float],
        joint_name: str | None,
        joint_position: float | None,
        duration: float,
    ) -> None:
        super().__init__("move_to")
        self._target_positions = target_positions
        self._joint_name = joint_name
        self._joint_position = joint_position
        self._duration = max(float(duration), 0.1)
        self._latest_joint_state: JointState | None = None
        self._last_feedback_log = 0.0

        self.create_subscription(
            JointState,
            f"/{_NAMESPACE}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self._follow_joint_trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{_NAMESPACE}/follow_joint_trajectory",
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def run(self) -> bool:
        if not self._wait_for_joint_state():
            self.get_logger().error("joint_states not available")
            return False

        if not self._follow_joint_trajectory.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("follow_joint_trajectory action not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_trajectory()

        send_future = self._follow_joint_trajectory.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"error_code={result.error_code} message={result.error_string}"
        )
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def _wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self._latest_joint_state is None:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state is not None

    def _make_trajectory(self) -> JointTrajectory:
        assert self._latest_joint_state is not None

        joint_names = list(self._latest_joint_state.name)
        current = np.array(self._latest_joint_state.position, dtype=np.float64)
        target = self._resolve_target(current)

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points = []

        steps = max(2, int(self._duration / 0.05))
        for step in range(steps + 1):
            ratio = step / float(steps)
            blend = _smoothstep(ratio)
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in current + (target - current) * blend]
            point.time_from_start = _duration_msg(self._duration * ratio)
            trajectory.points.append(point)

        self.get_logger().info(
            "moving joints to "
            + ", ".join(f"{name}={value:+.3f}" for name, value in zip(joint_names, target))
        )
        return trajectory

    def _resolve_target(self, current: np.ndarray) -> np.ndarray:
        assert self._latest_joint_state is not None

        if self._joint_name is not None:
            if self._target_positions:
                raise ValueError("use either 6 joint positions or --joint/--position")
            if self._joint_position is None:
                raise ValueError("--joint requires --position")
            if self._joint_name not in self._latest_joint_state.name:
                raise ValueError(f"unknown joint: {self._joint_name}")
            target = current.copy()
            joint_index = self._latest_joint_state.name.index(self._joint_name)
            target[joint_index] = float(self._joint_position)
            return target

        if self._joint_position is not None:
            raise ValueError("--position requires --joint")
        if len(self._target_positions) != len(current):
            raise ValueError(
                f"expected {len(current)} absolute joint positions, "
                f"got {len(self._target_positions)}"
            )
        target = np.array(self._target_positions, dtype=np.float64)
        return target

    def _feedback_cb(self, feedback_msg) -> None:
        now = time.monotonic()
        if now - self._last_feedback_log < 0.5:
            return
        self._last_feedback_log = now
        actual = feedback_msg.feedback.actual.positions
        if actual:
            self.get_logger().info(
                "actual="
                + ", ".join(f"{value:+.3f}" for value in actual)
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "positions",
        nargs="*",
        type=float,
        help="absolute 6-axis joint target positions in radians",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--joint",
        help="single joint name, for example joint3",
    )
    parser.add_argument(
        "--position",
        type=float,
        help="absolute target position for --joint, in radians",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = DemoMoveTo(
        target_positions=args.positions,
        joint_name=args.joint,
        joint_position=args.position,
        duration=args.duration,
    )
    try:
        ok = node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
