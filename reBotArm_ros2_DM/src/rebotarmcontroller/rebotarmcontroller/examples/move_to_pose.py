#!/usr/bin/env python3
"""End-effector move_to_pose demo for the reBotArm ROS controller."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.action import MoveToPose
from sensor_msgs.msg import JointState


class DemoMoveToPose(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("move_to_pose")
        self._namespace = args.namespace.strip("/")
        self._target = args
        self._latest_joint_state: JointState | None = None
        self.create_subscription(
            JointState,
            f"/{self._namespace}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self._move_to_pose = ActionClient(
            self,
            MoveToPose,
            f"/{self._namespace}/move_to_pose",
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def run(self) -> bool:
        if not self._wait_for_joint_state():
            self.get_logger().error("joint_states not available")
            return False

        if not self._move_to_pose.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_to_pose action not available")
            return False

        goal = MoveToPose.Goal()
        goal.target_pose = Pose()
        goal.target_pose.position.x = float(self._target.x)
        goal.target_pose.position.y = float(self._target.y)
        goal.target_pose.position.z = float(self._target.z)
        goal.target_pose.orientation.x = float(self._target.qx)
        goal.target_pose.orientation.y = float(self._target.qy)
        goal.target_pose.orientation.z = float(self._target.qz)
        goal.target_pose.orientation.w = float(self._target.qw)
        goal.duration = float(self._target.duration)

        send_future = self._move_to_pose.send_goal_async(
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
        self.get_logger().info(f"success={result.success} message={result.message}")
        return bool(result.success)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"progress={feedback.progress:.2f} elapsed={feedback.time_elapsed:.2f}s"
        )

    def _wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self._latest_joint_state is None:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state is not None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--x", type=float, default=0.30)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.30)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = DemoMoveToPose(args)
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
