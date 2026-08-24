from __future__ import annotations

import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState, RobotTrajectory
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveItDemoBase:
    """Shared ROS and MoveIt plumbing for the demo nodes."""

    def __init__(self, node_name: str) -> None:
        self.node = rclpy.create_node(
            node_name,
            automatically_declare_parameters_from_overrides=True,
        )
        self.group_name = str(self._param("group_name"))
        self.joint_names = [str(name) for name in self._param("joint_names")]
        self._latest_joint_positions: dict[str, float] = {}

        self._execute = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        self._ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )

    def _param(self, name: str):
        return self.node.get_parameter(name).value

    @staticmethod
    def duration(seconds: float) -> Duration:
        return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1_000_000_000))

    def wait_for_execute_server(self) -> bool:
        if self._execute.wait_for_server(timeout_sec=30.0):
            return True
        self.node.get_logger().error("MoveIt action /execute_trajectory is not available")
        return False

    def wait_for_ik_service(self) -> bool:
        if self._ik.wait_for_service(timeout_sec=30.0):
            return True
        self.node.get_logger().error("MoveIt service /compute_ik is not available")
        return False

    def wait(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        return future.done()

    def _joint_state(self, joint_values: list[float], is_diff: bool = True) -> RobotState:
        return RobotState(
            is_diff=is_diff,
            joint_state=JointState(name=list(self.joint_names), position=list(joint_values)),
        )

    def _joint_state_cb(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            self._latest_joint_positions[name] = float(position)

    def current_joint_values(
        self,
        fallback_values: list[float],
        fallback_name: str,
        log_current: bool = False,
    ) -> list[float]:
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not all(
            name in self._latest_joint_positions for name in self.joint_names
        ):
            if time.monotonic() >= deadline:
                self.node.get_logger().warn(
                    "Timed out waiting for /joint_states or /rebotarm/joint_states; "
                    f"using configured {fallback_name}"
                )
                return fallback_values
            rclpy.spin_once(self.node, timeout_sec=0.1)

        current = [self._latest_joint_positions[name] for name in self.joint_names]
        if log_current:
            self.node.get_logger().info(
                f"current joint state: {[round(value, 4) for value in current]}"
            )
        return current

    def joint_trajectory(
        self,
        start_values: list[float],
        goal_values: list[float],
        duration_sec: float,
    ) -> RobotTrajectory:
        return RobotTrajectory(
            joint_trajectory=JointTrajectory(
                joint_names=list(self.joint_names),
                points=[
                    JointTrajectoryPoint(
                        positions=list(start_values),
                        time_from_start=self.duration(0.0),
                    ),
                    JointTrajectoryPoint(
                        positions=list(goal_values),
                        time_from_start=self.duration(duration_sec),
                    ),
                ],
            )
        )

    def joint_trajectory_points(
        self,
        joint_values: list[list[float]],
        duration_sec: float,
    ) -> RobotTrajectory:
        step_duration = duration_sec / max(len(joint_values) - 1, 1)
        return RobotTrajectory(
            joint_trajectory=JointTrajectory(
                joint_names=list(self.joint_names),
                points=[
                    JointTrajectoryPoint(
                        positions=list(values),
                        time_from_start=self.duration(index * step_duration),
                    )
                    for index, values in enumerate(joint_values)
                ],
            )
        )

    def compute_ik_joint_target(
        self,
        pose_stamped: PoseStamped,
        seed_values: list[float],
        ik_link_name: str,
        timeout_sec: float,
        avoid_collisions: bool,
        label: str,
        warn_only: bool = False,
    ) -> list[float] | None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state = self._joint_state(seed_values, is_diff=False)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout = self.duration(timeout_sec)

        future = self._ik.call_async(request)
        log = self.node.get_logger().warn if warn_only else self.node.get_logger().error
        if not self.wait(future, timeout_sec):
            log(f"Timed out computing {label}")
            return None

        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            code = response.error_code.val if response is not None else "empty"
            message = response.error_code.message if response is not None else ""
            log(f"Failed to compute {label}: {code} {message}")
            return None

        joint_map = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        return [float(joint_map[name]) for name in self.joint_names]

    def execute_trajectory(self, trajectory: RobotTrajectory, timeout_sec: float) -> bool:
        send_future = self._execute.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)
        )
        if not self.wait(send_future, 5.0):
            self.node.get_logger().error("Timed out sending ExecuteTrajectory goal")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("ExecuteTrajectory rejected goal")
            return False

        result_future = goal_handle.get_result_async()
        if not self.wait(result_future, timeout_sec):
            self.node.get_logger().error(
                f"ExecuteTrajectory did not return within {timeout_sec:.1f}s"
            )
            return False

        action_result = result_future.result()
        if action_result is None:
            self.node.get_logger().error("ExecuteTrajectory returned an empty result")
            return False

        result = action_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f"ExecuteTrajectory failed with code {result.error_code.val}: "
                f"{result.error_code.message}"
            )
            return False
        return True
