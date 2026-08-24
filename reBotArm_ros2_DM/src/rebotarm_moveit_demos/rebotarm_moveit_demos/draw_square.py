from __future__ import annotations

import sys
from math import pi

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import rclpy
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler

from rebotarm_moveit_demos.demo_common import MoveItDemoBase


class DrawSquare(MoveItDemoBase):
    """Move the TCP through four coplanar rectangle corners."""

    def __init__(self) -> None:
        super().__init__("draw_square")
        self.wrap_joint_names = {str(name) for name in self._param("wrap_joint_names")}
        self.max_wrap_joint_delta = float(self._param("max_wrap_joint_delta"))
        self.frame_id = str(self._param("frame_id"))
        self.tcp_link_name = str(self._param("tcp_link_name"))
        self.start_point = self._wrap_joints(
            [float(value) for value in self._param("start_point")]
        )
        self.rectangle_center = [float(value) for value in self._param("rectangle_center")]
        self.rectangle_width = float(self._param("rectangle_width"))
        self.rectangle_height = float(self._param("rectangle_height"))
        self.tcp_rpy = [float(value) for value in self._param("tcp_rpy")]
        self.tcp_yaw_offsets = [float(value) for value in self._param("tcp_yaw_offsets")]
        self._planner = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.ik_timeout = float(self._param("ik_timeout"))
        self.result_timeout = float(self._param("result_timeout"))
        self.avoid_collisions = bool(self._param("avoid_collisions"))

    def run(self) -> bool:
        if not self._planner.wait_for_service(timeout_sec=30.0):
            self.node.get_logger().error(
                "MoveIt service /plan_kinematic_path is not available"
            )
            return False
        if not self.wait_for_ik_service():
            return False
        if not self.wait_for_execute_server():
            return False

        current_joints = self._wrap_joints(self._current_joint_values())
        if not self._plan_to_joints("reset", current_joints, self.start_point):
            return False

        points = self._rectangle_points()
        first_corner = self.corner_joint_target(points[0], self.start_point, "corner 1")
        if first_corner is None or not self._plan_to_joints(
            "corner 1",
            self.start_point,
            first_corner,
        ):
            return False

        current_joints = first_corner
        for edge_index, end in enumerate(points[1:] + [points[0]], start=1):
            target = self.corner_joint_target(end, current_joints, f"corner {edge_index + 1}")
            if target is None:
                return False
            if not self._plan_to_joints(f"edge {edge_index}", current_joints, target):
                return False
            current_joints = target

        self.node.get_logger().info("rectangle draw demo finished")
        return True

    def _rectangle_points(self) -> list[list[float]]:
        center = self.rectangle_center
        half_width = self.rectangle_width * 0.5
        half_height = self.rectangle_height * 0.5
        return [
            [center[0] - half_width, center[1] - half_height, center[2]],
            [center[0] + half_width, center[1] - half_height, center[2]],
            [center[0] + half_width, center[1] + half_height, center[2]],
            [center[0] - half_width, center[1] + half_height, center[2]],
        ]

    def _waypoint(self, tcp_position: list[float], yaw_offset: float = 0.0) -> Pose:
        roll, pitch, yaw = self.tcp_rpy
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw + yaw_offset)
        return Pose(
            position=Point(x=tcp_position[0], y=tcp_position[1], z=tcp_position[2]),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )

    def corner_joint_target(
        self,
        tcp_position: list[float],
        seed_values: list[float],
        label: str,
    ) -> list[float] | None:
        seed_values = self._wrap_joints(seed_values)
        self.node.get_logger().info(
            f"compute IK for {label}: "
            f"[{tcp_position[0]:.3f}, {tcp_position[1]:.3f}, {tcp_position[2]:.3f}]"
        )

        best = None
        best_yaw_offset = 0.0
        best_cost = float("inf")
        for yaw_offset in self.tcp_yaw_offsets:
            target = self._corner_joint_target(tcp_position, seed_values, label, yaw_offset)
            if target is None:
                continue
            if any(
                name in self.wrap_joint_names
                and abs(goal - start) > self.max_wrap_joint_delta
                for name, start, goal in zip(self.joint_names, seed_values, target)
            ):
                continue
            cost = sum(abs(goal - start) for start, goal in zip(seed_values, target))
            if cost < best_cost:
                best = target
                best_yaw_offset = yaw_offset
                best_cost = cost

        if best is None:
            self.node.get_logger().error(
                f"Failed to compute IK for {label} without wrapped-joint flip"
            )
            return None

        self.node.get_logger().info(
            f"{label} target yaw_offset={best_yaw_offset:.4f}: "
            f"{[round(value, 4) for value in best]}"
        )
        return best

    def _corner_joint_target(
        self,
        tcp_position: list[float],
        seed_values: list[float],
        label: str,
        yaw_offset: float,
    ) -> list[float] | None:
        target = self.compute_ik_joint_target(
            PoseStamped(
                header=Header(frame_id=self.frame_id),
                pose=self._waypoint(tcp_position, yaw_offset),
            ),
            seed_values,
            self.tcp_link_name,
            self.ik_timeout,
            self.avoid_collisions,
            f"IK for {label} yaw_offset={yaw_offset:.4f}",
            warn_only=True,
        )
        return None if target is None else self._wrap_joints(target, seed_values)

    def _wrap_joints(
        self,
        values: list[float],
        reference: list[float] | None = None,
    ) -> list[float]:
        result = []
        references = reference if reference is not None else [0.0] * len(values)
        for name, value, ref in zip(self.joint_names, values, references):
            if name not in self.wrap_joint_names:
                result.append(value)
                continue
            wrapped = ref + (value - ref + pi) % (2.0 * pi) - pi
            if wrapped < -pi or wrapped > pi:
                wrapped = (value + pi) % (2.0 * pi) - pi
            result.append(pi if wrapped == -pi and value > 0.0 else wrapped)
        return result

    def _plan_to_joints(
        self,
        label: str,
        start_values: list[float],
        goal_values: list[float],
    ) -> bool:
        start_values = self._wrap_joints(start_values)
        goal_values = self._wrap_joints(goal_values)
        self.node.get_logger().info(f"move to {label}")
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.group_name
        request.motion_plan_request.pipeline_id = str(self._param("pipeline_id"))
        request.motion_plan_request.planner_id = str(self._param("planner_id"))
        request.motion_plan_request.allowed_planning_time = float(
            self._param("planning_time")
        )
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.max_velocity_scaling_factor = float(
            self._param("velocity_scaling")
        )
        request.motion_plan_request.max_acceleration_scaling_factor = float(
            self._param("acceleration_scaling")
        )
        request.motion_plan_request.start_state = self._joint_state(start_values)
        request.motion_plan_request.goal_constraints = [
            self._joint_constraints(goal_values)
        ]

        future = self._planner.call_async(request)
        if not self.wait(future, self.result_timeout):
            self.node.get_logger().error(
                f"MoveIt planner did not return within {self.result_timeout:.1f}s"
            )
            return False

        response = future.result()
        plan_response = response.motion_plan_response if response is not None else None
        if (
            plan_response is None
            or plan_response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            code = plan_response.error_code.val if plan_response is not None else "empty"
            message = (
                plan_response.error_code.message if plan_response is not None else ""
            )
            self.node.get_logger().error(
                f"MoveIt planning failed with code {code}: {message}"
            )
            return False

        self.node.get_logger().info(f"MoveIt planned {label}")
        return self.execute_trajectory(plan_response.trajectory, self.result_timeout)

    def _joint_constraints(self, joint_values: list[float]) -> Constraints:
        tolerance = float(self._param("joint_tolerance"))
        return Constraints(
            joint_constraints=[
                JointConstraint(
                    joint_name=name,
                    position=value,
                    tolerance_above=tolerance,
                    tolerance_below=tolerance,
                    weight=1.0,
                )
                for name, value in zip(self.joint_names, joint_values)
            ]
        )

    def _current_joint_values(self) -> list[float]:
        return self.current_joint_values(list(self.start_point), "start_point")


def main() -> None:
    rclpy.init()
    demo = DrawSquare()
    try:
        ok = demo.run()
    except Exception as exc:
        demo.node.get_logger().error(str(exc))
        ok = False
    finally:
        demo.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
