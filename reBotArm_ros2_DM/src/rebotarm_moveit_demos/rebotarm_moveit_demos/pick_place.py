from __future__ import annotations

import sys
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene
import rclpy
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarm_moveit_demos.demo_common import MoveItDemoBase


class PickPlace(MoveItDemoBase):
    """Pick a scene object using MoveIt planning and the simulated gripper."""

    def __init__(self) -> None:
        super().__init__("pick_place")
        self.use_gripper = bool(self._param("use_gripper"))

        self._planner = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._planning_scene = self.node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self._get_planning_scene = self.node.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )
        self._gripper_trajectory = ActionClient(
            self.node,
            FollowJointTrajectory,
            str(self._param("gripper_action_name")),
        )
        self._gripper_command = ActionClient(
            self.node,
            GripperCommand,
            str(self._param("hardware_gripper_action_name")),
        )

    def run(self) -> bool:
        if not self._planner.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error(
                "MoveIt service /plan_kinematic_path is not available"
            )
            return False
        if not self._planning_scene.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error(
                "MoveIt service /apply_planning_scene is not available"
            )
            return False
        if not self._get_planning_scene.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error(
                "MoveIt service /get_planning_scene is not available"
            )
            return False
        if not self.wait_for_ik_service():
            return False
        if not self.wait_for_execute_server():
            return False
        if self.use_gripper and not self._wait_for_gripper_server():
            self.node.get_logger().error("Gripper action is not available")
            return False

        zero_point = [float(value) for value in self._param("zero_point")]
        ready_point = [float(value) for value in self._param("ready_point")]
        current = self.current_joint_values(
            zero_point,
            "zero_point",
            log_current=True,
        )

        if not self._command_gripper("close"):
            return False
        self._clear_demo_object()

        pick_center_position = self._object_center_position(self._param("pick_position"))
        place_center_position = self._place_center_position()
        pick_target = self.pick_place_joint_target(
            pick_center_position,
            ready_point,
            "pick",
        )
        if pick_target is None:
            return False
        place_target = self.pick_place_joint_target(
            place_center_position,
            ready_point,
            "place",
        )
        if place_target is None:
            return False

        object_pose = self._pose(pick_center_position)

        if not self._add_collision_object(object_pose):
            return False

        if not self._plan_to_joints("ready", current, ready_point):
            return False
        current = ready_point
        if not self._command_gripper("open"):
            return False
        if not self._allow_object_touch_collisions(True):
            return False
        if not self._plan_to_joints("pick", current, pick_target):
            return False
        current = pick_target
        if not self._command_gripper("grasp"):
            return False
        if not self._attach_object(object_pose):
            return False
        if not self._plan_to_joints("ready", current, ready_point):
            return False
        current = ready_point
        if not self._plan_to_joints("place", current, place_target):
            return False
        current = place_target
        if not self._detach_object_at_place():
            return False
        if not self._command_gripper("open"):
            return False
        if not self._allow_object_touch_collisions(False):
            return False
        if not self._plan_to_joints("ready", current, ready_point):
            return False

        self.node.get_logger().info("pick demo finished")
        return True

    def _plan_to_joints(
        self,
        label: str,
        start_values: list[float],
        goal_values: list[float],
    ) -> bool:
        self.node.get_logger().info(f"move to {label}")
        if self._joint_values_close(start_values, goal_values):
            self.node.get_logger().info(f"already at {label}, skip planning")
            return True
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
        timeout = float(self._param("result_timeout"))
        if not self.wait(future, timeout):
            self.node.get_logger().error(
                f"MoveIt planner did not return within {timeout:.1f}s"
            )
            return False

        response = future.result()
        if response is None:
            self.node.get_logger().error("MoveIt planner returned an empty response")
            return False

        plan_response = response.motion_plan_response
        if plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f"MoveIt planning failed with code {plan_response.error_code.val}: "
                f"{plan_response.error_code.message}"
            )
            return False

        self.node.get_logger().info("MoveIt planned joint target")

        return self.execute_trajectory(plan_response.trajectory, timeout)

    def _joint_values_close(
        self,
        first: list[float],
        second: list[float],
    ) -> bool:
        tolerance = float(self._param("joint_tolerance"))
        return all(abs(a - b) <= tolerance for a, b in zip(first, second))

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

    def _command_gripper(self, mode: str) -> bool:
        if not self.use_gripper:
            self.node.get_logger().info(f"gripper disabled, skip {mode}")
            return True

        if mode == "open":
            position = self._open_gripper_position()
        elif mode == "grasp":
            position = self._grasp_gripper_position()
        else:
            position = self._closed_gripper_position()

        if self._gripper_kind == "command":
            return self._command_gripper_action(mode, position)
        return self._command_gripper_trajectory(mode, position)

    def _command_gripper_trajectory(self, mode: str, position: float) -> bool:
        joint_names = [str(name) for name in self._param("gripper_joint_names")]
        duration = float(self._param("gripper_motion_duration"))
        goal = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(
                joint_names=joint_names,
                points=[
                    JointTrajectoryPoint(
                        positions=[position] * len(joint_names),
                        time_from_start=self.duration(duration),
                    )
                ],
            )
        )

        self.node.get_logger().info(
            f"{mode} gripper to {position:.4f} on {joint_names}"
        )
        return self._send_gripper_goal(goal)

    def _command_gripper_action(self, mode: str, position: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = self._hardware_gripper_position(position)
        goal.command.max_effort = float(self._param("gripper_max_effort"))

        self.node.get_logger().info(
            f"{mode} gripper to {goal.command.position:.4f} via "
            f"{self._param('hardware_gripper_action_name')}"
        )
        return self._send_gripper_goal(goal)

    def _send_gripper_goal(self, goal) -> bool:
        send_future = self._gripper.send_goal_async(goal)
        if not self.wait(send_future, 5.0):
            self.node.get_logger().error("Timed out sending gripper goal")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not self.wait(result_future, float(self._param("result_timeout"))):
            self.node.get_logger().error("gripper did not return a result in time")
            return False

        action_result = result_future.result()
        if action_result is None:
            self.node.get_logger().error("gripper returned an empty result")
            return False

        result = action_result.result
        if hasattr(result, "error_code") and result.error_code != 0:
            self.node.get_logger().error(
                f"gripper trajectory failed with code {result.error_code}: "
                f"{result.error_string}"
            )
            return False
        return True

    def _open_gripper_position(self) -> float:
        return self._clamp_gripper_width(float(self._param("open_gripper_position")))

    def _closed_gripper_position(self) -> float:
        return self._clamp_gripper_width(float(self._param("closed_gripper_position")))

    def _grasp_gripper_position(self) -> float:
        return self._clamp_gripper_width(float(self._param("grasp_gripper_position")))

    def _clamp_gripper_width(self, position: float) -> float:
        low = float(self._param("closed_gripper_position"))
        high = float(self._param("open_gripper_position"))
        return max(low, min(high, position))

    def _hardware_gripper_position(self, sim_position: float) -> float:
        max_width = float(self._param("max_gripper_width"))
        ratio = 0.0 if max_width <= 0.0 else (2.0 * sim_position / max_width)
        open_position = float(self._param("hardware_open_gripper_position"))
        closed_position = float(self._param("hardware_closed_gripper_position"))
        ratio = max(0.0, min(1.0, ratio))
        return closed_position + (open_position - closed_position) * ratio

    def _add_collision_object(self, object_pose: Pose) -> bool:
        object_id = str(self._param("object_id"))
        frame_id = str(self._param("object_frame_id"))

        self.node.get_logger().info(
            f"add object '{object_id}' at "
            f"[{object_pose.position.x:.3f}, {object_pose.position.y:.3f}, "
            f"{object_pose.position.z:.3f}] in {frame_id}"
        )
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(
            CollisionObject(
                id=object_id,
                header=Header(frame_id=frame_id),
                primitives=[self._object_primitive()],
                primitive_poses=[object_pose],
                operation=CollisionObject.ADD,
            )
        )
        return self._apply_planning_scene(scene)

    def _clear_demo_object(self) -> bool:
        object_id = str(self._param("object_id"))
        attached_link_name = str(self._param("attached_link_name"))

        scene = PlanningScene(is_diff=True, robot_state=RobotState(is_diff=True))
        scene.robot_state.attached_collision_objects.append(
            AttachedCollisionObject(
                link_name=attached_link_name,
                object=CollisionObject(id=object_id, operation=CollisionObject.REMOVE),
            )
        )
        scene.world.collision_objects.append(
            CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
        )

        self.node.get_logger().info(f"clear stale object '{object_id}'")
        return self._apply_planning_scene(scene)

    def pick_place_joint_target(
        self,
        tcp_position: list[float],
        seed_values: list[float],
        label: str,
    ) -> list[float] | None:
        parameter_name = "place_tcp_rpy" if label == "place" else "pick_tcp_rpy"
        tcp_rpy = [float(value) for value in self._param(parameter_name)]
        qx, qy, qz, qw = quaternion_from_euler(*tcp_rpy)
        target_pose = Pose(
            position=Point(x=tcp_position[0], y=tcp_position[1], z=tcp_position[2]),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
        timeout_sec = float(self._param("ik_timeout"))
        attached_link = str(self._param("attached_link_name"))

        self.node.get_logger().info(
            f"compute {label} IK for absolute {attached_link} pose "
            f"[{target_pose.position.x:.3f}, {target_pose.position.y:.3f}, "
            f"{target_pose.position.z:.3f}] rpy "
            f"[{tcp_rpy[0]:.4f}, {tcp_rpy[1]:.4f}, {tcp_rpy[2]:.4f}]"
        )

        target = self.compute_ik_joint_target(
            PoseStamped(
                header=Header(frame_id=str(self._param("object_frame_id"))),
                pose=target_pose,
            ),
            seed_values,
            attached_link,
            timeout_sec,
            True,
            f"{label} IK for {attached_link} pose "
            f"[{target_pose.position.x:.3f}, {target_pose.position.y:.3f}, "
            f"{target_pose.position.z:.3f}]",
        )
        if target is None:
            return None

        self.node.get_logger().info(
            f"{label} IK target: {[round(value, 4) for value in target]}"
        )
        return target

    def _attach_object(self, object_pose: Pose) -> bool:
        object_id = str(self._param("object_id"))
        attached_link_name = str(self._param("attached_link_name"))

        self.node.get_logger().info(
            f"attach object '{object_id}' to {attached_link_name} without changing pose"
        )
        attached_object = AttachedCollisionObject(
            link_name=attached_link_name,
            touch_links=[str(link) for link in self._param("touch_links")],
            object=CollisionObject(
                id=object_id,
                header=Header(frame_id=str(self._param("object_frame_id"))),
                primitives=[self._object_primitive()],
                primitive_poses=[object_pose],
                operation=CollisionObject.ADD,
            ),
        )
        scene = PlanningScene(is_diff=True, robot_state=RobotState(is_diff=True))
        scene.robot_state.attached_collision_objects.append(attached_object)
        return self._apply_planning_scene(scene)

    def _detach_object_at_place(self) -> bool:
        object_id = str(self._param("object_id"))
        attached_link_name = str(self._param("attached_link_name"))

        self.node.get_logger().info(
            f"detach object '{object_id}' at place position with initial orientation"
        )
        scene = PlanningScene(is_diff=True, robot_state=RobotState(is_diff=True))
        scene.robot_state.attached_collision_objects.append(
            AttachedCollisionObject(
                link_name=attached_link_name,
                object=CollisionObject(id=object_id, operation=CollisionObject.REMOVE),
            )
        )
        scene.world.collision_objects.append(
            CollisionObject(
                id=object_id,
                header=Header(frame_id=str(self._param("object_frame_id"))),
                primitives=[self._object_primitive()],
                primitive_poses=[self._pose(self._place_center_position())],
                operation=CollisionObject.ADD,
            )
        )
        return self._apply_planning_scene(scene)

    def _allow_object_touch_collisions(self, allowed: bool) -> bool:
        object_id = str(self._param("object_id"))
        touch_links = [str(link) for link in self._param("touch_links")]

        state = "allow" if allowed else "forbid"
        self.node.get_logger().info(
            f"{state} collisions between '{object_id}' and {touch_links}"
        )
        acm = self._current_allowed_collision_matrix()
        if acm is None:
            return False
        for link in touch_links:
            self._set_allowed_collision(acm, object_id, link, allowed)

        scene = PlanningScene(is_diff=True)
        scene.allowed_collision_matrix = acm
        return self._apply_planning_scene(scene)

    def _current_allowed_collision_matrix(self) -> AllowedCollisionMatrix | None:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self._get_planning_scene.call_async(request)
        if not self.wait(future, float(self._param("result_timeout"))):
            self.node.get_logger().error("Timed out reading planning scene ACM")
            return None
        response = future.result()
        if response is None:
            self.node.get_logger().error("MoveIt returned an empty planning scene")
            return None
        return response.scene.allowed_collision_matrix

    @staticmethod
    def _set_allowed_collision(
        acm: AllowedCollisionMatrix,
        first: str,
        second: str,
        allowed: bool,
    ) -> None:
        for name in (first, second):
            if name in acm.entry_names:
                continue
            acm.entry_names.append(name)
            for entry in acm.entry_values:
                entry.enabled.append(False)
            acm.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(acm.entry_names))
            )

        first_index = acm.entry_names.index(first)
        second_index = acm.entry_names.index(second)
        acm.entry_values[first_index].enabled[second_index] = allowed
        acm.entry_values[second_index].enabled[first_index] = allowed

    def _object_primitive(self) -> SolidPrimitive:
        return SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[float(value) for value in self._param("object_dimensions")],
        )

    def _object_center_position(self, surface_position) -> list[float]:
        position = [float(value) for value in surface_position]
        dimensions = [float(value) for value in self._param("object_dimensions")]
        position[2] = position[2] + dimensions[2] * 0.5
        return position

    def _place_center_position(self) -> list[float]:
        pick_position = [float(value) for value in self._param("pick_position")]
        return self._object_center_position(
            [pick_position[0], -pick_position[1], pick_position[2]]
        )

    def _pose(self, position: list[float]) -> Pose:
        return Pose(
            position=Point(x=position[0], y=position[1], z=position[2]),
            orientation=Quaternion(w=1.0),
        )

    def _apply_planning_scene(self, scene: PlanningScene) -> bool:
        future = self._planning_scene.call_async(
            ApplyPlanningScene.Request(scene=scene)
        )
        if not self.wait(future, float(self._param("result_timeout"))):
            self.node.get_logger().error("Timed out applying planning scene")
            return False

        response = future.result()
        if response is None or not response.success:
            self.node.get_logger().error("Failed to apply planning scene")
            return False
        return True

    def _wait_for_gripper_server(self) -> bool:
        if self._gripper_trajectory.wait_for_server(timeout_sec=1.0):
            self._gripper = self._gripper_trajectory
            self._gripper_kind = "trajectory"
            return True
        if self._gripper_command.wait_for_server(timeout_sec=10.0):
            self._gripper = self._gripper_command
            self._gripper_kind = "command"
            return True
        return False


def main() -> None:
    rclpy.init()
    demo = PickPlace()
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
