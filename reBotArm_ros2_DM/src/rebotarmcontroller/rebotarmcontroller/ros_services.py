from __future__ import annotations

import math

from rebotarm_msgs.srv import MoveToPoseIK, SetGripper, SetMode, SetZero
from std_srvs.srv import Trigger

from .conversions import pose_to_xyz_rpy


class ArmServices:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace

        node.create_service(
            Trigger,
            self._service("enable"),
            self.enable,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("disable"),
            self.disable,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("safe_home"),
            self.safe_home,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("gravity_compensation/start"),
            self.start_gravity_compensation,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("gravity_compensation/stop"),
            self.stop_gravity_compensation,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("gravity_compensation/status"),
            self.gravity_compensation_status,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            SetZero,
            self._service("set_zero"),
            self.set_zero,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetMode,
            self._service("set_mode"),
            self.set_mode,
            callback_group=node.slow_group,
        )
        node.create_service(
            MoveToPoseIK,
            self._service("move_to_pose_ik"),
            self.move_to_pose_ik,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            SetGripper,
            self._service("gripper/set"),
            self.set_gripper,
            callback_group=node.reentrant_group,
        )

    def _service(self, name: str) -> str:
        return f"/{self._namespace}/{name}"

    def enable(self, _request, response):
        try:
            self._hardware.enable()
            response.success = True
            response.message = "enabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def disable(self, _request, response):
        try:
            self._hardware.disable()
            response.success = True
            response.message = "disabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def safe_home(self, _request, response):
        try:
            self._hardware.stop_gravity_compensation()
            self._hardware.ensure_pos_vel_control()
            self._hardware.safe_home()
            response.success = True
            response.message = "safe_home complete"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def start_gravity_compensation(self, _request, response):
        try:
            already_active = self._hardware.gravity_compensation_active()
            self._hardware.start_gravity_compensation()
            target = self._hardware.gravity_compensation_target()
            if already_active:
                self._node.get_logger().info(
                    "gravity compensation already active; keeping current control loop"
                )
                response.success = True
                response.message = "gravity compensation already active"
                self._node.publish_arm_status()
                return response
            if target is not None:
                deg = ", ".join(f"{math.degrees(float(v)):+.1f}" for v in target)
                self._node.get_logger().info(
                    f"gravity compensation started, lock target deg=[{deg}]"
                )
            else:
                self._node.get_logger().info("gravity compensation started")
            response.success = True
            response.message = "gravity compensation started"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def stop_gravity_compensation(self, _request, response):
        try:
            active = self._hardware.gravity_compensation_active()
            self._hardware.stop_gravity_compensation()
            if active:
                self._node.get_logger().info(
                    "gravity compensation stopped, returned to pos_vel hold"
                )
            else:
                self._node.get_logger().info("gravity compensation was not active")
            response.success = True
            response.message = "gravity compensation stopped"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def gravity_compensation_status(self, _request, response):
        try:
            active = self._hardware.gravity_compensation_active()
            target = self._hardware.gravity_compensation_target()
            response.success = bool(active)
            if target is not None:
                deg = ", ".join(f"{math.degrees(float(v)):+.1f}" for v in target)
                response.message = f"gravity compensation active; lock target deg=[{deg}]"
            else:
                response.message = "gravity compensation inactive"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_zero(self, request, response):
        try:
            self._hardware.stop_gravity_compensation()
            ok = self._hardware.set_zero(request.joint_name)
            response.success = bool(ok)
            response.message = "set_zero complete" if ok else "set_zero failed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_mode(self, request, response):
        try:
            self._hardware.stop_gravity_compensation()
            ok = self._hardware.set_mode(request.mode)
            response.success = bool(ok)
            response.message = f"mode set to {request.mode}" if ok else "mode switch incomplete"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def move_to_pose_ik(self, request, response):
        try:
            self._hardware.stop_gravity_compensation()
            self._hardware.ensure_pos_vel_control()
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(request.target_pose)
            ok = self._hardware.endpos_ctrl.move_to_ik(x, y, z, roll, pitch, yaw)
            response.success = bool(ok)
            response.message = "IK target accepted" if ok else "IK failed"
            response.q_solution = [
                float(v) for v in self._hardware.endpos_ctrl._q_target.copy()
            ]
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.q_solution = []
        self._node.publish_arm_status()
        return response

    def set_gripper(self, request, response):
        try:
            reached, reached_position = self._hardware.set_gripper_position(
                request.position,
                request.max_effort,
            )
            response.success = bool(reached)
            response.reached_position = float(reached_position)
            self._node.get_logger().info(
                "gripper set "
                f"target={float(request.position):.3f}m "
                f"reached={response.reached_position:.3f}m "
                f"success={response.success}"
            )
        except Exception as exc:
            response.success = False
            response.reached_position = 0.0
            self._node.get_logger().error(f"gripper set failed: {exc}")
        self._node.publish_arm_status()
        return response
