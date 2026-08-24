from __future__ import annotations

import json
import math
import threading
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from .real2sim_sync import (
    JointMapping,
    MujocoReal2Sim,
    ResolvedJointTarget,
    VirtualGraspObject,
    _DEFAULT_GRASP_OBJECTS,
    _GRIPPER_BASE_GAP_FALLBACK_M,
    _GRIPPER_GRASP_CLEARANCE_M,
    _GRIPPER_GRASP_RELEASE_MARGIN_M,
    _GRIPPER_GRASP_TOLERANCE_M,
    _GRIPPER_QPOS_MAX_FALLBACK_M,
)


_TARGET_MATERIAL_RGBA = np.array([1.0, 0.55, 0.12, 0.72], dtype=np.float32)
_TARGET_SITE_RGBA = np.array([1.0, 0.55, 0.12, 0.35], dtype=np.float32)
_TARGET_HIDDEN_POS = np.array([0.0, 0.0, -10.0], dtype=np.float64)
_TARGET_HIDDEN_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


class MujocoSimRgbCamera(Node):
    """Publish a fixed MuJoCo RGB camera as ROS Image and CameraInfo topics."""

    def __init__(self) -> None:
        super().__init__("mujoco_sim_rgb_camera")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_map_file", "")
        self.declare_parameter("joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("object_states_topic", "")
        self.declare_parameter("target_pose_topic", "/rebotarm/mujoco/target_pose")
        self.declare_parameter("target_body_name", "ik_target")
        self.declare_parameter("target_visible_timeout", 0.7)
        self.declare_parameter("render_target_marker", True)
        self.declare_parameter("camera_name", "overhead_rgb")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("frame_id", "overhead_rgb_frame")
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("publish_hz", 8.0)
        self.declare_parameter("virtual_grasp_enabled", True)
        self.declare_parameter("virtual_grasp_objects", _DEFAULT_GRASP_OBJECTS)
        self.declare_parameter("virtual_grasp_close_threshold", 0.010)
        self.declare_parameter("virtual_grasp_release_threshold", 0.020)
        self.declare_parameter("virtual_grasp_radius", 0.130)
        self.declare_parameter("virtual_grasp_target_timeout", 6.0)
        self.declare_parameter("virtual_grasp_drop_gravity", 9.81)

        namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.object_states_topic = str(self.get_parameter("object_states_topic").value or "").strip()
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_body_name = str(self.get_parameter("target_body_name").value or "ik_target")
        self.target_visible_timeout = max(
            float(self.get_parameter("target_visible_timeout").value),
            0.05,
        )
        self.render_target_marker = MujocoReal2Sim._as_bool(
            self.get_parameter("render_target_marker").value
        )
        self.camera_name = str(self.get_parameter("camera_name").value or "overhead_rgb")
        self.frame_id = str(self.get_parameter("frame_id").value or f"{self.camera_name}_frame")
        self.width = max(int(self.get_parameter("width").value), 16)
        self.height = max(int(self.get_parameter("height").value), 16)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 0.2)
        self.virtual_grasp_enabled = MujocoReal2Sim._as_bool(
            self.get_parameter("virtual_grasp_enabled").value
        )
        self.virtual_grasp_close_threshold = max(
            float(self.get_parameter("virtual_grasp_close_threshold").value),
            0.0,
        )
        self.virtual_grasp_release_threshold = max(
            float(self.get_parameter("virtual_grasp_release_threshold").value),
            self.virtual_grasp_close_threshold,
        )
        self.virtual_grasp_radius = max(
            float(self.get_parameter("virtual_grasp_radius").value),
            0.01,
        )
        self.virtual_grasp_target_timeout = max(
            float(self.get_parameter("virtual_grasp_target_timeout").value),
            0.0,
        )
        self.virtual_grasp_drop_gravity = max(
            float(self.get_parameter("virtual_grasp_drop_gravity").value),
            0.0,
        )

        image_topic = str(self.get_parameter("image_topic").value or "").strip()
        camera_info_topic = str(self.get_parameter("camera_info_topic").value or "").strip()
        self.image_topic = image_topic or f"/{namespace}/mujoco/{self.camera_name}/image_raw"
        self.camera_info_topic = (
            camera_info_topic or f"/{namespace}/mujoco/{self.camera_name}/camera_info"
        )

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
        self.data = mujoco.MjData(self.model)
        self.joint_map = MujocoReal2Sim._load_joint_map(joint_map_path)
        self.qpos_targets = self._build_qpos_targets()
        self.camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            self.camera_name,
        )
        if self.camera_id < 0:
            raise RuntimeError(f"camera {self.camera_name!r} was not found in {self.model_path}")
        self.target_mocap_id = self._find_mocap_id(self.target_body_name)
        self.target_mat_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_MATERIAL,
            "target_mat",
        )
        self.target_geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "ik_target_sphere",
        )
        self.target_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "ik_target_site",
        )
        self._target_geom_size = (
            self.model.geom_size[self.target_geom_id].copy()
            if self.target_geom_id >= 0
            else None
        )
        self._target_site_size = (
            self.model.site_size[self.target_site_id].copy()
            if self.target_site_id >= 0
            else None
        )
        self.tcp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.finger_left_qpos_addr = self._joint_qpos_addr("finger_left")
        self.finger_right_qpos_addr = self._joint_qpos_addr("finger_right")
        self.virtual_grasp_objects = self._build_virtual_grasp_objects(
            str(self.get_parameter("virtual_grasp_objects").value or _DEFAULT_GRASP_OBJECTS)
        )
        self.object_qpos_by_name = {
            obj.name: obj.qpos_addr for obj in self.virtual_grasp_objects
        }

        self._lock = threading.Lock()
        self._target_qpos: dict[int, float] = {}
        self._target_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._target_pose_monotonic: float | None = None
        self._target_visible = True
        self._warned_missing: set[str] = set()
        self._renderer: mujoco.Renderer | None = None
        self._last_renderer_warning = 0.0
        self._attached_object: VirtualGraspObject | None = None
        self._attached_offset = np.zeros(3, dtype=np.float64)
        self._falling_objects: dict[str, float] = {}
        self._last_grasp_update_time: float | None = None
        self._set_target_visible(False)

        self.image_pub = self.create_publisher(Image, self.image_topic, qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        if self.object_states_topic:
            self.create_subscription(
                String,
                self.object_states_topic,
                self._object_states_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.publish_hz, self._render_timer)

        self.get_logger().info(
            "MuJoCo RGB camera ready: "
            f"model={self.model_path}, camera={self.camera_name}, "
            f"image={self.image_topic}, camera_info={self.camera_info_topic}, "
            f"size={self.width}x{self.height}@{self.publish_hz:g}Hz, "
            f"object_states={self.object_states_topic or 'disabled'}, "
            f"render_target_marker={self.render_target_marker}"
        )

    def destroy_node(self) -> bool:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        return super().destroy_node()

    def _build_qpos_targets(self) -> dict[str, list[ResolvedJointTarget]]:
        qpos_targets: dict[str, list[ResolvedJointTarget]] = {}
        for ros_name, mappings in self.joint_map.items():
            resolved_targets: list[ResolvedJointTarget] = []
            for mapping in mappings:
                resolved_targets.append(self._resolve_joint_target(ros_name, mapping))
            qpos_targets[ros_name] = resolved_targets
        return qpos_targets

    def _resolve_joint_target(
        self,
        ros_name: str,
        mapping: JointMapping,
    ) -> ResolvedJointTarget:
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
            raise RuntimeError(f"MuJoCo joint {mapping.mujoco!r} must be hinge or slide")
        return ResolvedJointTarget(
            mujoco=mapping.mujoco,
            qpos_addr=int(self.model.jnt_qposadr[joint_id]),
            scale=mapping.scale,
            offset=mapping.offset,
            lower=float(self.model.jnt_range[joint_id][0])
            if self.model.jnt_limited[joint_id]
            else None,
            upper=float(self.model.jnt_range[joint_id][1])
            if self.model.jnt_limited[joint_id]
            else None,
        )

    def _joint_qpos_addr(self, joint_name: str) -> int | None:
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            return None
        return int(self.model.jnt_qposadr[joint_id])

    def _joint_qpos_upper(self, joint_name: str, fallback: float) -> float:
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0 or not self.model.jnt_limited[joint_id]:
            return fallback
        return max(float(self.model.jnt_range[joint_id][1]), fallback)

    def _estimate_gripper_base_gap(self) -> float:
        left_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "finger_left_collision",
        )
        right_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "finger_right_collision",
        )
        if left_id >= 0 and right_id >= 0:
            left_inner = float(self.model.geom_pos[left_id][1] - self.model.geom_size[left_id][1])
            right_inner = float(
                self.model.geom_pos[right_id][1] + self.model.geom_size[right_id][1]
            )
            gap = left_inner - right_inner
            if gap > 0.0:
                return gap
        return _GRIPPER_BASE_GAP_FALLBACK_M

    def _body_footprint_size(self, body_id: int) -> tuple[float, float]:
        geom_addr = int(self.model.body_geomadr[body_id])
        geom_count = int(self.model.body_geomnum[body_id])
        longest = 0.0
        shortest = float("inf")
        for geom_id in range(geom_addr, geom_addr + geom_count):
            geom_type = int(self.model.geom_type[geom_id])
            geom_size = self.model.geom_size[geom_id]
            if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
                width, depth = float(2.0 * geom_size[0]), float(2.0 * geom_size[1])
            elif geom_type in (
                int(mujoco.mjtGeom.mjGEOM_CYLINDER),
                int(mujoco.mjtGeom.mjGEOM_SPHERE),
            ):
                width = depth = float(2.0 * geom_size[0])
            else:
                continue
            longest = max(longest, width, depth)
            shortest = min(shortest, width, depth)

        if longest <= 0.0 or not np.isfinite(shortest):
            return 0.05, 0.05
        return longest, shortest

    def _object_grasp_width(self, body_id: int) -> float:
        longest, shortest = self._body_footprint_size(body_id)
        finger_upper = self._joint_qpos_upper("finger_left", _GRIPPER_QPOS_MAX_FALLBACK_M)
        max_gap = self._estimate_gripper_base_gap() + 2.0 * finger_upper
        preferred = shortest if shortest + _GRIPPER_GRASP_CLEARANCE_M <= max_gap else longest
        return float(np.clip(preferred, 0.0, max_gap - _GRIPPER_GRASP_CLEARANCE_M))

    def _grasp_width_to_qpos(self, grasp_width: float) -> float:
        finger_upper = self._joint_qpos_upper("finger_left", _GRIPPER_QPOS_MAX_FALLBACK_M)
        base_gap = self._estimate_gripper_base_gap()
        qpos = (grasp_width + _GRIPPER_GRASP_CLEARANCE_M - base_gap) * 0.5
        return float(np.clip(qpos, self.virtual_grasp_close_threshold, finger_upper))

    def _build_virtual_grasp_objects(self, names: str) -> list[VirtualGraspObject]:
        objects: list[VirtualGraspObject] = []
        for raw_name in names.split(","):
            name = raw_name.strip()
            if not name:
                continue
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id < 0:
                self.get_logger().warn(f"virtual grasp object body {name!r} was not found")
                continue
            joint_addr = int(self.model.body_jntadr[body_id])
            joint_count = int(self.model.body_jntnum[body_id])
            free_joint_id = -1
            for joint_id in range(joint_addr, joint_addr + joint_count):
                if int(self.model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
                    free_joint_id = joint_id
                    break
            if free_joint_id < 0:
                self.get_logger().warn(f"virtual grasp object {name!r} has no freejoint")
                continue
            grasp_width = self._object_grasp_width(body_id)
            grasp_qpos = self._grasp_width_to_qpos(grasp_width)
            release_qpos = max(
                self.virtual_grasp_release_threshold,
                grasp_qpos + _GRIPPER_GRASP_RELEASE_MARGIN_M,
            )
            objects.append(
                VirtualGraspObject(
                    name=name,
                    body_id=body_id,
                    qpos_addr=int(self.model.jnt_qposadr[free_joint_id]),
                    grasp_width=grasp_width,
                    grasp_qpos=grasp_qpos,
                    release_qpos=release_qpos,
                    rest_z=float(self.model.qpos0[int(self.model.jnt_qposadr[free_joint_id]) + 2]),
                )
            )
        return objects

    def _find_mocap_id(self, body_name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            self.get_logger().warn(
                f"target body {body_name!r} not found; target pose is not rendered"
            )
            return -1
        mocap_id = int(self.model.body_mocapid[body_id])
        if mocap_id < 0:
            self.get_logger().warn(
                f"target body {body_name!r} is not a mocap body; target pose is not rendered"
            )
        return mocap_id

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        updates: dict[int, float] = {}
        for name, position in zip(msg.name, msg.position):
            if name not in self.qpos_targets:
                if name not in self._warned_missing:
                    self._warned_missing.add(name)
                    self.get_logger().warn(
                        f"ignoring unmapped ROS joint {name!r} from {self.joint_state_topic}"
                    )
                continue
            for target in self.qpos_targets[name]:
                updates[target.qpos_addr] = target.clamp(
                    float(position) * target.scale + target.offset
                )

        if updates:
            with self._lock:
                self._target_qpos.update(updates)

    def _object_states_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
        except Exception:
            return

        objects = payload.get("objects")
        if not isinstance(objects, list):
            return

        with self._lock:
            for item in objects:
                if not isinstance(item, dict):
                    continue
                name = str(item.get("name", ""))
                qpos_addr = self.object_qpos_by_name.get(name)
                if qpos_addr is None:
                    continue
                position = item.get("position")
                quat = item.get("quat_wxyz")
                if (
                    not isinstance(position, list)
                    or len(position) < 3
                    or not isinstance(quat, list)
                    or len(quat) < 4
                ):
                    continue
                values = [*position[:3], *quat[:4]]
                try:
                    self.data.qpos[qpos_addr : qpos_addr + 7] = np.asarray(
                        [float(value) for value in values],
                        dtype=np.float64,
                    )
                except Exception:
                    continue

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        pos = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=np.float64,
        )
        quat = np.array(
            [
                float(msg.pose.orientation.w),
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quat))
        if norm < 1e-9:
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            quat /= norm

        with self._lock:
            self._target_pose = (pos, quat)
            self._target_pose_monotonic = self.get_clock().now().nanoseconds * 1e-9

    def _render_timer(self) -> None:
        renderer = self._ensure_renderer()
        if renderer is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            for qpos_addr, value in self._target_qpos.items():
                self.data.qpos[qpos_addr] = value
            target_visible = (
                self.render_target_marker
                and
                self._target_pose is not None
                and self._target_pose_monotonic is not None
                and now - self._target_pose_monotonic <= self.target_visible_timeout
            )
            self._set_target_visible(target_visible)
            if target_visible and self.target_mocap_id >= 0:
                pos, quat = self._target_pose
                self.data.mocap_pos[self.target_mocap_id] = pos
                self.data.mocap_quat[self.target_mocap_id] = quat
            elif self.target_mocap_id >= 0:
                self.data.mocap_pos[self.target_mocap_id] = _TARGET_HIDDEN_POS
                self.data.mocap_quat[self.target_mocap_id] = _TARGET_HIDDEN_QUAT

        mujoco.mj_forward(self.model, self.data)
        if self._update_virtual_grasp():
            mujoco.mj_forward(self.model, self.data)
        renderer.update_scene(self.data, camera=self.camera_name)
        rgb = np.ascontiguousarray(renderer.render())
        stamp = self.get_clock().now().to_msg()

        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        image_msg.height = self.height
        image_msg.width = self.width
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = 0
        image_msg.step = self.width * 3
        image_msg.data = rgb.tobytes()

        info_msg = self._camera_info(stamp)

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(info_msg)

    def _set_target_visible(self, visible: bool) -> None:
        if self._target_visible == visible:
            return
        self._target_visible = visible
        alpha = 1.0 if visible else 0.0
        if self.target_mat_id >= 0:
            rgba = _TARGET_MATERIAL_RGBA.copy()
            rgba[3] *= alpha
            self.model.mat_rgba[self.target_mat_id] = rgba
        if self.target_geom_id >= 0:
            rgba = _TARGET_MATERIAL_RGBA.copy()
            rgba[3] *= alpha
            self.model.geom_rgba[self.target_geom_id] = rgba
            if self._target_geom_size is not None:
                self.model.geom_size[self.target_geom_id] = (
                    self._target_geom_size if visible else 0.0
                )
        if self.target_site_id >= 0:
            rgba = _TARGET_SITE_RGBA.copy()
            rgba[3] *= alpha
            self.model.site_rgba[self.target_site_id] = rgba
            if self._target_site_size is not None:
                self.model.site_size[self.target_site_id] = (
                    self._target_site_size if visible else 0.0
                )

    def _update_virtual_grasp(self) -> bool:
        if (
            not self.virtual_grasp_enabled
            or self.tcp_site_id < 0
            or self.finger_left_qpos_addr is None
            or not self.virtual_grasp_objects
        ):
            return False

        opening = float(abs(self.data.qpos[self.finger_left_qpos_addr]))
        tcp_pos = np.array(self.data.site_xpos[self.tcp_site_id], dtype=np.float64)
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._last_grasp_update_time is None else float(
            np.clip(now - self._last_grasp_update_time, 0.0, 0.12)
        )
        self._last_grasp_update_time = now

        if self._attached_object is not None:
            if opening >= self._attached_object.release_qpos:
                self._falling_objects[self._attached_object.name] = 0.0
                self._attached_object = None
                return self._update_falling_objects(dt)
            qpos_addr = self._attached_object.qpos_addr
            next_pos = tcp_pos + self._attached_offset
            changed = not np.allclose(self.data.qpos[qpos_addr : qpos_addr + 3], next_pos)
            self.data.qpos[qpos_addr : qpos_addr + 3] = next_pos
            return bool(changed or self._hold_gripper_at_object_width(self._attached_object))

        origins = [("tcp", tcp_pos)]
        with self._lock:
            target_pose = self._target_pose
            target_pose_monotonic = self._target_pose_monotonic
        if (
            target_pose is not None
            and target_pose_monotonic is not None
            and now - target_pose_monotonic <= self.virtual_grasp_target_timeout
        ):
            origins.append(("target", np.array(target_pose[0], dtype=np.float64)))

        nearest: tuple[float, VirtualGraspObject] | None = None
        for obj in self.virtual_grasp_objects:
            obj_pos = np.array(self.data.qpos[obj.qpos_addr : obj.qpos_addr + 3], dtype=np.float64)
            distance = min(float(np.linalg.norm(obj_pos - origin)) for _name, origin in origins)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, obj)

        if nearest is None or nearest[0] > self.virtual_grasp_radius:
            return self._update_falling_objects(dt)

        _, obj = nearest
        if opening > obj.grasp_qpos + _GRIPPER_GRASP_TOLERANCE_M:
            return self._update_falling_objects(dt)

        obj_pos = np.array(self.data.qpos[obj.qpos_addr : obj.qpos_addr + 3], dtype=np.float64)
        self._attached_object = obj
        self._attached_offset = obj_pos - tcp_pos
        self._falling_objects.pop(obj.name, None)
        return self._hold_gripper_at_object_width(obj)

    def _hold_gripper_at_object_width(self, obj: VirtualGraspObject) -> bool:
        changed = False
        if self.finger_left_qpos_addr is not None:
            current_left = float(self.data.qpos[self.finger_left_qpos_addr])
            next_left = max(current_left, obj.grasp_qpos)
            if not np.isclose(current_left, next_left):
                self.data.qpos[self.finger_left_qpos_addr] = next_left
                changed = True
        if self.finger_right_qpos_addr is not None:
            current_right = float(self.data.qpos[self.finger_right_qpos_addr])
            next_right = min(current_right, -obj.grasp_qpos)
            if not np.isclose(current_right, next_right):
                self.data.qpos[self.finger_right_qpos_addr] = next_right
                changed = True
        return changed

    def _update_falling_objects(self, dt: float) -> bool:
        if not self._falling_objects or dt <= 0.0:
            return False

        changed = False
        finished: list[str] = []
        by_name = {obj.name: obj for obj in self.virtual_grasp_objects}
        for name, velocity in list(self._falling_objects.items()):
            obj = by_name.get(name)
            if obj is None:
                finished.append(name)
                continue
            qpos_addr = obj.qpos_addr
            current_z = float(self.data.qpos[qpos_addr + 2])
            if current_z <= obj.rest_z + 1e-4:
                self.data.qpos[qpos_addr + 2] = obj.rest_z
                finished.append(name)
                changed = True
                continue
            next_velocity = velocity - self.virtual_grasp_drop_gravity * dt
            next_z = max(obj.rest_z, current_z + next_velocity * dt)
            self.data.qpos[qpos_addr + 2] = next_z
            self._falling_objects[name] = next_velocity
            changed = True
            if next_z <= obj.rest_z + 1e-4:
                self.data.qpos[qpos_addr + 2] = obj.rest_z
                finished.append(name)

        for name in finished:
            self._falling_objects.pop(name, None)
        return changed

    def _ensure_renderer(self) -> mujoco.Renderer | None:
        if self._renderer is not None:
            return self._renderer

        try:
            self._renderer = mujoco.Renderer(
                self.model,
                height=self.height,
                width=self.width,
            )
        except Exception as exc:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_renderer_warning > 5.0:
                self._last_renderer_warning = now
                self.get_logger().warn(f"RGB camera renderer unavailable: {exc}")
            return None
        return self._renderer

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fovy_rad = math.radians(float(self.model.cam_fovy[self.camera_id]))
        fy = self.height / (2.0 * math.tan(fovy_rad / 2.0))
        fx = fy
        cx = (self.width - 1.0) / 2.0
        cy = (self.height - 1.0) / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info


def main(args=None) -> None:
    rclpy.init(args=args)
    node: MujocoSimRgbCamera | None = None
    try:
        node = MujocoSimRgbCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
