from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
import threading
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml


@dataclass(frozen=True)
class JointMapping:
    mujoco: str
    scale: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class ResolvedJointTarget:
    mujoco: str
    qpos_addr: int
    scale: float = 1.0
    offset: float = 0.0
    lower: float | None = None
    upper: float | None = None

    def clamp(self, value: float) -> float:
        if self.lower is not None:
            value = max(value, self.lower)
        if self.upper is not None:
            value = min(value, self.upper)
        return value


@dataclass(frozen=True)
class VirtualGraspObject:
    name: str
    body_id: int
    qpos_addr: int
    grasp_width: float
    grasp_qpos: float
    release_qpos: float
    rest_z: float


_DEFAULT_JOINT_MAP = {
    "joint1": [JointMapping("joint1")],
    "joint2": [JointMapping("joint2")],
    "joint3": [JointMapping("joint3")],
    "joint4": [JointMapping("joint4")],
    "joint5": [JointMapping("joint5")],
    "joint6": [JointMapping("joint6")],
    "finger_left": [JointMapping("finger_left")],
}

_TARGET_MATERIAL_RGBA = np.array([1.0, 0.55, 0.12, 0.72], dtype=np.float32)
_TARGET_SITE_RGBA = np.array([1.0, 0.55, 0.12, 0.35], dtype=np.float32)
_TARGET_HIDDEN_POS = np.array([0.0, 0.0, -10.0], dtype=np.float64)
_TARGET_HIDDEN_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
_DEFAULT_GRASP_OBJECTS = "red_cube,blue_block,yellow_cylinder"
_GRIPPER_BASE_GAP_FALLBACK_M = 0.014
_GRIPPER_GRASP_CLEARANCE_M = 0.004
_GRIPPER_GRASP_RELEASE_MARGIN_M = 0.006
_GRIPPER_GRASP_TOLERANCE_M = 0.0015
_GRIPPER_QPOS_MAX_FALLBACK_M = 0.0285


class MujocoReal2Sim(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_real2sim")

        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_map_file", "")
        self.declare_parameter("joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("target_pose_topic", "/rebotarm/mujoco/target_pose")
        self.declare_parameter("target_body_name", "ik_target")
        self.declare_parameter("target_visible_timeout", 0.7)
        self.declare_parameter("open_viewer", True)
        self.declare_parameter("sync_hz", 60.0)
        self.declare_parameter("smoothing_alpha", 1.0)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("virtual_grasp_enabled", True)
        self.declare_parameter("virtual_grasp_objects", _DEFAULT_GRASP_OBJECTS)
        self.declare_parameter("virtual_grasp_close_threshold", 0.010)
        self.declare_parameter("virtual_grasp_release_threshold", 0.020)
        self.declare_parameter("virtual_grasp_radius", 0.130)
        self.declare_parameter("virtual_grasp_target_timeout", 6.0)
        self.declare_parameter("virtual_grasp_drop_gravity", 9.81)

        self.model_path = self._resolve_package_path(
            str(self.get_parameter("model_path").value or ""),
            "models",
            "rebotarm_b601_colored.xml",
        )
        joint_map_path = self._resolve_package_path(
            str(self.get_parameter("joint_map_file").value or ""),
            "config",
            "joint_map_kinematic.yaml",
        )
        self.joint_state_topic = str(
            self.get_parameter("joint_state_topic").value or "/rebotarm/joint_states"
        )
        self.target_pose_topic = str(
            self.get_parameter("target_pose_topic").value or "/rebotarm/mujoco/target_pose"
        )
        self.target_body_name = str(self.get_parameter("target_body_name").value or "ik_target")
        self.target_visible_timeout = max(
            float(self.get_parameter("target_visible_timeout").value),
            0.05,
        )
        self.open_viewer = self._as_bool(self.get_parameter("open_viewer").value)
        self.sync_hz = max(float(self.get_parameter("sync_hz").value), 1.0)
        self.smoothing_alpha = float(
            np.clip(float(self.get_parameter("smoothing_alpha").value), 0.0, 1.0)
        )
        self.stale_timeout = max(float(self.get_parameter("stale_timeout").value), 0.0)
        self.virtual_grasp_enabled = self._as_bool(
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

        self.model = self._load_mujoco_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.joint_map = self._load_joint_map(joint_map_path)
        self.qpos_targets = self._build_qpos_targets()
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
        self.tcp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.finger_left_qpos_addr = self._joint_qpos_addr("finger_left")
        self.finger_right_qpos_addr = self._joint_qpos_addr("finger_right")
        self.virtual_grasp_objects = self._build_virtual_grasp_objects(
            str(self.get_parameter("virtual_grasp_objects").value or _DEFAULT_GRASP_OBJECTS)
        )

        self._lock = threading.Lock()
        self._target_qpos: dict[int, float] = {}
        self._target_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._target_pose_monotonic: float | None = None
        self._target_visible = True
        self._last_msg_monotonic: float | None = None
        self._warned_missing: set[str] = set()
        self._viewer = None
        self._attached_object: VirtualGraspObject | None = None
        self._attached_offset = np.zeros(3, dtype=np.float64)
        self._falling_objects: dict[str, float] = {}
        self._last_grasp_update_time: float | None = None
        self._set_target_visible_locked(False)

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.sync_hz, self._sync_timer)

        if self.open_viewer:
            self._open_viewer_window()

        self.get_logger().info(
            "MuJoCo real2sim ready: "
            f"model={self.model_path}, topic={self.joint_state_topic}, "
            f"joints={self._describe_joint_targets()}, "
            f"target_body={self.target_body_name if self.target_mocap_id >= 0 else 'unavailable'}, "
            f"virtual_grasp={self.virtual_grasp_enabled}"
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _package_root() -> Path:
        try:
            return Path(get_package_share_directory("rebotarm_mujoco"))
        except Exception:
            return Path(__file__).resolve().parents[1]

    @classmethod
    def _resolve_package_path(cls, value: str, *fallback_parts: str) -> Path:
        path = Path(value).expanduser() if value else cls._package_root().joinpath(*fallback_parts)
        if not path.is_absolute():
            path = cls._package_root() / path
        return path.resolve()

    @classmethod
    def _load_mujoco_model(cls, path: Path) -> mujoco.MjModel:
        resolved = cls._xml_with_resolved_mesh_assets(path)
        if resolved is None:
            return mujoco.MjModel.from_xml_path(str(path))
        resolved_xml, assets = resolved
        return mujoco.MjModel.from_xml_string(resolved_xml, assets=assets)

    @classmethod
    def _xml_with_resolved_mesh_assets(cls, path: Path) -> tuple[str, dict[str, bytes]] | None:
        tree = ET.parse(path)
        root = tree.getroot()
        mesh_elements = root.findall(".//mesh[@file]")
        if not mesh_elements:
            return None

        compiler = root.find("compiler")
        meshdir_value = compiler.get("meshdir") if compiler is not None else None
        meshdir = Path(meshdir_value).expanduser() if meshdir_value else path.parent
        if not meshdir.is_absolute():
            meshdir = (path.parent / meshdir).resolve()

        fallback_mesh_dirs = [
            meshdir,
            cls._package_root() / "meshes",
            cls._package_root().parent / "rebotarm_bringup" / "description" / "meshes",
        ]

        assets: dict[str, bytes] = {}
        for mesh in mesh_elements:
            file_value = mesh.get("file")
            if not file_value:
                continue

            mesh_path = Path(file_value).expanduser()
            if not mesh_path.is_absolute():
                mesh_path = cls._resolve_existing_mesh_path(mesh_path, fallback_mesh_dirs)

            asset_name = mesh_path.name
            asset_data = mesh_path.read_bytes()
            if asset_name in assets and assets[asset_name] != asset_data:
                asset_name = f"{len(assets)}_{asset_name}"
            assets[asset_name] = asset_data
            mesh.set("file", asset_name)

        if compiler is not None:
            compiler.attrib.pop("meshdir", None)

        return ET.tostring(root, encoding="unicode"), assets

    @staticmethod
    def _resolve_existing_mesh_path(mesh_path: Path, mesh_dirs: list[Path]) -> Path:
        for mesh_dir in mesh_dirs:
            candidate = (mesh_dir / mesh_path).resolve()
            if candidate.exists():
                return candidate

        checked = ", ".join(str((mesh_dir / mesh_path).resolve()) for mesh_dir in mesh_dirs)
        raise FileNotFoundError(f"MuJoCo mesh {mesh_path} not found; checked: {checked}")

    @staticmethod
    def _default_joint_map() -> dict[str, list[JointMapping]]:
        return {ros_name: list(mappings) for ros_name, mappings in _DEFAULT_JOINT_MAP.items()}

    @classmethod
    def _load_joint_map(cls, path: Path) -> dict[str, list[JointMapping]]:
        if not path.exists():
            return cls._default_joint_map()

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        raw_joints = data.get("joints", data)
        result: dict[str, list[JointMapping]] = {}
        for ros_name, spec in raw_joints.items():
            mappings = cls._parse_joint_mapping(str(ros_name), spec)
            if mappings:
                result[str(ros_name)] = mappings
        return result or cls._default_joint_map()

    @classmethod
    def _parse_joint_mapping(cls, ros_name: str, spec) -> list[JointMapping]:
        if isinstance(spec, str):
            return [JointMapping(spec)]

        if isinstance(spec, list):
            mappings: list[JointMapping] = []
            for item in spec:
                mappings.extend(cls._parse_joint_mapping(ros_name, item))
            return mappings

        if not isinstance(spec, dict):
            raise ValueError(f"joint mapping for {ros_name!r} must be a string, dict, or list")

        targets = spec.get("targets")
        if targets is not None:
            if not isinstance(targets, list):
                raise ValueError(f"joint mapping targets for {ros_name!r} must be a list")
            mappings: list[JointMapping] = []
            for target in targets:
                mappings.extend(cls._parse_joint_mapping(ros_name, target))
            return mappings

        return [
            JointMapping(
                mujoco=str(spec.get("mujoco", ros_name)),
                scale=float(spec.get("scale", 1.0)),
                offset=float(spec.get("offset", 0.0)),
            )
        ]

    def _build_qpos_targets(self) -> dict[str, list[ResolvedJointTarget]]:
        qpos_targets: dict[str, list[ResolvedJointTarget]] = {}
        for ros_name, mappings in self.joint_map.items():
            resolved_targets: list[ResolvedJointTarget] = []
            for mapping in mappings:
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
                        "for scalar real2sim synchronization"
                    )
                resolved_targets.append(
                    ResolvedJointTarget(
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
                )
            qpos_targets[ros_name] = resolved_targets
        return qpos_targets

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
                f"target body {body_name!r} not found; target pose visualization disabled"
            )
            return -1
        mocap_id = int(self.model.body_mocapid[body_id])
        if mocap_id < 0:
            self.get_logger().warn(
                f"target body {body_name!r} is not a mocap body; target pose visualization disabled"
            )
        return mocap_id

    def _describe_joint_targets(self) -> dict[str, list[str]]:
        return {
            ros_name: [target.mujoco for target in targets]
            for ros_name, targets in self.qpos_targets.items()
        }

    def _open_viewer_window(self) -> None:
        import mujoco.viewer

        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._viewer.sync()

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
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

        if not updates:
            return
        with self._lock:
            self._target_qpos.update(updates)
            self._last_msg_monotonic = now

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
            quat[:] = [1.0, 0.0, 0.0, 0.0]
        else:
            quat /= norm
        with self._lock:
            self._target_pose = (pos, quat)
            self._target_pose_monotonic = self.get_clock().now().nanoseconds * 1e-9

    def _sync_timer(self) -> None:
        if self._viewer is not None and not self._viewer.is_running():
            self.get_logger().info("MuJoCo viewer closed")
            rclpy.shutdown()
            return

        with self._lock:
            targets = dict(self._target_qpos)
            last_msg = self._last_msg_monotonic

        if last_msg is None:
            self._sync_viewer_only()
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.stale_timeout > 0.0 and now - last_msg > self.stale_timeout:
            self._sync_viewer_only()
            return

        viewer_lock = self._viewer.lock() if self._viewer is not None else nullcontext()
        with viewer_lock:
            target_pose_changed = self._apply_target_pose_locked(now)
            for addr, target in targets.items():
                current = float(self.data.qpos[addr])
                if self.smoothing_alpha >= 1.0:
                    self.data.qpos[addr] = target
                else:
                    self.data.qpos[addr] = current + self.smoothing_alpha * (target - current)
            mujoco.mj_forward(self.model, self.data)
            if self._update_virtual_grasp_locked(now):
                mujoco.mj_forward(self.model, self.data)

        self._sync_viewer_only()

    def _sync_viewer_only(self) -> None:
        if self._viewer is None or not self._viewer.is_running():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._viewer.lock():
            if self._apply_target_pose_locked(now):
                mujoco.mj_forward(self.model, self.data)
        self._viewer.sync()

    def _apply_target_pose_locked(self, now: float | None = None) -> bool:
        if self.target_mocap_id < 0:
            return False
        with self._lock:
            target_pose = self._target_pose
            target_pose_monotonic = self._target_pose_monotonic
        if target_pose is None:
            return self._set_target_visible_locked(False)
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        visible = (
            target_pose_monotonic is not None
            and now - target_pose_monotonic <= self.target_visible_timeout
        )
        visual_changed = self._set_target_visible_locked(visible)
        if not visible:
            if not np.allclose(self.data.mocap_pos[self.target_mocap_id], _TARGET_HIDDEN_POS):
                self.data.mocap_pos[self.target_mocap_id] = _TARGET_HIDDEN_POS
                self.data.mocap_quat[self.target_mocap_id] = _TARGET_HIDDEN_QUAT
                return True
            return visual_changed
        pos, quat = target_pose
        changed = not (
            np.allclose(self.data.mocap_pos[self.target_mocap_id], pos)
            and np.allclose(self.data.mocap_quat[self.target_mocap_id], quat)
        )
        self.data.mocap_pos[self.target_mocap_id] = pos
        self.data.mocap_quat[self.target_mocap_id] = quat
        return bool(changed or visual_changed)

    def _set_target_visible_locked(self, visible: bool) -> bool:
        if self._target_visible == visible:
            return False
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
        if self.target_site_id >= 0:
            rgba = _TARGET_SITE_RGBA.copy()
            rgba[3] *= alpha
            self.model.site_rgba[self.target_site_id] = rgba
        return True

    def _update_virtual_grasp_locked(self, now: float | None = None) -> bool:
        if (
            not self.virtual_grasp_enabled
            or self.tcp_site_id < 0
            or self.finger_left_qpos_addr is None
            or not self.virtual_grasp_objects
        ):
            return False

        opening = float(abs(self.data.qpos[self.finger_left_qpos_addr]))
        tcp_pos = np.array(self.data.site_xpos[self.tcp_site_id], dtype=np.float64)
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._last_grasp_update_time is None else float(
            np.clip(now - self._last_grasp_update_time, 0.0, 0.08)
        )
        self._last_grasp_update_time = now

        if self._attached_object is not None:
            if opening >= self._attached_object.release_qpos:
                self.get_logger().info(f"released {self._attached_object.name}")
                self._falling_objects[self._attached_object.name] = 0.0
                self._attached_object = None
                return self._update_falling_objects_locked(dt)
            qpos_addr = self._attached_object.qpos_addr
            next_pos = tcp_pos + self._attached_offset
            changed = not np.allclose(self.data.qpos[qpos_addr : qpos_addr + 3], next_pos)
            self.data.qpos[qpos_addr : qpos_addr + 3] = next_pos
            return bool(changed or self._hold_gripper_at_object_width_locked(self._attached_object))

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
            return self._update_falling_objects_locked(dt)

        _, obj = nearest
        if opening > obj.grasp_qpos + _GRIPPER_GRASP_TOLERANCE_M:
            return self._update_falling_objects_locked(dt)

        obj_pos = np.array(self.data.qpos[obj.qpos_addr : obj.qpos_addr + 3], dtype=np.float64)
        self._attached_object = obj
        self._attached_offset = obj_pos - tcp_pos
        self._falling_objects.pop(obj.name, None)
        self.get_logger().info(
            f"attached {obj.name} at gripper={opening:.4f}m "
            f"(object_width={obj.grasp_width:.3f}m)"
        )
        return self._hold_gripper_at_object_width_locked(obj)

    def _hold_gripper_at_object_width_locked(self, obj: VirtualGraspObject) -> bool:
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

    def _update_falling_objects_locked(self, dt: float) -> bool:
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

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoReal2Sim()
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
