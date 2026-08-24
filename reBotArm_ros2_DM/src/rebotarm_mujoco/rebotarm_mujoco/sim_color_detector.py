from __future__ import annotations

from collections import deque
from contextlib import redirect_stderr
from dataclasses import dataclass, replace
import io
import json
import math
import time

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    # Some ROS desktop images provide a system OpenCV built against NumPy 1.x,
    # while this project's venv uses NumPy 2.x.  NumPy prints a full traceback
    # to stderr before the optional import raises; keep startup output clean and
    # fall back to the built-in NumPy connected-components implementation.
    with redirect_stderr(io.StringIO()):
        import cv2  # type: ignore
except Exception:  # pragma: no cover - optional runtime dependency
    cv2 = None


_BOX_COLORS = {
    "red": (255, 40, 30),
    "blue": (0, 220, 255),
    "yellow": (255, 230, 40),
}

_OBJECT_COLOR_BY_NAME = {
    "red_cube": "red",
    "blue_block": "blue",
    "yellow_cylinder": "yellow",
}


@dataclass(frozen=True)
class Detection:
    color: str
    area: int
    u: float
    v: float
    bbox: tuple[int, int, int, int]
    x: float
    y: float
    z: float
    width_m: float
    height_m: float
    grasp_yaw_rad: float

    def as_dict(self) -> dict[str, float | int | str | list[int]]:
        x0, y0, x1, y1 = self.bbox
        longest = max(self.width_m, self.height_m)
        shortest = min(self.width_m, self.height_m)
        return {
            "color": self.color,
            "area": self.area,
            "u": round(self.u, 2),
            "v": round(self.v, 2),
            "bbox": [x0, y0, x1, y1],
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "z": round(self.z, 4),
            "width_m": round(self.width_m, 4),
            "height_m": round(self.height_m, 4),
            "longest_m": round(longest, 4),
            "shortest_m": round(shortest, 4),
            "grasp_yaw_rad": round(self.grasp_yaw_rad, 4),
        }


class MujocoSimColorDetector(Node):
    """Detect simple colored blocks from the simulated overhead RGB camera."""

    def __init__(self) -> None:
        super().__init__("mujoco_sim_color_detector")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("annotated_topic", "")
        self.declare_parameter("detections_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("poses_topic", "")
        self.declare_parameter("object_states_topic", "")
        self.declare_parameter("object_association_max_distance_m", 0.055)
        self.declare_parameter("object_states_max_age_s", 0.5)
        self.declare_parameter("target_color", "red")
        self.declare_parameter("process_hz", 8.0)
        self.declare_parameter("min_area_px", 35)
        self.declare_parameter("max_area_px", 5000)
        self.declare_parameter("min_bbox_width_px", 4)
        self.declare_parameter("min_bbox_height_px", 4)
        self.declare_parameter("max_bbox_width_px", 52)
        self.declare_parameter("max_bbox_height_px", 34)
        self.declare_parameter("object_max_size_m", 0.13)
        self.declare_parameter("table_edge_margin_m", 0.025)
        self.declare_parameter("roi_u_min", 0.24)
        self.declare_parameter("roi_u_max", 0.82)
        self.declare_parameter("roi_v_min", 0.29)
        self.declare_parameter("roi_v_max", 0.86)
        self.declare_parameter("camera_x", 0.42)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.86)
        self.declare_parameter("table_z", 0.033)
        self.declare_parameter("table_x_min", 0.24)
        self.declare_parameter("table_x_max", 0.72)
        self.declare_parameter("table_y_min", -0.26)
        self.declare_parameter("table_y_max", 0.26)
        self.declare_parameter("fovy_deg", 50.0)
        self.declare_parameter("target_z", 0.18)

        namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        self.image_topic = str(
            self.get_parameter("image_topic").value
            or f"/{namespace}/mujoco/overhead_rgb/image_raw"
        )
        self.annotated_topic = str(
            self.get_parameter("annotated_topic").value
            or f"/{namespace}/vision/color_blocks/annotated"
        )
        self.detections_topic = str(
            self.get_parameter("detections_topic").value
            or f"/{namespace}/vision/color_blocks/detections"
        )
        self.target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
            or f"/{namespace}/vision/color_blocks/target_pose"
        )
        self.poses_topic = str(
            self.get_parameter("poses_topic").value
            or f"/{namespace}/vision/color_blocks/poses"
        )
        self.object_states_topic = str(
            self.get_parameter("object_states_topic").value
            or f"/{namespace}/mujoco/object_states"
        )
        self.object_association_max_distance_m = max(
            float(self.get_parameter("object_association_max_distance_m").value),
            0.0,
        )
        self.object_states_max_age_s = max(
            float(self.get_parameter("object_states_max_age_s").value),
            0.0,
        )
        self.target_color = str(self.get_parameter("target_color").value or "red").lower()
        self.process_hz = max(float(self.get_parameter("process_hz").value), 0.5)
        self.min_area_px = max(int(self.get_parameter("min_area_px").value), 1)
        self.max_area_px = max(int(self.get_parameter("max_area_px").value), self.min_area_px)
        self.min_bbox_width_px = max(int(self.get_parameter("min_bbox_width_px").value), 1)
        self.min_bbox_height_px = max(int(self.get_parameter("min_bbox_height_px").value), 1)
        self.max_bbox_width_px = max(
            int(self.get_parameter("max_bbox_width_px").value),
            self.min_bbox_width_px,
        )
        self.max_bbox_height_px = max(
            int(self.get_parameter("max_bbox_height_px").value),
            self.min_bbox_height_px,
        )
        self.object_max_size_m = max(float(self.get_parameter("object_max_size_m").value), 0.01)
        self.table_edge_margin_m = max(float(self.get_parameter("table_edge_margin_m").value), 0.0)
        self.roi = (
            float(self.get_parameter("roi_u_min").value),
            float(self.get_parameter("roi_u_max").value),
            float(self.get_parameter("roi_v_min").value),
            float(self.get_parameter("roi_v_max").value),
        )
        self.camera_x = float(self.get_parameter("camera_x").value)
        self.camera_y = float(self.get_parameter("camera_y").value)
        self.camera_z = float(self.get_parameter("camera_z").value)
        self.table_z = float(self.get_parameter("table_z").value)
        self.table_bounds = (
            float(self.get_parameter("table_x_min").value),
            float(self.get_parameter("table_x_max").value),
            float(self.get_parameter("table_y_min").value),
            float(self.get_parameter("table_y_max").value),
        )
        self.fovy_deg = float(self.get_parameter("fovy_deg").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self._last_process = 0.0
        self._object_states_at = 0.0
        self._object_xy_by_color: dict[str, tuple[float, float]] = {}

        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, qos_profile_sensor_data)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.poses_pub = self.create_publisher(PoseArray, self.poses_topic, 10)
        self.create_subscription(Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
        self.create_subscription(
            String,
            self.object_states_topic,
            self._object_states_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "MuJoCo color detector ready: "
            f"image={self.image_topic}, detections={self.detections_topic}, "
            f"target_pose={self.target_pose_topic}, target_color={self.target_color}, "
            f"object_states={self.object_states_topic}, "
            f"opencv={'yes' if cv2 is not None else 'no'}"
        )

    def _object_states_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
        except Exception:
            return
        objects = payload.get("objects")
        if not isinstance(objects, list):
            return

        next_positions: dict[str, tuple[float, float]] = {}
        for item in objects:
            if not isinstance(item, dict):
                continue
            color = _OBJECT_COLOR_BY_NAME.get(str(item.get("name", "")))
            position = item.get("position")
            if color is None or not isinstance(position, list) or len(position) < 2:
                continue
            try:
                x = float(position[0])
                y = float(position[1])
            except (TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                next_positions[color] = (x, y)

        if next_positions:
            self._object_xy_by_color = next_positions
            self._object_states_at = time.monotonic()

    def _image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_process < 1.0 / self.process_hz:
            return
        self._last_process = now

        rgb = self._decode_rgb(msg)
        if rgb is None:
            return

        detections = self._detect(rgb)
        target = self._choose_target(detections)
        annotated = self._annotate(rgb, detections, target)

        stamp = self.get_clock().now().to_msg()
        self._publish_annotated(annotated, msg.header.frame_id, stamp)
        self._publish_detection_text(detections, target, stamp)
        self._publish_poses(detections, stamp)
        if target is not None:
            self._publish_target_pose(target, stamp)

    def _decode_rgb(self, msg: Image) -> np.ndarray | None:
        encoding = str(msg.encoding or "rgb8").lower()
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        supported = encoding in ("rgb8", "bgr8", "rgba8", "bgra8")
        if not supported:
            self.get_logger().warn(f"unsupported image encoding: {msg.encoding}")
            return None

        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        row_step = int(msg.step) if msg.step else int(msg.width) * channels
        expected = row_step * int(msg.height)
        if data.size < expected:
            self.get_logger().warn("image data is shorter than expected")
            return None

        rows = data[:expected].reshape((int(msg.height), row_step))
        packed = rows[:, : int(msg.width) * channels].reshape(
            (int(msg.height), int(msg.width), channels)
        )
        rgb = packed[:, :, :3].copy()
        if encoding in ("bgr8", "bgra8"):
            rgb = rgb[:, :, ::-1]
        return rgb

    def _detect(self, rgb: np.ndarray) -> list[Detection]:
        r = rgb[:, :, 0].astype(np.int16)
        g = rgb[:, :, 1].astype(np.int16)
        b = rgb[:, :, 2].astype(np.int16)
        workspace_mask = self._workspace_mask(rgb.shape[1], rgb.shape[0])

        masks = {
            "red": (r > 135) & (r - g > 45) & (r - b > 35),
            "blue": (b > 135) & (g > 125) & (r < 125) & (b - r > 55) & (g - r > 55),
            "yellow": (r > 140) & (g > 125) & (b < 135) & (r - b > 45) & (g - b > 35),
        }

        detections: list[Detection] = []
        for color, mask in masks.items():
            components = self._components(mask & workspace_mask)
            for area, cx, cy, bbox in components:
                if area < self.min_area_px or area > self.max_area_px:
                    continue
                if not self._passes_shape_filter(bbox, rgb.shape[1], rgb.shape[0]):
                    continue
                x, y = self._pixel_to_world(cx, cy, rgb.shape[1], rgb.shape[0])
                if not self._inside_table(x, y):
                    continue
                obj_w, obj_h = self._bbox_world_size(bbox, rgb.shape[1], rgb.shape[0])
                grasp_yaw = -math.pi / 2.0 if obj_w <= obj_h else 0.0
                detections.append(
                    Detection(
                        color=color,
                        area=int(area),
                        u=float(cx),
                        v=float(cy),
                        bbox=bbox,
                        x=x,
                        y=y,
                        z=self.target_z,
                        width_m=obj_w,
                        height_m=obj_h,
                        grasp_yaw_rad=grasp_yaw,
                    )
                )
        detections = self._associate_with_mujoco_objects(detections)
        detections.sort(key=lambda item: (-item.area, item.color))
        return detections

    def _associate_with_mujoco_objects(self, detections: list[Detection]) -> list[Detection]:
        if (
            not self._object_xy_by_color
            or time.monotonic() - self._object_states_at > self.object_states_max_age_s
        ):
            return detections

        associated: list[Detection] = []
        colors_with_reference = set(self._object_xy_by_color)
        for color, reference in self._object_xy_by_color.items():
            candidates = [item for item in detections if item.color == color]
            if not candidates:
                continue
            nearest = min(
                candidates,
                key=lambda item: math.hypot(item.x - reference[0], item.y - reference[1]),
            )
            distance = math.hypot(nearest.x - reference[0], nearest.y - reference[1])
            if distance <= self.object_association_max_distance_m:
                # This detector is simulation-specific and already receives the
                # authoritative MuJoCo object poses.  Keep the image-derived
                # footprint/yaw, but use the associated body's exact center for
                # motion planning so pixel quantization and height parallax do
                # not turn into a one-sided grasp.
                associated.append(replace(nearest, x=reference[0], y=reference[1]))

        associated.extend(
            item for item in detections if item.color not in colors_with_reference
        )
        return associated

    def _workspace_mask(self, width: int, height: int) -> np.ndarray:
        u_min, u_max, v_min, v_max = self.roi
        x0 = int(np.clip(u_min, 0.0, 1.0) * width)
        x1 = int(np.clip(u_max, 0.0, 1.0) * width)
        y0 = int(np.clip(v_min, 0.0, 1.0) * height)
        y1 = int(np.clip(v_max, 0.0, 1.0) * height)
        mask = np.zeros((height, width), dtype=bool)
        mask[min(y0, y1) : max(y0, y1), min(x0, x1) : max(x0, x1)] = True

        xs = np.arange(width, dtype=np.float64)
        ys = np.arange(height, dtype=np.float64)
        grid_u, grid_v = np.meshgrid(xs, ys)
        world_x, world_y = self._pixels_to_world(grid_u, grid_v, width, height)
        x_min, x_max, y_min, y_max = self.table_bounds
        table_mask = (
            (world_x >= min(x_min, x_max))
            & (world_x <= max(x_min, x_max))
            & (world_y >= min(y_min, y_max))
            & (world_y <= max(y_min, y_max))
        )
        return mask & table_mask

    def _components(self, mask: np.ndarray) -> list[tuple[int, float, float, tuple[int, int, int, int]]]:
        if cv2 is not None:
            count, labels, stats, centroids = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8),
                connectivity=8,
            )
            components = []
            for label in range(1, count):
                x, y, w, h, area = stats[label]
                cx, cy = centroids[label]
                components.append((int(area), float(cx), float(cy), (int(x), int(y), int(x + w), int(y + h))))
            return components
        return self._components_numpy(mask)

    @staticmethod
    def _components_numpy(mask: np.ndarray) -> list[tuple[int, float, float, tuple[int, int, int, int]]]:
        height, width = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        components: list[tuple[int, float, float, tuple[int, int, int, int]]] = []
        ys, xs = np.nonzero(mask)
        for start_x, start_y in zip(xs, ys):
            if visited[start_y, start_x]:
                continue
            queue: deque[tuple[int, int]] = deque([(int(start_x), int(start_y))])
            visited[start_y, start_x] = True
            area = 0
            sum_x = 0
            sum_y = 0
            x0 = x1 = int(start_x)
            y0 = y1 = int(start_y)
            while queue:
                x, y = queue.popleft()
                area += 1
                sum_x += x
                sum_y += y
                x0 = min(x0, x)
                x1 = max(x1, x)
                y0 = min(y0, y)
                y1 = max(y1, y)
                for ny in range(max(0, y - 1), min(height, y + 2)):
                    for nx in range(max(0, x - 1), min(width, x + 2)):
                        if visited[ny, nx] or not mask[ny, nx]:
                            continue
                        visited[ny, nx] = True
                        queue.append((nx, ny))
            components.append((area, sum_x / area, sum_y / area, (x0, y0, x1 + 1, y1 + 1)))
        return components

    def _pixel_to_world(self, u: float, v: float, width: int, height: int) -> tuple[float, float]:
        x, y = self._pixels_to_world(float(u), float(v), width, height)
        return float(x), float(y)

    def _pixels_to_world(self, u, v, width: int, height: int):
        depth = max(self.camera_z - self.table_z, 0.05)
        fovy = math.radians(self.fovy_deg)
        focal = height / (2.0 * math.tan(fovy / 2.0))
        cx = (width - 1.0) / 2.0
        cy = (height - 1.0) / 2.0
        x = self.camera_x + (u - cx) * depth / focal
        y = self.camera_y - (v - cy) * depth / focal
        return x, y

    def _inside_table(self, x: float, y: float) -> bool:
        x_min, x_max, y_min, y_max = self.table_bounds
        return min(x_min, x_max) <= x <= max(x_min, x_max) and min(y_min, y_max) <= y <= max(y_min, y_max)

    def _passes_shape_filter(self, bbox: tuple[int, int, int, int], width: int, height: int) -> bool:
        x0, y0, x1, y1 = bbox
        bbox_w = max(int(x1 - x0), 0)
        bbox_h = max(int(y1 - y0), 0)
        if bbox_w < self.min_bbox_width_px or bbox_h < self.min_bbox_height_px:
            return False
        if bbox_w > self.max_bbox_width_px or bbox_h > self.max_bbox_height_px:
            return False

        obj_w, obj_h = self._bbox_world_size(bbox, width, height)
        if max(obj_w, obj_h) > self.object_max_size_m:
            return False

        min_x, max_x, min_y, max_y = self._bbox_world_bounds(bbox, width, height)
        table_x_min, table_x_max, table_y_min, table_y_max = self.table_bounds
        margin = self.table_edge_margin_m
        if min_x <= min(table_x_min, table_x_max) + margin:
            return False
        if max_x >= max(table_x_min, table_x_max) - margin:
            return False
        if min_y <= min(table_y_min, table_y_max) + margin:
            return False
        if max_y >= max(table_y_min, table_y_max) - margin:
            return False
        return True

    def _bbox_world_size(self, bbox: tuple[int, int, int, int], width: int, height: int) -> tuple[float, float]:
        x0, y0, x1, y1 = bbox
        world_x0, world_y0 = self._pixel_to_world(x0, y0, width, height)
        world_x1, world_y1 = self._pixel_to_world(
            max(x1 - 1, x0),
            max(y1 - 1, y0),
            width,
            height,
        )
        return abs(world_x1 - world_x0), abs(world_y1 - world_y0)

    def _bbox_world_bounds(self, bbox: tuple[int, int, int, int], width: int, height: int) -> tuple[float, float, float, float]:
        x0, y0, x1, y1 = bbox
        corners = [
            self._pixel_to_world(x0, y0, width, height),
            self._pixel_to_world(max(x1 - 1, x0), y0, width, height),
            self._pixel_to_world(x0, max(y1 - 1, y0), width, height),
            self._pixel_to_world(max(x1 - 1, x0), max(y1 - 1, y0), width, height),
        ]
        xs = [corner[0] for corner in corners]
        ys = [corner[1] for corner in corners]
        return min(xs), max(xs), min(ys), max(ys)

    def _choose_target(self, detections: list[Detection]) -> Detection | None:
        preferred = [item for item in detections if item.color == self.target_color]
        candidates = preferred or detections
        return max(candidates, key=lambda item: item.area) if candidates else None

    def _annotate(
        self,
        rgb: np.ndarray,
        detections: list[Detection],
        target: Detection | None,
    ) -> np.ndarray:
        annotated = rgb.copy()
        workspace_mask = self._workspace_mask(rgb.shape[1], rgb.shape[0])
        annotated[~workspace_mask] = (annotated[~workspace_mask].astype(np.float32) * 0.22).astype(np.uint8)
        for detection in detections:
            color = _BOX_COLORS[detection.color]
            self._draw_rect(annotated, detection.bbox, color)
            self._draw_cross(annotated, int(round(detection.u)), int(round(detection.v)), color)
            if cv2 is not None:
                cv2.putText(
                    annotated,
                    detection.color,
                    (detection.bbox[0], max(detection.bbox[1] - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.36,
                    color,
                    1,
                    cv2.LINE_AA,
                )
        if target is not None:
            self._draw_cross(annotated, int(round(target.u)), int(round(target.v)), (255, 255, 255), radius=8)
        return annotated

    @staticmethod
    def _draw_rect(image: np.ndarray, bbox: tuple[int, int, int, int], color: tuple[int, int, int]) -> None:
        x0, y0, x1, y1 = bbox
        x0 = int(np.clip(x0, 0, image.shape[1] - 1))
        x1 = int(np.clip(x1, 0, image.shape[1] - 1))
        y0 = int(np.clip(y0, 0, image.shape[0] - 1))
        y1 = int(np.clip(y1, 0, image.shape[0] - 1))
        image[y0 : y0 + 2, x0:x1] = color
        image[max(y1 - 2, y0) : y1, x0:x1] = color
        image[y0:y1, x0 : x0 + 2] = color
        image[y0:y1, max(x1 - 2, x0) : x1] = color

    @staticmethod
    def _draw_cross(
        image: np.ndarray,
        x: int,
        y: int,
        color: tuple[int, int, int],
        radius: int = 5,
    ) -> None:
        x0 = max(x - radius, 0)
        x1 = min(x + radius + 1, image.shape[1])
        y0 = max(y - radius, 0)
        y1 = min(y + radius + 1, image.shape[0])
        if 0 <= y < image.shape[0]:
            image[y, x0:x1] = color
        if 0 <= x < image.shape[1]:
            image[y0:y1, x] = color

    def _publish_annotated(self, rgb: np.ndarray, frame_id: str, stamp) -> None:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id or "overhead_rgb_frame"
        msg.height = int(rgb.shape[0])
        msg.width = int(rgb.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = int(rgb.shape[1] * 3)
        msg.data = np.ascontiguousarray(rgb).tobytes()
        self.annotated_pub.publish(msg)

    def _publish_detection_text(
        self,
        detections: list[Detection],
        target: Detection | None,
        stamp,
    ) -> None:
        payload = {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "target_color": self.target_color,
            "count": len(detections),
            "target": target.as_dict() if target is not None else None,
            "detections": [item.as_dict() for item in detections],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(msg)

    def _publish_poses(self, detections: list[Detection], stamp) -> None:
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        for detection in detections:
            msg.poses.append(self._pose_from_detection(detection))
        self.poses_pub.publish(msg)

    def _publish_target_pose(self, detection: Detection, stamp) -> None:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.pose = self._pose_from_detection(detection)
        self.target_pose_pub.publish(msg)

    @staticmethod
    def _pose_from_detection(detection: Detection) -> Pose:
        pose = Pose()
        pose.position.x = detection.x
        pose.position.y = detection.y
        pose.position.z = detection.z
        pose.orientation.w = 1.0
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node: MujocoSimColorDetector | None = None
    try:
        node = MujocoSimColorDetector()
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
