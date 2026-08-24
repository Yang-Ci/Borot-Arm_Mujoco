from __future__ import annotations

from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class SliderSpec:
    name: str
    label: str
    lower: float
    upper: float
    default: float
    resolution: float
    unit: str


SLIDERS = [
    SliderSpec("joint1", "关节1 / 底座旋转", -2.8, 2.8, 0.0, 0.001, "rad"),
    SliderSpec("joint2", "关节2 / 肩部俯仰", -3.14, 0.0, 0.0, 0.001, "rad"),
    SliderSpec("joint3", "关节3 / 肘部俯仰", -3.14, 0.0, 0.0, 0.001, "rad"),
    SliderSpec("joint4", "关节4 / 腕部旋转", -1.87, 1.57, 0.0, 0.001, "rad"),
    SliderSpec("joint5", "关节5 / 腕部俯仰", -1.57, 1.57, 0.0, 0.001, "rad"),
    SliderSpec("joint6", "关节6 / 末端旋转", -3.14, 3.14, 0.0, 0.001, "rad"),
    SliderSpec("finger_left", "夹爪开合", 0.0, 0.0285, 0.0, 0.0005, "m"),
]


class JointSliderGui(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_joint_slider_gui")

        self.declare_parameter("joint_state_topic", "/rebotarm/joint_states")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("arm_namespace", "rebotarm")

        self.topic = str(
            self.get_parameter("joint_state_topic").value or "/rebotarm/joint_states"
        )
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.arm_namespace = str(
            self.get_parameter("arm_namespace").value or "rebotarm"
        ).strip("/")
        self.publisher = self.create_publisher(JointState, self.topic, 10)
        self.gc_start_client = self.create_client(
            Trigger,
            f"/{self.arm_namespace}/gravity_compensation/start",
        )
        self.gc_stop_client = self.create_client(
            Trigger,
            f"/{self.arm_namespace}/gravity_compensation/stop",
        )
        self.gc_status_client = self.create_client(
            Trigger,
            f"/{self.arm_namespace}/gravity_compensation/status",
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            ArmStatus,
            f"/{self.arm_namespace}/arm_status",
            self._arm_status_callback,
            latched_qos,
        )

        self.root = tk.Tk()
        self.root.title("reBotArm real2sim 关节滑块")
        self.root.geometry("760x590")
        self.root.minsize(680, 530)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.values: dict[str, tk.DoubleVar] = {}
        self.value_labels: dict[str, ttk.Label] = {}
        self.continuous_publish = tk.BooleanVar(value=True)
        self.status_text = tk.StringVar()
        self.gc_status_text = tk.StringVar(value="重力补偿：未查询")

        self._build_ui()
        self._publish_once()
        self._schedule_publish()
        self._schedule_ros_spin()

        self.get_logger().info(
            f"joint slider GUI publishing {len(SLIDERS)} joints to {self.topic} "
            f"at {self.publish_hz:.1f} Hz"
        )

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(16, 12, 16, 8))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        title = ttk.Label(header, text="reBotArm 关节滑块控制", font=("", 16, "bold"))
        title.grid(row=0, column=0, sticky="w")
        subtitle = ttk.Label(
            header,
            text=f"发布话题：{self.topic}    频率：{self.publish_hz:.1f} Hz",
        )
        subtitle.grid(row=1, column=0, sticky="w", pady=(4, 0))

        slider_frame = ttk.Frame(self.root, padding=(16, 0, 16, 8))
        slider_frame.grid(row=1, column=0, sticky="nsew")
        slider_frame.columnconfigure(1, weight=1)

        for row, spec in enumerate(SLIDERS):
            value = tk.DoubleVar(value=spec.default)
            self.values[spec.name] = value

            label = ttk.Label(slider_frame, text=spec.label, width=18)
            label.grid(row=row, column=0, sticky="w", padx=(0, 10), pady=6)

            scale = ttk.Scale(
                slider_frame,
                from_=spec.lower,
                to=spec.upper,
                variable=value,
                command=lambda _unused, s=spec: self._on_slider_change(s),
            )
            scale.grid(row=row, column=1, sticky="ew", pady=6)

            value_label = ttk.Label(slider_frame, width=13)
            value_label.grid(row=row, column=2, sticky="e", padx=(12, 0), pady=6)
            self.value_labels[spec.name] = value_label
            self._refresh_value_label(spec)

            range_label = ttk.Label(
                slider_frame,
                text=f"{spec.lower:g} ~ {spec.upper:g} {spec.unit}",
                width=18,
            )
            range_label.grid(row=row, column=3, sticky="e", padx=(12, 0), pady=6)

        controls = ttk.Frame(self.root, padding=(16, 4, 16, 8))
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(8, weight=1)

        ttk.Button(controls, text="复位", command=self.reset_all).grid(
            row=0, column=0, padx=(0, 8)
        )
        ttk.Button(controls, text="夹爪闭合", command=lambda: self.set_gripper(0.0)).grid(
            row=0, column=1, padx=8
        )
        ttk.Button(controls, text="夹爪半开", command=lambda: self.set_gripper(0.014)).grid(
            row=0, column=2, padx=8
        )
        ttk.Button(controls, text="夹爪全开", command=lambda: self.set_gripper(0.0285)).grid(
            row=0, column=3, padx=8
        )
        ttk.Button(controls, text="发布一次", command=self._publish_once).grid(
            row=0, column=4, padx=8
        )
        ttk.Checkbutton(
            controls,
            text="连续发布",
            variable=self.continuous_publish,
        ).grid(row=0, column=5, padx=8)

        gravity = ttk.LabelFrame(self.root, text="重力补偿", padding=(16, 8, 16, 10))
        gravity.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 8))
        gravity.columnconfigure(3, weight=1)

        ttk.Button(
            gravity,
            text="启动",
            command=self.start_gravity_compensation,
        ).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(
            gravity,
            text="停止",
            command=self.stop_gravity_compensation,
        ).grid(row=0, column=1, padx=8)
        ttk.Button(
            gravity,
            text="查询状态",
            command=self.query_gravity_compensation,
        ).grid(row=0, column=2, padx=8)
        ttk.Label(gravity, textvariable=self.gc_status_text).grid(
            row=0,
            column=3,
            sticky="w",
            padx=(14, 0),
        )

        footer = ttk.Frame(self.root, padding=(16, 0, 16, 14))
        footer.grid(row=4, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        self.status_text.set("就绪")
        ttk.Label(footer, textvariable=self.status_text).grid(row=0, column=0, sticky="w")

    def _on_slider_change(self, spec: SliderSpec) -> None:
        self._refresh_value_label(spec)
        if not self.continuous_publish.get():
            self.status_text.set("滑块已改变，点击“发布一次”发送当前姿态")

    def _refresh_value_label(self, spec: SliderSpec) -> None:
        value = self.values[spec.name].get()
        self.value_labels[spec.name].configure(text=f"{value:.4f} {spec.unit}")

    def _schedule_publish(self) -> None:
        if rclpy.ok():
            if self.continuous_publish.get():
                self._publish_once()
            delay_ms = max(int(1000.0 / self.publish_hz), 1)
            self.root.after(delay_ms, self._schedule_publish)

    def _schedule_ros_spin(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(30, self._schedule_ros_spin)

    def _publish_once(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [spec.name for spec in SLIDERS]
        msg.position = [float(self.values[spec.name].get()) for spec in SLIDERS]
        msg.velocity = [0.0 for _spec in SLIDERS]
        msg.effort = [0.0 for _spec in SLIDERS]
        self.publisher.publish(msg)

        gripper = self.values["finger_left"].get()
        self.status_text.set(
            f"已发布 {len(msg.name)} 个关节到 {self.topic}，夹爪={gripper:.4f} m"
        )

    def reset_all(self) -> None:
        for spec in SLIDERS:
            self.values[spec.name].set(spec.default)
            self._refresh_value_label(spec)
        self._publish_once()

    def set_gripper(self, value: float) -> None:
        self.values["finger_left"].set(value)
        self._refresh_value_label(SLIDERS[-1])
        self._publish_once()

    def start_gravity_compensation(self) -> None:
        self._call_gravity_service(self.gc_start_client, "启动重力补偿")

    def stop_gravity_compensation(self) -> None:
        self._call_gravity_service(self.gc_stop_client, "停止重力补偿")

    def query_gravity_compensation(self) -> None:
        self._call_gravity_service(self.gc_status_client, "查询重力补偿")

    def _call_gravity_service(self, client, label: str) -> None:
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.05)
        if not client.service_is_ready():
            self.gc_status_text.set(f"{label}：服务不可用")
            return

        self.gc_status_text.set(f"{label}：请求中")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done_future, done_label=label: self._handle_gravity_response(
                done_future,
                done_label,
            )
        )

    def _handle_gravity_response(self, future, label: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.gc_status_text.set(f"{label}失败：{exc}")
            return
        if label.startswith("停止") and response.success:
            active = False
        else:
            active = bool(response.success)
        state = "运行中" if active else "未运行"
        self.gc_status_text.set(
            f"重力补偿：{state} / {response.message or label}"
        )

    def _arm_status_callback(self, msg: ArmStatus) -> None:
        active = msg.state_machine == "GRAVITY_COMP"
        state = "运行中" if active else "未运行"
        detail = msg.state_machine or "UNKNOWN"
        self.gc_status_text.set(f"重力补偿：{state} / {detail}")

    def close(self) -> None:
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JointSliderGui()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
