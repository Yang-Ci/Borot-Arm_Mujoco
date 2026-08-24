from __future__ import annotations

import threading
import time
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from .conversions import fk_to_pose

_G_MAX_DIST_M = 0.09
_G_ANGLE_OPEN = -5.0
_G_OPEN_SOFT_LIMIT = -4.9
_G_ARRIVE_TOL = 0.12
_G_TAU_MAX = 1.5
_G_DEFAULT_FORCE = 0.30
_G_CTRL_RATE = 50.0
_GC_VEL_THRESHOLD = 0.04
_GC_W_VEL_THRESHOLD = 0.08
_GC_EE_FRAME = "end_link"
_GC_KP = 7.0
_GC_KD = 0.8


class HardwareManager:
    """Owns the single RobotArm instance used by the ROS driver."""

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
    ) -> None:
        self._sdk_root = self._ensure_rebot_sdk_in_syspath()

        from reBotArm_control_py.actuator import RebotArm
        from reBotArm_control_py.controllers import RebotArmEndPose
        from reBotArm_control_py.kinematics import load_robot_model
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        import pinocchio as pin

        cfg_path = self._resolve_hw_cfg(
            Path(arm_cfg).expanduser() if arm_cfg else self.default_arm_cfg(),
            channel,
        )
        self._arm = RebotArm(hw_yaml=str(cfg_path))
        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._gc_ee_frame_id = self._gc_model.getFrameId(_GC_EE_FRAME)
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        self._gc_pin = pin

        self._gripper_cfg_path = (
            Path(gripper_cfg).expanduser() if gripper_cfg else self.default_gripper_cfg()
        )
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        self._gripper_target_angle = 0.0
        self._gripper_target_effort = _G_DEFAULT_FORCE
        self._gripper_active = False
        self._gripper_pos = 0.0
        self._gripper_vel = 0.0
        self._gripper_torque = 0.0
        self._gripper_loop_stop = threading.Event()
        self._gripper_loop_thread: threading.Thread | None = None
        self._gripper_loop_running = False
        self._gripper_lock = threading.Lock()

        self._endpos_ctrl = RebotArmEndPose(self._arm, arm_control_mode="posvel")
        # Wrapper manages gripper via its own safety loop; stop the SDK
        # control-loop callback from also sending gripper MIT commands.
        self._endpos_ctrl._has_gripper = False
        self._connected = False
        self._enabled = False
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_target: np.ndarray | None = None
        self._gravity_comp_integral: np.ndarray | None = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last: np.ndarray | None = None

    def default_arm_cfg(self) -> Path:
        return self._sdk_root / "config" / "rebotarm_dm.yaml"

    def default_gripper_cfg(self) -> Path:
        return self.default_arm_cfg()

    @staticmethod
    def _workspace_root() -> Path:
        return Path(__file__).resolve().parents[3]

    @classmethod
    def _sdk_candidates(cls) -> list[Path]:
        workspace = cls._workspace_root()
        return [
            workspace / "third_party" / "reBotArm_control_py",
            workspace / "sdk" / "reBotArm_control_py",
            Path.home() / "reBotArm_control_py",
            Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    @classmethod
    def _ensure_rebot_sdk_in_syspath(cls) -> Path:
        for root in cls._sdk_candidates():
            if (root / "reBotArm_control_py").is_dir():
                root_str = str(root)
                if root_str not in sys.path:
                    sys.path.insert(0, root_str)
                return root
        candidates = "\n".join(f"  - {path}" for path in cls._sdk_candidates())
        raise FileNotFoundError(
            "Cannot find reBotArm_control_py. Clone it into one of:\n"
            f"{candidates}"
        )

    @staticmethod
    def _resolve_hw_cfg(cfg_path: Path, channel: str) -> Path:
        """Resolve hw config; fall back to SDK DM config for legacy
        (group-less) files, and optionally override the channel."""
        data = None
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except (FileNotFoundError, OSError):
            pass
        if not data or "groups" not in data:
            for root in HardwareManager._sdk_candidates():
                dm = root / "config" / "rebotarm_dm.yaml"
                if dm.exists():
                    cfg_path = dm
                    with open(cfg_path, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f)
                    break
        if not channel:
            return cfg_path
        data["channel"] = channel
        tmp_dir = Path("/tmp") / "rebotarm_ros2"
        tmp_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = tmp_dir / "hw_channel_override.yaml"
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return tmp_path

    @property
    def arm(self):
        return self._arm

    @property
    def endpos_ctrl(self):
        return self._endpos_ctrl

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm.arm.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm.arm.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._arm.has_gripper

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    def set_state_machine(self, state: str) -> None:
        if state not in ("IDLE", "TRAJ_RUNNING", "LOWLEVEL_STREAMING", "GRAVITY_COMP"):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state

    def connect(self) -> None:
        if self._connected:
            return
        self._arm.connect()
        self._patch_arm_bus_lock()
        self._arm.arm.mode_pos_vel()
        self._arm.arm.enable()
        self._enabled = True
        self._start_pos_vel_loop()
        self._connected = True
        self.init_gripper()

    def shutdown(self) -> None:
        if not self._connected:
            return
        try:
            self._stop_gripper_loop()
            gravity_comp_active = self._gravity_comp_active
            self.stop_gravity_compensation()
            if gravity_comp_active:
                self.ensure_pos_vel_control()
            if self._endpos_ctrl._running:
                self._endpos_ctrl.end()
            else:
                self._arm.disconnect()
        finally:
            self._connected = False
            self._enabled = False

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos, vel, torq = self._arm.get_state()
        n = self._arm.arm.num_joints
        return pos[:n], vel[:n], torq[:n]

    def hold_current_position(self) -> np.ndarray:
        q, _, _ = self.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._endpos_ctrl._q_target[:] = current
        return current

    def enable(self) -> None:
        from motorbridge import Mode

        self._arm.arm.enable()
        if self._gripper_mot is not None:
            self._gripper_mot.ensure_mode(Mode.POS_VEL, 1000)
        self._enabled = True
        if self.mode == "pos_vel" and not self.control_loop_active:
            self._start_pos_vel_loop()
        self.set_state_machine("IDLE")

    def disable(self) -> None:
        self.stop_gravity_compensation()
        self._stop_control_loop()
        self._arm.arm.disable()
        self._enabled = False
        self.set_state_machine("IDLE")

    def set_mode(self, mode: str) -> bool:
        mode = mode.strip().lower()
        if mode not in ("mit", "pos_vel", "vel"):
            raise ValueError(f"unsupported mode: {mode}")
        self.stop_gravity_compensation()

        if mode == self.mode:
            if mode == "pos_vel" and self._enabled:
                if self.control_loop_active:
                    self.hold_current_position()
                else:
                    self._start_pos_vel_loop()
            self.set_state_machine("IDLE")
            return True

        self._stop_control_loop()
        if mode == "mit":
            ok = self._arm.arm.mode_mit()
        elif mode == "pos_vel":
            ok = self._arm.arm.mode_pos_vel()
            if self._enabled:
                self._start_pos_vel_loop()
        else:
            ok = self._arm.arm.mode_vel()
        self.set_state_machine("IDLE")
        return bool(ok)

    def set_zero(self, joint_name: str = "") -> bool:
        self._stop_control_loop()
        if joint_name:
            mot = self._arm._motor_map.get(joint_name)
            if mot is None:
                raise KeyError(f"unknown joint: {joint_name}")
            try:
                mot.set_zero_position()
                ok = True
            except Exception:
                ok = False
        else:
            self._arm.set_zero()
            ok = True
        self._enabled = False
        self.set_state_machine("IDLE")
        return bool(ok)

    def ensure_pos_vel_control(self) -> None:
        if self.mode != "pos_vel":
            self._stop_control_loop()
            self._arm.arm.mode_pos_vel()
        if not self._enabled:
            self._arm.arm.enable()
            self._enabled = True
        if not self.control_loop_active:
            self._start_pos_vel_loop()
        else:
            self.hold_current_position()

    def safe_home(self) -> None:
        """Home the arm joints and the gripper to their zero positions.

        The SDK endpose controller only homes the arm joints (its gripper
        control is disabled here via ``_has_gripper = False``), so the
        gripper must be homed separately through this wrapper's own loop.
        """
        self._endpos_ctrl.safe_home()
        if self._gripper_mot is not None:
            self.set_gripper_position(0.0)
        self.set_state_machine("IDLE")

    def send_joint_motor_cmd(self, joint_name: str, cmd) -> None:
        if joint_name not in self._arm._motor_map:
            raise KeyError(f"unknown joint: {joint_name}")

        mot = self._arm._motor_map[joint_name]
        jc = next(j for j in self._arm._all_joints if j.name == joint_name)
        state = mot.get_state()

        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(jc.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(jc.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(jc.vlim)

        arm_joints = self._arm.arm.joint_names
        if joint_name in arm_joints:
            idx = arm_joints.index(joint_name)
            self._endpos_ctrl._q_target[idx] = pos
            if cmd.use_vlim:
                if self._endpos_ctrl._vlim_override is None:
                    self._endpos_ctrl._vlim_override = np.full(
                        self._endpos_ctrl._n, float(jc.vlim), dtype=np.float64
                    )
                self._endpos_ctrl._vlim_override[idx] = vlim

        if int(cmd.mode) == 0:
            mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(mot, "send_vel"):
                raise RuntimeError(f"{joint_name} does not support send_vel")
            mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        self.set_state_machine("LOWLEVEL_STREAMING")

    def start_gravity_compensation(self) -> None:
        if self._gravity_comp_active:
            return
        self.stop_gravity_compensation()
        if not self._enabled:
            self._arm.arm.enable()
            self._enabled = True
        self._stop_control_loop()
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False
        self._gravity_comp_q_target = self._arm.arm.get_positions(request_feedback=True).copy()
        self._gravity_comp_q_last = self._gravity_comp_q_target.copy()
        initial_tau_g = self._gravity_torque(self._gravity_comp_q_target)
        self._enter_gravity_compensation_mode(
            self._gravity_comp_q_target,
            initial_tau_g,
        )
        self._gravity_comp_integral = np.zeros_like(self._gravity_comp_q_target)
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_active = True
        self._gravity_comp_tick(self._arm, 1.0 / float(self._arm._rate))
        self._arm.start_control_loop(self._gravity_comp_tick, rate=self._arm._rate)
        self.set_state_machine("GRAVITY_COMP")

    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            return
        hold_target = (
            self._gravity_comp_q_last.copy()
            if self._gravity_comp_q_last is not None
            else None
        )
        self._arm.stop_control_loop()
        self._gravity_comp_active = False
        self._gravity_comp_q_target = None
        self._gravity_comp_integral = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last = None
        if self._enabled:
            self._arm.arm.mode_pos_vel()
            self._start_pos_vel_loop(target=hold_target)
        self.set_state_machine("IDLE")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    def gravity_compensation_target(self) -> np.ndarray | None:
        if self._gravity_comp_q_target is None:
            return None
        return self._gravity_comp_q_target.copy()

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        q = self._arm.arm.get_positions(request_feedback=request)
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_torque(self, q: np.ndarray) -> np.ndarray:
        from reBotArm_control_py.kinematics import pad_q_for_model

        q_padded = pad_q_for_model(self._gc_model, q, len(q))
        tau_g = self._gc_compute_generalized_gravity(
            self._gc_model,
            q_padded,
            self._gc_data,
        )
        return np.asarray(tau_g[: len(q)], dtype=np.float64)

    def _enter_gravity_compensation_mode(
        self,
        q_hold: np.ndarray,
        tau_g: np.ndarray,
    ) -> None:
        """Enter MIT one joint at a time while holding the measured pose.

        ``JointGroup.mode_mit`` switches every motor before the first MIT
        command is sent. During that gap a newly switched motor can briefly
        use its default zero target. Send a hold command immediately after
        switching each motor so the arm never sees that uncommanded window.
        """
        from motorbridge import Mode

        group = self._arm.arm
        kp = np.full(group.num_joints, _GC_KP, dtype=np.float64)
        kd = np.full(group.num_joints, _GC_KD, dtype=np.float64)
        for index, joint_name in enumerate(group.joint_names):
            motor = self._arm._motor_map[joint_name]
            motor.ensure_mode(Mode.MIT, 1000)
            motor.send_mit(
                float(q_hold[index]),
                0.0,
                float(kp[index]),
                float(kd[index]),
                float(tau_g[index]),
            )
        group._mode = "mit"
        group._mit_kp = kp
        group._mit_kd = kd

    def _gravity_comp_tick(self, arm, dt: float) -> None:
        del dt
        if not self._gravity_comp_active or self._gravity_comp_q_target is None:
            return

        q = self._read_gravity_comp_positions()
        qd = arm.arm.get_velocities()
        tau_g = self._gravity_torque(q)

        q_error = self._gravity_comp_q_target - q
        if self._gravity_comp_integral is None:
            self._gravity_comp_integral = np.zeros_like(q)
        self._gravity_comp_integral += q_error * 1.0
        np.clip(self._gravity_comp_integral, -0.5, 0.5, out=self._gravity_comp_integral)

        self._gc_pin.computeJointJacobians(self._gc_model, self._gc_data, q)
        self._gc_pin.updateFramePlacements(self._gc_model, self._gc_data)
        jacobian = self._gc_pin.getFrameJacobian(
            self._gc_model,
            self._gc_data,
            self._gc_ee_frame_id,
            self._gc_pin.ReferenceFrame.WORLD,
        )
        spatial_velocity = jacobian @ qd
        linear_speed = float(np.linalg.norm(spatial_velocity[:3]))
        angular_speed = float(np.linalg.norm(spatial_velocity[3:]))

        if linear_speed > _GC_VEL_THRESHOLD or angular_speed > _GC_W_VEL_THRESHOLD:
            self._gravity_comp_q_target = q.copy()
            self._gravity_comp_lock_counter = 0
            self._gravity_comp_integral *= 0.9
        else:
            self._gravity_comp_lock_counter += 1

        arm.arm.send_mit(
            pos=self._gravity_comp_q_target,
            vel=np.zeros(arm.arm.num_joints),
            kp=np.full(arm.arm.num_joints, _GC_KP),
            kd=np.full(arm.arm.num_joints, _GC_KD),
            tau=tau_g + self._gravity_comp_integral,
        )

    def current_pose(self):
        from reBotArm_control_py.kinematics import compute_fk, pad_q_for_model

        q, _, _ = self.get_joint_state()
        q = pad_q_for_model(self._endpos_ctrl._model, q, len(q))
        position, rotation, _ = compute_fk(self._endpos_ctrl._model, q)
        return fk_to_pose(position, rotation)

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._arm._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    def init_gripper(self, cfg_path: str = "") -> None:
        from motorbridge import CallError, Mode

        gripper_jc = next(
            (j for j in self._arm._all_joints if j.name == "gripper"), None
        )
        if gripper_jc is None:
            return
        self._gripper_cfg = gripper_jc
        self._gripper_mot = self._arm._motor_map.get("gripper")
        self._gripper_ctrl = self._arm._ctrl_map.get(gripper_jc.vendor)
        if self._gripper_mot is None or self._gripper_ctrl is None:
            return

        self._patch_controller_bus(self._gripper_ctrl)
        self._wrap_motor_bus(self._gripper_mot, self._gripper_ctrl._bus_lock)

        try:
            self._gripper_ctrl.enable_all()
            time.sleep(0.3)
        except CallError:
            pass
        self._gripper_mot.ensure_mode(Mode.POS_VEL, 1000)
        self._start_gripper_loop()

    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        distance = float(np.clip(position_m, 0.0, _G_MAX_DIST_M))
        target = max((distance / _G_MAX_DIST_M) * _G_ANGLE_OPEN, _G_OPEN_SOFT_LIMIT)
        effort = _G_DEFAULT_FORCE if max_effort <= 0.0 else float(max_effort)
        with self._gripper_lock:
            self._gripper_target_angle = target
            self._gripper_target_effort = float(np.clip(effort, 0.05, _G_TAU_MAX))
            self._gripper_active = True
        vlim = float(self._gripper_cfg.vlim) if self._gripper_cfg is not None else 3.0
        try:
            self._gripper_mot.send_pos_vel(target, vlim)
        except Exception:
            pass

    def wait_gripper_target(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._gripper_lock:
                target = self._gripper_target_angle
            if abs(self._gripper_pos - target) < _G_ARRIVE_TOL:
                return True
            time.sleep(0.02)
        return False

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target()
        return reached, self.gripper_position_m()

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        status = 0
        if self._gripper_mot is not None:
            try:
                st = self._gripper_mot.get_state()
                if st is not None:
                    status = int(st.status_code)
            except Exception:
                status = 0
        return self._gripper_pos, self._gripper_vel, self._gripper_torque, status

    def gripper_position_m(self) -> float:
        distance = (self._gripper_pos / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_velocity_m(self) -> float:
        return float(self._gripper_vel * (_G_MAX_DIST_M / _G_ANGLE_OPEN))

    def gripper_reached_target(self) -> bool:
        with self._gripper_lock:
            if not self._gripper_active:
                return True
            target = self._gripper_target_angle
        return abs(self._gripper_pos - target) < _G_ARRIVE_TOL

    def send_gripper_motor_cmd(self, cmd) -> None:
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        state = self._gripper_mot.get_state()
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        if int(cmd.mode) == 0:
            self._gripper_mot.send_mit(pos, vel, kp, kd, tau)
            with self._gripper_lock:
                self._gripper_active = False
        elif int(cmd.mode) == 1:
            self.set_gripper_target(pos, tau)
        elif int(cmd.mode) == 2:
            if not hasattr(self._gripper_mot, "send_vel"):
                raise RuntimeError("gripper does not support send_vel")
            self._gripper_mot.send_vel(vel)
            with self._gripper_lock:
                self._gripper_active = False
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")

    def _patch_arm_bus_lock(self) -> None:
        for ctrl in self._arm._ctrl_map.values():
            self._patch_controller_bus(ctrl)

        if not hasattr(self._arm, "_bus_lock_patched"):
            for jc in self._arm._all_joints:
                mot = self._arm._motor_map[jc.name]
                ctrl = self._arm._ctrl_map[jc.vendor]
                self._wrap_motor_bus(mot, ctrl._bus_lock)
            self._arm._bus_lock_patched = True

    @staticmethod
    def _patch_controller_bus(ctrl) -> None:
        if not hasattr(ctrl, "_bus_lock"):
            ctrl._bus_lock = threading.RLock()
        if hasattr(ctrl, "_bus_lock_patched"):
            return
        lock = ctrl._bus_lock

        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in ("poll_feedback_once", "enable_all", "disable_all"):
            if hasattr(ctrl, attr):
                wrapped = _wrap(getattr(ctrl, attr))
                wrapped._rebotarm_locked = True
                setattr(ctrl, attr, wrapped)
        ctrl._bus_lock_patched = True

    @staticmethod
    def _wrap_motor_bus(mot, lock) -> None:
        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in (
            "send_pos_vel",
            "send_mit",
            "send_vel",
            "request_feedback",
            "ensure_mode",
            "write_register_f32",
            "set_zero_position",
        ):
            if hasattr(mot, attr) and not hasattr(getattr(mot, attr), "_rebotarm_locked"):
                wrapped = _wrap(getattr(mot, attr))
                wrapped._rebotarm_locked = True
                setattr(mot, attr, wrapped)

    def _start_pos_vel_loop(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.array(target, dtype=np.float64)
        self._arm.start_control_loop(self._endpos_ctrl._loop_cb)
        self._endpos_ctrl._running = True

    def _stop_control_loop(self) -> None:
        self._arm.stop_control_loop()
        self._endpos_ctrl._running = False

    def _gripper_tick(self) -> None:
        if self._gripper_mot is None or self._gripper_ctrl is None:
            return
        try:
            self._gripper_mot.request_feedback()
            self._gripper_ctrl.poll_feedback_once()
        except Exception:
            pass
        try:
            st = self._gripper_mot.get_state()
            if st is not None:
                self._gripper_pos = float(st.pos)
                self._gripper_vel = float(st.vel)
                self._gripper_torque = float(st.torq)
        except Exception:
            pass

    def _gripper_loop(self) -> None:
        dt = 1.0 / _G_CTRL_RATE
        last = time.perf_counter()
        while not self._gripper_loop_stop.is_set():
            now = time.perf_counter()
            if now - last >= dt:
                last += dt
                self._gripper_tick()
            else:
                time.sleep(1e-4)

    def _start_gripper_loop(self) -> None:
        if self._gripper_loop_running:
            return
        self._gripper_loop_stop.clear()
        self._gripper_loop_thread = threading.Thread(
            target=self._gripper_loop,
            name="rebotarm-gripper-loop",
            daemon=True,
        )
        self._gripper_loop_thread.start()
        self._gripper_loop_running = True

    def _stop_gripper_loop(self) -> None:
        if not self._gripper_loop_running:
            return
        self._gripper_loop_stop.set()
        if self._gripper_loop_thread is not None:
            self._gripper_loop_thread.join(timeout=1.0)
            self._gripper_loop_thread = None
        self._gripper_loop_running = False
