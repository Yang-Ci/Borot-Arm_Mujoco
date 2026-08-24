# reBot Arm MuJoCo DM

> 中文文档：[README_zh.md](./README_zh.md)

A ROS 2 + MuJoCo simulation and web control project for the reBot Arm B601-DM (Damiao motor edition).

This repository covers the full software stack: arm ROS 2 interface, real-robot driver, Fake Driver, MuJoCo dynamics and grasping, real2sim sync, torque comparison, virtual camera, color detection, Three.js web console, and LLM/MCP control.

## Hardware Requirements

- **Arm**: reBot Arm B601-DM
- **Motors**: Damiao 4340P (joint1-3) + 4310 (joint4-6 + gripper)
- **Communication**: USB-CAN (`/dev/ttyACM0`)
- **Host**: Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12, or Ubuntu 22.04 + ROS 2 Humble + Python 3.10

## Software Prerequisites

| Component | Version | Install |
| --- | --- | --- |
| Ubuntu | 24.04 or 22.04 | System |
| ROS 2 | Jazzy or Humble | `apt install ros-${ROS_DISTRO}-desktop` |
| Python | 3.12 or 3.10 | System |
| Node.js | ≥ 18 | `apt install nodejs` |
| MuJoCo | 3.10+ | `pip install mujoco` |
| Pinocchio | 4.1+ | `pip install pin` |
| MotorBridge | 0.5+ | `pip install motorbridge` |

## Dependencies

### In-Workspace (bundled)

| Contents | Location | Description |
| --- | --- | --- |
| ROS 2 packages (7) | `reBotArm_ros2_DM/src/` | msgs, controller, bringup, mujoco, agent |
| URDF + STL meshes | `rebotarm_bringup/description/` | Arm model files |
| MuJoCo models | `rebotarm_mujoco/models/` | STL / kinematics XML |
| Web frontend | `reBotArm_simulator-DM/` | Three.js + HTML/CSS/JS |
| Launch scripts | `reBotArm_ros2_DM/scripts/` | One-click launch |

### External Dependencies (installed separately)

| Dependency | Source | Purpose | Install |
| --- | --- | --- | --- |
| ROS 2 (Jazzy/Humble) | System | rclpy, message types, rosbridge | `apt install ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-rosbridge-suite` |
| reBotArm_control_py SDK | GitHub | RebotArm, IK, dynamics, gravity compensation | See install steps below |
| motorbridge | pip | Damiao motor CAN communication | `pip install motorbridge` |
| pinocchio (pin) | pip (cmeel) | Rigid-body dynamics model | `pip install pin` |
| mujoco | pip | Physics simulation engine | `pip install mujoco` |
| numpy | pip | Numerical computing | `pip install numpy` |
| pyyaml | pip | YAML config parsing | `pip install pyyaml` |
| transforms3d | pip | Coordinate transforms | `pip install transforms3d` |
| tf_transformations | ROS .deb | Quaternion/Euler conversion | See install steps below |
| Node.js | System | Web server | `apt install nodejs` |

## Installation

### 1. System Prerequisites

```bash
# ROS 2 (Jazzy or Humble, if not installed)
sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-rosbridge-suite

# Node.js (web console)
sudo apt install -y nodejs

# Verify
ros2 --version    # Jazzy or Humble
node --version    # >= 18
```

### 2. Install reBotArm_control_py SDK

The SDK provides real-robot driver, inverse kinematics, dynamics, and gravity compensation. It is a core external dependency.

```bash
cd ~
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
pip install -e .    # Editable install, or use sys.path directly
```

Directory structure after install:
```text
~/reBotArm_control_py/
├─ reBotArm_control_py/
│  ├─ actuator/          RebotArm class, JointGroup, motor control
│  ├─ controllers/       RebotArmEndPose (trajectory, IK, gravity comp)
│  ├─ kinematics/        FK/IK, load_robot_model, pad_q_for_model
│  └─ dynamics/          compute_generalized_gravity and other dynamics functions
├─ config/
│  └─ rebotarm_dm.yaml   DM motor config (IDs, baud rate, limits, PID)
├─ urdf/                 Pinocchio dynamics model URDF
└─ pyproject.toml
```

> **Note**: The SDK's `pyproject.toml` declares `requires-python >=3.10,<3.12`, but this project imports it via `sys.path` rather than pip install, so it works fine on Python 3.12. If pip install reports a version conflict, skip `pip install -e .` and just ensure the directory is at `~/reBotArm_control_py/` (the code auto-searches this path).

### 3. Run the Install Script

`setup.sh` automates venv creation (`--system-site-packages`), Python dependency installation, `tf_transformations` extraction, import verification, and `colcon build`:

```bash
cd ~/reBot_Arm_Mujoco-DM
./setup.sh
```

> Check only, no install: `./setup.sh --check`

## Project Structure

```text
.
├─ PROJECT_ARCHITECTURE_ZH.md       Architecture, simulation principles, anti-jitter notes
├─ setup.sh                         Idempotent one-click install and version check
├─ rebotarm                         Unified start/stop/status/diagnostic entry
├─ requirements.txt                 Python dependency version ranges
├─ reBotArm_ros2_DM/    ROS 2 workspace
│  ├─ scripts/                      One-click launch scripts
│  ├─ third_party/                  reBotArm_control_py SDK for fresh installs
│  ├─ .venv/                        Project Python venv (created by setup.sh)
│  └─ src/
│     ├─ rebotarm_msgs/             Custom msg/srv/action
│     ├─ rebotarmcontroller/        Real-robot driver, Fake Driver, hardware manager
│     ├─ rebotarm_bringup/          URDF, STL, launch, motor config
│     ├─ rebotarm_mujoco/           MuJoCo sim, IK, camera, vision
│     ├─ rebotarm_moveit_config/   MoveIt motion planning config
│     ├─ rebotarm_moveit_demos/    MoveIt demos (draw square, pick & place)
│     └─ rebotarm_agent/            MCP Server and text Agent
└─ reBotArm_simulator-DM/           Node.js + Three.js web console
   ├─ public/                       Pages, styles, frontend logic
   └─ split_meshes/grouped_gripper/ Web gripper meshes
```

## Simulation Modes

| Mode | Dynamics Step | Purpose |
| --- | --- | --- |
| Three.js web simulator | None | Browser display, interaction, teaching, ROS control panel |
| MuJoCo real2sim | `mj_forward` | Real-robot joint angles mapped to MuJoCo model in real time |
| MuJoCo physics grasp | `mj_step` | Gravity, inertia, collision, friction, physics-based grasping |
| MuJoCo torque control | `mj_step` | `tau_g + PD` closed-loop, gravity model comparison |

## Quick Start

### One-Click Install After Clone (Recommended)

```bash
git clone https://github.com/Seeed-Projects/Borot-Arm_Mujoco.git
cd reBot_Arm_Mujoco-DM
./setup.sh
./rebotarm doctor
```

The installer is idempotent: existing components are preserved and the web `.env` is never overwritten. A different SDK revision produces a warning but does not block setup; if the venv uses an incompatible Python version, the old directory is backed up before it is rebuilt. A summary of installed, skipped, version-mismatched, and failed items is printed at the end. Check-only mode (no system modifications):

```bash
./setup.sh --check
```

Unified launch entry:

```bash
# Verify device node and set permissions before launch
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM*
```
```bash
./rebotarm start web   # rosbridge + web
./rebotarm start dm    # DM real robot (separate terminal)
./rebotarm start sim   # MuJoCo simulation; do not launch alongside DM real robot
./rebotarm status
```

Source the environment before all commands (auto-sourced by the unified entry):

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArm_ros2_DM
source scripts/source_rebotarm_env.sh
```

### 1. Fake Driver (Pure Simulation, No Real Robot)

```bash
ros2 launch rebotarm_bringup fake_bringup.launch.py
```

Verify: `ros2 topic echo /rebotarm/joint_states --once` should return non-zero angles.

### 2. Real Robot Control

```bash
# Verify device node and set permissions
ls /dev/ttyACM0    # default is ACM0
sudo chmod 666 /dev/ttyACM0

# Launch real-robot driver
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

Verify:
```bash
ros2 topic echo /rebotarm/joint_states --once   # Should show real joint angles
ros2 service call /rebotarm/enable std_srvs/srv/Trigger   # Enable
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger   # Safe home
```

### 3. Web Console

**Terminal 2 — rosbridge:**

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArm_ros2_DM
source scripts/source_rebotarm_env.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

**Terminal 3 — Web server:**

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArm_simulator-DM
cp .env.example .env   # First use: copy env template
# Edit .env, change localhost to your VM IP (e.g. ws://192.168.x.x:9090)
node server.js
```

**Browser:**

1. Open `http://localhost:3001`
2. Enter the rosbridge address in the WebSocket input (e.g. `ws://localhost:9090`); the address is saved automatically
3. Click "Connect ROS"
4. The 3D model should display the real robot's current pose

A "中/EN" toggle at the top of the panel switches between Chinese and English. All panels, sub-menus, tooltips, and status messages update instantly without page reload.

**Controlling the real robot (two-step unlock):**

1. Check "Allow web to send control to real robot" (a confirmation dialog appears when connecting to the real controller)
2. Click the "Enable" button

After both steps, drag the joint sliders or click the gripper button to control the real robot.

### 4. Real2Sim Sync

With the real-robot controller running, open a new terminal:

```bash
source scripts/source_rebotarm_env.sh
ros2 launch rebotarm_mujoco real2sim.launch.py
```

Expected: MuJoCo viewer window opens; the 3D model follows real-robot motion in real time (including gripper).

### 5. Torque Control Comparison

With the real-robot controller running, open a new terminal:

```bash
source scripts/source_rebotarm_env.sh
scripts/start_mujoco_torque_control.sh
```

Expected: MuJoCo viewer opens; the console prints torque comparison every 0.5 s:

```text
tau_g compare: max_abs=0.5080 Nm at joint3, mujoco=-7.4782, sdk=-6.9702, rms=0.2478
```

- `mujoco`: gravity torque from the MuJoCo physics engine
- `sdk`: gravity torque computed by Pinocchio
- `diff`: difference between the two
- RMS < 0.3 Nm indicates good dynamics model consistency

### 6. Full MuJoCo Simulation Stack

```bash
source scripts/source_rebotarm_env.sh
scripts/start_rebot_mujoco_all.sh
```

One-click launch: Fake Driver + MuJoCo physics grasping + task server + RGB camera + color detection + rosbridge.

## ROS 2 Interface

### Topics

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `/rebotarm/joint_states` | `sensor_msgs/JointState` | Publish | 6 joints + finger_left position/velocity/effort |
| `/rebotarm/arm_status` | `rebotarm_msgs/ArmStatus` | Publish | Mode, enable state, state machine, error codes |
| `/rebotarm/gripper/state` | `rebotarm_msgs/JointMotorState` | Publish | Gripper position (m)/velocity/effort |
| `/rebotarm/joints/jointN/cmd` | `rebotarm_msgs/JointMotorCmd` | Subscribe | Single-joint position/velocity/MIT command |
| `/rebotarm/gripper/cmd` | `rebotarm_msgs/JointMotorCmd` | Subscribe | Gripper command (pos in meters) |

### Services

| Service | Type | Description |
| --- | --- | --- |
| `/rebotarm/enable` | `std_srvs/Trigger` | Enable the arm |
| `/rebotarm/disable` | `std_srvs/Trigger` | Disable the arm |
| `/rebotarm/safe_home` | `std_srvs/Trigger` | Safe return to home |
| `/rebotarm/gripper/set` | `rebotarm_msgs/SetGripper` | Set gripper (position in meters, max_effort) |
| `/rebotarm/gravity_compensation/start` | `std_srvs/Trigger` | Start gravity compensation |
| `/rebotarm/gravity_compensation/stop` | `std_srvs/Trigger` | Stop gravity compensation |
| `/rebotarm/gravity_compensation/status` | `std_srvs/Trigger` | Query gravity compensation status |
| `/rebotarm/set_mode` | `rebotarm_msgs/SetMode` | Switch control mode |
| `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/MoveToPoseIK` | IK move to target pose |

## Configuration

### Motor Configuration

SDK config file: `~/reBotArm_control_py/config/rebotarm_dm.yaml`

Contains motor IDs, baud rates, joint limits, PID parameters, etc. Restart the controller after modifying.

#### Motor Overview

| Joint | Motor ID | Feedback ID | Model | Purpose |
|-------|---------|-------------|-------|---------|
| joint1 | 0x01 | 0x11 | 4340P | Base rotation |
| joint2 | 0x02 | 0x12 | 4340P | Shoulder pitch |
| joint3 | 0x03 | 0x13 | 4340P | Elbow pitch |
| joint4 | 0x04 | 0x14 | 4310  | Wrist roll |
| joint5 | 0x05 | 0x15 | 4310  | Wrist pitch |
| joint6 | 0x06 | 0x16 | 4310  | Wrist roll |
| gripper | 0x07 | 0x17 | 4310  | Gripper |

#### Control Architecture: Arm Joints and Gripper Use POS_VEL

All 7 motors (joint1–joint6 + gripper) use **POS_VEL mode**, closed-loop by the motor firmware's internal PID controller. The host PC does no external PD computation. This avoids oscillation from double PD stacking.

**Data flow:**

```
Host PC (ROS 2 / Web)
  │
  ├─ Arm joint1-6:   SDK control loop 500 Hz → arm.send_pos_vel(q_target, vlim)
  │                   → motor firmware internal PID (pos_kp/pos_ki + vel_kp/vel_ki) closed-loop
  │
  └─ Gripper:         set_gripper_target() → gripper.send_pos_vel(target, vlim)
                       → motor firmware internal PID (pos_kp/pos_ki + vel_kp/vel_ki) closed-loop
```

**Key design:**

- Arm joints are driven by the SDK's `RebotArmEndPose` control loop (`_loop_cb`) at 500 Hz calling `arm.send_pos_vel(q_target, vlim)`; target positions are written to the `_q_target` array
- The gripper is driven by `HardwareManager.set_gripper_target()` calling `send_pos_vel(target, vlim)` directly — no control loop needed; in POS_VEL mode the motor holds the target position automatically
- The gripper feedback polling thread calls `request_feedback()` + `poll_feedback_once()` at 50 Hz, reading state only (position/velocity/effort) without sending control commands
- Single-joint commands (`/rebotarm/joints/jointN/cmd`) also use `send_pos_vel` in mode=1, while synchronizing `_q_target[idx]` to prevent the control loop from overwriting

#### POS_VEL PID Parameters

| Joint | pos_kp | pos_ki | vel_kp | vel_ki | vlim (rad/s) |
|-------|--------|--------|--------|--------|--------------|
| joint1–3 (4340P) | 150.0 | 0.5 | 0.0125 | 0.004 | 5.0 |
| joint4–6 (4310)  | 50.0  | 1.0 | 0.0008 | 0.002 | 3.0 |
| gripper (4310)   | 50.0  | 1.0 | 0.0008 | 0.002 | 3.0 |

These parameters are stored in motor firmware registers and written by the SDK during `ensure_mode(Mode.POS_VEL)`. To modify PID, edit the `POS_VEL` section for the corresponding joint in `rebotarm_dm.yaml` and restart the controller.

#### Gripper Unit Conversion

The web and ROS interfaces use **meters** (0.0 = fully closed, 0.09 = fully open); the motor firmware uses **radians** (0.0 = closed, −5.0 = open). Conversion is done in `HardwareManager`:

```
radians = (distance_m / 0.09) × (−5.0)
distance_m = (radians / −5.0) × 0.09
```

### Web rosbridge Address

The rosbridge WebSocket address is entered manually by the user in the web "ROS2 Bridge" panel; it is not hardcoded by default. `rebot-ros-ui.js` reads the last saved address from `localStorage`. On first connection, enter the actual address, e.g. `ws://<Ubuntu IP>:9090`.

## Environment

`scripts/source_rebotarm_env.sh` loads in order:

1. ROS 2 (`/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash`)
2. Python venv (`.venv/bin/activate`)
3. cmeel.prefix paths (Pinocchio C extensions and shared libraries)
4. Workspace (`install/setup.bash`)

The venv must have `include-system-site-packages = true` (`.venv/pyvenv.cfg`), otherwise rosbridge's `tornado`, `psutil`, `argcomplete`, `bson` and other system packages are invisible.

## Troubleshooting

### `ModuleNotFoundError: No module named 'tornado'/'psutil'/'argcomplete'/'bson'`

Venv does not include system site-packages:
```bash
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' .venv/pyvenv.cfg
```

### `write_register_f32 failed: register 25 write ack not received within 50ms`

Motor bus communication timeout. Check:
- USB-CAN cable connection
- `/dev/ttyACM0` existence
- Motor power on
- No other process occupying the bus

### Cannot control real robot from web

Confirm the two-step unlock:
1. Connect ROS in the "ROS2 Bridge" panel (WebSocket to the real controller's rosbridge)
2. Check "Allow control"
3. Click the "Enable" button

## Known Fixes (DM Adaptation)

| Issue | Root Cause | Fix |
| --- | --- | --- |
| SDK API incompatibility | Repo uses old API (`RobotArm`), DM SDK uses new API (`RebotArm`) | Rewrote `hardware_manager.py` in 21 places |
| rosbridge missing deps | Venv isolated system packages | Enabled `include-system-site-packages` |
| Joint slider cannot control real robot | pos_vel loop overwrites single-joint commands | |
| Gravity compensation stop exception | `self._arm.mode_pos_vel()` should be `self._arm.arm.mode_pos_vel()` | Fixed call path |

## Documentation
- [Project architecture, MuJoCo & web notes](./PROJECT_ARCHITECTURE_ZH.md)
- [DM real-robot data link and flow](./DATA_FLOW_ZH.md)
- [B601-DM user manual](./USER_MANUAL_ZH.md)
- [ROS 2 workspace readme](./reBotArm_ros2_DM/README_zh.md)
- [MuJoCo package readme](./reBotArm_ros2_DM/src/rebotarm_mujoco/README.md)
- [Web console readme](./reBotArm_simulator-DM/README.md)
- [AI Agent/MCP readme](./reBotArm_ros2_DM/src/rebotarm_agent/README.md)

## License

Software code follows the Apache-2.0 license as indicated in the repository. Model and asset usage terms should also reference the original reBot-DevArm project.
