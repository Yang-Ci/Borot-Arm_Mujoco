# reBotArm ROS2 SDK

<p align="center">
  <strong>ROS2 Jazzy · 机械臂控制 · 夹爪控制 · JointTrajectory · 自定义单电机调试接口</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue.svg" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Python-3.12-blue.svg" alt="Python 3.12">
  <img src="https://img.shields.io/badge/Version-v0.0.5-brightgreen.svg" alt="Version v0.0.5">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2024.04+-orange.svg" alt="Ubuntu 24.04+">
  <img src="https://img.shields.io/badge/Controller-reBotArmController-green.svg" alt="reBotArmController">
</p>

---

## 项目介绍

当前版本：`v0.0.5`

`rebotarm_ros2` 是 reBotArm B601 机械臂的 ROS2 SDK 工作空间。它将现有的
`reBotArm_control_py` Python 控制库封装为 ROS2 topic、service 和 action，
作为二次开发、上层规划、可视化和单电机调试的统一入口。

当前工作空间包含四个 ROS2 包：

| 包 | 作用 |
|---|---|
| `rebotarm_msgs` | 自定义 msg / srv / action 接口 |
| `rebotarmcontroller` | 控制节点包，提供 `reBotArmController` 节点 |
| `rebotarm_bringup` | launch、配置、URDF、RViz 等启动资源 |
| `rebotarm_mujoco` | MuJoCo real2sim、tau_g 对比和纯 MuJoCo 力矩闭环工具 |

---

## 核心功能

- 发布机械臂状态：`/rebotarm/joint_states`、`/rebotarm/arm_status`
- 提供基础服务：`enable`、`disable`、`set_mode`、`set_zero`、`safe_home`
- 支持笛卡尔目标：`MoveToPoseIK` service、`MoveToPose` action
- 支持标准轨迹接口：`control_msgs/action/FollowJointTrajectory`
- 支持夹爪控制：`SetGripper` service、`GripperCommand` action
- 支持 controller 内部重力补偿：`gravity_compensation/start`、`gravity_compensation/stop`、`gravity_compensation/status`
- 支持 per-joint sparse raw command：`JointMotorCmd`
- 复用 `reBotArm_control_py` 的 `RobotArm`、`ArmEndPos`、FK/IK、动力学和夹爪配置加载

---

## 环境要求

| 项 | 要求 |
|---|---|
| 操作系统 | Ubuntu 24.04+ |
| ROS2 | Jazzy，安装路径 `/opt/ros/jazzy` |
| Python | 系统 Python 3.12 |
| Python 环境 | 不使用 conda |
| 底层 SDK | 推荐 `~/seeed/rebotarm_ros2/third_party/reBotArm_control_py/` |
| 默认串口 | `/dev/ttyACM0`，可通过 launch 参数覆盖 |

每次构建或运行前先 source ROS2：

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 配置开发环境

### Step 1. 安装 ROS2 依赖

请参考[ROS官方下载文档](https://www.ros.org/blog/getting-started/)选择适合的版本进行安装。

### Step 2. 安装 motorbridge

`motorbridge` 从 PyPI 官方源安装：

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

### Step 3. 获取底层 SDK


```bash
cd ~/seeed/rebotarm_ros2
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

### Step 4. 确认底层 SDK 可导入

如果还没有把 SDK 安装到系统 Python，可临时设置 `PYTHONPATH` 验证：

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=$PWD/third_party/reBotArm_control_py:$PYTHONPATH
python3 -c "import rclpy, motorbridge, reBotArm_control_py; print('core imports OK')"
python3 -c "import pinocchio; print('pinocchio', pinocchio.__version__)"
python3 -c "from reBotArm_control_py.actuator import RobotArm; from reBotArm_control_py.controllers import ArmEndPos; from reBotArm_control_py.kinematics import compute_fk; from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik; from reBotArm_control_py.actuator.gripper import load_cfg; print('SDK required APIs OK')"
```

## 构建工作空间

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

验证包和入口：

```bash
ros2 pkg executables rebotarmcontroller
```

期望输出：

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## 目录结构

```text
rebotarm_ros2/
├── README_zh.md
├── PLAN.md
├── instruction.md
└── src/
    ├── rebotarm_msgs/
    │   ├── msg/
    │   ├── srv/
    │   └── action/
    ├── rebotarmcontroller/
    │   ├── rebotarmcontroller/
    │   │   ├── rebotarm_controller.py
    │   │   ├── hardware_manager.py
    │   │   ├── ros_publishers.py
    │   │   ├── ros_services.py
    │   │   ├── ros_actions.py
    │   │   ├── motor_passthrough.py
    │   │   ├── conversions.py
    │   │   └── examples/
    └── rebotarm_bringup/
        ├── launch/
        ├── config/
        ├── description/
        └── rviz/
```

---

## 快速启动

### 启动完整系统

启动控制节点、`robot_state_publisher`，可选 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

如果机械臂不在默认串口 `/dev/ttyACM0`：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

启用 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### 只启动控制节点

```bash
ros2 launch rebotarm_bringup driver_only.launch.py
```

### 直接运行控制节点

```bash
ros2 run rebotarmcontroller reBotArmController
```

---

## MuJoCo 工具

常用启动入口在 `scripts/` 顶层：

```bash
./scripts/start_fake_bringup.sh
./scripts/start_joint_slider_gui.sh
REAL2SIM_MODEL=stl ./scripts/start_real2sim.sh
./scripts/start_mujoco_torque_control.sh
```

中文滑块 GUI 发布 `/rebotarm/joint_states`，并带有重力补偿 start / stop /
status 按钮：

```bash
./scripts/start_joint_slider_gui.sh
```

如果要让重力补偿按钮有响应，先在另一个终端启动 fake driver 或真实 controller：

```bash
./scripts/start_fake_bringup.sh
```

MuJoCo `tau_g` 对比 SDK `tau_g`，以及纯 MuJoCo `tau_g + PD` 力矩闭环仍可单独调试：

```bash
./scripts/start_mujoco_torque_control.sh
```

该脚本默认使用 STL 模型，并把 `/rebotarm/joint_states` 作为 MuJoCo 力矩闭环目标，
所以可以跟随网页/fake driver 发布的关节状态。若要完全独立测试纯 MuJoCo 目标话题，
用：

```bash
MUJOCO_TORQUE_TARGET_TOPIC=/rebotarm/mujoco/target_joint_states ./scripts/start_mujoco_torque_control.sh
```

常用观察话题：

```bash
ros2 topic echo /rebotarm/mujoco/tau_g --once
ros2 topic echo /rebotarm/mujoco/sdk_tau_g --once
ros2 topic echo /rebotarm/mujoco/tau_g_diff --once
ros2 topic echo /rebotarm/mujoco/joint_states --once
```

给纯 MuJoCo 闭环下发一个目标姿态，不会控制真实硬件：

```bash
ros2 topic pub --once /rebotarm/mujoco/target_joint_states sensor_msgs/msg/JointState \
"{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.0, -0.7, -0.8, 0.0, 0.5, 0.0]}"
```

---

## 直接移动到 Pose

不运行 demo 时，可以直接调用 ROS service 和 action 完成一次末端位姿移动。
先在一个终端启动控制节点：

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

然后在另一个终端执行控制命令：

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

1. 使能机械臂：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

2. 移动末端到目标 pose：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}" \
  --feedback
```

`move_to_pose` action 内部会确保进入 `pos_vel` 控制并启动轨迹控制循环。

3. 回到安全零位：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

4. 失能并退出：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

---

## 示例脚本

所有示例都假设已经启动 `reBotArmController`：

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

示例已注册为 ROS2 可执行入口，可以直接通过 `ros2 run` 调用。

源文件位于：

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

关节空间绝对角移动示例。脚本会等待 `/rebotarm/joint_states`，然后通过
`/rebotarm/follow_joint_trajectory` 发送关节轨迹；控制节点会在 action 内部进入
`pos_vel`，并使用限速和最终到位检查。

这是单次动作 demo，不会在结束后自动 `safe_home` 或 `disable`。

一次性控制 6 个电机，参数为 6 个绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

一次性控制 1 个电机，参数为目标关节名和绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

这个入口只作为 joint trajectory 链路验证，不建议用它做大范围规划。轨迹仍需满足
`follow_joint_trajectory` 的首点、速度和最终到位校验。

### move_to_pose.py

末端位姿移动示例。脚本会通过 `/rebotarm/move_to_pose` action 发送
`geometry_msgs/Pose` 目标；控制节点内部负责模式切换和轨迹执行。

这是单次动作 demo，不会在结束后自动 `safe_home` 或 `disable`。

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

这个示例对应底层 SDK 的 `ArmEndPos.move_to_traj(...)` 能力链路，也就是
`third_party/reBotArm_control_py/example/8_arm_traj_control.py` 的 ROS action 版本。

### gravity_compensation.py

重力补偿锁止示例。脚本本身不再在外部节点重写控制循环，而是直接调用
`reBotArmController` 内部的重力补偿服务；真正的重力补偿闭环在 controller
进程内部直接使用 SDK 的 `RobotArm.get_positions/get_velocities/mit` 完成。
内部启动流程参考新版 SDK 示例：切入 MIT 后先执行 `fresh()` 空指令，再进入重力补偿闭环，
并对多圈角度反馈做就近连续化处理，避免启动瞬态弹跳。

```bash
ros2 run rebotarmcontroller GravityCompensation
```

脚本启动时会先调用 `/rebotarm/enable`，再启动重力补偿。按 `Ctrl+C` 退出时，
脚本会依次调用 `/rebotarm/safe_home` 和 `/rebotarm/disable`，让机械臂回到安全零位后失能。

对应底层服务：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/status std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

该示例只是这些服务的薄客户端，不会绕过 ROS 节点直接连接硬件。硬件访问仍由
`reBotArmController` 统一持有。

### gripper_control.py

交互式夹爪开闭示例。脚本只调用 `/rebotarm/gripper/set` service，不直接访问硬件。
脚本启动时会先调用 `/rebotarm/enable`，退出时先闭合夹爪，再调用 `/rebotarm/disable`。

```bash
ros2 run rebotarmcontroller GripperControl
```

运行后输入：

```text
o / open    打开夹爪
c / close   闭合夹爪
q / quit    退出
```

`open` 对应夹爪开口 `0.09 m`，`close` 对应 `0.0 m`。

---

## ROS API 速查

默认命名空间为 `/rebotarm`。如果 launch 时设置了 `arm_namespace:=xxx`，下面所有
`/rebotarm/...` 都替换为 `/xxx/...`。

### 状态 Topic

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/joint_states` | `sensor_msgs/msg/JointState` | 6 轴关节位置、速度、力矩 | `ros2 topic echo /rebotarm/joint_states --once` |
| `/rebotarm/arm_status` | `rebotarm_msgs/msg/ArmStatus` | 控制模式、使能状态、状态机、关节状态码 | `ros2 topic echo /rebotarm/arm_status --once` |
| `/rebotarm/joints/<joint>/state` | `rebotarm_msgs/msg/JointMotorState` | 单关节电机状态，`<joint>` 为 `joint1` 到 `joint6` | `ros2 topic echo /rebotarm/joints/joint1/state --once` |
| `/rebotarm/gripper/state` | `rebotarm_msgs/msg/JointMotorState` | 夹爪状态，未配置夹爪时不发布 | `ros2 topic echo /rebotarm/gripper/state --once` |

### 服务 Service

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/enable` | `std_srvs/srv/Trigger` | 使能机械臂和夹爪 | `ros2 service call /rebotarm/enable std_srvs/srv/Trigger` |
| `/rebotarm/disable` | `std_srvs/srv/Trigger` | 停止控制循环并失能机械臂 | `ros2 service call /rebotarm/disable std_srvs/srv/Trigger` |
| `/rebotarm/safe_home` | `std_srvs/srv/Trigger` | 以安全速度回零 | `ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger` |
| `/rebotarm/gravity_compensation/start` | `std_srvs/srv/Trigger` | 启动 controller 内部重力补偿闭环 | `ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger` |
| `/rebotarm/gravity_compensation/stop` | `std_srvs/srv/Trigger` | 停止 controller 内部重力补偿闭环 | `ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger` |
| `/rebotarm/gravity_compensation/status` | `std_srvs/srv/Trigger` | 查询 controller 内部重力补偿状态，`success=true` 表示正在运行 | `ros2 service call /rebotarm/gravity_compensation/status std_srvs/srv/Trigger` |
| `/rebotarm/set_mode` | `rebotarm_msgs/srv/SetMode` | 切换 `mit`、`pos_vel`、`vel` | `ros2 service call /rebotarm/set_mode rebotarm_msgs/srv/SetMode "{mode: 'pos_vel'}"` |
| `/rebotarm/set_zero` | `rebotarm_msgs/srv/SetZero` | 设置全部或指定关节零点，空 `joint_name` 表示全部 | `ros2 service call /rebotarm/set_zero rebotarm_msgs/srv/SetZero "{joint_name: ''}"` |
| `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/srv/MoveToPoseIK` | 只做 IK 求解并更新目标关节角，适合小步位姿调整 | 见下方预留 API |
| `/rebotarm/gripper/set` | `rebotarm_msgs/srv/SetGripper` | 设置夹爪开合距离和最大力矩 | `ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.05, max_effort: 0.5}"` |

### 动作 Action

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/move_to_pose` | `rebotarm_msgs/action/MoveToPose` | 末端笛卡尔位姿轨迹，内部走 `ArmEndPos.move_to_traj(...)` | 适合应用层发 `Pose + duration` |
| `/rebotarm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 标准关节轨迹接口，按输入轨迹点定时下发，并做首点/速度/到位校验 | 适合上层规划器或 `move_to.py` |
| `/rebotarm/gripper/command` | `control_msgs/action/GripperCommand` | 标准夹爪 action | 适合行为树、任务编排或 MoveIt 风格接口 |

示例：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}, duration: 2.0}" \
  --feedback
```

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 2}},
             {positions: [0.1,0,0,0,0,0], time_from_start: {sec: 5}}]}}"
```

`follow_joint_trajectory` 要求首个轨迹点接近当前关节角，最终误差默认需小于
`0.03 rad`。底层 `pos_vel` 速度限制使用 `arm.yaml` 中的电机配置。

### 低层 Command Topic

| API | 类型 | 说明 | 简要使用 |
|---|---|---|---|
| `/rebotarm/joints/<joint>/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 单关节 sparse raw command，可发 MIT、位置速度、速度模式 | 调试、电机直通、重力补偿 |
| `/rebotarm/gripper/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 夹爪 sparse raw command | 低层夹爪调试 |

`JointMotorCmd` 使用 sparse-flag 设计：只有 `use_pos/use_vel/use_kp/use_kd/use_tau/use_vlim`
为 `true` 的字段才覆盖默认值。`mode` 可选：

| mode | 常量 | 说明 |
|---|---|---|
| `0` | `MODE_MIT` | MIT 模式，使用 `pos/vel/kp/kd/tau` |
| `1` | `MODE_POS_VEL` | 位置速度模式，使用 `pos/vlim` |
| `2` | `MODE_VEL` | 速度模式，使用 `vel` |

单关节 MIT 示例：

```bash
ros2 topic pub --once /rebotarm/joints/joint1/cmd rebotarm_msgs/msg/JointMotorCmd \
  "{mode: 0, use_pos: true, use_kp: true, use_kd: true, pos: 0.0, kp: 80.0, kd: 4.0}"
```

---

## 预留 API 与上层集成入口

这些接口已经在节点中注册，建议作为后续上层应用、规划器或调试工具的稳定接入点。

| 入口 | 当前用途 | 预留方向 | 使用说明 |
|---|---|---|---|
| `/rebotarm/follow_joint_trajectory` | 关节轨迹 action | MoveIt2、任务规划器、离线轨迹回放 | 发送完整 `joint_names` 和 `points`，首点需接近当前关节角 |
| `/rebotarm/move_to_pose` | 末端位姿 action | 视觉抓取、点击到达、任务级 API | 输入 `Pose + duration`，返回 `final_pose` 和执行结果 |
| `/rebotarm/move_to_pose_ik` | IK 服务 | IK 预检查、小步 servo、UI 预览 | 服务返回 `q_solution`，适合先验证目标是否可达 |
| `/rebotarm/joints/<joint>/cmd` | 单电机直通 topic | 调参、重力补偿、低层控制实验 | 轨迹运行时默认 `reject`，可用参数 `cmd_arbitration:=preempt` 改为抢占 |
| `/rebotarm/gripper/command` | 标准夹爪 action | 行为树、抓取 pipeline | 使用 `control_msgs/action/GripperCommand`，位置单位为米 |
| `/rebotarm/arm_status` | latched 状态 topic | UI、健康监控、状态机同步 | 关注 `state_machine`：`IDLE`、`TRAJ_RUNNING`、`LOWLEVEL_STREAMING`、`GRAVITY_COMP` |
| `arm_namespace` 参数 | 命名空间 | 多臂或仿真/真机并行 | launch 时传 `arm_namespace:=left_arm` |
| `frame_id`、`ee_frame_id` 参数 | 坐标帧标识 | 后续 TF、MoveIt2、视觉坐标对齐 | 默认 `base_link`、`end_link` |

---

## 配置说明

`rebotarm_bringup/config/` 提供默认配置：

| 文件 | 说明 |
|---|---|
| `arm.yaml` | 机械臂 6 个关节的电机、反馈 ID、控制参数 |
| `gripper.yaml` | 夹爪电机配置 |
| `driver_params.yaml` | ROS 参数示例 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm_config` | bringup 内置 `arm.yaml` | 机械臂配置路径 |
| `gripper_config` | bringup 内置 `gripper.yaml` | 夹爪配置路径 |
| `channel` | 空字符串 | 留空使用 YAML，非空时覆盖串口 |
| `joint_state_rate` | `100.0` | `/rebotarm/joint_states` 发布频率 |
| `cmd_arbitration` | `reject` | 轨迹运行中 per-joint cmd 仲裁，`reject` 或 `preempt` |
| `arm_namespace` | `rebotarm` | ROS 命名空间前缀 |
| `frame_id` | `base_link` | 机械臂基座坐标系，预留给 TF、视觉和规划集成 |
| `ee_frame_id` | `end_link` | 末端坐标系，预留给 TF、视觉和规划集成 |
| `use_rviz` | `false` | 是否启动 RViz |

---

## 排障

### 找不到串口

如果启动时报：

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

说明默认串口不存在。先查看实际设备：

```bash
ls /dev/ttyACM*
```

然后用 `channel:=...` 覆盖：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

### 权限不足

如果串口存在但无权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### RViz 模型不显示

确认 URDF mesh 路径已经是：

```text
package://rebotarm_bringup/description/meshes/...
```

### FastDDS SHM 端口提示

如果终端出现类似：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

通常是之前的 ROS2 进程异常退出后，FastDDS shared memory 锁文件残留。服务和 action
能正常响应时，这个提示一般不影响控制。需要清理时，先停掉相关 ROS2 进程，再执行：

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

如果希望临时绕开 shared memory transport，可在启动 ROS2 前设置：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

---

## 已知状态

`reBotArmController` 启动时会直接连接真实硬件；如果默认 `/dev/ttyACM0` 不存在，
需要通过 `channel:=/dev/ttyACM*` 指定正确串口。当前示例入口均通过 ROS2
service/action/topic 访问 controller，不会绕过 controller 直接占用硬件。
