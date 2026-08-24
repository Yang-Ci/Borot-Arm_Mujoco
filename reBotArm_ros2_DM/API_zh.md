# reBotArm ROS2 API Reference

适用版本：`v0.3.0`

本文档是 `rebotarm_ros2` 的 ROS2 API 参考文档。它面向二次开发、上层规划、视觉抓取、调试工具和脚本集成，按主流 API 文档习惯组织为：约定、Topic、Service、Action、低层 Command、消息定义、调用示例和集成注意事项。

## 目录

- [1. 基本约定](#1-基本约定)
- [2. Topic API](#2-topic-api)
- [3. Service API](#3-service-api)
- [4. Action API](#4-action-api)
- [5. 低层 Command Topic](#5-低层-command-topic)
- [6. 自定义消息定义](#6-自定义消息定义)
- [7. 推荐调用流程](#7-推荐调用流程)
- [8. 集成注意事项](#8-集成注意事项)

## 接口总览

| 类别 | 接口 | 类型 | 主要用途 |
|---|---|---|---|
| Topic | `/rebotarm/joint_states` | `sensor_msgs/msg/JointState` | 6 轴关节状态 |
| Topic | `/rebotarm/arm_status` | `rebotarm_msgs/msg/ArmStatus` | controller 状态机和健康状态 |
| Topic | `/rebotarm/joints/<joint>/state` | `rebotarm_msgs/msg/JointMotorState` | 单关节电机状态 |
| Topic | `/rebotarm/gripper/state` | `rebotarm_msgs/msg/JointMotorState` | 夹爪电机状态 |
| Service | `/rebotarm/enable` | `std_srvs/srv/Trigger` | 使能机械臂 |
| Service | `/rebotarm/disable` | `std_srvs/srv/Trigger` | 失能机械臂 |
| Service | `/rebotarm/safe_home` | `std_srvs/srv/Trigger` | 安全回零 |
| Service | `/rebotarm/set_mode` | `rebotarm_msgs/srv/SetMode` | 切换底层控制模式 |
| Service | `/rebotarm/set_zero` | `rebotarm_msgs/srv/SetZero` | 设置关节零点 |
| Service | `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/srv/MoveToPoseIK` | IK 预检查和目标关节角求解 |
| Service | `/rebotarm/gripper/set` | `rebotarm_msgs/srv/SetGripper` | 设置夹爪电机位置 |
| Service | `/rebotarm/gripper/open` | `rebotarm_msgs/srv/GripperCommand` | 打开夹爪到指定或默认位置 |
| Service | `/rebotarm/gripper/close` | `rebotarm_msgs/srv/GripperCommand` | 闭合夹爪到指定或默认位置 |
| Service | `/rebotarm/gravity_compensation/start` | `std_srvs/srv/Trigger` | 启动 controller 内部重力补偿 |
| Service | `/rebotarm/gravity_compensation/stop` | `std_srvs/srv/Trigger` | 停止 controller 内部重力补偿 |
| Action | `/rebotarm/move_to_pose` | `rebotarm_msgs/action/MoveToPose` | 末端位姿轨迹 |
| Action | `/rebotarm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 标准关节轨迹 |
| Action | `/rebotarm/gripper/command` | `control_msgs/action/GripperCommand` | 标准夹爪 action |
| Command Topic | `/rebotarm/joints/<joint>/cmd/mit` | `rebotarm_msgs/msg/JointMitCmd` | 单关节 MIT raw command |
| Command Topic | `/rebotarm/joints/<joint>/cmd/pos_vel` | `rebotarm_msgs/msg/JointPosVelCmd` | 单关节位置速度 raw command |
| Command Topic | `/rebotarm/joints/<joint>/cmd/vel` | `rebotarm_msgs/msg/JointVelCmd` | 单关节速度 raw command |
| Command Topic | `/rebotarm/gripper/cmd/mit` | `rebotarm_msgs/msg/JointMitCmd` | 夹爪 MIT raw command |
| Command Topic | `/rebotarm/gripper/cmd/pos_vel` | `rebotarm_msgs/msg/JointPosVelCmd` | 夹爪位置速度 raw command |
| Command Topic | `/rebotarm/gripper/cmd/vel` | `rebotarm_msgs/msg/JointVelCmd` | 夹爪速度 raw command |

## 1. 基本约定

### 命名空间

默认命名空间为：

```text
/rebotarm
```

本文档中的所有接口默认都以 `/rebotarm` 为前缀。如果启动时设置：

```bash
ros2 launch rebotarm_bringup bringup.launch.py arm_namespace:=left_arm
```

则 `/rebotarm/...` 替换为 `/left_arm/...`。

示例：

```text
/rebotarm/joint_states  ->  /left_arm/joint_states
/rebotarm/enable        ->  /left_arm/enable
```

命名空间只影响 ROS graph 中的 topic / service / action 名字，不会自动改变 TF frame。默认 TF frame 仍为 `base_link`、`end_link` 等 URDF 中定义的名字。

### 单位

| 字段 | 单位 |
|---|---|
| 关节位置 | rad |
| 关节速度 | rad/s |
| 力矩 / effort | N*m |
| 末端位置 | m |
| 四元数 | 无量纲，`x/y/z/w` |
| 夹爪电机位置 | rad |
| duration | s |

### 运动合法性

SDK 路径上的运动接口直接复用 `reBotArm_control_py` 的 IK、轨迹规划和控制逻辑：

- `/rebotarm/move_to_pose_ik` 调用 SDK `RebotArmEndPose.move_to_ik(...)`。
- `/rebotarm/move_to_pose` 调用 SDK `RebotArmEndPose.move_to_traj(...)`。

这两个接口不在 ROS 层重复实现 IK、轨迹规划或 joint limit 逻辑。标准外部关节轨迹接口
`/rebotarm/follow_joint_trajectory` 会接收上层直接给出的关节轨迹；ROS action 负责基础消息格式、
关节名顺序检查和按 `time_from_start` 执行多点轨迹，不在 ROS 层重复实现 IK 或 URDF joint limit
二次校验。

低层 command topic 面向调试，不承担 IK、轨迹规划或 URDF 合法性检查；上层应用应优先使用 service / action。

### QoS

| 类别 | QoS | 说明 |
|---|---|---|
| 高频状态 topic | `qos_profile_sensor_data` | BEST_EFFORT，适合 `/joint_states` 和电机状态 |
| `arm_status` | TRANSIENT_LOCAL + RELIABLE，depth 1 | 类似 latched 状态，后启动的订阅者也能拿到最后状态 |
| command topic | RELIABLE，depth 10 | 低层命令不应丢包 |

订阅 `/rebotarm/joint_states` 时建议使用 sensor-data QoS，否则可能出现：

```text
offering incompatible QoS. Last incompatible policy: RELIABILITY
```

### 控制状态机

`/rebotarm/arm_status.state_machine` 可能值：

| 值 | 说明 |
|---|---|
| `IDLE` | 空闲或位置保持 |
| `TRAJ_RUNNING` | 正在执行轨迹 action |
| `LOWLEVEL_STREAMING` | 收到过低层 raw command |
| `GRAVITY_COMP` | controller 内部重力补偿运行中 |

## 2. Topic API

### `/rebotarm/joint_states`

类型：

```text
sensor_msgs/msg/JointState
```

QoS：sensor-data

说明：发布 6 轴机械臂关节状态。`name` 顺序来自底层 `RebotArm.groups["arm"].joint_names`，通常为：

```text
joint1, joint2, joint3, joint4, joint5, joint6
```

字段：

| 字段 | 说明 |
|---|---|
| `name` | 关节名 |
| `position` | 关节角，rad |
| `velocity` | 关节速度，rad/s |
| `effort` | 关节力矩，N*m |

示例：

```bash
ros2 topic echo /rebotarm/joint_states --once
```

### `/rebotarm/arm_status`

类型：

```text
rebotarm_msgs/msg/ArmStatus
```

QoS：TRANSIENT_LOCAL + RELIABLE，depth 1

说明：发布 controller 的控制模式、使能状态、状态机和每个关节的状态码。

字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `mode` | `string` | 底层模式：`mit`、`pos_vel`、`vel` |
| `enabled` | `bool` | 机械臂是否使能 |
| `control_loop_active` | `bool` | 底层控制循环是否运行 |
| `state_machine` | `string` | `IDLE`、`TRAJ_RUNNING`、`LOWLEVEL_STREAMING`、`GRAVITY_COMP` |
| `joint_names` | `string[]` | 关节名 |
| `per_joint_status_code` | `uint8[]` | 每个关节的底层状态码 |
| `error_codes` | `string[]` | 当前错误码列表 |

示例：

```bash
ros2 topic echo /rebotarm/arm_status --once
```

### `/rebotarm/joints/<joint>/state`

类型：

```text
rebotarm_msgs/msg/JointMotorState
```

QoS：sensor-data

说明：发布单个关节电机状态。`<joint>` 可为 `joint1` 到 `joint6`。

示例：

```bash
ros2 topic echo /rebotarm/joints/joint1/state --once
```

### `/rebotarm/gripper/state`

类型：

```text
rebotarm_msgs/msg/JointMotorState
```

QoS：sensor-data

说明：发布夹爪电机状态。未配置夹爪时不发布。

示例：

```bash
ros2 topic echo /rebotarm/gripper/state --once
```

## 3. Service API

### `/rebotarm/enable`

类型：

```text
std_srvs/srv/Trigger
```

说明：使能机械臂；如果夹爪已初始化，也会确保夹爪处于 MIT 模式。

示例：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

### `/rebotarm/disable`

类型：

```text
std_srvs/srv/Trigger
```

说明：停止控制循环并失能机械臂。调用前如果重力补偿正在运行，会先停止重力补偿。

示例：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

### `/rebotarm/safe_home`

类型：

```text
std_srvs/srv/Trigger
```

说明：调用前会先停止重力补偿；如果夹爪已初始化，会先闭合夹爪到 `0.0rad`，
然后切回 `pos_vel` 控制并调用 SDK `RebotArmEndPose.safe_home()`，让机械臂以安全速度回零。

示例：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

### `/rebotarm/set_mode`

类型：

```text
rebotarm_msgs/srv/SetMode
```

请求：

| 字段 | 类型 | 说明 |
|---|---|---|
| `mode` | `string` | `mit`、`pos_vel` 或 `vel` |

响应：

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否切换成功 |
| `message` | `string` | 结果说明 |

示例：

```bash
ros2 service call /rebotarm/set_mode rebotarm_msgs/srv/SetMode "{mode: 'pos_vel'}"
```

### `/rebotarm/set_zero`

类型：

```text
rebotarm_msgs/srv/SetZero
```

请求：

| 字段 | 类型 | 说明 |
|---|---|---|
| `joint_name` | `string` | 空字符串表示全部关节；否则为单个关节名 |

示例：

```bash
ros2 service call /rebotarm/set_zero rebotarm_msgs/srv/SetZero "{joint_name: ''}"
```

单关节：

```bash
ros2 service call /rebotarm/set_zero rebotarm_msgs/srv/SetZero "{joint_name: 'joint3'}"
```

### `/rebotarm/move_to_pose_ik`

类型：

```text
rebotarm_msgs/srv/MoveToPoseIK
```

说明：调用 SDK `RebotArmEndPose.move_to_ik(...)` 求解并更新内部目标关节角，适合 IK 预检查或小步位姿调整。它不是完整轨迹 action；应用层需要更完整的轨迹执行时优先使用 `/rebotarm/move_to_pose`。

请求：

| 字段 | 类型 | 说明 |
|---|---|---|
| `target_pose` | `geometry_msgs/Pose` | 目标末端位姿，默认以 `base_link` 为参考 |

响应：

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | IK 是否成功 |
| `message` | `string` | 结果说明 |
| `q_solution` | `float64[]` | 求解得到的关节角，rad |

示例：

```bash
ros2 service call /rebotarm/move_to_pose_ik rebotarm_msgs/srv/MoveToPoseIK \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}"
```

### `/rebotarm/gripper/set`

类型：

```text
rebotarm_msgs/srv/SetGripper
```

说明：设置夹爪电机位置。ROS 层直接调用 SDK `Gripper.pos_vel()`，因此这里沿用
SDK 夹爪电机角度单位 rad，不再做开口距离到电机角度的二次映射。

请求：

| 字段 | 类型 | 说明 |
|---|---|---|
| `position` | `float64` | 目标夹爪电机位置，rad |
| `max_effort` | `float64` | 预留字段；当前 POS_VEL 控制不使用 |

响应：

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否到达目标 |
| `reached_position` | `float64` | 实际夹爪电机位置，rad |

示例：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: -5.0, max_effort: 0.0}"
```

### `/rebotarm/gripper/open`

类型：

```text
rebotarm_msgs/srv/GripperCommand
```

说明：用位置控制打开夹爪。`position` 为目标夹爪电机角度，单位 rad；传 `0.0`
时使用 controller 默认打开位置。`timeout` 小于等于 `0.0` 时使用默认超时。

请求：

| 字段 | 类型 | 说明 |
|---|---|---|
| `position` | `float64` | 目标夹爪电机位置，rad；`0.0` 表示默认打开位置 |
| `timeout` | `float64` | 等待到位超时，s |

响应：

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否在超时内到达目标 |
| `reached_position` | `float64` | 实际夹爪电机位置，rad |
| `message` | `string` | 结果说明 |

示例：

```bash
ros2 service call /rebotarm/gripper/open rebotarm_msgs/srv/GripperCommand \
  "{position: 0.0, timeout: 3.0}"
```

### `/rebotarm/gripper/close`

类型：

```text
rebotarm_msgs/srv/GripperCommand
```

说明：用位置控制闭合夹爪。`position` 为目标夹爪电机角度，单位 rad；传 `0.0`
时使用 controller 默认闭合位置。该接口不做力反馈夹取判断。

示例：

```bash
ros2 service call /rebotarm/gripper/close rebotarm_msgs/srv/GripperCommand \
  "{position: 0.0, timeout: 3.0}"
```

### `/rebotarm/gravity_compensation/start`

类型：

```text
std_srvs/srv/Trigger
```

说明：启动 controller 内部重力补偿闭环。闭环直接使用底层 `RebotArm` 的 `arm` group 状态和 MIT 命令，不通过外部 ROS topic 重写控制器。

实现要点：

- 进入 MIT 模式后锁定当前关节位置，并启动 controller 内部控制循环。
- 使用 `compute_generalized_gravity(q)` 计算重力前馈。
- 对多圈角度反馈做就近连续化，避免 `-4π` 类读数污染锁定目标。

示例：

```bash
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
```

### `/rebotarm/gravity_compensation/stop`

类型：

```text
std_srvs/srv/Trigger
```

说明：停止 controller 内部重力补偿，并切回 `pos_vel` hold。

示例：

```bash
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
```

## 4. Action API

### `/rebotarm/move_to_pose`

类型：

```text
rebotarm_msgs/action/MoveToPose
```

说明：末端笛卡尔位姿 action。内部直接调用 SDK `RebotArmEndPose.move_to_traj(...)`，
action 接受目标后返回当前 SDK `get_positions()` / `get_velocities()` 读数到 `message`。

Goal：

| 字段 | 类型 | 说明 |
|---|---|---|
| `target_pose` | `geometry_msgs/Pose` | 目标末端位姿 |
| `duration` | `float64` | 轨迹时长，s |

Result：

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否执行成功 |
| `message` | `string` | 结果说明，包含当前 positions / velocities |
| `final_pose` | `geometry_msgs/Pose` | action 返回时的末端位姿 |

Feedback 字段在消息定义中保留，但当前薄封装实现不发布反馈。

示例：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}, duration: 2.0}"
```

### `/rebotarm/follow_joint_trajectory`

类型：

```text
control_msgs/action/FollowJointTrajectory
```

说明：标准关节轨迹 action 兼容入口。当前实现会读取 trajectory 中的所有 point，
按每个 point 的 `time_from_start` 更新 SDK `RebotArmEndPose` 的关节目标；实际发送由 SDK
的 `pos_vel` 控制循环完成。action 存活期间会把当前 `/joint_states` 对应的
positions / velocities 填入 `FollowJointTrajectory.Feedback.actual`。

约束：

- `joint_names` 必须与底层 `RebotArm.groups["arm"].joint_names` 顺序完全一致。
- 每个 trajectory point 必须包含完整 `positions`。
- 当前按 point 之间的时间线性更新关节目标，不在 ROS 层重复实现 IK 或 URDF 合法性检查。

示例：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [{positions: [0.1,0,0,0,0,0], time_from_start: {sec: 5}}]}}"
```

### `/rebotarm/gripper/command`

类型：

```text
control_msgs/action/GripperCommand
```

说明：标准夹爪 action。适合行为树、任务编排或 MoveIt 风格接口。

Goal 中 `command.position` 单位为夹爪电机角度 rad。`command.max_effort` 为标准
action 字段，当前 ROS 适配层直接使用 SDK POS_VEL 控制，暂不使用该字段做力矩限制。

示例：

```bash
ros2 action send_goal /rebotarm/gripper/command control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 0.0}}"
```

## 5. 低层 Command Topic

低层 command topic 面向调试和实验，不建议作为应用层常规运动接口。轨迹运行时的仲裁由 launch 参数 `cmd_arbitration` 控制：

| 值 | 行为 |
|---|---|
| `reject` | 轨迹运行时拒绝低层命令，默认 |
| `preempt` | 轨迹运行时 arm joint 低层命令抢占轨迹 |

低层 command 按控制模式拆分为 MIT、位置速度、速度三类 topic。

注意：低层 command topic 使用电机原始单位，`pos` 为 rad，`vel` 为 rad/s。
夹爪 service、action 和低层 command 都使用夹爪电机角度 rad。

安全约束：

- 重力补偿运行时拒绝全部低层 command。
- 轨迹运行时，arm joint 低层 command 默认拒绝；只有 `cmd_arbitration:=preempt` 时才会先停止轨迹再进入低层模式。
- 轨迹运行时，gripper 低层 command 始终拒绝，不抢占 arm 轨迹。
- 低层 command 是调试入口，不做轨迹规划、IK 或 URDF joint limit 校验；应用层运动优先使用 action / service。

### `/rebotarm/joints/<joint>/cmd/mit`

类型：

```text
rebotarm_msgs/msg/JointMitCmd
```

QoS：RELIABLE，depth 10

说明：单关节 MIT 模式 raw command。`<joint>` 可为 `joint1` 到 `joint6`。

示例：

```bash
ros2 topic pub --once /rebotarm/joints/joint1/cmd/mit rebotarm_msgs/msg/JointMitCmd \
  "{pos: 0.0, vel: 0.0, kp: 80.0, kd: 4.0, tau: 0.0}"
```

### `/rebotarm/joints/<joint>/cmd/pos_vel`

类型：

```text
rebotarm_msgs/msg/JointPosVelCmd
```

QoS：RELIABLE，depth 10

说明：单关节位置速度模式 raw command。

示例：

```bash
ros2 topic pub --once /rebotarm/joints/joint1/cmd/pos_vel rebotarm_msgs/msg/JointPosVelCmd \
  "{pos: 0.1, vlim: 0.2}"
```

### `/rebotarm/joints/<joint>/cmd/vel`

类型：

```text
rebotarm_msgs/msg/JointVelCmd
```

QoS：RELIABLE，depth 10

说明：单关节速度模式 raw command。

示例：

```bash
ros2 topic pub --once /rebotarm/joints/joint1/cmd/vel rebotarm_msgs/msg/JointVelCmd \
  "{vel: 0.05}"
```

### `/rebotarm/gripper/cmd/mit`

类型：

```text
rebotarm_msgs/msg/JointMitCmd
```

QoS：RELIABLE，depth 10

说明：夹爪 MIT 模式 raw command。

### `/rebotarm/gripper/cmd/pos_vel`

类型：

```text
rebotarm_msgs/msg/JointPosVelCmd
```

QoS：RELIABLE，depth 10

说明：夹爪位置速度模式 raw command。

### `/rebotarm/gripper/cmd/vel`

类型：

```text
rebotarm_msgs/msg/JointVelCmd
```

QoS：RELIABLE，depth 10

说明：夹爪速度模式 raw command。

## 6. 自定义消息定义

### `ArmStatus.msg`

```text
std_msgs/Header header
string mode
bool enabled
bool control_loop_active
string state_machine
string[] joint_names
uint8[] per_joint_status_code
string[] error_codes
```

### `JointMotorState.msg`

```text
std_msgs/Header header
string joint_name
float64 position
float64 velocity
float64 torque
uint8 status_code
```

### `JointMitCmd.msg`

```text
float64 pos
float64 vel
float64 kp
float64 kd
float64 tau
builtin_interfaces/Time stamp
```

### `JointPosVelCmd.msg`

```text
float64 pos
float64 vlim
builtin_interfaces/Time stamp
```

### `JointVelCmd.msg`

```text
float64 vel
builtin_interfaces/Time stamp
```

### `SetMode.srv`

```text
string mode
---
bool success
string message
```

### `SetZero.srv`

```text
string joint_name
---
bool success
string message
```

### `MoveToPoseIK.srv`

```text
geometry_msgs/Pose target_pose
---
bool success
string message
float64[] q_solution
```

### `SetGripper.srv`

```text
float64 position
float64 max_effort
---
bool success
float64 reached_position
```

### `GripperCommand.srv`

```text
float64 position
float64 timeout
---
bool success
float64 reached_position
string message
```

### `MoveToPose.action`

```text
geometry_msgs/Pose target_pose
float64 duration
---
bool success
string message
geometry_msgs/Pose final_pose
---
geometry_msgs/Pose current_pose
float64 progress
float64 time_elapsed
```

## 7. 推荐调用流程

### 末端位姿移动

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger

ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}, duration: 2.0}"

ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

### 夹爪控制

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger

ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: -5.0, max_effort: 0.0}"

ros2 service call /rebotarm/gripper/close rebotarm_msgs/srv/GripperCommand \
  "{position: 0.0, timeout: 3.0}"

ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

### 重力补偿

推荐优先使用 demo：

```bash
ros2 run rebotarmcontroller GravityCompensation
```

手动服务调用：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

## 8. 集成注意事项

- `MoveTo` 和 `/follow_joint_trajectory` 都使用弧度，不是角度。
- `/follow_joint_trajectory` 按 point 的 `time_from_start` 执行多点关节轨迹。
- `/move_to_pose` 更适合应用层“到达某个末端位姿”的常规使用。
- 重力补偿必须在 controller 内部运行；不要在外部 ROS 节点用 `/joint_states` + raw command 重写高频闭环。
- `/safe_home` 会先闭合夹爪再执行机械臂安全回零；`reBotArmController` 退出时默认也走同一流程。
- 夹爪 `open` / `close` service 是位置控制接口，不包含力反馈夹取判断。
- 多机械臂场景中，`arm_namespace` 只解决 ROS graph 命名冲突；TF frame 仍需额外规划 frame 前缀或 URDF 命名。
- 修改 `hardware_manager.py`、`ros_services.py` 或 `ros_actions.py` 后，需要重启 `driver.launch.py` 才会加载新 controller 逻辑。
