# reBotArm B601-DM ROS2 命令速查

> 前提：已执行 `source scripts/source_rebotarm_env.sh`，且 Fake Driver 或真机驱动已启动。

## 话题查看

```bash
# 列出所有活跃话题
ros2 topic list

# 实时查看 6 关节 + 夹爪状态
ros2 topic echo /rebotarm/joint_states

# 查看机械臂状态（使能、模式、状态机）
ros2 topic echo /rebotarm/arm_status

# 查看夹爪状态（位置 / 速度 / 力矩）
ros2 topic echo /rebotarm/gripper/state

# 查看话题发布频率
ros2 topic hz /rebotarm/joint_states

# 查看话题信息（类型、发布者、订阅者）
ros2 topic info /rebotarm/joint_states
```

## 服务调用（命令控制）

```bash
# 使能机械臂
ros2 service call /rebotarm/enable std_srvs/srv/Trigger

# 失能机械臂
ros2 service call /rebotarm/disable std_srvs/srv/Trigger

# 安全回零
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger

# 启动重力补偿
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger

# 停止重力补偿
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger

# 查询重力补偿状态
ros2 service call /rebotarm/gravity_compensation/status std_srvs/srv/Trigger

# 设置夹爪开度（0 = 闭合，0.09 = 全开，单位米）
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.05, timeout: 3.0}"

# IK 可达性检查（不执行运动）
ros2 service call /rebotarm/move_to_pose_ik rebotarm_msgs/srv/MoveToPoseIK "{x: 0.3, y: 0.0, z: 0.3, roll: 0.0, pitch: 0.0, yaw: 0.0}"

# 列出所有服务
ros2 service list
```

## Action 调用（轨迹运动）

```bash
# 列出所有 action
ros2 action list

# 查看 action 信息
ros2 action info /rebotarm/move_to_pose

# 笛卡尔位姿运动
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose "{x: 0.3, y: 0.0, z: 0.3, roll: 0.0, pitch: 0.0, yaw: 0.0, duration: 3.0}"

# 关节轨迹执行
ros2 action send_goal /rebotarm/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

## 话题发布（直接发命令）

```bash
# 单关节位置命令（joint1 转 0.5 弧度）
ros2 topic pub --once /rebotarm/joints/joint1/cmd rebotarm_msgs/msg/JointMotorCmd "{pos: 0.5, vel: 0.0, kp: 0.0, kd: 0.0, tau: 0.0}"

# 夹爪命令（0.05 米开度）
ros2 topic pub --once /rebotarm/gripper/cmd rebotarm_msgs/msg/JointMotorCmd "{pos: 0.05, vel: 0.0, kp: 0.0, kd: 0.0, tau: 0.0}"

# TCP 目标位姿（拖拽模式）
ros2 topic pub --once /rebotarm/mujoco/target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 节点与参数

```bash
# 列出所有节点
ros2 node list

# 查看控制器节点信息
ros2 node info /rebotarm_controller

# 列出参数
ros2 param list /rebotarm_controller

# 查看单个参数
ros2 param get /rebotarm_controller arm_config
ros2 param get /rebotarm_controller channel
ros2 param get /rebotarm_controller joint_state_rate
```

## MuJoCo 仿真相关

```bash
# 查看仿真关节状态
ros2 topic echo /rebotarm/mujoco/physics_joint_states

# 查看虚拟相机图像（需安装 image_tools）
ros2 run image_tools showimage --ros-args -r /image:=/rebotarm/mujoco/overhead_rgb/image_raw

# 查看颜色检测结果
ros2 topic echo /rebotarm/vision/color_blocks/detections

# 查看仿真动画事件（抓取 / 释放）
ros2 topic echo /rebotarm/sim/animation_event

# 录制开始
ros2 service call /rebotarm/mujoco/record/start std_srvs/srv/Trigger

# 录制停止
ros2 service call /rebotarm/mujoco/record/stop std_srvs/srv/Trigger

# 回放录制
ros2 service call /rebotarm/mujoco/record/replay std_srvs/srv/Trigger
```

## 录制与回放

```bash
# 录制关节话题 10 秒
ros2 bag record -o /tmp/arm_demo /rebotarm/joint_states /rebotarm/gripper/state /rebotarm/arm_status --duration 10

# 回放录制
ros2 bag play /tmp/arm_demo
```

## 接口总览

### Topic

| Topic | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/rebotarm/joint_states` | `sensor_msgs/msg/JointState` | 订阅 | 6 关节 + 夹爪实时位置 |
| `/rebotarm/gripper/state` | `rebotarm_msgs/msg/JointMotorState` | 订阅 | 夹爪位置 / 速度 / 力矩 |
| `/rebotarm/arm_status` | `rebotarm_msgs/msg/ArmStatus` | 订阅 | 使能、模式、状态机 |
| `/rebotarm/joints/<N>/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 发布 | 单关节稀疏命令 |
| `/rebotarm/gripper/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 发布 | 夹爪命令（米，0~0.09） |
| `/rebotarm/mujoco/target_pose` | `geometry_msgs/msg/PoseStamped` | 发布 | TCP 拖拽目标位姿 |
| `/rebotarm/mujoco/overhead_rgb/image_raw` | `sensor_msgs/msg/Image` | 订阅 | 桌面俯视相机 RGB |
| `/rebotarm/vision/color_blocks/detections` | `std_msgs/msg/String` | 订阅 | 颜色块检测结果（JSON） |
| `/rebotarm/sim/animation_event` | `std_msgs/msg/String` | 订阅 | 仿真动画事件 |

### Service

| Service | 类型 | 说明 |
|---|---|---|
| `/rebotarm/enable` | `std_srvs/srv/Trigger` | 使能所有电机 |
| `/rebotarm/disable` | `std_srvs/srv/Trigger` | 失能所有电机 |
| `/rebotarm/safe_home` | `std_srvs/srv/Trigger` | 安全回零 |
| `/rebotarm/gravity_compensation/start` | `std_srvs/srv/Trigger` | 启动重力补偿 |
| `/rebotarm/gravity_compensation/stop` | `std_srvs/srv/Trigger` | 停止重力补偿 |
| `/rebotarm/gravity_compensation/status` | `std_srvs/srv/Trigger` | 查询重力补偿状态 |
| `/rebotarm/gripper/set` | `rebotarm_msgs/srv/SetGripper` | 夹爪到位服务 |
| `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/srv/MoveToPoseIK` | IK 解算服务 |
| `/rebotarm/mujoco/record/start` | `std_srvs/srv/Trigger` | 开始录制 |
| `/rebotarm/mujoco/record/stop` | `std_srvs/srv/Trigger` | 停止录制 |
| `/rebotarm/mujoco/record/replay` | `std_srvs/srv/Trigger` | 回放录制 |
| `/rebotarm/mujoco/record/clear` | `std_srvs/srv/Trigger` | 清空录制 |

### Action

| Action | 类型 | 说明 |
|---|---|---|
| `/rebotarm/move_to_pose` | `rebotarm_msgs/action/MoveToPose` | 笛卡尔位姿运动 |
| `/rebotarm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 关节轨迹执行 |
