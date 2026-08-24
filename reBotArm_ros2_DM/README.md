# reBotArm ROS2 SDK

版本：`v0.0.2`

中文完整使用说明见 [README_zh.md](README_zh.md)。

## ROS 端启动脚本

常用启动入口在 `scripts/` 顶层：

```bash
scripts/start_fake_bringup.sh
scripts/start_joint_slider_gui.sh
scripts/start_real2sim.sh
scripts/start_mujoco_torque_control.sh
scripts/start_rebotarm_all.sh
```

`scripts/source_rebotarm_env.sh` 仍然是环境加载脚本，不是直接启动入口。

### GUI 控制与重力补偿

```bash
cd reBotArm_ros2_DM
./scripts/start_joint_slider_gui.sh
```

中文 joint slider GUI 会发布 `/rebotarm/joint_states`，并提供 controller
重力补偿按钮。按钮调用：

```bash
/rebotarm/gravity_compensation/start
/rebotarm/gravity_compensation/stop
/rebotarm/gravity_compensation/status
```

如果要让重力补偿按钮有响应，需要先在另一个终端启动 fake driver 或真实
controller。

## 使用方法

进入 ROS2 工作区后直接运行需要的主入口：

```bash
cd reBotArm_ros2_DM
chmod +x scripts/*.sh
./scripts/start_joint_slider_gui.sh
```

如果工作区还没有编译：

```bash
cd reBotArm_ros2_DM
colcon build
source install/setup.bash
chmod +x scripts/*.sh
./scripts/start_joint_slider_gui.sh
```

脚本启动时会自动尝试加载：

```bash
/opt/ros/jazzy/setup.bash
.venv/bin/activate
.venv/lib/python*/site-packages
install/setup.bash
```

其中 `.venv/lib/python*/site-packages` 会被自动加入 `PYTHONPATH`，用于让 ROS2
节点找到虚拟环境里的 `mujoco` 等 Python 包。

如果没有找到 Jazzy，并且当前终端还没有加载 ROS2，脚本也会继续尝试加载：

```bash
/opt/ros/*/setup.bash
```

## STL MuJoCo 快速验证

如果刚修改过 MuJoCo 包，先构建一次：

```bash
colcon build --symlink-install --packages-select rebotarm_mujoco
source install/setup.bash
REAL2SIM_MODEL=stl ./scripts/start_real2sim.sh
```

## MuJoCo tau_g 与力矩闭环

单独调试 tau_g / torque loop：

```bash
./scripts/start_mujoco_torque_control.sh
```

默认使用 STL 模型，并把 `/rebotarm/joint_states` 作为 MuJoCo 力矩闭环目标，
所以可以跟随网页/fake driver 发布的关节状态。该节点同时发布 MuJoCo
`tau_g`、SDK `tau_g` 以及差值：

```bash
ros2 topic echo /rebotarm/mujoco/tau_g --once
ros2 topic echo /rebotarm/mujoco/sdk_tau_g --once
ros2 topic echo /rebotarm/mujoco/tau_g_diff --once
```

如果要完全独立于网页/fake driver 的纯 MuJoCo 目标话题，可以这样启动：

```bash
MUJOCO_TORQUE_TARGET_TOPIC=/rebotarm/mujoco/target_joint_states ./scripts/start_mujoco_torque_control.sh
```

然后下发目标姿态，不会控制真实硬件：

```bash
ros2 topic pub --once /rebotarm/mujoco/target_joint_states sensor_msgs/msg/JointState \
"{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.0, -0.7, -0.8, 0.0, 0.5, 0.0]}"
```

## 可选参数

可以通过环境变量修改默认配置：

```bash
SERIAL_CHANNEL=/dev/ttyACM0 \
ROSBRIDGE_PORT=9090 \
ROSBRIDGE_ADDRESS=0.0.0.0 \
USE_RVIZ=true \
ros2 launch rebotarm_bringup bringup.launch.py
```

默认值如下：

- `SERIAL_CHANNEL=/dev/ttyACM0`
- `ROSBRIDGE_PORT=9090`
- `ROSBRIDGE_ADDRESS=0.0.0.0`
- `USE_RVIZ=true`

## 注意事项

- 请在 ROS2/Linux 环境中运行该脚本。
- 如果真实机械臂串口不是 `/dev/ttyACM0`，请用 `SERIAL_CHANNEL` 修改。
- `rosbridge_server` 需要已经安装，否则 rosbridge websocket 启动会失败。
- `start_joint_slider_gui.sh` 会发布 `/rebotarm/joint_states`，不要和其他
  同时发布该话题的节点混用，除非你明确在做多发布源测试。
