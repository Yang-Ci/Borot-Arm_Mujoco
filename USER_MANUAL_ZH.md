# reBot Arm B601-DM 用户使用手册

本手册面向第一次接触本项目的用户，说明从克隆代码、一键安装、环境检查到启动网页、连接 DM 真机、使用重力补偿和安全退出的完整流程。

## 1. 项目简介

本项目面向 reBot Arm B601-DM（达妙电机版），提供：

- ROS 2 Jazzy 真机驱动；
- MuJoCo 物理仿真与 real2sim；
- Three.js 网页控制台；
- 关节、轨迹、IK、夹爪和重力补偿控制；
- rosbridge WebSocket；
- MCP 与文本 Agent；
- 一键安装、环境诊断和统一启动命令。

主要目录：

| 路径 | 作用 |
| --- | --- |
| `setup.sh` | 增量安装和版本检查 |
| `rebotarm` | 启动、停止、状态和诊断入口 |
| `requirements.txt` | Python 依赖兼容范围 |
| `reBotArm_ros2_DM/` | ROS 2 工作空间 |
| `reBotArm_simulator-DM/` | 网页控制台 |
| `reBotArm_ros2_DM/third_party/reBotArm_control_py/` | 新安装时的默认 SDK 位置 |

## 2. 支持环境

推荐环境：

| 组件 | 推荐版本 |
| --- | --- |
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy |
| Python | 3.12 |
| Node.js | 18 或更高 |
| MuJoCo | 3.10.x |
| Pinocchio (`pin`) | 4.1.x |
| MotorBridge | 0.5.x |

硬件真机模式还需要：

- reBot Arm B601-DM；
- 达妙电机及电源；
- USB-CAN 串口桥；
- `/dev/ttyACM0` 设备节点。

## 3. 克隆项目

```bash
cd ~
git clone <本仓库地址> reBot_Arm_Mujoco-DM
cd reBot_Arm_Mujoco-DM
```

确认一键入口存在：

```bash
ls -l setup.sh rebotarm requirements.txt
```

如果脚本没有执行权限：

```bash
chmod +x setup.sh rebotarm
```

## 4. 一键安装

### 4.1 先做只读检查

```bash
./setup.sh --check
```

`--check` 不安装、不克隆、不复制、不编译，也不会修改系统。结果分为四组：

| 分组 | 含义 |
| --- | --- |
| `Installed/updated` | 本次安装或更新的组件 |
| `Already usable; skipped` | 已存在且可用，因此保留并跳过 |
| `Version/platform mismatches` | 版本或平台与验证环境不一致 |
| `Failed or still missing` | 缺失、安装失败或检查失败 |

首次运行出现缺失项是正常的。

### 4.2 执行安装

```bash
./setup.sh
```

需要安装系统包时会提示：

```text
[sudo] password for <用户名>:
```

输入 Ubuntu 当前用户密码。终端不显示密码字符属于正常行为。

安装器遵循以下原则：

- 已存在且能正常使用的组件直接跳过；
- 只安装缺失或版本不兼容的项目 Python 包；
- 不删除已有 SDK；
- 不切换或覆盖已有 SDK Git 分支；
- 已有 SDK 提交号与验证版本不同时只提示，不阻断安装；
- `.venv` 的 Python 版本不兼容时，先将原目录重命名备份，再创建正确版本；
- 不覆盖网页 `.env`；
- 不删除用户配置的软件源；
- 可以安全重复运行。

安装器会处理：

1. Ubuntu、CPU 架构、Python 和 Node.js 检查；
2. ROS 2 Jazzy、rosbridge、MoveIt 和开发工具；
3. `reBotArm_control_py` SDK；
4. 项目 `.venv`；
5. MuJoCo、Pinocchio、MotorBridge、FastMCP 等 Python 依赖；
6. 网页 `.env`；
7. rosdep 依赖；
8. `colcon build --symlink-install`；
9. Python/SDK 导入和 `/dev/ttyACM0` 检查。

系统 Python 和项目虚拟环境的 Python 小版本必须一致，并与 ROS 2 对应。
- Ubuntu 24.04 + ROS 2 Jazzy：使用系统 /usr/bin/python3.12
- Ubuntu 22.04 + ROS 2 Humble：使用系统 /usr/bin/python3.10
项目 .venv 必须由对应的系统 Python 创建，因为它通过 --system-site-packages 使用 ROS 的 Python 包。

需要 Ubuntu 系统自带的 Python 版本，并用它创建项目虚拟环境。Ubuntu 24.04 要求 Python 3.12，Ubuntu 22.04 要求 Python 3.10。Conda base 中的 Python 3.13 可以保留，安装脚本不会使用它。

### 4.3 安装后复检

```bash
./rebotarm doctor
```

`doctor` 等价于：

```bash
./setup.sh --check
```

理想结果：

```text
Version/platform mismatches (0)
  - none

Failed or still missing (0)
  - none
```

如果机械臂尚未接入，`/dev/ttyACM0 not connected` 只表示硬件未连接，不影响网页或纯仿真。

如果 `doctor` 仍显示系统包缺失，说明上一次安装没有真正完成。修复 apt 问题后重新运行：

```bash
./setup.sh
./rebotarm doctor
```

## 5. 安装位置说明

### 5.1 系统环境

系统组件通过 apt 安装，需要 sudo，主要位于 `/usr/` 和 `/opt/ros/jazzy/`：

```text
ROS 2 Jazzy
rosbridge
MoveIt
ros-dev-tools
Node.js / npm
Python 3.12 / venv
Git / Curl / 编译工具
```

### 5.2 项目虚拟环境

Python 依赖安装在：

```text
reBotArm_ros2_DM/.venv
```

包括：

```text
mujoco, pin, motorbridge, numpy, PyYAML, transforms3d,
tornado, psutil, fastmcp, openai
```

### 5.3 SDK

新用户默认 SDK 路径：

```text
reBot_Arm_Mujoco-DM/reBotArm_ros2_DM/third_party/reBotArm_control_py
```

如果已经存在以下 SDK，安装器会优先保留：

```text
~/reBotArm_control_py
```

当前控制器会按顺序搜索：

1. `reBotArm_ros2_DM/third_party/reBotArm_control_py`；
2. `reBotArm_ros2_DM/sdk/reBotArm_control_py`；
3. `~/reBotArm_control_py`；
4. `~/seeed/cameraws/sdk/reBotArm_control_py`。

## 6. 统一命令入口

```bash
./rebotarm doctor
./rebotarm status
./rebotarm start web
./rebotarm start dm
./rebotarm start sim
./rebotarm stop
```

| 命令 | 作用 |
| --- | --- |
| `doctor` | 只读环境检查和版本报告 |
| `status` | 查看管理进程、端口、串口和 ROS 节点 |
| `start web` | 启动 rosbridge 和网页服务 |
| `start dm` | 启动 DM 真机控制器 |
| `start sim` | 启动 MuJoCo 仿真栈 |
| `stop` | 停止由统一入口记录的后台进程 |

DM 真机与仿真模式应分开使用，不要同时启动。

## 7. 启动网页控制台

打开终端 1：

```bash
cd ~/reBot_Arm_Mujoco-DM
./rebotarm start web
```

正常输出：

```text
Web: http://localhost:3001
ROS WebSocket: ws://localhost:9090
Ctrl+C stops both.
```

浏览器访问：

```text
http://localhost:3001
```

如果浏览器和 Ubuntu 在同一台电脑，WebSocket 使用：

```text
ws://localhost:9090
```

如果浏览器位于另一台电脑，使用 Ubuntu 主机 IP：

```text
ws://<Ubuntu IP>:9090
```

查询 Ubuntu IP：

```bash
hostname -I
```

## 8. 启动 DM 真机

### 8.1 启动前安全检查

1. 确认机械臂周围没有人员和障碍物；
2. 确认机械臂安装稳固；
3. 确认急停/断电方式随时可用；
4. 确认 USB-CAN 串口桥连接；
5. 确认没有 RS 控制器或 Fake Driver 使用 `/rebotarm`。

检查串口：

```bash
ls -l /dev/ttyACM0
```

临时权限不足时：

```bash
sudo chmod 666 /dev/ttyACM0
```

### 8.2 启动控制器

打开终端 2：

```bash
cd ~/reBot_Arm_Mujoco-DM
./rebotarm start dm
```

默认使用：

```text
/dev/ttyACM0
```

指定其他设备：

```bash
SERIAL_CHANNEL=/dev/ttyACM1 ./rebotarm start dm
```

看到控制器启动后先不要立即使能，先执行状态检查。

## 9. 检查 DM 数据链路

打开终端 3：

```bash
cd ~/reBot_Arm_Mujoco-DM
./rebotarm status
```

真机链路通常包含：

```text
/reBotArmController
/robot_state_publisher
/rosapi
/rosbridge_websocket
```

读取一次关节状态：

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArm_ros2_DM
source scripts/source_rebotarm_env.sh
ros2 topic echo /rebotarm/joint_states --once
```

读取机械臂状态：

```bash
ros2 topic echo /rebotarm/arm_status --once
```

查看动作接口：

```bash
ros2 action list -t
```

应该包含：

```text
/rebotarm/follow_joint_trajectory
/rebotarm/gripper/command
/rebotarm/move_to_pose
```

## 10. 网页连接和控制

### 10.1 连接 ROS

1. 在网页填写 rosbridge WebSocket 地址；
2. 点击“连接 ROS”；
3. 等待日志显示 `ROS 已连接`；
4. 确认 rosapi 检测到 topics、services 和 actions。

连接 ROS 不会自动查询或启动重力补偿。

### 10.2 镜像真机

勾选：

```text
镜像真实关节状态到网页
```

确认网页模型姿态与真机一致。如果姿态明显不一致，不要使能，先检查关节映射和反馈。

### 10.3 打开控制锁

勾选：

```text
允许网页向真实机械臂发控制
```

未打开控制锁时，网页只显示和更新仿真，不向真机发送运动命令。

### 10.4 建议测试顺序

1. 点击“使能”；
2. 测试夹爪打开和闭合；
3. 小幅测试 joint6、joint5 等末端关节；
4. 再测试近端关节；
5. 最后测试轨迹、IK 和重力补偿。

夹爪开/闭按钮不弹出二次确认，但仍受控制锁保护。

## 11. 重力补偿

### 11.1 启动

点击“启动重补”后，控制器会：

1. 读取当前关节位置；
2. 计算当前位置重力力矩；
3. 每个关节切入 MIT 后立即发送当前位置保持；
4. 启动连续重力补偿循环。

机械臂不应先运动到零位再启动重力补偿。

重力补偿已经运行时再次点击“启动重补”会被忽略，不切模式、不停止控制循环、也不重新上电。

### 11.2 停止

点击“停止重补”后：

```text
停止 MIT 重力补偿
→ 保存最后反馈位置
→ 切回 POS_VEL
→ 保持当前位置
```

### 11.3 查询

连接 ROS 时不会自动查询重力补偿。用户手动点击“查询状态”，或控制器确认已经进入重力补偿后，网页才会调用状态服务。

重力补偿运行期间，网页每 500 ms 静默刷新一次当前锁定目标角度；停止重补、失能或断开 ROS 后自动停止刷新。静默刷新不会反复写入实时日志。

## 12. 安全断开

网页提供：

```text
断开时自动安全回零并失能
```

该选项默认关闭。

未勾选时：

```text
点击断开 → 直接断开 WebSocket
```

勾选后：

```text
点击断开
→ 等待安全回零完成
→ 等待电机失能完成
→ 断开 ROS
```

如果回零或失能失败，网页保持 ROS 连接，避免在机械臂未安全处理时失去控制通道。

## 13. MuJoCo 仿真

确保 DM 真机控制器已经停止，然后执行：

```bash
cd ~/reBot_Arm_Mujoco-DM
./rebotarm start sim
```

不要同时运行：

```text
./rebotarm start dm
./rebotarm start sim
```

因为它们可能同时使用 `/rebotarm` 命名空间，造成服务和状态来源混乱。

## 14. 停止项目

推荐顺序：

1. 网页勾选安全断开；
2. 点击“断开”；
3. DM 控制器终端按 `Ctrl+C`；
4. Web 终端按 `Ctrl+C`；
5. 执行状态检查。

```bash
cd ~/reBot_Arm_Mujoco-DM
./rebotarm status
```

`./rebotarm stop` 只停止统一入口记录的进程，不会使用宽泛的 `pkill` 去终止用户的其他 ROS 项目。

## 15. 更新代码后重新编译

拉取代码：

```bash
cd ~/reBot_Arm_Mujoco-DM
git pull
```

重新运行增量安装：

```bash
./setup.sh
```

安装器会跳过可用组件，并重新执行依赖检查和工作空间编译。

只重新编译 ROS 工作空间：

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArm_ros2_DM
source scripts/source_rebotarm_env.sh
colcon build --symlink-install
```

## 16. 常见故障

### 16.1 apt update 被第三方源阻塞

典型错误：

```text
仓库 https://deb.nodesource.com/... 不再含有 Release 文件
```

安装器不会自动删除或禁用用户的软件源。它会记录警告，并尝试使用已有 apt 索引继续安装。

先查看第三方源：

```bash
grep -R "nodesource" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null
```

如果确认该源已经失效，应由用户自行修复或禁用，然后重新运行：

```bash
sudo apt update
./setup.sh
./rebotarm doctor
```

不要仅根据 `Setup complete` 判断环境可用，最终以 `doctor` 的 `Failed or still missing (0)` 为准。

### 16.2 rosbridge 端口被占用

错误：

```text
Address already in use
```

检查 9090：

```bash
ss -ltnp 'sport = :9090'
```

通常说明已经有一个 rosbridge 在运行，不要重复启动。

### 16.3 网页端口被占用

```bash
ss -ltnp 'sport = :3001'
```

确认是否已经有本项目网页服务运行。

### 16.4 找不到 `/dev/ttyACM0`

```bash
lsusb
ls -l /dev/ttyACM*
dmesg | tail -n 50
```

检查 USB 连接、电机电源和串口桥。

### 16.5 串口权限不足

临时处理：

```bash
sudo chmod 666 /dev/ttyACM0
```

长期处理可将当前用户加入 `dialout`：

```bash
sudo usermod -aG dialout "$USER"
```

注销并重新登录后生效。

### 16.6 `tf_transformations` 缺失

优先重新运行：

```bash
./setup.sh
```

安装器会检测系统 ROS 包或项目 `.venv` 中是否已有可用模块。

### 16.7 同名 ROS 节点或状态来源混乱

```bash
ros2 node list
ps -eo pid,tty,cmd | grep -E 'reBotArmController|FakeReBotArmDriver'
```

真机模式只应保留一个 `/reBotArmController`，不要同时运行 RS 控制器或 Fake Driver。

### 16.8 控制器启动后立即退出

查看当前终端的完整 Python Traceback，并检查：

```bash
./rebotarm doctor
ls -l /dev/ttyACM0
fuser -v /dev/ttyACM0
```

### 16.9 网页连接成功但不能控制

依次确认：

1. `/reBotArmController` 正在运行；
2. 网页连接到正确的 rosbridge；
3. 已勾选控制锁；
4. 机械臂已经使能；
5. `/rebotarm/arm_status` 没有错误码。

## 17. 快速命令表

```bash
# 安装和诊断
./setup.sh
./setup.sh --check
./rebotarm doctor

# 启动
./rebotarm start web
./rebotarm start dm
./rebotarm start sim

# 状态和停止
./rebotarm status
./rebotarm stop

# ROS 环境
cd reBotArm_ros2_DM
source scripts/source_rebotarm_env.sh

# 状态反馈
ros2 topic echo /rebotarm/joint_states --once
ros2 topic echo /rebotarm/arm_status --once
ros2 action list -t

# 端口和串口
ss -ltnp | grep -E ':3001|:9090'
ls -l /dev/ttyACM0
fuser -v /dev/ttyACM0
```

## 18. 相关文档

- [项目总 README](./README.md)
- [DM 数据链路说明](./DATA_FLOW_ZH.md)
- [项目架构说明](./PROJECT_ARCHITECTURE_ZH.md)
- [ROS 2 工作空间说明](./reBotArm_ros2_DM/README_zh.md)
- [MuJoCo 包说明](./reBotArm_ros2_DM/src/rebotarm_mujoco/README.md)
- [网页控制台说明](./reBotArm_simulator-DM/README.md)
- [Agent/MCP 说明](./reBotArm_ros2_DM/src/rebotarm_agent/README.md)
