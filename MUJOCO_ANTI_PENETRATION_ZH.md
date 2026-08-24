# MuJoCo 夹爪防穿模方案

## 问题原因

原来每根手指使用一个整体凸包碰撞体。凸包会填满手指中间的凹槽，导致碰撞面与视觉模型不一致；同时接触过软、摩擦过大和夹爪宽度标定不正确，也会造成物体穿入或单侧卡住。

## 解决办法

1. **拆分碰撞体**
   - 左右手指分别拆成 `front`、`mid`、`rear` 三段凸包。
   - 三段使用连续的 X 切面，避免段与段之间留下碰撞空隙。
   - 共新增 6 个 STL 碰撞网格。

2. **调整接触参数**
   - 手指碰撞余量：`margin="0.00015"`（0.15 mm）。
   - 接触刚度：`solref="0.002 1"`。
   - 摩擦：`friction="1.5 0.04 0.01"`。
   - 物体使用 MuJoCo 原生 `box`、`cylinder` 几何体，保证外形准确。

3. **避免双指自锁**
   - 排除左右手指之间的碰撞：

     ```xml
     <exclude body1="finger_left_link" body2="finger_right_link"/>
     ```

   - 这样可以避免分段凸包的切面互相摩擦，导致夹爪无法打开。

4. **修正夹爪宽度标定**
   - 实际物理开口范围改为 `0–57 mm`。
   - 抓取指令按物体短边减去约 4 mm 计算，由物理接触阻止继续闭合。

## 最终效果

- 手指与物体不再明显穿模。
- 接触表面保持约 `0.13 mm` 的正间距。
- 左右夹持更对称，夹爪可以正常打开。
- 网页端直接显示 MuJoCo 物理关节反馈，避免网页动画显示穿模。

## 主要文件

- `reBotArm_ros2_DM/src/rebotarm_mujoco/models/rebotarm_b601_colored.xml`
- `reBotArm_ros2_DM/src/rebotarm_bringup/description/meshes/*_finger_{front,mid,rear}.stl`
- `reBotArm_ros2_DM/src/rebotarm_agent/rebotarm_agent/rebotarm_mcp_server.py`
- `reBotArm_simulator-DM/public/js/ros/rebot-ros-ui.js`

修改后重新构建并重启：

```bash
cd reBotArm_ros2_DM
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rebotarm_mujoco rebotarm_agent
./scripts/start_rebot_mujoco_all.sh
```
