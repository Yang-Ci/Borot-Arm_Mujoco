# ReBot Arm DM 上色版（独立可分享包）

本文件夹是一份**自包含**的上色版机械臂资源。URDF 内的 mesh 引用均为相对路径（`meshes/...`），任何支持相对路径的加载器（three.js URDFLoader、RViz、MuJoCo、PyBullet）均可直接使用。

## 文件结构

```
description/
├── ReBot_Arm_DM.urdf           # 唯一上色 URDF（visual 按材质分件 + 顶层 <material>）
└── meshes/
    ├── colored_*_<material>.stl # 23 个按材质拆分的手臂分件 STL（visual）
    ├── split_meshes/...         # 夹爪分件 STL（visual）
    └── *.STL                    # 7 个整件碰撞网格 STL（collision）
```

- 44 个 mesh 引用（37 个小写 `*.stl` + 7 个大写 `*.STL`）全部在本文件夹内，无外部依赖。
- **颜色全部定义在 URDF 顶层 `<material>`，不改动任何 STL**，换色只需编辑 URDF。

## 颜色对照表（当前配色）

| 材质名 | RGBA | 实体 |
| --- | --- | --- |
| `anodized_grey` | 0.720 0.760 0.730 | 铝结构件（CNC 银色，螺丝孔所在件）|
| `hardware_black` | 0.055 0.065 0.060 | 螺丝/连接件（黑）|
| `matte_black` | 0.14 0.16 0.15 | 主体壳体（哑光黑）|
| `silver_trim` | 0.72 0.76 0.73 | 银边装饰 |
| `seeed_yellow` | 0.73 0.84 0.12 | 明黄色点缀 |
| `gripper_finger_black` | 0.090 0.106 0.102 | 夹爪手指 |
| `gripper_carriage_grey` | 0.239 0.278 0.271 | 夹爪滑块 |
| `gripper_rack_metal` | 0.682 0.710 0.694 | 齿条金属 |
| `gripper_seeed_yellow` | 0.725 0.835 0.118 | 夹爪黄色件 |
| `gripper_hardware_black` | 0.063 0.071 0.067 | 夹爪螺丝件 |
| `gripper_base_metal` | 0.720 0.760 0.730 | 夹爪底座金属 |

## 三种引擎引用方法

### 1. Web（three.js URDFLoader）

```js
const loader = new URDFLoader();
// basePath 指向本 description 文件夹，或让 URDF 相对 URL 可用
loader.load('ReBot_Arm_DM.urdf', robot => scene.add(robot));
```

注意：URDFLoader 会给每个材质设 `material.name = <material name>`，可用它表映射 metalness/roughness 光泽感：

```js
{
  matte_black: { roughness: 0.48, metalness: 0.48 },
  hardware_black: { roughness: 0.20, metalness: 0.82 },
  anodized_grey: { roughness: 0.34, metalness: 0.72 },
  seeed_yellow: { roughness: 0.34, metalness: 0.18 },
  silver_trim: { roughness: 0.18, metalness: 0.88 },
}
```

### 2. RViz

把本文件夹拷到任意 catkin/colcon 包内，替换 URDF 里 `meshes/` 为 `package://<你的包>/description/meshes/`，或直接用绝对路径加载。

### 3. MuJoCo

用 `load_model_from_path`（mujoco>=3.x）直接加载 URDF：
```python
import mujoco
m = mujoco.MjModel.from_xml_path("ReBot_Arm_DM.urdf")  # 需编译 URDF 支持
```
或用本仓库的 MJCF 版 `rebotarm_b601_colored.xml`（同一颜色体系，带光泽/金属度参数）。

## 如何修改颜色

只改 URDF 里 `<material name="...">` 的 `<color rgba="..."/>`，三端（web/RViz/MuJoCoMJCF）自然生效；JS/XML 里的光泽参数保持不动即可。

## 来源

- 从 reBotArm 工程的 `reBotArm_ros2_DM/src/rebotarm_bringup/description/` 复制而来
- web 模拟器：`reBotArm_simulator-DM/`（server.js 已指向该 URDF）
- 颜色加工工具：根目录 `scripts/extract_dae_colors.py` / `scripts/unify_arm_urdf.py`（本文件是它们的输出）

明白了，是对比"整个项目早期怎么上色的" vs "现在"。核心差异：
最初（DAE 烘焙 + 混搭）
部位	上色方案
手臂（base_link~link6）	DAE 文件内烘色：colored_linkN.dae 网格自带材质色，URDF 只引用 DAE 文件，不加 color
夹爪（gripper）	拆分 STL + URDF <material>：颜色写在 URDF 标签里
Web 端	以上都读不到 → 用 REAL_FINISH_GROUPS 硬编码三角形面索引，按 [材质索引, 三角形数] 把 STL 的面涂色
现在（统一为 URDF <material> 单一来源）
部位	上色方案
手臂	拆分 STL + URDF <material>：把 DAE 里的颜色提取成 rgba 写进 URDF 顶层 material（extract_dae_colors.py 干的）
夹爪	不变（本来就是这套）
Web 端	颜色直接从 URDF material 读（URDFLoader 给 mesh 设定名称）→ 删掉了面索引硬编码死代码，只留光泽参数表
MuJoCo	material rgba 与 URDF 完全同源
本质区别
1. 颜色存放位置：DAE 二进制内嵌（专用容器、难以读）→ 纯文本 URDF 标签（任何人可见可改）
2. 驱动方式：改色需重新导 DAE → 改 <color rgba> 一行即换色
3. 文件格式：DAE（COLLADA，体积大、易碎）→ STL（轻量通用）
4. 代码 hack → 数据驱动：web 端从"面索引导死表"变成"URDF 数据 + 光泽映射表"
5. 兼容性：旧方案只有 RViz 能正确显示颜色（MuJoCo/URDFLoader 都读不到 DAE 色），新方案三端一致
一句话：从"颜色藏在 DAE 网格里、各端各想各法"变成了"颜色是 URDF 的一等公民、所有人看同一份数据"。
