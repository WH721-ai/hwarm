# hw_arm_description

六轴机械臂的 URDF 描述包（参考 OpenArm v2.0 的 link/joint 结构）。

## 特点

- **link 关系与 joint 数值沿用 OpenArm v2.0**（`assets/robot/openarm_v2.0/config/arm`）
- **6 轴**：OpenArm joint1/2/4/5/6/7；大臂的"平转关节"(OpenArm joint3)取消，改为固定连接
- **固定法兰 `tool0`**：接在 link7 之后（z=0），方便 IK / 末端标定
- **暂不使用外观 mesh**：visual / collision 全部用圆柱占位

## visual / collision 的命名

每个条目都有 `name` 字段，生成的 URDF 里就是有意义的名字：

| 数据 | URDF 里的名字 | 含义 |
|---|---|---|
| `name: body` | `link1_body` | 连杆本体 |
| `name: housing` | `link1_housing` | **关节壳体**（恒为蓝色 0.2/0.45/0.75） |
| `name: flange` | `tool0_flange` | 末端法兰 |
| `name: collision` | `link1_collision` | 碰撞包络 |

`hw_emit_visuals` / `hw_emit_collisions` 的逻辑：

```xml
<visual name="${(link_name + '_' + item['name']) if 'name' in item
               else (link_name + '_visual' + str(index))}">
```

**没有 `name` 时自动退回 `link1_visual0`**，所以老数据不会坏。
凸分解后一个 link 会有多个碰撞块，那时用 `name: collision_0/1/2` 即可。

## base_link 是虚拟节点（无几何、无惯性）

`base_link` 只是一个坐标系原点，URDF 里是彻底空的：

```xml
<link name="base_link">
</link>
```

| 项 | 有没有 | 原因 |
|---|---|---|
| visual | ❌ | 底座属于"安装结构"，不属于机械臂本体 |
| collision | ❌ | 同上；这样 SRDF 里也不会再出现 base 相关碰撞对 |
| inertial | ❌ | **根 link 不能有惯性**（KDL/MoveIt 会警告 `root link ... has an inertia`） |

⚠️ 不要用"零尺寸圆柱 + 透明"的方式假装虚拟节点 —— 那仍会生成 `<visual>`/`<collision>` 元素，
且 `radius=0 / length=0` 违反 URDF 规范（部分引擎会炸）。
**正确做法就是从 `visuals.yaml` / `collisions.yaml` / `inertials/nominal.yaml` 里删掉 `base_link:` 条目**，
`hw_emit_link` 的 `source_name in xxx` 判断会自动跳过。

代价：底座那 1.14 kg 不再计入模型总质量（当前 4.2688 kg）。因为它是固定根，
本来就不影响任何关节力矩 —— 实测零位重力力矩删前删后完全一致。

## 结构（v2：照搬 OpenArm 的分层）

```
hw_arm_description/
├── config/                        # 数据层（5 个子目录，按功能分类）
│   ├── struct/                    #   ① 结构：谁连谁、叫什么
│   │   ├── topology.yaml          #      links[] 顺序 + chain[] 关节顺序 + parent/child/type
│   │   ├── name_mapping.yaml      #      规范名 ↔ 原始名（接 CAD 后填右列）
│   │   └── reference_points.yaml  #      装配参考点 / TCP（当前空）
│   ├── joint/                     #   ② 关节
│   │   ├── joint_origins.yaml     #      关节在哪 + 子坐标系怎么摆
│   │   ├── joint_axes.yaml        #      绕哪个方向转（当前恒为 z）
│   │   └── joint_limits.yaml      #      能转多少
│   ├── link/                      #   ③ 连杆外形
│   │   ├── visuals.yaml           #      视觉（每个 link 可多个：本体 + 关节壳体）
│   │   └── collisions.yaml        #      碰撞（比视觉小一圈，无壳体）
│   ├── inertials/nominal.yaml     #   ④ 惯性（质量 / 质心 / 惯量）
│   └── control/                   #   ⑤ 控制（当前空，等上机标定）
│       ├── control_gains.yaml     #      MIT kp/kd
│       └── friction.yaml          #      摩擦模型
├── urdf/                          # 模板层
│   ├── hw_arm.urdf.xacro          #   入口（读 config → 调 robot）
│   ├── utils/
│   │   ├── hw_arm_utils.xacro     #   原语：唯一写 XML 标签的地方
│   │   ├── hw_arm_arm.xacro       #   组件：串联臂（递归遍历 topology.chain）
│   │   ├── hw_arm_ee.xacro        #   组件：末端（当前空）
│   │   ├── hw_arm_sensor.xacro    #   组件：传感器（当前空）
│   │   └── hw_arm_robot.xacro     #   总装：读全部 YAML 并分发
│   └── ros2_control/
│       └── hw_arm.ros2_control.xacro   # 控制接口（当前空）
├── scripts/
│   ├── migrate_to_openarm_layout.py   # 旧 YAML → 新版分层（一次性迁移，可重跑）
│   ├── gen_frames.py                  # 【legacy】旧版生成器，仅作参考
│   └── gen_srdf.py                    # 采样生成 SRDF 自碰撞矩阵
├── srdf/hw_arm.srdf               # 自碰撞禁用矩阵（采样生成）
├── launch/display.launch.py
├── rviz/display.rviz
└── docs/                          # OpenArm 生态分析 + 迁移方案
```

数据流：

```
config/{struct,joint,link,inertials}  ──┐
                                        ├─→ xacro → hw_arm.urdf → (MoveIt / RViz / Pinocchio)
urdf/utils/{utils,arm,robot} ───────────┘
                                        └─→ gen_srdf.py → srdf/hw_arm.srdf
```

**当前为空但已占位的文件**（等对应数据到位就能填）：
`struct/reference_points.yaml`、`control/{control_gains,friction}.yaml`、
`urdf/utils/hw_arm_ee.xacro`、`urdf/utils/hw_arm_sensor.xacro`、`urdf/ros2_control/hw_arm.ros2_control.xacro`

## 关节参数（6 关节 ↔ 6 连杆，一一对应）

| 本臂 joint | 对应 OpenArm | parent → child | origin xyz (m) | origin rpy (rad) | axis | limit (rad) | effort |
|---|---|---|---|---|---|---|---|
| joint1 | joint1 | base_link → **link1** | (0, −0.0625, 0) | (0, 0, 0) | **0 0 1** | −1.3963 ~ 3.4907 | 40 |
| joint2 | joint2 | link1 → **link2** | (0, 0, 0.06) | (0, −1.571, 0) | **0 0 1** | −0.17453 ~ 3.3161 | 40 |
| joint3 | joint3(取消)+joint4 | link2 → **link3** | (0, −0.22, 0) | (3.142, 0, 3.142) | **0 0 1** | 0 ~ 2.4435 | 27 |
| joint4 | joint5 | link3 → **link4** | (0, −0.0955, 0) | (0, −1.571, 1.571) | **0 0 1** | −1.5708 ~ 1.5708 | 7 |
| joint5 | joint6 | link4 → **link5** | (0, 0, 0.1205) | (−1.571, 0, 1.571) | **0 0 1** | −0.7854 ~ 0.7854 | 7 |
| joint6 | joint7 | link5 → **link6** | (0, 0, 0) | (0, 1.571, −3.142) | **0 0 1** | −1.5708 ~ 1.5708 | 7 |
| tool0_joint | — | link6 → tool0 | (0,0,0) | (0,0,0) | — | fixed | — |

### 肘部转正：link3 绕自身 y 轴逆时针 90°

OpenArm 的 joint4（本臂 joint3，即"肘"）原本绕**竖直轴**转，动起来是"甩"而不是"折"，不像工业臂。

按需求把 **link3 及其下游前臂绕 link3 的局部 y 轴逆时针旋转 90°**（`scripts/gen_frames.py` 里的 `TWIST_J3_DEG = 90`）：

| | 修改前 | 修改后 |
|---|---|---|
| joint2 轴 | (−1, 0, 0) | (−1, 0, 0)（不变） |
| **joint3 轴** | **(0, 0, 1) 竖直** | **(1, 0, 0) 水平** |
| J2 与 J3 夹角 | 90°（垂直） | **0°（平行）** ← 工业臂特征 |
| joint4 轴 | (0, −1, 0) | (0, −1, 0) |

**验证**：只动 joint2 / joint3 时，末端 `x` 恒为 0 —— 整条臂在 **y-z 平面内**折弯，就像典型 6 轴工业臂。

```
joint2=0.8 joint3=0.0 → EE = (0, −0.366, 0.373)
joint2=0.8 joint3=0.6 → EE = (0, −0.428, 0.261)
joint2=0.8 joint3=1.2 → EE = (0, −0.415, 0.134)
```

实现：绕 `link3` 局部 y 轴（过 joint3 原点）把前臂整段刚性旋转 —— joint3 安装点不动，
joint4..joint6 的相对偏置与轴向整体跟着转，link3 之后的几何/惯性随之重新表达。

### 为什么没有 link7 / joint3_mount

早期版本为了保留 OpenArm 的 joint3（大臂平转）位置，插入了一个 **fixed 关节 `joint3_mount`**，
它把 `link2 → link3` 变成刚性连接。后果是：

- **link3 没有任何控制接口**（它是 fixed 的，拖不动）；
- 拖 `joint3` 滑条时转的是 **link4**，名字和视觉对不上。

现在把那个 fixed 关节去掉了，并把它的 **0.06625 m 偏置并进 joint3 的 origin**（0.06625 + 0.15375 = **0.22 m**），
同时把原 link3 的质量/质心/惯量**刚性合并进 link2**（平行轴定理），然后把后面的连杆名字整体前移一位：

```
旧: base_link → link1 → link2 ─(fixed joint3_mount)→ link3 → link4 → link5 → link6 → link7 → tool0
新: base_link → link1 → link2 ────────────────────→ link3 → link4 → link5 → link6 → tool0
```

**质量守恒验证**：合并前 5.4120 kg → 合并后 **5.4120 kg**（link2 由 0.2775 变为 1.3514）。

### 两条设计约定

1. **所有 revolute 关节的 `axis` 都是 `(0,0,1)`**，真实转动方向烘进 `origin` 的 `rpy` —— 这是 URDF 的标准写法（也是 SolidWorks 导出的默认做法），比"轴向量乱指"更好读、更好和 DH/几何参数对应。
   - 新旧两条链的**零位世界位姿完全相同**（已用 `scripts/gen_frames.py` 验证，误差 < 1e-6 m）；
   - 每个关节的 z 轴在世界系下仍指向原来的物理转轴。

2. **连杆本体与关节壳体分开建模**（几何与参数都分开）：
   - `config/links.yaml` 里的 `body`：两个关节之间的**连杆本体**（沿两关节连线，圆柱）；
   - `config/joints.yaml` 里的 `housing`：**关节壳体**（沿本关节 z 轴的短粗圆柱）。
   - 好处：以后换成真实 mesh 时，body 换 mesh、housing 保留；改关节尺寸不影响连杆，反之亦然。

### 坐标系重写是怎么做的

`scripts/gen_frames.py` 做的是：给每个 child link 引入一个朝向 `F_i`，满足 `F_i·e_z = 原轴向量 a_i`；
新旧两链满足 `T_new = F_{i-1}ᵀ · T_old · F_i`，级联后 `FᵢFᵢᵀ` 相互抵消，所以**世界位姿不变，只是每个 link 的坐标系被转了**。
因此连杆几何、质心、惯量张量都用 `F_i` 重新表达（惯量按 `Fᵀ I F` 正交变换）。

## 基座朝向：joint1 轴 = base_link 的 z 轴

按需求把整条臂绕 **joint1 的轴线**刚性旋转了 90°，使 **joint1 的转轴与 base_link 的 +z 重合**（标准"基座回转"）。

| | 修改前 | 修改后 |
|---|---|---|
| joint1 origin rpy | (1.571, 0, 0) | **(0, 0, 0)** |
| joint1 轴（世界系） | (0, −1, 0) | **(0, 0, 1)** |
| 重力方向 | — | base 的 −z（Pinocchio 默认 `(0,0,−9.81)`） |
| 其余关节 | — | **完全不变**（相对几何、限位一字未改） |

实现方式：`scripts/gen_frames.py` 里的 `BASE_UP = True` —— 先求 `G = R_align_z(a1)ᵀ`（把 joint1 的原轴转到 +z），
再把 `joint2..joint6` 的平移与轴向整体左乘 `G`，最后照原流程生成"每轴绕 z"的坐标系。
**整臂是刚性旋转，所以各关节之间的相对几何一点没变。**

### ⚠️ 一个几何事实：零位下臂是水平的

joint1 的转轴与臂身方向**垂直**（这是机械结构决定的，改不了）。所以一旦把 joint1 轴立成 z，
零位下臂身必然落在**水平面**内（沿 −y 伸出，高度 0.06 m）：

```
joint1 (0, -0.0625, 0)   joint4 (0, -0.378, 0.06)
joint2 (0, -0.0625, 0.06) joint5 (0, -0.4985, 0.06)
joint3 (0, -0.2825, 0.06) joint6 (0, -0.4985, 0.06)
```

**要让臂竖直向上，把它当成一个"姿态"而不是改 URDF**：`joint2 = +1.5708`（在限位 [−0.175, 3.316] 内）。
SRDF 里已加好这个姿态，MoveIt 里可直接调用：

```xml
<group_state name="upright" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="1.5708"/>
  ...
</group_state>
```

RViz 里拖 joint2 滑条到 +90° 也能立刻看到"立起来"的样子。

## 碰撞几何与 SRDF

### visual ≠ collision（这是必须的）

第一版把 visual 和 collision 写成同一套圆柱，结果**在零位就有 17 对碰撞体互相穿透**
（其中 `link1 ↔ link3` 还是**非相邻**的）—— 任何规划器都会认为初始位形非法。

现在分开：

| | visual | collision |
|---|---|---|
| 连杆本体 | 全长圆柱 | **两端各缩进一个间隙**（`gap = min(15 mm, 25% 长度)`） |
| 关节壳体 | 蓝色短粗圆柱 | **不参与碰撞**（否则相邻 link 恒重叠） |
| 半径 | 本体半径 | **本体半径 × 0.80**（碰撞包络应小于视觉模型） |
| 零长度 link（球形腕：joint5/joint6 共点） | 有 | **不生成**（没有体积，否则会和前臂误撞） |

### 采样验证结果（Pinocchio/coal，openarm 限位内）

| 项 | 修改前 | 修改后 |
|---|---|---|
| 零位相交对数 | **17**（含 1 对非相邻） | **2**（全是相邻对） |
| 随机/顶点采样中非相邻碰撞 | — | **0 / 2201 组** |

采样集 = 零位 + 200 组限位顶点 + 2000 组均匀随机，限位全部沿用 OpenArm：
`joint1 [−1.396, 3.491]`、`joint2 [−0.175, 3.316]`、`joint3 [0, 2.444]`、
`joint4 [−1.571, 1.571]`、`joint5 [−0.785, 0.785]`、`joint6 [−1.571, 1.571]`。

### SRDF（`srdf/hw_arm.srdf`）

由 `scripts/gen_srdf.py` 采样 3301 组位形自动生成（等价 MoveIt Setup Assistant 的做法）：

- **8 对** `reason="Adjacent"`（同一关节相连）
- **28 对** `reason="Never"`（采样中从未碰撞）
- **保留检查 0 对**

> ⚠️ **重要提醒**：当前是**简化几何**（圆柱占位），在 OpenArm 限位内任何位形都不会真实自碰撞，
> 所以 SRDF 把所有对都禁用了。**换成真实 mesh 之后必须重跑** `scripts/gen_srdf.py`，
> 否则会漏掉真实碰撞。

### 重新生成

```bash
# 1) 由原始轴向生成"每轴绕 z"的坐标系 + link/joint 分离的 YAML
python3 scripts/gen_frames.py

# 2) 展开 URDF
xacro src/hw_arm_description/urdf/hw_arm.urdf.xacro > /tmp/hw_arm.urdf

# 3) 采样生成 SRDF 自碰撞矩阵
python3 scripts/gen_srdf.py /tmp/hw_arm.urdf src/hw_arm_description/srdf/hw_arm.srdf
```

## 参考资料

- 与达妙生态另外两套开源臂（OpenArm / Panther）的逐层对比与参考价值：
  [`docs/COMPARE-OPENARM-PANTHER.md`](docs/COMPARE-OPENARM-PANTHER.md)

## 用法

```bash
cd ~/hw_arm/hw_arm_ws
colcon build --packages-select hw_arm_description
source install/setup.bash

# 看模型
ros2 launch hw_arm_description display.launch.py

# 只导出 URDF
xacro src/hw_arm_description/urdf/hw_arm.urdf.xacro > /tmp/hw_arm.urdf
```

## 已知简化

1. 没有 mesh（圆柱占位，半径/长度按 OpenArm 关节间距估算）
2. 没有 ros2_control 段（下一步加）
3. 没有 SRDF（自碰撞矩阵，下一步加）
