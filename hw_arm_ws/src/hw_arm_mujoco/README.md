# hw_arm_mujoco

hw_arm 的 MuJoCo 物理仿真。**只做仿真，不参与 ros2_control。**

```
├── launch/
│   ├── display.launch.py   ← ① URDF 验证（RViz，转换前必跑）
│   └── mujoco.launch.py    ← ③ 用 ros2 launch 起 MuJoCo
├── scripts/
│   ├── urdf_to_mjcf.py     ← ② URDF → MJCF 转换
│   └── launch_mujoco.py    ← ③ MuJoCo viewer + 力矩控制
└── mjcf/
    ├── hw_arm.xml          ← 生成物：机器人
    ├── scene.xml           ← 生成物：地面 + 光照 + 相机
    └── assets/*.stl        ← 生成物：从 hw_arm_description 复制
```

---

## 三步走

### ① 验证 URDF（**不能跳**）

```bash
ros2 launch hw_arm_mujoco display.launch.py
```

起 `robot_state_publisher` + `joint_state_publisher_gui` + `rviz2`，
拖滑条逐个关节检查。

**为什么必须做这一步**：MuJoCo 只吃「几何 + 惯性 + 关节链」，
它**不显示**关节轴、限位、origin 意图、惯性张量朝向。这些在 MJCF 里
全被烘焙成 `pos`/`quat` 数字，转换后才发现轴反了，几乎无法反查。

检查清单：

- [ ] 每个关节转动方向符合直觉（不是反的）
- [ ] 零位姿态与 CAD 装配一致
- [ ] 贴限位时没有明显穿模
- [ ] mesh 完整（缺失会在终端报 `Could not load resource`）
- [ ] Mass Properties 里质量/惯量不是 0 或天文数字

### ② 转换

```bash
python3 scripts/urdf_to_mjcf.py              # 转换 + 自检
python3 scripts/urdf_to_mjcf.py --check      # 只自检，不生成
python3 scripts/urdf_to_mjcf.py --motor-tmax # forcerange 用电机峰值(200/45/27/11/7/7)
```

转换器做的 8 件事（脚本头部有完整注释）：

1. xacro 展开
2. 文本自检：axis 归一化、限位合理、惯量正定 + 三角不等式、mesh 存在
3. `package://` → `assets/`，STL 复制过来（MuJoCo 不认 `package://`）
4. `mujoco.MjSpec.from_file(urdf)` —— MuJoCo 自带 URDF 导入器
5. 补 URDF 没有的物理参数：`armature` / `damping` / `frictionloss`
6. 加 6 个 `<motor>`（纯力矩），`forcerange` 取 `joint_limits.yaml` 的 effort
7. 加 keyframe `home`
8. 导出 `hw_arm.xml` + `scene.xml`，并自检（质量 / `nu` / `q=0` 重力矩）

### ③ 在物理世界里看

```bash
python3 scripts/launch_mujoco.py                     # 默认 hold
ros2 launch hw_arm_mujoco mujoco.launch.py mode:=gravity
```

---

## 两种模式（纯物理，无控制算法）

| mode | 说明 |
|---|---|
| `gravity`（默认） | `ctrl = 0`，机械臂在重力下自由下坠，撞到关节限位停住。底座不动 |
| `torque` | `ctrl` 交给 viewer 右侧 **Control 面板滑条**手动给每个关节 N·m（范围 = 额定扭矩）。**你自己当控制器** |

```bash
python3 scripts/launch_mujoco.py                     # gravity
python3 scripts/launch_mujoco.py --mode torque       # 手动给力矩
python3 scripts/launch_mujoco.py --headless --duration 2
```

其他开关：

```bash
--scene PATH        指定 scene.xml
--duration N        跑 N 秒（仿真时间）后退出
--headless          不开 viewer，只跑物理（CI / 远程验证）
--print-period N    打印周期（秒，仿真时间），默认 0.1
```

> ⚠️ **本包不含任何控制算法**（没有 PD、没有重力补偿、没有 IK、不读 `control_gains.yaml`）。
> 控制算法在 `hw_arm_algorithms`，由外部控制器产生力矩后喂进来。

---

## 底座是固定的（不会跟着掉）

URDF 里 `base_link` 通过 `world_joint`（fixed）挂在 `world` 上，
MuJoCo 把固定关节**合并进 world body**：

```
nbody = 7  =  world + link1 + link2 + link3 + link4 + link5 + link6
                ↑ base_link 的几何体在 world body 里，焊死
```

跑 `gravity` 模式可以验证：

```
$ python3 scripts/launch_mujoco.py --headless --mode gravity --duration 3
  最终 q = [0, -0.175, -0.0004, -0.0706, -0.7859, 0]
  底座几何体位移 = 0.000e+00 m   ← base_link 一格没动
```

想让整机倒下来，得给 `base_link` 加 `<freejoint>`（本包不做）。

---

## ⚠️ 自碰撞：零件之间**故意不碰**

CAD 导出的 visual mesh 本来就是**嵌套**的（`link1` 塞在 `base_link` 里，
重叠 **3 mm**）。直接拿它当碰撞体，会在 q=0 就产生 4 个接触点、
**上百万牛**的约束力，把 **joint1 彻底锁死**：

```
joint1 给 20 N·m，跑 2 秒
  开自碰撞:  q1 =   +0.01 deg   ← 完全不动 ❌
  关自碰撞:  q1 = +200.32 deg   ← 正常转到限位 ✅
```

（约束力 −20.0 N·m 正好抵消 ctrl = +20.0 N·m，所以看着"有力矩但不动"。）

**修法**（转换器已经自动做）：

```
机械臂 geom : contype=1, conaffinity=0     ← 零件之间互不碰撞
地面 geom   : contype=2, conaffinity=1     ← 但和机械臂会碰
```

碰撞判定是 `(contype1 & conaffinity2) || (contype2 & conaffinity1)`：

- 机械臂 vs 机械臂：`(1&0) || (1&0) = 0` → 不碰 ✓
- 机械臂 vs 地面：`(1&1) || (2&0) = 1` → 碰 ✓

实测六个关节都能被驱动（q=0 起始，给额定力矩 0.5 秒）：

| joint | 扭矩 | q 变化 | 判定 |
|---|---|---|---|
| j1 | 20 N·m | 194.0° | ✅ |
| j2 | 10 N·m | 92.5° | ✅ |
| j3 | 9 N·m | 113.2° | ✅ |
| j4 | 3.5 N·m | 90.8° | ✅ |
| j5 | 3 N·m | 45.3° | ✅ |
| j6 | 3 N·m | 91.2° | ✅ |

> 想恢复自碰撞（比如以后换成简化碰撞包络），把转换器里 `g.conaffinity = 0`
> 那行去掉、地面改回 `contype=1` 即可。

---

## 职责边界：一个仿真器 = 一个包

```
hw_arm_mujoco/          仿真环境 + 本体（★ 不含任何控制算法）
  mjcf/                 MJCF（几何/惯性/接触/执行器/地面）
  scripts/urdf_to_mjcf.py
  scripts/launch_mujoco.py   ctrl 原样喂给 MuJoCo

hw_arm_gazebo/          Gazebo 仿真 + 算法节点（验证算法用这里）
  src/gravity_node.cpp  直接 link hw_arm_algorithms::computeGravity

hw_arm_algorithms/      纯算法库（只依赖 Eigen + Pinocchio）
  computeGravity / computeFK / solveIK / ...
```

**为什么 MuJoCo 里不验证算法**

| | MuJoCo | Gazebo |
|---|---|---|
| 算法接入方式 | 跨语言（C API / ctypes） | **同进程直接 link C++ 库** |
| 和真机路径 | 差别大 | 和 `ros2_control` 一致 |
| 算法库纯净性 | 要加 C API 层 | **不用动算法库** |

所以：**算法验证放 Gazebo，MuJoCo 只做环境与对齐真机的物理场**。
两者没有代码依赖，`hw_arm_algorithms` 保持纯净。

---

## 电机力矩上限（额定扭矩）

`actuator_ctrlrange` / `forcerange` 都设成**额定扭矩**，所以：

- MuJoCo viewer 右侧 Control 面板的**滑条范围就是额定值**（不设置的话默认只有 ±1，没法做实验）
- 电机实际施加的力矩永远不会超过额定值

| joint | 电机 | **额定 (滑条范围)** | 峰值 | armature |
|---|---|---|---|---|
| joint1 | DM10010L | **±20 N·m** | 200 | 0.0100 |
| joint2 | DM6216P | **±10 N·m** | 45 | 0.0100 |
| joint3 | DM4340 | **±9 N·m** | 27 | 0.1600 |
| joint4 | DM4310P | **±3.5 N·m** | 11 | 0.0100 |
| joint5 | DM4310 | **±3 N·m** | 7 | 0.0100 |
| joint6 | DM4310 | **±3 N·m** | 7 | 0.0100 |

实测（指令给 999，看实际输出被裁到多少）：

```
joint1: 指令 999 → 实际输出 +20.00 N·m  ✅
joint2: 指令 999 → 实际输出 +10.00 N·m  ✅
joint3: 指令 999 → 实际输出  +9.00 N·m  ✅
joint4: 指令 999 → 实际输出  +3.50 N·m  ✅
joint5: 指令 999 → 实际输出  +3.00 N·m  ✅
joint6: 指令 999 → 实际输出  +3.00 N·m  ✅
```

> ⚠️ `data.ctrl` 本身不会被改写（MuJoCo 只在 `qfrc_actuator` 上裁剪），
> 但滑条拖不出 `ctrlrange`，所以指令从源头就被限制了。

想用**峰值**做实验（比如验证电机的瞬时能力）：

```bash
python3 scripts/urdf_to_mjcf.py --motor-tmax     # 滑条变成 ±200/45/27/11/7/7
```

> 注意这和 `joint_limits.yaml` 里的 `effort`（40/40/27/7/7/7）是两回事：
> `effort` 是 ros2_control 的安全上限，MuJoCo 这里用的是电机自己的额定/峰值。

---

## 终端输出：看每个电机的力矩

`gravity` / `zero` / `torque` 模式每 0.1 s（仿真时间）打印一张力矩表：

```
         |       j1       j2       j3       j4       j5       j6
  ── t= 0.201s
   q(deg)|    -0.00   -10.27    -0.02    -0.02    -5.79    -0.00
     ctrl|    0.000    0.000    0.000    0.000    0.000    0.000   外部给的
 qfrc_act|    0.000    0.000    0.000    0.000    0.000    0.000   实际输出
     mj_g|    0.000    2.553    2.536    0.014    0.188    0.000   MuJoCo 内部量
 qfrc_con|   -0.001    2.115    2.538    0.014    0.100    0.000   限位反力
```

| 行 | MuJoCo 字段 | 含义 |
|---|---|---|
| `ctrl` | `data.ctrl` | 外部下发的力矩（gravity 恒 0；torque 来自滑条） |
| `qfrc_act` | `data.qfrc_actuator` | 电机**真正输出**到关节的力矩（= ctrl 经 forcerange 限幅） |
| `mj_g` | `data.qfrc_bias` | MuJoCo 自己的重力+科氏力矩（**仅观察用**，不参与控制） |
| `qfrc_con` | `data.qfrc_constraint` | **限位/接触反力**（撞到限位后迅速涨到 ≈ `qfrc_bias`） |

### 怎么看

**自由下坠（`gravity`）**：`ctrl` 和 `qfrc_act` **全是 0** —— 电机不出力，
所以机械臂往下掉。掉到限位后 `qfrc_con` 立刻涨起来顶住（joint2 → 2.5 N·m，
joint5 → 0.1 N·m + 限位），`q` 就不动了。

**按住不动（`hold --show-torque`）**：三者相等，这是**重力补偿正确的标志**。

```
     ctrl|   -0.000    2.285    2.821    0.016    0.202    0.000
 qfrc_act|   -0.000    2.285    2.821    0.016    0.202    0.000
     mj_g|   -0.000    2.285    2.821    0.016    0.202    0.000
```

### 相关开关

```bash
--print-period N   打印周期（秒，仿真时间）：hold/teach 默认 1.0，其它 0.1
--show-torque      所有模式都打印力矩表（hold/teach 默认只打跟踪误差）
```

> 时间戳 `t` 是**仿真时间**，不是墙钟。`--headless` 跑得比实时快几十倍，
> 所以看数据时以 `t` 为准，不要用秒表。

---

## 手动拖关节（所有模式通用）

在 MuJoCo 窗口里：

1. **Ctrl + 左键双击**某个连杆 → 选中它（出现箭头）
2. **按住 Ctrl + 左键拖动** → 对选中的连杆施加力/力矩
3. 松开 → 力撤掉

脚本里已经接好了：

```python
if viewer.perturb.active:
    mujoco.mjv_applyPerturbPose(model, data, viewer.perturb, 0)
    mujoco.mjv_applyPerturbForce(model, data, viewer.perturb)
```

> ⚠️ `mujoco.viewer` 的 passive 模式**只更新** `viewer.perturb`，
> **不会**替你施加力。这一步必须自己调，否则鼠标拖了没反应。

两种手感：

| 模式 | 拖起来的感觉 |
|---|---|
| `gravity` | 有重力，松手掉回去 |
| `teach` | 失重，松手停在原地 ← **想"摆 pose"用这个** |

实测（给 link3 加 8 N 推力 0.6 s，然后松手 2.4 s）：

```
teach    松手后 q = [0, 0.098, 0.101, 0, 0, 0]     ← 停在拖到的位置
gravity  松手后 q = [0, -0.175, 0, -0.004, -0.243, 0]  ← 掉回限位
```

---

## 实测结果（纯物理）

自由下坠（`ctrl = 0`，默认 `gravity` 模式）：

```
$ python3 scripts/launch_mujoco.py --headless --duration 2

t=0.0s  q = [0,   0,   0,  0,   0,  0] deg
t=0.3s  q = [0, -10,   0,  0, -10,  0]      ← joint2 撞下限位(-0.1745)
t=0.9s  q = [0, -10,   0, -0.1, -35.7, 0]
t=1.2s  q = [0, -10,   0, -0.1, -45,  0]    ← joint5 撞上限位(-0.7854)，停住

[headless] 最终 q = [0, -0.175, -0.0004, -0.0006, -0.247, 0]
[headless] 底座几何体位移 = 0.000e+00 m     ← base_link 焊死在地面
```

撞限位后 `qfrc_con` 会立刻涨到 ≈ `mj_g`（限位在替电机顶住重力）：

```
t=0.3s  mj_g   = [0, 2.545, 2.543, 0.014, 0.190, 0]
        qfrc_con= [0, 2.533, 2.541, 0.013, 0.100, 0]
```

> 阻尼设为 0.05（不是 OpenArm motor class 的 1.0）：1.0 会让机械臂"渗"下去，
> 30 秒才到限位，看起来像卡住了。`damping` 只用于数值稳定。

---

## 与 Pinocchio 的一致性

URDF 直接导入 MuJoCo 后，两者算出的动力学完全一致（随机 5 个姿态）：

```
质量矩阵   MuJoCo vs Pinocchio CRBA :  差 0.16（来自 armature，Pinocchio 不计）
重力矩     qfrc_bias vs rnea        :  最大绝对误差 9.8e-06 N·m
总质量     MuJoCo 2.7513 kg  ==  Pinocchio computeTotalMass 2.7513 kg
q=0 link6 世界位置差                 :  7.3e-07 m
```

所以 MuJoCo 里调出来的东西可以放心搬回 Pinocchio / 真机。

---

## 已知参数缺口（TODO）

| 项 | 现状 | 待办 |
|---|---|---|
| joint1/joint2 的 `armature` | 占位 0.01 | 需要 DM10010L / DM6216P 的转子惯量 + 减速比，或 sysid |
| `damping` / `frictionloss` | 全局 1.0 / 0.1 | 已有实测 `fc`（j1 0.43, j2 0.23, j3 0.56, j4 0.10, j6 0.09），可填进 `frictionloss` |
| `forcerange` / `ctrlrange` | **额定扭矩**（20/10/9/3.5/3/3），滑条同范围 | 想用峰值就加 `--motor-tmax`（200/45/27/11/7/7） |
| 碰撞几何 | 直接用 visual 的 STL（很密） | 想加速可换简化包络；现在 6 个 mesh 跑 1 kHz 没问题 |

---

## 依赖

```
mujoco >= 3.6   （当前 4.0.1，pip 装的用户级包）
numpy
```

不需要 `mujoco_ros2_control`。要接 ros2_control 是另一条路线（C）。
