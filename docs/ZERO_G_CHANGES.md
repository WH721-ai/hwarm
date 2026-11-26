# 零重力拖拽示教 —— 代码改动报告

> 2026-11-26 · 从 C-05 参数辨识转入 `hw_arm_algorithms` 实测

---

## 0. 一句话总结

**为了让机械臂能"失重"被手推动，需要把 C-05 辨识出的重力参数喂进硬件插件，并加上四道安全机制。**
代码全部改完并验证参数链路打通；**物理上还差一步：机械臂必须自由站立**（当前被托住，测不出重力）。

---

## 1. 提交清单

| 提交 | 内容 | 文件数 |
|---|---|---|
| `8147fd1` | 重力参数接入运行时 + 零重力拖拽示教的安全机制 | 10 |
| `32aa50c` | 文档：退出时的 WARN 是正常产物 | 1 |
| `5f190c6` | 修：重力补偿参数根本传不到插件（三处断链） | 4 |
| `e442bff` | 修：参数链路真正打通 + 定位"时好时坏"根因 | 2 |
| `d93ea0d` | 文档：零重力测试必须先确认机械臂自由站立 | 1 |

**累计：11 个文件，+560 / −51 行**

```
algo/data/gravity_theta.yaml                       |  78 +++----
docs/C05_ZERO_G_TEACHING.md                        | 240 +++++++++++++
docs/ZERO_G_CHANGES.md                             | 本文件
hw_arm_ws/src/hw_arm_algorithms/CMakeLists.txt     |   1 +
.../include/hw_arm_algorithms/gravity_identify.hpp |  19 ++
.../src/hw_arm_algorithms/src/gravity_identify.cpp |  38 ++++
.../src/hw_arm_bringup/launch/hw_arm.launch.py     |  34 +++
.../src/hw_arm_description/scripts/preflight.sh    |  15 +-
.../src/hw_arm_description/urdf/hw_arm.urdf.xacro  |  16 ++
.../urdf/ros2_control/hw_arm.ros2_control.xacro    |  25 ++-
.../include/hw_arm_hardware/hw_arm_hardware.hpp    |  19 ++
.../src/hw_arm_hardware/src/hw_arm_hardware.cpp    | 126 +++++++-
```

---

## 2. ⭐ C-05 辨识参数的位置

### 2.1 文件

```
~/hw_arm/algo/data/gravity_theta.yaml        1429 字节
```

[gravity_theta.yaml](algo/data/gravity_theta.yaml)

**这个文件是"唯一真源"** —— 运行时重力补偿只认它，不读 URDF 的惯性参数。

### 2.2 文件结构

```yaml
theta:     60 个数   # 重力回归参数，10 个/连杆 × 6 连杆
offsets:    6 个数   # 每轴恒定力矩偏置（线束/零点/基座不水平）
friction:   6 个数   # 每轴库仑摩擦幅值
meta:
  urdf:        ...                       # 辨识时用的 URDF
  rank:        22                        # 有效秩（10 重力 + 6 偏置 + 6 摩擦）
  n_used:      24                        # 参与拟合的样本数
  rms:         0.1836                    # 训练集残差 N·m
  max_abs:     0.8371
  holdout_rms: 0.2210                    # ← 泛化精度，这才是关键指标
  param_layout: "per body 10: [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]"
```

### 2.3 θ̂ 的实际数值（按连杆分组，只列非零项）

| 连杆 | 非零参数 |
|---|---|
| link1 | 全 0（约 1e-8，机器零） |
| link2 | `m·cx = −0.6165`、`m·cy = +0.0833` |
| link3 | `m = +0.1708`、`m·cx = +0.1947`、`m·cy = +0.1091` |
| link4 | `m = +0.2199`、`m·cx = +0.0187`、`m·cy = +0.0358`、`m·cz = +0.1851` |
| link5 | `m = +0.2287`、`m·cx = +0.0337`、`m·cy = −0.0305`、`m·cz = +0.0223` |
| link6 | `m = +0.2308`、`m·cx = −0.0229`、`m·cy = −0.0022`、`m·cz = +0.0300` |

```
60 个参数里只有 21 个非零，||θ̂|| = 0.8123
```

**⚠️ 单个参数不具物理意义。** 只有 10 维子空间可辨识（见 C-05 分析），
最小二乘解会落在某个"规范"上 —— 例如 `link2` 出现 `m=0` 但 `m·cx=−0.62`，
这在物理上不成立，但**预测力矩是对的**（rms 0.184）。

**判断 θ̂ 好不好，只看 `meta.holdout_rms`，不要看单个参数是否合理。**

### 2.4 残余偏置与摩擦

```
offsets  = [-0.1506, +0.1227, -0.1046, +0.3314, -0.6328, -0.1309]  N·m
friction = [+0.0041, +0.0379, +0.0095, +0.0322, +0.0026, -0.0044]  N·m
```

`offsets` 是与位形无关的恒定力矩（joint5 的 −0.63 最大），
`friction` 是库仑摩擦幅值（都很小，说明抖动采集把静摩擦消掉了）。

---

## 3. θ̂ 是怎么产生、怎么被消费的

### 3.1 生产链（离线）

```
collect_static_gravity.py          采集 40 个样本（20 位形 × A/B 双向 + 每步抖动 3 次）
        ↓  algo/data/static_gravity_dither.yaml
gravity_id_cli                     最小二乘（SVD 可辨识子空间 + 正则化可选项）
        ↓  algo/data/gravity_theta.yaml     ← **C-05 的产物**
```

**复现命令：**

```bash
source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh
$HW_ARM_BIN/gravity_id_cli --urdf $HW_ARM_URDF \
    --samples ~/hw_arm/algo/data/static_gravity_dither.yaml \
    --out     ~/hw_arm/algo/data/gravity_theta.yaml
```

### 3.2 消费链（运行时）

```
gravity_theta.yaml
        ↓  hw_arm_algorithms::loadGravityParams()        gravity_identify.cpp:76
GravityParams gparams_{theta, offsets, friction}          gravity_identify.hpp:119
        ↓  HwArmHW::on_init()                            hw_arm_hardware.cpp:417
g = Y(q)·θ̂ + offset                                     gravity.cpp（computeGravityFromParamsAndOffset）
    + friction·tanh(dq/0.05)                             hw_arm_hardware.cpp:802
        ↓  × 斜坡 × gravity_scale，再逐轴限幅             hw_arm_hardware.cpp:811-819
tau_ff
        ↓  加到控制器的力矩上
τ_motor = τ_cmd + tau_ff                                 hw_arm_hardware.cpp:826
```

**关键设计：** `gravity_params_path` 给了就必须读成功，否则 `on_init` **直接 return false**。
绝不静默退回标称 URDF —— 那样使用者以为在用辨识参数（0.18 N·m），实际用的是 CAD 参数（0.88 N·m）。

### 3.3 精度对比（同一批真机数据）

| 模型 | rms | 泛化（留出位形） |
|---|---|---|
| 标称 URDF（CAD 惯性） | 0.884 | — |
| **辨识 θ̂** | **0.184** | **0.221** |
| 改善 | **4.8×** | — |

**用一个完全未参与辨识的实测点验证（00:40 那次静止读数）：**

| 轴 | 实测 | 标称误差 | **θ̂ 误差** |
|---|---|---|---|
| joint1 | −0.83 | −0.83 | −0.68 |
| joint2 | −4.94 | +0.56 | +0.67 |
| **joint3** | **6.24** | **+2.19** | **+0.11** |
| joint4 | 0.02 | +0.05 | −0.07 |
| joint5 | 0.40 | −0.18 | +0.23 |
| joint6 | −0.10 | −0.07 | −0.13 |
| **整体 rms** | | **0.987** | **0.407** |

**joint3 从 +2.19 降到 +0.11 —— 辨识确实抓住了真实偏差。**
joint1 的 −0.68 仍在（竖直轴重力恒为 0，θ 建模不了，符合预期）。

---

## 4. 代码改动明细（按文件）

### 4.1 `hw_arm_algorithms` —— 新增读参数能力

| 文件 | 行 | 内容 |
|---|---|---|
| [gravity_identify.hpp](hw_arm_ws/src/hw_arm_algorithms/include/hw_arm_algorithms/gravity_identify.hpp#L119) | **119–140** | `struct GravityParams` + `loadGravityParams()` 声明 |
| [gravity_identify.cpp](hw_arm_ws/src/hw_arm_algorithms/src/gravity_identify.cpp#L76) | **76–112** | 实现：yaml-cpp 读回 theta/offsets/friction/meta |
| [CMakeLists.txt](hw_arm_ws/src/hw_arm_algorithms/CMakeLists.txt#L57) | **57** | `yaml-cpp` 提到库的 PUBLIC 依赖 |

### 4.2 `hw_arm_hardware` —— 核心

**[hw_arm_hardware.hpp](hw_arm_ws/src/hw_arm_hardware/include/hw_arm_hardware/hw_arm_hardware.hpp#L166)**

| 行 | 内容 |
|---|---|
| **166–180** | 拖动示教的注释块 + 4 道保险的说明 |
| **174** | `gravity_params_path_` |
| **175** | `GravityParams gparams_` |
| **176–177** | `gravity_ramp_time_` / `ramp_t0_` |
| **178** | `tau_limit_ = {120, 45, 28, 10, 7, 7}` |
| **179–180** | `zero_gravity_vel_limit_` / `vel_guard_count_` |

**[hw_arm_hardware.cpp](hw_arm_ws/src/hw_arm_hardware/src/hw_arm_hardware.cpp)**

| 行 | 内容 | 说明 |
|---|---|---|
| **29–33** | `steady_now()` | 斜坡用的单调时钟 |
| **41–52** | `to_bool` 兼容 `True/False` | xacro 渲染出的是 Python 风格 |
| **348–383** | 解析 4 个新参数 + 校验 | `tau_limit` 必须 6 个正数 |
| **417–432** | `on_init` 加载 θ̂，失败即 return false | 保险① |
| **436–441** | 未指定参数时的 WARN | |
| **791–799** | `write()` 用 θ̂ 算 g | 保险① |
| **802–807** | 库仑摩擦 `f_c·tanh(dq/0.05)` | 平滑，静止不抖 |
| **811–819** | 斜坡 + 逐轴力矩限幅 | 保险②③ |
| **826** | `τ_motor = τ_cmd + tau_ff` | |
| **843–850** | 速度守护：超速 kd×4 当刹车 | 保险④，**不去激活** |
| `on_activate` | `ramp_t0_ = steady_now()` | 保险② |

### 4.3 参数链路（xacro / launch）

**[hw_arm.urdf.xacro](hw_arm_ws/src/hw_arm_description/urdf/hw_arm.urdf.xacro)**

| 行 | 内容 |
|---|---|
| **32–38** | 7 个 `<xacro:arg>`（默认全关，**不改现有行为**） |
| **79–86** | 宏调用补上这 7 个参数 |

**[ros2_control/hw_arm.ros2_control.xacro](hw_arm_ws/src/hw_arm_description/urdf/ros2_control/hw_arm.ros2_control.xacro)**

| 行 | 内容 |
|---|---|
| **51–59** | 宏 `params` 列表补 7 个 |
| **106–115** | `<param>` 用 `${...}`（宏参数），`gravity_params_path` 空值不发射 |
| **115** | `tau_limit` |
| **128–130** | `zero_gravity` / `zero_gravity_kd` / `zero_gravity_vel_limit` |

**[hw_arm.launch.py](hw_arm_ws/src/hw_arm_bringup/launch/hw_arm.launch.py)**

| 行 | 内容 |
|---|---|
| **47–54** | 7 个 `LaunchConfiguration(...).perform(context)` |
| **64–70** | `Command([...])` 补 7 个 `" gravity_X:=", gX` |
| **127–141** | 7 个 `DeclareLaunchArgument` |

### 4.4 工装

**[preflight.sh](hw_arm_ws/src/hw_arm_description/scripts/preflight.sh#L87)**

| 行 | 内容 |
|---|---|
| **31** | 新增 `count_rsp()` |
| **87–101** | 残留 `robot_state_publisher` > 1 判为**致命**；`ros2 launch` 重复也报错 |

### 4.5 文档

| 文件 | 内容 |
|---|---|
| [C05_ZERO_G_TEACHING.md](docs/C05_ZERO_G_TEACHING.md) | 操作手册：自由站立自检 / 三模式对比 / 四道保险 / 分三阶段 / kd 手感表 / 已知限制 / 退出 WARN |
| 本文件 | 改动报告 + θ̂ 位置 |

---

## 5. 四道安全机制的对应位置

| # | 机制 | 参数 | 默认 | 代码 |
|---|---|---|---|---|
| ① | 用辨识参数 | `gravity_params_path` | 空 = 标称 | `hw_arm_hardware.cpp:417` 加载 / `:791` 使用 |
| ② | 斜坡 | `gravity_ramp_time` | **3.0 s** | `:811` + `on_activate` |
| ③ | 力矩限幅 | `tau_limit` | `40 10 12 3.5 3 3` | `:818` |
| ④ | 速度守护 | `zero_gravity_vel_limit` | 2.0 rad/s | `:843` |

**④ 的要点：超速时把 `kd` 提到 4 倍当刹车，而**不是**去激活。**
去激活会变成自由落体，比溜车更危险。

---

## 6. 过程中修掉的两个"我自己的 bug"

### 6.1 `str.replace` 静默 no-op

```
用 Python 的 str.replace 改 C++ 源码
        ↓
锚点没匹配上（缩进/上下文变了）
        ↓
**静默什么都没改**，而脚本还打印了 "✅"
        ↓
表现：参数解析块根本没进代码，插件一直用默认值
```

**教训：改代码必须 `grep` 验证。** 后续改为按行定位 + `assert`。

### 6.2 残留 `robot_state_publisher` 覆盖参数

```
ros2_control_node 对 /robot_description **只取第一条**收到的描述
        ↓
桌面残留 4 个 rsp，一直发**旧的** URDF
        ↓
参数**时好时坏**：这次 True 下次 False，path 时有时无
        ↓
日志里没有任何错误
```

**已修：`preflight.sh` 把「rsp 数量 > 1」判为致命。**

---

## 7. 当前状态与阻塞点

```
✅ 参数链路全通
   日志确认：重力参数 rank=22 rms=0.1836 holdout=0.2210 偏移 6 / 摩擦 6
✅ dry-run 能算出 tau_ff
   g(q)=[-0.150 2.421 6.056 -0.141 0.094 0.040]
❌ 但实测 effort = [-0.147 0.051 0.034 -0.022 -0.012 -0.007]
   joint2 差 2.37、joint3 差 6.02 N·m
```

**根因（已用实验确认）：机械臂被托住了**

```
关掉重力补偿、纯 kp=70 位置控制，使能 15 秒：
    q 一个数都没变（-0.0017 → -0.0017）
    effort ≈ 0.05（量化台阶 0.034，基本是 0）
        ↓
有重力就一定会下垂 g/kp = 2.421/70 = 0.0346 rad = 2°
        ↓
**没有下垂 ⇒ 重力作用不到关节上 ⇒ 被托住**
```

**解锁条件：让机械臂自由站立，joint2 应可见地下垂约 2°。**

---

## 8. 下一步

| 步骤 | 命令 | 验收 |
|---|---|---|
| ① 自由站立自检 | 只起 `joint_state_broadcaster,hw_arm_forward_position_controller`（**不加**重力补偿），松手 10 s | joint2 下垂 1.5~2° |
| ② 阶段 1（dry-run） | `gravity_compensation:=true gravity_dry_run:=true` + θ̂ 路径 | `tau_ff` 与 `effort` 各轴差 ≤ 0.4 N·m |
| ③ 阶段 2（重力补偿） | `gravity_dry_run:=false gravity_scale:=0.5` → 1.0 | joint2 稳态下垂 12.6 mrad → 2.6 mrad |
| ④ 阶段 3（零重力） | `zero_gravity:=true`，**只起 broadcaster** | 松手停住、手推可动、松手不振荡 |

**每次测试前先跑 `preflight.sh`，五项必须全 ✅。**
