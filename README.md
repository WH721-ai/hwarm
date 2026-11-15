# hw_arm — 6 轴机械臂软件栈

自研 6 轴机械臂的完整软件栈：**CAD 描述 → 动力学算法 → ros2_control 硬件驱动 → 两种仿真器 → 参数辨识**。

目标场景：桌面级 6 轴臂，达妙（DAMIAO）DM 系列电机 + CAN/CANFD，ROS 2 Jazzy，Pinocchio 动力学。
精度目标 0.1 mm，当前阶段做**模型已知的重力补偿**（`tau_ff = RNEA(q, 0, 0)`）。

```
hw_arm/
├── hw_arm_ws/                     ROS 2 工作空间
│   └── src/
│       ├── hw_arm_description/    数据驱动的 URDF/xacro + SRDF + mesh
│       ├── hw_arm_algorithms/     ★ 纯算法库（C++ / Pinocchio）
│       ├── hw_arm_hardware/       ros2_control SystemInterface（CAN）
│       ├── hw_arm_bringup/        launch + controller 配置
│       ├── hw_arm_mujoco/         MuJoCo 仿真环境（不含算法）
│       ├── hw_arm_gazebo/         Gazebo 仿真 + 算法节点
│       └── openarm_can/           第三方 CAN 驱动（git submodule）
├── algo/                          离线算法与辨识工具箱（Python）
└── reference/                     参考用的第三方代码（不入库）
```

---

## 一、这套代码在做什么

| 层 | 负责 | 代码 |
|---|---|---|
| 描述 | 从 CAD 导出 URDF，数据驱动（YAML）生成 xacro | `hw_arm_description` |
| 算法 | 重力补偿、FK/Jacobian/IK、奇异度 | `hw_arm_algorithms` |
| 驱动 | CAN 收发 + MIT 模式力矩指令 + 状态反馈 | `hw_arm_hardware` |
| 运行时 | controller_manager + JTC + launch | `hw_arm_bringup` |
| 仿真 A | MuJoCo：物理环境 + 对齐真机的参数 | `hw_arm_mujoco` |
| 仿真 B | Gazebo：同进程 link 算法库，验证算法 | `hw_arm_gazebo` |
| 辨识 | 激励轨迹设计 + 最小二乘参数辨识 | `algo/src/m30_sysid` |

**架构原则**

- **算法只写一次**（`hw_arm_algorithms`），仿真器和真机都是它的消费者
- **一个仿真器 = 一个包**，仿真包不含控制算法
- **描述数据驱动**：改 CAD 只需重跑导入脚本，不碰 xacro

---

## 二、快速开始

### 依赖

```bash
# ROS 2 Jazzy + Pinocchio + Eigen
sudo apt install ros-jazzy-pinocchio ros-jazzy-eigen3-cmake-module
# MuJoCo（可选，仅仿真）
pip install mujoco>=3.6
# Gazebo（可选）
sudo apt install ros-jazzy-ros-gz
```

### 编译

```bash
git clone --recursive <this-repo> hw_arm
cd hw_arm/hw_arm_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

> `openarm_can` 是 git submodule，用 `--recursive` 克隆或
> `git submodule update --init --recursive`。

### 跑测试

```bash
colcon test --packages-select hw_arm_algorithms
colcon test-result --verbose        # 32 个 gtest
```

### 看模型

```bash
# ① 先验证 URDF（RViz）
ros2 launch hw_arm_mujoco display.launch.py

# ② MuJoCo 物理场（自由下坠 / 手动给力矩）
python3 src/hw_arm_mujoco/scripts/urdf_to_mjcf.py
python3 src/hw_arm_mujoco/scripts/launch_mujoco.py
```

---

## 三、`hw_arm_algorithms`（核心）

纯算法库，**只依赖 Eigen + Pinocchio**，无 rclcpp / hardware_interface 依赖 ——
可以被实时线程、离线脚本、单元测试、将来的代码生成共同复用。

```cpp
#include "hw_arm_algorithms/model.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/kinematics.hpp"

hw_arm_algorithms::RobotModel model;
model.load("hw_arm.urdf");                     // 加载后自动校验
hw_arm_algorithms::RobotWorkspace ws(model);   // 每线程一份，预分配

// 重力补偿：tau_ff = RNEA(q, 0, 0)
Eigen::VectorXd g;
hw_arm_algorithms::computeGravity(model, ws, q, g);

// 运动学
pinocchio::SE3 pose;
hw_arm_algorithms::computeFK(model, ws, q, frame, pose);
hw_arm_algorithms::solveIKMultiSeed(model, ws, frame, target, opt, result);
```

| 文件 | 内容 |
|---|---|
| `model.hpp` | `RobotModel`：加载 + 校验（关节名/质量/末端帧） |
| `data.hpp` | `RobotWorkspace`：Pinocchio Data + 预分配缓冲 |
| `gravity.hpp` | `computeGravity` / `computeGravityMotor`（含 `direction ±1`） |
| `kinematics.hpp` | `computeFK` / `computeJacobian`（4 种参考系）/ `solveIK`（DLS）/ `computeManipulability` |

---

## 四、`hw_arm_description`（数据驱动）

改 CAD 之后**只改 YAML**，xacro 一行不用动：

```
config/
├── struct/topology.yaml        链结构（谁接谁）
├── joint/joint_origins.yaml    原点 xyz + rpy
├── joint/joint_axes.yaml       转轴
├── joint/joint_limits.yaml     限位 / effort
├── link/visuals.yaml           visual 几何（mesh）
├── link/collisions.yaml        collision 几何
├── inertials/nominal.yaml      质量 / 质心 / 惯量
└── control/{control_gains,friction}.yaml
```

配套脚本：

```bash
scripts/import_plugin_urdf.py      # CAD 插件导出的 URDF → 上面这些 YAML
scripts/gen_srdf.py               # 采样生成 SRDF 自碰撞矩阵
```

---

## 五、两个仿真器

| | `hw_arm_mujoco` | `hw_arm_gazebo` |
|---|---|---|
| 定位 | 物理环境 + 对齐真机 | 仿真 + **算法验证** |
| 算法接入 | ❌ 不接（纯环境） | ✅ 同进程 link `hw_arm_algorithms` |
| 用途 | 看几何/惯性/接触，手动给力矩 | 验证重力补偿端到端闭环 |

MuJoCo 侧的两个关键设置（都是踩过坑的）：

```xml
<!-- ctrlrange = 额定扭矩 → viewer 滑条范围正确 -->
<general joint="joint1" ctrlrange="-20 20" ctrllimited="true"/>

<!-- 零件之间不自碰撞（CAD mesh 本来就是嵌套装配，直接当碰撞体会锁死关节） -->
<geom ... conaffinity="0"/>
```

---

## 六、已知限制

| 项 | 现状 |
|---|---|
| 夹爪 | ❌ URDF 里没有末端执行器 |
| 手眼标定 | ❌ 未做（待办 C-21） |
| 质量参数 | ⚠️ CAD 只含外壳，缺电机/减速器，重力矩偏小 1.4~1.8 倍 |
| 摩擦 | ⚠️ MuJoCo 的 `frictionloss` 表达不了静摩擦死区 |
| 实时性 | 250 Hz，未做 `SCHED_FIFO` / `mlockall` |
| 力控 | ❌ 未做（待办 C-16/19/20） |

完整待办见 [`algo/docs/90_附录/Appendix_C_待办清单.md`](algo/docs/90_附录/Appendix_C_待办清单.md)。

---

## 七、许可

本仓库自有代码：Apache-2.0。
`hw_arm_ws/src/openarm_can` 为第三方（Enactic, Apache-2.0），通过 submodule 引入。
