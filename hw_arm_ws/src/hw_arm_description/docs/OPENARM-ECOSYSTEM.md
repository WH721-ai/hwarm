# OpenArm 生态：10 个仓库分别是什么

> 目录：`/mnt/hgfs/VMshare/OpenArm/`（只读）
> 归属：Enactic, Inc.（达妙生态的整机品牌），Apache-2.0

## 一句话结构

```
            ┌────────────── openarm-main（总仓库 + 文档站）
            │
 硬件 ──────┼── OpenArm-hardware（SolidWorks 全模型）
            │
 模型 ──────┼── openarm_description（URDF/xacro）
            │        └── openarm_ros2（ROS2 集成 + MoveIt2 配置）
            │
 驱动 ──────┼── openarm_can（C++ SocketCAN 库）
            │        └── openarm_teleop（遥操作：重力补偿 + 双边控制）
            │
 数据 ──────┼── dora-openarm（dora-rs 采集节点）
            │        └── openarm_dataset（LeRobot/IsaacLab 数据集工具）
            │
 仿真 ──────┴── openarm_mujoco（MJCF） / openarm_isaac_lab（Isaac Lab 环境）
```

---

## 逐个说明

### 1. `openarm-main` —— 总仓库 / 文档站
- **是什么**：整个 OpenArm 项目的入口仓库，主体是 `website/`（Docusaurus 文档站，即 docs.openarm.dev 的源码）+ 项目说明。
- **内容**：产品介绍（$6500 双臂系统）、装配指南、软件文档入口、贡献指南。
- **对我们**：**查资料的入口**。硬件的安装步骤、引脚、CAN 配置都在这套文档里。

### 2. `OpenArm-hardware` —— 机械硬件
- **是什么**：SolidWorks 源文件（**89 个 SLDPRT + 21 个 SLDASM**，110 个文件），含 `DM-J4340_1(urdf).SLDPRT` 这类与 URDF 对应的零件。
- **对我们**：机械结构参考（关节怎么布置、电机怎么装）。若要复刻或改结构，这是唯一的真值源。

### 3. `openarm_description` —— URDF/xacro（**我们已经深度用过**）
- **是什么**：机器人描述包，`assets/{robot,end_effector,sensor}` + `config/{struct,joint,link,…}` + `robot_presets`。
- **内容**：v1.0 / v2.0 两代；双臂/单臂 preset；夹爪（pinch_gripper / parallel_link）；ZED 相机挂载。
- **对我们**：**`hw_arm_description` 的模板**。数据驱动结构、housing/body 分离、preset 机制都抄自这里。

### 4. `openarm_ros2` —— ROS 2 集成 + MoveIt2
- **是什么**：ROS 2 包集合：`openarm/`（硬件接口配置与 launch）+ `openarm_bimanual_moveit_config/`（MoveIt2 配置）。
- **对我们**：**接 MoveIt2 的直接参考**——`ros2_controllers.yaml`（`update_rate: 200`）、`moveit_controllers.yaml`、launch 组织方式。

### 5. `openarm_can` —— C++ CAN 通信库（**最可直接复用**）
- **是什么**：独立 C++ 库，`include/openarm/` + `src/openarm/` + `examples/`，走 **SocketCAN（CAN / CAN-FD）**，封装达妙电机协议；还带 deb/rpm 打包。
- **对我们**：**ros2_control 硬件插件的地基**。我们不用从零写 CAN 帧打包和电机状态机，直接链接它。

### 6. `openarm_teleop` —— 遥操作
- **是什么**：C++ 控制节点：`gravity_compensation.cpp`、`openarm_unilateral_control.cpp`、`openarm_bilateral_control.cpp`、`openarm_communication_test.cpp`，配 `leader.yaml` / `follower.yaml`。
- **对我们**：**主从 + 重力补偿 + 力反馈**的实现参考。我们做拖动示教/遥操作时最有价值的一份代码。

### 7. `dora-openarm` —— 数据采集节点（dora-rs）
- **是什么**：一个 **dora-rs**（不是 ROS）dataflow 节点，`--side right/left`、`--align-trigger gripper`，用于组装"相机 + 臂 + 夹爪"的采集流水线。
- **对我们**：了解"非 ROS 的数据采集架构"长什么样；如果以后做大规模采集，dora 比 ROS 2 轻。

### 8. `openarm_dataset` —— 数据集工具（**我们之前看过**）
- **是什么**：Python 包 `openarm_dataset`：读写/转换/校验/上传，支持 **LeRobot v2.1 / v3.0**、Isaac Lab、RRD 等格式。
- **对我们**：以后做模仿学习时，数据格式就用它的（LeRobot 是当前事实标准）。

### 9. `openarm_mujoco` —— MuJoCo 仿真
- **是什么**：MJCF 模型（v0.3 / v1 / v2），pip 包 `openarm-mujoco`，`openarm-mujoco-launch` 直接起仿真。
- **对我们**：**最快的动力学/控制验证环境**（不用装 Isaac）。MuJoCo 的 `mj_inverse` 可以当第二个重力补偿对拍基准。

### 10. `openarm_isaac_lab` —— Isaac Sim / Isaac Lab 环境
- **是什么**：Isaac Sim 5.1 + Isaac Lab 2.3 的训练环境（reach / lift cube / open drawer），面向 RL / IL / sim-to-real。
- **对我们**：**重装备**（要 GPU + Omniverse）。现阶段用不上，做 RL 时再回来看。

---

## 三条对我们最有价值的

| 优先级 | 仓库 | 用途 |
|---|---|---|
| ★★★★★ | `openarm_can` | 直接当 ros2_control 插件的 CAN 层，省掉协议调试 |
| ★★★★☆ | `openarm_ros2` | MoveIt2 + 控制器的配置模板 |
| ★★★★☆ | `openarm_teleop` | 重力补偿 + 主从控制的工程实现 |

## 一个提醒

这 10 个仓库是**同一个产品的完整闭环**（硬件 → 描述 → 驱动 → 遥操作 → 采集 → 仿真 → 训练），
我们的 `hw_arm` 目前只覆盖"描述"这一层，后面按 `openarm_ros2` → `openarm_can` → `openarm_teleop` 的顺序补齐最省力。
