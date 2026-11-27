# PC 端控制与通信框架：四层分工与数据流

> 回答三个问题：① PC 端框架怎么分层；② Pinocchio / MoveIt2 / BT 各在哪一层干什么；
> ③ "决策层"到底是什么算法框架。
> 依据：教程 M12（ros2_control）、M13（BehaviorTree）、M14（MoveIt2+MTC）、M07（OMPL）、M10（时间参数化）、F05（导纳）。

---

## 1. 四层架构（自顶向下）

```
┌─ ① 决策层（Task / Behavior）  10–20 Hz，事件驱动，非实时 ─────────────┐
│  BehaviorTree.CPP（或 py_trees）＋ 少量服务/动作调用                  │
│  职责：先抓哪个？失败重试几次？抓不到怎么办？——"做什么"                │
│  输出：Action Goal（/move_action、/follow_joint_trajectory、夹爪、IO） │
└──────────────────────────┬─────────────────────────────────────────┘
                           ▼ Goal
┌─ ② 规划层（Motion Planning）  事件驱动，100 ms–2 s，允许失败 ─────────┐
│  MoveIt2（框架）＝ PlanningPipeline ＋ 插件（OMPL/pilz/CHOMP/STOMP）   │
│                        ＋ IK 插件（KDL/pick_ik/opw）                  │
│                        ＋ 碰撞检查（FCL/Coal）＋ PlanningScene/ACM      │
│                        ＋ 时间参数化（TOTG / Ruckig）                  │
│  职责：怎么走？——产出一条带时间戳的 JointTrajectory                   │
│  输出：/joint_trajectory（Action）                                    │
└──────────────────────────┬─────────────────────────────────────────┘
                           ▼ Trajectory
┌─ ③ 控制层（Controller）  500–1000 Hz，周期，软实时 ───────────────────┐
│  ros2_control controller_manager 里的插件：                           │
│  joint_trajectory_controller（JTC）/ forward_command / admittance / Servo │
│  职责：每周期把 reference 变成 command（含 PID、重力前馈、导纳）        │
│  输出：command interface（进程内指针，不走 DDS）                       │
└──────────────────────────┬─────────────────────────────────────────┘
                           ▼ write()
┌─ ④ 硬件层（Hardware）  1 kHz，硬实时 ─────────────────────────────────┐
│  hardware_interface 插件：SystemInterface / ActuatorInterface        │
│  职责：read() 读编码器、write() 打包 CANFD ——只做 I/O，不算控制        │
│  可选：中间插一块 MCU（我们的方案 D：1 kHz 插补 + 重力补偿 + 限幅）      │
│  输出：CANFD 帧（1M 仲裁 / 5M 数据）→ 达妙电机 MIT 模式                │
└──────────────────────────────────────────────────────────────────────┘
```

**频率跨度 2–3 个数量级**，这是所有架构决策的根源：规划层 100 ms–2 s、控制层 1 ms、电机内环 50–100 µs。
**下层绝不调用上层，上层绝不进实时环。**

---

## 2. 逐条修正你现在的理解

### 2.1 "ROS 处理算法节点通信和 PC-MCU 通信"

**一半对。**
- 对的部分：ROS 2 是**中间件 + 组件框架**，进程间通信（话题/服务/Action）、生命周期、参数、插件加载都归它。
- 要修正的部分：**PC↔MCU 通信不是 ROS 提供的**。它是你在 `hardware_interface::read()/write()` 里自己写的：
  - 走 SocketCAN：`socket(PF_CAN)` + `write(canfd_frame)`，插件里完成
  - 走串口/USB CDC：`termios` + `O_NONBLOCK`，插件里完成
  - ROS 只负责"把这个插件加载进 controller_manager，并保证它每 1 ms 被调一次"

### 2.2 "Pinocchio 处理机械臂算法，输入末端位置/力矩，输出每个电机的值"

**这句把三个阶段揉在一起了，而且方向反了一个。**

| 你以为 | 实际 |
|---|---|
| Pinocchio 是一个"层" | Pinocchio 是一个**库**（C++），被上面三层里任意一处 `#include` 调用。它不是架构组件 |
| 输入末端位姿 → 输出电机值 | 中间还有两步：**IK**（位姿→关节角）和**控制器**（关节角→command）、**hardware interface**（command→CAN 帧） |
| 输入"力矩" | 力矩是**输出方向**的东西：`RNEA(q, q̇, q̈) → τ`，或者重力补偿 `g(q) → τ_g`。它不能当"输入末端位置"的同级输入 |

正确的三段式：

```
末端位姿 T*  ──IK──▶  q_des（N 维关节角）
q_des, q̇_des ──控制器(JTC/PID/导纳)──▶  command（位置/速度/力矩）
command ──hardware_interface.write()──▶  MIT 报文（每台电机 8 字节）
```

而**力矩**的来源是另一条链：`RNEA(q, q̇, q̈) → τ_ff`（前馈），或 `g(q)`（重力补偿）。

**Pinocchio 在各层的出现位置**
| 层 | 用 Pinocchio 干什么 |
|---|---|
| 规划层 | IK 插件内部（KDL 是默认，pick_ik 用 Pinocchio）；碰撞检查前的 FK |
| 控制层 | 导纳控制里算 FK/Jacobian；重力前馈 `g(q)` |
| 硬件层 / MCU | **不用**（太重）→ 换成我们生成的纯 C |
| 离线 | 我们 armgen 用它录 tape、生成代码 |

### 2.3 "MoveIt2 是用来做避障决策的吗？"

**不是"决策"，是"运动规划"。避障只是它的一个副产品。**

MoveIt2 是一个**框架**（"MoveIt2 = OMPL" 等价于 "Linux = kernel"），它管五件事：

| 职责 | 具体组件 | 说明 |
|---|---|---|
| **运动规划** | OMPL（RRT-Connect/RRT*/PRM）、pilz（直线/圆弧）、CHOMP/STOMP（优化型） | 在关节空间搜一条无碰撞路径 |
| **碰撞检查** | FCL/Coal + PlanningScene + ACM（自碰撞矩阵） | 这才是"避障"，但它是**约束检查**，不是决策 |
| **IK** | `kinematics_solver` 插件（KDL/pick_ik/opw） | 位姿目标 → 关节目标 |
| **时间参数化** | TOTG（速度+加速度）/ Ruckig（多一层 jerk） | 几何路径 → 带时间戳轨迹 |
| **执行与监控** | Action + JTC 的 `FollowJointTrajectory` | 下发轨迹、监控执行、超时中止 |

**所以：MoveIt2 回答"怎么走"，不回答"做什么"。**"做什么"在它上面一层。

### 2.4 "决策层的算法框架是什么？"

教程给的答案是 **BehaviorTree.CPP**（ROS 2 生态里也有 `py_trees`、SMACH 状态机、以及工业界的 `MTC`）。

| 框架 | 定位 | 什么时候用 |
|---|---|---|
| **FSM（状态机）** | ≤5 状态、线性流程 | 简单 demo、单动作 |
| **BehaviorTree.CPP** | 5–15 状态、需要重试/降级/并行监控 | **推荐**：工业装配、抓取流水线 |
| **MTC（MoveIt Task Constructor）** | 规划层的"任务分解"，Stage 组合（Generator/Propagator/Connector） | 复杂抓取（多解搜索、约束传播、attach/detach） |
| **PDDL/TAMP**（如 PDDLStream） | 符号推理 + 几何验证 | 研究向，多物体任务规划 |

**BT 的本质**：三态（SUCCESS/FAILURE/RUNNING）、协作式非抢占 tick、节点组合（Sequence/Fallback/Parallel/Decorator）。
它的价值不是"更强"，而是**可维护性**：30 状态、100+ 边的 FSM 在 BT 里只是一个 Sequence 套 Fallback。

**BT 与 MTC 是互补，不是替代**：
- **BT 决定"做什么"**（秒–分钟：先抓 A 再放 B，失败了重试）
- **MTC 决定"怎么做"**（毫秒–秒：从哪个角度接近、IK 哪个解、attach 哪个物体）

**判据**：BT 里一旦出现轨迹点、IK 调用、控制增益，就说明职责泄漏了。

---

## 3. 一次 pick-and-place 的完整数据流

```
① BT tick（10 Hz）
   /vision/object_pose ──▶ Blackboard ──▶ Action Goal: "抓红色方块"
        │
        ▼
② MoveIt2（事件驱动，100 ms–2 s）
   PlanningScene（URDF + 物体 + /joint_states）
   → IK 插件（pick_ik：µs 级）求抓取位姿的 q_goal
   → 规划器（RRT-Connect）搜无碰撞路径 [q0 … qT]
   → Ruckig 时间参数化（带 jerk 限制）→ JointTrajectory
        │
        ▼ /follow_joint_trajectory
③ JTC（500–1000 Hz）
   按时间插值 reference → 若配了导纳，叠加 F/T 传感器的柔顺偏移
   → command interface: q_cmd, dq_cmd, (τ_ff)
        │  （进程内指针，不走 DDS）
        ▼
④ hardware_interface::write()（1 kHz）
   （方案 D）PC→MCU 下发送定点 {q_des, dq_des, Kp, Kd, τ_ff} + seq
        │
        ▼ USB CDC / 以太网，200–500 Hz
⑤ MCU（1 kHz，确定性）
   插补到 1 kHz → 重力补偿 g(q)（我们生成的代码）→ 限幅 → 打包 MIT 帧
   → CANFD（1M/5M）→ 6 台达妙电机
        │
        ▼ 反馈
⑥ 电机 → MCU → PC → /joint_states + /ft_sensor → 回到 ①
```

**每一层的"止损点"**：
- BT：任务级失败 → 重试/换策略/求人
- MoveIt2：规划失败 → 换规划器/换 IK/放宽约束
- JTC：跟随误差超限 → 中止轨迹、进 hold
- MCU：超差/过流/断链 → 限幅、零力矩、保持

---

## 4. 我们项目的具体落位

| 层 | 用什么 | 自己写什么 |
|---|---|---|
| 决策 | BehaviorTree.CPP（先只做 3 个节点：MoveTo、Gripper、Retry） | 动作节点（30 行/个，用 `RosActionNode` 基类） |
| 规划 | MoveIt2 + OMPL + pick_ik + Ruckig | `moveit_config` 包（URDF/SRDF/kinematics.yaml/joint_limits.yaml） |
| 控制 | JTC（+ 可选导纳） | 若做力控：自定义 controller |
| 硬件 | `ros2_control` SystemInterface 插件 | **CANFD 收发 + 达妙 MIT 协议**（或直接用官方包） |
| MCU | 生成代码 | 重力补偿 + 插补 + 限幅 + 看门狗（< 10 KB） |
| 离线 | armgen | 生成 FK/雅可比/重力/RNEA 的纯 C |

**能直接用的**：达妙官方 7 轴包的协议库与 `ros2_control` 插件（BSD-3）、MoveIt2 全家桶、BT.CPP 基类。
**必须自己写的**：URDF/SRDF 与 moveit_config、我们的生成器与 MCU 侧固件、BT 的业务节点。

---

## 5. 职责边界（防止架构腐化）

| 检查项 | 正确 | 错误信号 |
|---|---|---|
| 实时环里有 ROS 话题？ | ❌ 不该有 | 在 1 kHz 里 `rclcpp::spin_some` |
| 规划层里有 CAN 帧？ | ❌ | MoveIt 插件里直接发 CAN |
| BT 里有 IK/轨迹点？ | ❌ | 动作节点里算 q_des |
| 控制器里有 `new`/日志？ | ❌ | `update()` 里 `RCLCPP_INFO` |
| MCU 里有规划？ | ❌ | MCU 上跑 OMPL |
| 上位机做重力补偿？ | ⚠️ 可做但不安全 | PC 挂了机械臂塌下去 |

---

## 6. 几个会直接咬人的坑（来自教程）

| 坑 | 后果 | 对策 |
|---|---|---|
| **TOTG 不限制 jerk** | 加速度阶跃 → 1 kHz 伺服出现力矩尖峰 | 显式换成 **Ruckig**（多一层 jerk 限制） |
| **PlanningScene diff 只传变化量** | `addCollisionObjects()` 返回 ≠ 已生效，紧接着规划会用旧场景 | 用服务式 `applyCollisionObject()` |
| **ACM 误禁用碰撞对** | 撞坏设备（漏禁用只是浪费微秒） | 只禁相邻连杆，逐条验证 |
| **`on_activate` 不设 `cmd = state`** | 命令默认 0，关节在 1.5 rad 时瞬间满速回零 → **上电飞车** | 激活时对齐当前位置 |
| **Servo 100 Hz 命令 vs 10 Hz 碰撞检测** | 中间 10 步未检查 | 安全间距 ≥ v/rate（5 cm） |
| **USB CDC 默认 latency_timer = 16 ms** | "1 kHz 控制"实际是 62 Hz 设定点 | `echo 1 > .../latency_timer` |
| **规划器选型错** | pilz 不避障、RRT 抖 | 通用避障用 RRT-Connect，直线用 pilz + 环境清空 |
