# hw_arm 待办清单

> 最后更新：2026-11-22
> 编号规则延续 README 里的 `C-xx`；新增的按时间顺序往后排，不复用旧号。

---

## P0 — 会影响真机安全 / 必须先做

### C-30 MoveIt 2 集成（**用户指定：先记下，后面再做**）

**动机**：当前执行层是自研"类 MoveIt"的控制库（`kinematics.cpp` + `ik_selection.cpp`
+ `goto_xyz.py` → JTC），**没有碰撞检测，也没有复杂运动序列能力**。
MoveIt 2 补的正是这两块。

**定位**：方案 B —— MoveIt 只做**规划 + 避障**，执行仍走现有 JTC；
现有 IK 链（点对点）保持不动。长期可选方案 C：把 `ik_selection` 包成
MoveIt 的 `KinematicsBase` 插件，让 MoveIt 也享受"选解 + 路径不穿奇异"。

**前置条件（不满足就别开始）**
- [ ] `sudo apt install ros-jazzy-moveit ros-jazzy-moveit-planners-ompl \
        ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-ros-visualization`
      （2026-11-22 核对：四个包均未安装，apt 源里有 2.12.4）
- [ ] 先修 C-31（URDF 速度上限）—— 否则 MoveIt 按错误速度做时间参数化
- [ ] 先补加速度参数 —— MoveIt 的 TOTG/Ruckig 必须有 `max_acceleration`

**要新建的包** `hw_arm_ws/src/hw_arm_moveit_config/`
```
config/
  hw_arm.srdf              用现成的 hw_arm_description/scripts/gen_srdf.py 生成，
                           再手补 <group>/<end_effector>/<virtual_joint>/<group_state>
  joint_limits.yaml        MoveIt 格式（max_velocity / max_acceleration / max_jerk）
  kinematics.yaml          KDL 起步；q5≈0 腕部奇异附近成功率低，后续换 pick_ik
  ompl_planning.yaml       range 要小（臂展只有 28 cm），
                           longest_valid_segment_fraction ≈ 0.005
  moveit_controllers.yaml  type: FollowJointTrajectory,
                           action_ns: follow_joint_trajectory,
                           controller: hw_arm_controller  ← 与现有 JTC 天然对齐
  hw_arm.rviz
launch/
  demo.launch.py           全 mock，只规划不执行
  plan_execute.launch.py   接真机 JTC
```

**已知的 5 个数据问题（写配置前必须处理）**
1. `joint_limits.yaml`（我们的）里 joint1 `velocity: 16.755`，
   但 24V 空载 100 rpm = **10.47** —— 见 C-31
2. 加速度完全没有 → MoveIt 需要：`[84.8, 4.0, 80.8, 785, 852, 1700]` rad/s²
3. `effort` 与名称牌不符：现有 j2=40/j3=27/j4~6=7，
   实测额定 j2=10/j3=12/j4=3.5/j5,6=3
4. joint1/joint6 是有界 ±π / ±2π（不是 continuous）：MoveIt 不会绕圈，
   但边界附近可能选"绕远路"的解
5. q5≈0 是腕部奇异（j2/j3/j5 三轴平行）→ KDL IK 成功率低，
   要 `kinematics_solver_timeout: 0.05`、`attempts: 5`

**与现有框架的冲突点**
- 两套 IK（MoveIt KDL vs `ik_selection`）：同一目标可能给不同解 → 长期用方案 C 统一
- 两套时间参数化（MoveIt TOTG/Ruckig vs 两点轨迹）
- JTC 容差（C-29 刚加）：太紧会让 MoveIt 报 `execution failed`
- **执行互斥**：MoveIt 和 `goto_xyz.py` 都会发 FollowJointTrajectory，
  JTC 会用新目标抢占旧目标 → 需要一把执行锁

**验收顺序**（每步都能单独验证）
1. `demo.launch.py`（mock）：RViz 能拖末端、能规划、能预览 —— 能暴露 ~90% 配置错误
2. 接真机、速度缩放 0.1：`default_velocity_scaling_factor: 0.1`
3. 逐步提速到实测边界
4. 调 JTC 容差与 `max_acceleration` 对齐

**估计**：装包 10 min + 只规划 2 h + 接 JTC 1 h + 调参 2 h ≈ **半天到一天**

---

### C-31 按实测数据修正 URDF / joint_limits 的速度与力矩上限

`config/joint/joint_limits.yaml` 是 URDF 限位的唯一来源，但数值没跟实测对齐：

| 关节 | 现在写的 velocity | 24V 物理上限 | 现写的 effort | 实测额定 |
|---|---|---|---|---|
| joint1 | 16.755 | **10.47** | 40 | 40 ✅ |
| joint2 | 16.755 | 27.23（偏保守） | 40 | **10** |
| joint3 | 5.4454 | 5.86 ✅ | 27 | **12** |
| joint4 | 20.944 | 20.94 ✅ | 7 | **3.5** |
| joint5 | 20.944 | 20.94 ✅ | 7 | **3** |
| joint6 | 20.944 | 20.94 ✅ | 7 | **3** |

数据源：`config/control/motor_specs.yaml`（说明书 + 固件 RID 实读）。
**注意**：改了这个会影响 URDF → 也会影响 `ik_selection` 的限位余量判据。

---

### C-32 把 armature（转子惯量）接进动力学模型

`motor_specs.yaml` 里已有实测的 `armature`（RID 12 × Gr²），但
`RobotModel::load(urdf)` 只读 URDF，URDF 没有 armature 字段。

现状后果：`M66 = 9.1e-7`（≈0），`cond(M) = 2.4e5`，动力学不可用。
加上后：`M66 = 1.77e-3`，`cond(M) = 182`。

建议做法（方案 A）：`RobotModel::load(urdf, armature)` 重载，
加载后加到 `inertias[i](2,2)`。Pinocchio 里 `model.inertias[i]` 就是 joint i 的
坐标系、转轴是它的 z 轴，加法是**精确正确**的。

---

### C-41 BT.CPP 决策层接入（**依赖已就绪，待实现**）

2026-11-22 在 `~/BehaviorTree/BC_ws` 验证通过：

| 组件 | 版本 | 状态 |
|---|---|---|
| BehaviorTree.CPP | **4.11.1** | ✅ 编译通过（关掉 tools/examples/tests） |
| behaviortree_ros2 | 0.3.0（humble 分支） | ✅ **在 Jazzy + v4.11.1 上编译通过** |
| btcpp_ros2_interfaces | 0.3.0 | ✅ |

**两个踩过的坑（记下来免得重踩）**
1. `git clone -b ros2` 会失败 —— **该分支不存在**。远端只有
   `bug_fix/registry`、`humble`、`issue_76` 三个分支，克隆别加 `-b`。
2. GitHub 直连在本机不稳（DNS 把 github.com 解析成 57.155.223.240，
   连接被重置）。`--depth 1` + 重试可过。
   Tsinghua TUNA **不镜像 GitHub 仓库**（只镜像 apt/pypi/conda）。
3. BT.CPP 本体要关 4 个可选组件才能编（缺 flatbuffers / nlohmann_json）：
   `-DBTCPP_BUILD_TOOLS=OFF -DBTCPP_EXAMPLES=OFF -DBUILD_TESTING=OFF`
   Groot2（cppzmq）与 SQLite 日志（libsqlite3-dev）都可用。

**第一阶段范围（不依赖 MoveIt C-30）**
- 包 `mini_manip_bt`：`FollowJointTrajectory`（走现有 JTC）、
  `IsJointLimitOK`、`IsEmergencyStopInactive`、`SwitchController`
- 树形：`ReactiveSequence( 条件…, FollowJointTrajectory )`
  —— 必须用 Reactive 版，否则前置条件只查一次，长动作套不住安全监控
- `onHalted()` 里必须 `cancel_goal_async()`，否则 BT 以为中断了、JTC 还在跑
- tick 10–20 Hz，独立非实时线程（MultiThreadedExecutor + 单独 callback group）

**第二阶段**：MoveIt 就绪后加 `MoveToPose`（`RosActionNode<MoveGroup>`）
**第三阶段**：把 `goto_xyz.py` 的"选解 → 轨迹 → 执行 → 验收"整体搬进 BT

---

### C-42 视觉抓取规划（D435 已有，夹爪即将到位）

**硬件现状**：Intel RealSense D435（深度 FOV 87°×58°）；臂展仅 0.28 m，
相机装 0.5 m 处即可覆盖整个工作空间（视野 0.95×0.55 m，848 宽下 1.12 mm/px）。
**深度精度约 1~2% of distance**：0.3 m → 3~6 mm，0.5 m → 5~10 mm（需实测确认）。

**开发机（真实主机，2026-11-22 用户提供）**
| 项 | 配置 |
|---|---|
| CPU | **i7-14650HX** 16C/24T（8P+8E），Raptor Lake-HX |
| 内存 | 16 GB DDR5-5600 **单条**（15.8 GB 可用） |
| **GPU** | **NVIDIA RTX 4060 Laptop，8 GB 显存**，驱动 616.56，PCIe 4.0 x8 |
| 硬盘 | YMTC 954 GB NVMe |
| 计划 | 在新 SSD 上装**原生 Linux** 开发 |

**⇒ 之前"两个都跑不动"的结论要改**（那是 VMware 虚拟机：只有 SVGA、无 CUDA、7.7 GB 内存）

算力换算（FP32 峰值）：RTX 4060 Laptop ≈ 13.8 TFLOPS = **V100 的 0.88 倍**
→ 教程里 GraspNet 的 200 ms（V100/3090 口径）在 4060 上约 **200~300 ms**
→ 8 GB 显存：GraspNet 推理约 2~4 GB、AnyGrasp 约 1~2 GB，**都放得下**

**原生 Linux 的 5 个坑**
1. NVIDIA 驱动 + Secure Boot：需 MOK 签名或关闭 Secure Boot
2. RViz / Gazebo 建议 X11；Wayland + 165 Hz + 独显 易有渲染问题
3. CUDA 版本与 PyTorch 对齐；Jazzy 是 Python 3.12
4. D435 必须 USB 3.0 直连（虚拟机下深度流会掉帧）
5. 250 Hz 控制要关降频：`governor=performance`，注意 E-core 调度抖动

**GraspNet vs AnyGrasp**

| | GraspNet-1Billion | AnyGrasp |
|---|---|---|
| 开源 | ✅ PyTorch | ❌ 二进制 SDK |
| 许可 | 自由 | **需申请（学术/商用分开）** |
| 推理 | ~200 ms（GPU） | ~50 ms |
| 成功率 | ~80% | 90%+ |
| **本机可跑** | ✅ 4060 上约 200~300 ms | ✅ 硬件满足，但**需申请许可** |

**→ 硬件上两个都可行。但第一版仍推荐几何抓取**（桌面单物体顶抓不需要学习模型），
GraspNet 留作"堆叠/杂乱场景"的备用路径。

**第 1 阶段（推荐先做）：解析几何抓取，纯 CPU，无模型**
1. RANSAC 去桌面平面
2. 欧氏聚类取物体点云
3. 质心 + PCA 主轴 → 生成 2~8 个候选（顶抓优先，减少自由度）
4. 过滤：`ik_selection`（实测 159 µs/次，比 KDL 快 13 倍）+ 碰撞（C-36 未做）
5. 预计 50~100 ms/帧

**第 2 阶段**：若需要堆叠/杂乱场景，再上 GraspNet（先在 CPU 上测实际耗时）

**第 3 阶段**：若要 AnyGrasp，必须先有 GPU 工作站 + 许可

**前置依赖**
- [ ] C-36 碰撞检测（没有它，候选 90% 会撞）
- [ ] C-37 夹爪进 URDF + 重新做重力辨识
- [ ] C-33 延伸：手眼标定 AX=XB（M15:16，15~25 组，绕三轴各 ≥20°）
- [ ] 装 `ros-humble-realsense2-camera`（Jazzy 对应包）+ 实测深度精度

---

### C-36 碰撞检测（拆解：先只做自碰撞）

> **方案与实现细节见 [`docs/COLLISION_DESIGN.md`](COLLISION_DESIGN.md)**（选型数据、文件结构、API、接入点、测试计划）

**用户确认**：工作空间没有外部障碍物，只需查自碰撞。
**先说结论：自碰撞会在限位内发生（同目标的多数 IK 分支都碰），但只需用来"选分支"，成本要分两级。**

**"在关节限位内就不会自碰撞吗？"—— 不会，但要看具体含义**

实测（Pinocchio + coal 3.0.3，本机）：
- 排除相邻连杆后，真正要查 **15 对**
- 3000 个随机构型：**1.6% 有真实自碰撞**；碰撞位形长这样：
  `joint2 ≈ 105~180°`、`joint3 ≈ 0°`（大臂转下去、小臂完全折回），
  TCP 高度 z ≈ −11 ~ +3 cm（**低于基座**）。
  碰撞对 90% 是 `base_link ↔ link4/5/6`，不是典型肘部自碰。
  joint2 限位 −2°~180° ⇒ **"折下去"完全在限位内**。

**更关键的统计**：对每个可达目标枚举 8 个去重 IK 解

| 目标 | 解数 | 碰撞解 | 全部解都碰的目标 |
|---|---|---|---|
| r=12 cm, z=5 cm | 320 | 243（**76%**） | **15%** |
| r=18 cm, z=10 cm | 320 | 206（64%） | 2.5% |
| r=24 cm, z=15 cm | 320 | 213（67%） | 0% |

⇒ **限位内一半以上的 IK 分支是自碰撞的**；
⇒ 但 85%+ 的目标**存在**无碰撞解。

**所以碰撞检测的作用是"选分支"，不是"禁止目标"**：
`ik_selection` 现在的打分只有 travel / manipulability / gravity / margin，
**没有碰撞项**。风险集中在大位移、以及目标靠近基座（r<15 cm 且 z 低）时。

**成本（这是关键）**
| 碰撞体表示 | 单次耗时 | 100 个候选 |
|---|---|---|
| STL mesh（现状） | **2293 µs** | 230 ms |
| Box 近似 | **10 µs** | 1 ms |

→ **两级策略**：L1 用 Box 近似粗筛（10 µs），L2 对通过的少数候选做 STL 精查（2.3 ms）。

**⚠️ 口径会变**
- 夹爪进 URDF 后，手指是运动部件；被抓物体应 attach 到手臂当作碰撞体
- 桌面通常仍要作为障碍（顶抓时夹爪不能扎进桌面）——"只有自碰撞"的假设要重新审

**实施**
- [ ] `hw_arm_algorithms` 加 `checkSelfCollision(model, ws, q)`：L1 Box / L2 mesh
- [ ] 接进 `ik_selection` 的硬过滤（放在路径奇异检查之后）
- [ ] 抓取候选过滤复用同一个函数

---

## P1 — 功能推进

### C-37a 安全壳阈值实测标定（刚加，值是拍出来的）
- `hw_arm_hardware.cpp` 的 `stale_timeout_ms_ = 200`：要看真实丢帧分布
- `hw_arm_controllers.yaml` 的 `constraints`（trajectory 0.09 / goal 0.05 rad）：
  按真机重复性收紧
- 建议二者都改成 ROS param，避免每次改数值都要重编译

### C-33 几何标定
- 1 mm 几何误差 = 1 mm 末端误差（已实测）
- 编码器本身限在 ~0.14 mm
- 先用"双构型重合"验证（`solveIKMultiSeed` 能枚举多解）

### C-34 C-05 重力辨识剩余工作
- 上次只采到 13 个姿态，拟合残差 RMS 0.198 N·m（目标 0.2，勉强）
- 需要补齐到 ~28 个姿态（joint2 20~120°，joint3 10~140°）
- 然后 `fit_static_gravity.py` 重拟合

### C-35 阻抗控制接入（`cartesian_impedance` 已写，未接）
- POSITION/EFFORT 模式互斥（`prepare/perform_command_mode_switch`，目前没有）
- `previous_applied_torque` 上报（硬件层没有记录上一周期实际发出力矩）
- 重力 g(q) 的单一 owner + 互锁
- 转子惯量（C-32）是前提：`M_d = Λ` 依赖 `M⁻¹`


### C-37 夹爪
- URDF 里没有末端执行器
- 装上夹爪后要**重做重力辨识**（C-34 的流程）
- 可选 `--fixed-arm` 模式 + `payload_mass` 补偿

---

## P2 — 工程化

### C-38 实时性
- 现在 250 Hz，未做 `SCHED_FIFO` / `mlockall`
- 目标 1 kHz 需要先解决 USB-CAN 的可持续帧率（实测 gs_usb ≈ 3000 fps）

### C-39 模型一致性校验
- 运行时 `robot_description` 是 xacro 展开的，磁盘上还有构建产物
- 改了 YAML 只编译 algorithms 时，URDF 可能是旧的
- 需要校验模型哈希 / TCP / 关节顺序 / 零偏 / 参数版本

### C-40 执行互斥
- 多个客户端（`goto_xyz.py`、`jog.py`、MoveIt）都会发 FollowJointTrajectory
- JTC 会抢占旧目标，不会拒绝新目标
- 需要执行锁 / 会话标识

---

## 已完成（留档）

- ✅ C-05 静态重力辨识采集器（`collect_static_gravity.py`，含内核时间戳、三门前置检查）
- ✅ 关节限位真机标定（`calibrate_limits.py`）
- ✅ direction ±1 标定与验证（`mirror_check.py`）
- ✅ 24V 电机规格 + 转子惯量实读（`motor_specs.yaml`，RID 12 全精度）
- ✅ 笛卡尔点到点（`ik_selection` 多解筛选 + `goto_xyz.py`）
- ✅ 外部代码审查 R1~R8、R11（点对点路径）已修复并加回归测试
