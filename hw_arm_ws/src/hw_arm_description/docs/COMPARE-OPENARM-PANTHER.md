# OpenArm vs Panther：两套达妙生态机械臂的对比与参考价值

> OpenArm = 达妙官方 7 轴谐波臂（`kit-miao/7-axis-harmonic-drive-robotic-arm` + `openarm_description`）
> Panther = 社区 6 轴钣金臂（`wEch1ng/Panther_RobotArm`，Ragtime-LAB）

## 0. 一句话定位

| | OpenArm | Panther |
|---|---|---|
| 定位 | **工程化 / 产品化**：可量产的双臂平台 | **低成本教学**：学生党能玩上动力学 |
| 形态 | 7 轴 ×2 + 夹爪（也支持单臂 preset） | 6 轴 + 2 指平行夹爪 |
| 生态 | ROS 2（Humble/Jazzy） | ROS 1（Noetic，已 EOL） |
| 时代 | 2026，活跃维护 | 2025 前后，后续迁到 `Ragtime_Panthera` |

---

## 1. 硬件与机械

| 项 | OpenArm v2.0 | Panther |
|---|---|---|
| 自由度 | 7（每臂），双臂形态 | 6 + 2 个 prismatic 夹爪指 |
| 结构 | 谐波减速 + 铝合金加工件 | 钣金 + 标准件 |
| 电机 | DM-JH14 ×4 / DM-JH11 ×3 / DM3507（夹爪） | DM4310 / DM4340 等 |
| 总线 | **CANFD（仲裁 1 Mbps / 数据 5 Mbps）**，8 电机一条总线 | **USB-U2C 串口 921600**，2 个适配器（1-3 关节 / 4-6 关节） |
| 开源件 | STEP/3D、BOM、装配说明、图纸 | SolidWorks 全模型、BOM、钣金图 |
| 关节限位 | 按设计/实测填写（如 J1 −1.396~3.491） | **全部 ±3.14**（sw2urdf 默认，未调） |

---

## 2. 软件栈逐层对比

| 层 | OpenArm | Panther |
|---|---|---|
| **描述文件** | `assets/{robot,end_effector,sensor}` + `config/{struct,joint,link,inertials,control}/*.yaml` + xacro 组合器 + `robot_presets/*.yaml` | `sw2urdf` 一次性导出的**扁平 URDF** + STL（10 个 inertial、20 个 mesh） |
| **关节轴** | 原始是任意轴，可改造成全 `(0,0,1)` | 天生全 `(0,0,1)`（sw2urdf 约定） |
| **控制框架** | **ros2_control**：自定义 `SystemInterface` 插件（`left_arm_hardware`）+ 生命周期 + `control_executor_thread` | **手写 ROS 节点**，无 `hardware_interface`：`dm_controller` / `ZeroGraMode` / `teach` / `changeMode` |
| **动力学** | **Pinocchio**（URDF 驱动，`computeGeneralizedGravity` + `compute_inertial_coriolis`） | **手写 DH + Newton-Euler**（`6Dof_dyna.h`，180 行，硬编码质量/质心/DH/零点偏置） |
| **重力补偿位置** | PC（硬件接口内部） | PC（节点内部） |
| **示教** | `teaching_manager_node` | `teach.cpp`（460 行，含轨迹录制/复现 + 速度阻尼手感） |
| **规划** | MoveIt2 + `left_description_grip.srdf` + moveit.rviz | MoveIt1 `panther_moveit_config`（标准模板生成） |
| **工具节点** | `gravity_comp_node` / `real_gravity_comp_node` / `motor_controller_node` | `catesian_plan`（笛卡尔画圆，作者注明"可能有问题"） |
| **总线协议实现** | `left_arm_serial`：`socketcan.cpp` + `damiao.cpp`（SocketCAN + CANFD） | `damiao.h`（822 行）：串口 + CAN 帧封装 + 多控制模式枚举 |

---

## 3. 相同点（比想象中多）

1. **都是"PC 跑算法 + 电机内置 FOC/伺服"，中间没有 MCU**。
2. **都用 MIT 模式**：`control_mit(kp, kd, pos, vel, tau)` 一次发全。
3. **重力补偿都在 PC 侧**，本质都是 RNEA 静态化（一个用 Pinocchio，一个手写）。
4. **都以 URDF 为唯一几何/惯性真值源**（Panther 虽然手写动力学，但 URDF 仍是 RViz/MoveIt 的真值）。
5. **都做拖动示教**：重力补偿 + 速度阻尼。
6. **都用 MoveIt 做规划**（版本不同）。
7. **硬件都需要**：USB-CAN 适配器 + 120 Ω 终端电阻 + 独立供电。
8. **关节轴最终都是 `(0,0,1)`** —— 这印证了我们给 `hw_arm` 做的改造是行业常规做法。

---

## 4. 关键差异

| 维度 | OpenArm | Panther | 影响 |
|---|---|---|---|
| **抽象层次** | 数据驱动（YAML + 组合器 + preset） | 单体文件（一个 URDF 走天下） | 换零件：OpenArm 改 YAML；Panther 重新导出 |
| **参数可信度** | 限位/质量/惯量按设计值维护 | 限位全 ±π、质量来自 CAD 自动算 | Panther 的 URDF **不能直接用于动力学标定** |
| **动力学实现** | 运行时库调用（Pinocchio） | 手写递推（无依赖、快） | Panther 的可移植性强，但改臂必须重推 DH |
| **控制接口** | 标准化（ros2_control 插件） | 自定义节点 | OpenArm 能直接接 MoveIt2/控制器生态 |
| **失败处理** | 生命周期状态机 + 错误码 | 基本没有（`return 0`） | 上产线必须补 |
| **可读性** | 注释齐全、有 meta.md | 魔法数多、注释少 | 学习曲线不同 |
| **夹爪** | revolute + **mimic**（拓扑驱动） | **prismatic** 0~0.06 m（两指独立） | 我们的平行夹爪可直接抄 Panther 的 prismatic |
| **实时性** | 200 Hz（`update_rate`），CANFD | 500 Hz（`ros::Rate`），串口 921600 | 都够用，瓶颈都是总线不是 CPU |

---

## 5. 各部分参考价值（给 `hw_arm` 的取舍）

| 模块 | OpenArm | Panther | 我们的选择 |
|---|---|---|---|
| **URDF 组织方式** | ★★★★★ 数据驱动 + preset + 校验宏 | ★★ 扁平单体 | **抄 OpenArm**（已做） |
| **惯性参数来源** | ★★★★ `inertials/nominal.yaml`，可扩展 `measured/` | ★★★ CAD 自动算，正负未知 | 抄 OpenArm 的分层，数值待实测 |
| **关节限位** | ★★★★★ 真实设计值 | ★ 全 ±π（不可用） | **抄 OpenArm**（已做） |
| **关节轴/坐标系** | ★★★ 原始轴混乱，需改造 | ★★★★★ 天生 `(0,0,1)` | 我们已改造，与 Panther 一致 |
| **夹爪建模** | ★★★★ mimic + 拓扑驱动（7 轴用） | ★★★★ prismatic（简单直接） | **抄 Panther 的 prismatic**（更简单） |
| **ros2_control 插件** | ★★★★★ 标准 `SystemInterface` + CANFD | ★ 无（ROS1 手写节点） | **抄 OpenArm** |
| **CAN 协议实现** | ★★★★★ SocketCAN + CANFD + 模式切换 | ★★★ 串口封装，兼容性好 | 抄 OpenArm（CANFD 带宽够） |
| **重力补偿** | ★★★★ Pinocchio 运行时 | ★★★ 手写 NE（可以当对拍基准） | **生成代码**（比两者都轻，还能上 MCU） |
| **示教手感（阻尼系数）** | ★★★ | ★★★★★ 按关节手调的 `f[i]` 经验值 | **抄 Panther** 的调参经验 |
| **MoveIt 集成** | ★★★★ ROS2 全家桶 | ★★★ ROS1 模板（无优化） | 抄 OpenArm |
| **硬件/装配文档** | ★★★★ BOM + 3D + 装配说明 | ★★★★★ SolidWorks 全模型 + 钣金图 | 各自取用：Panther 适合低成本复刻 |
| **代码可读性/可维护性** | ★★★★ | ★★★ | — |

---

## 6. 五条结论

1. **描述文件抄 OpenArm**：数据驱动 + preset + 分层 config，我们已经按这个路线做了 `hw_arm_description`。
2. **控制接口抄 OpenArm**：ros2_control `SystemInterface` 是接 MoveIt2 的唯一正确姿势；Panther 的"手写节点"路线在 ROS2 时代不划算。
3. **夹爪抄 Panther**：prismatic + 行程映射（0~0.06 m）比 OpenArm 的 revolute+mimic 简单，且更符合"平行夹爪"的实际结构。
4. **重力补偿两者都不抄**：它们都在 PC 跑；我们生成纯 C 后可以放 MCU，这是两者都没有的能力（`docs/ARCHITECTURE-OPTIONS.md` 里的方案 D）。
5. **Panther 的最大价值是"被验证过的低成本路线"**：钣金 + 达妙电机 + 5.4 kg 级的 6 轴臂能做出可用的拖动示教，这条工程路径值得复刻；但它的 URDF 限位和动力学参数不能直接用。

## 7. 坑

- Panther 是 **ROS 1 Noetic**（2025 已 EOL），代码不能直接搬到 ROS 2。
- Panther 的 `6Dof_dyna.h` **硬编码 DH、q_offset、质量**，换臂必须重新推导（这正是我们做代码生成的原因）。
- OpenArm 的 `presets/校验宏` 复杂度高，学习成本不低；简单 6 轴用不上全部机制。
- 两者的 URDF 都依赖 mesh；我们的圆柱占位只是过渡，换真实网格后必须重跑 `gen_srdf.py`。
