# openarm_ros2：四个 ROS 2 包各干什么

> 目录：`/mnt/hgfs/VMshare/OpenArm/openarm_ros2-main`（Apache-2.0，Enactic）
> 定位：OpenArm 整机的 **ROS 2 集成层** —— 把 `openarm_description`（模型）和 `openarm_can`（驱动）粘成能跑的机器人

## 结构

```
openarm_ros2/
├── openarm/                        # ① 元包（metapackage），只做依赖聚合
├── openarm_hardware/               # ② ros2_control 硬件接口插件（核心，414 行）
├── openarm_bringup/                # ③ launch + 控制器配置 + rviz
└── openarm_bimanual_moveit_config/ # ④ MoveIt2 配置（v1.0 / v2.0 两套）
```

依赖关系：`openarm` → {`openarm_hardware`, `openarm_bringup`, `openarm_bimanual_moveit_config`}
`openarm_hardware` → **`openarm_can`**（我们刚编译的那个）+ `hardware_interface` + `pluginlib`
`openarm_bimanual_moveit_config` → `openarm_description`（我们参考的那个）

---

## ① openarm（元包）

```xml
<name>openarm</name>
<description>Metapackage for OpenArm</description>
<exec_depend>openarm_bringup</exec_depend>
<exec_depend>openarm_description</exec_depend>
<exec_depend>openarm_hardware</exec_depend>
<exec_depend>openarm_bimanual_moveit_config</exec_depend>
```

**作用**：一个空包，让你 `rosdep install openarm` 就能把整套装齐。没有代码。

---

## ② openarm_hardware（核心，最值得抄）

- 类：`OpenArmHW : public hardware_interface::SystemInterface`
- 文件：`src/openarm_simple_hardware.cpp`（414 行）+ `.hpp`（158 行）+ `openarm_hardware.xml`（pluginlib 声明）

### 配置项（全部从 URDF 的 `<ros2_control>` 参数读）

| 参数 | 含义 |
|---|---|
| `can_interface` | SocketCAN 接口名（默认 `can0`） |
| `can_fd` | 是否 CAN-FD |
| `kp1..kp6` / `kd1..kd6` | 每个关节的 MIT 增益 |
| `kp_hand` / `kd_hand` | 夹爪增益 |
| `hand` | 是否接夹爪 |

### 生命周期

```
on_init()     解析参数 → 建 openarm::can::socket::OpenArm(can_interface, can_fd)
on_activate() 使能电机（力矩 ON）
read()        refresh_all() + recv_all() → pos/vel/tau 写进 state interface
write()       把 command interface 组包成 MITParam{kp,kd,pos,vel,tau} → mit_control_all()
```

`write()` 的核心就三行：

```cpp
for (i...) arm_params.push_back({kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_commands_[i]});
openarm_->get_arm().mit_control_all(arm_params);   // 一次把 6 个关节全发出去
openarm_->recv_all(100);
```

**注意**：`kp/kd` 来自 URDF 参数（不是运行时改），`pos/vel/tau` 来自 command interface ——
所以 `joint_trajectory_controller` 输出的位置 + `forward_command_controller` 输出的力矩，走的是同一条 MIT 帧。

### 夹爪

- 电机弧度 ↔ 关节行程（0~0.044 m）做一个线性映射
- 代码里明确写着 **TODO：速度/力矩映射未实现**（`vel_states_[ARM_DOF] = 0`）

---

## ③ openarm_bringup（launch + 控制器）

```
config/controllers/openarm_bimanual_controllers.yaml        # 真机：update_rate 750 Hz
config/controllers/openarm_bimanual_moveit_controllers.yaml  # MoveIt demo：100 Hz
launch/openarm.bimanual.launch.py
rviz/bimanual.rviz
```

每只臂 **4 个控制器**：

| 控制器 | 类型 | 用途 |
|---|---|---|
| `left_joint_trajectory_controller` | JTC | MoveIt / 轨迹 |
| `left_forward_position_controller` | ForwardCommand | 直接发位置（RL/脚本） |
| `left_forward_velocity_controller` | ForwardCommand | 直接发速度 |
| `left_gripper_controller` | JTC | 夹爪 |

**两个 update_rate 的差异很说明问题**：
- 真机 **750 Hz**（CANFD 撑得住，MIT 全发）
- MoveIt demo **100 Hz**（规划层不需要那么快）

---

## ④ openarm_bimanual_moveit_config（MoveIt2）

按硬件版本分目录 `config/openarm_v1.0/` 和 `config/openarm_v2.0/`，每套包含：

| 文件 | 作用 |
|---|---|
| `openarm_bimanual.urdf.xacro` | 包一层：include description 的 xacro + 加 `world` 固定关节 |
| `openarm_bimanual.srdf` | **4 个 group**：left_arm / right_arm / left_gripper / right_gripper；2 个 `end_effector`；`home` / `hands_up` 两个 group_state；`disable_collisions` 列表 |
| `kinematics.yaml` | **KDL**（`kdl_kinematics_plugin`），搜索分辨率 0.005 / 超时 0.005 s |
| `joint_limits.yaml` | MoveIt 侧的速度/加速度缩放 |
| `moveit_controllers.yaml` | MoveIt → ros2_control 的控制器映射 |
| `pilz_cartesian_limits.yaml` | Pilz 直线规划的速度/加速度限制 |
| `sensors_3d.yaml` | 点云/深度相机（Octomap）配置 |
| `ros2_controllers.yaml` | 与 bringup 里那份对应 |
| `moveit.rviz` | 专用 RViz 布局 |

launch 是标准的 Setup Assistant 产物：`demo.launch.py`、`move_group.launch.py`、`spawn_controllers.launch.py`、`static_virtual_joint_tfs.launch.py`。

---

## 数据流（真机）

```
MoveIt2 → /joint_trajectory → JTC(750Hz) → command interface
                                              ↓
                            OpenArmHW::write() → MITParam 数组
                                              ↓
                            openarm_can（SocketCAN / CANFD）
                                              ↓
                                    6× 达妙电机（内部 FOC）
                                              ↑
                            OpenArmHW::read() ← refresh/recv
```

---

## 对我们 hw_arm 的复用方式

| 他们的东西 | 我们怎么用 |
|---|---|
| 4 包分层（meta / hardware / bringup / moveit_config） | **照搬结构**，我们只需 hardware + bringup + moveit_config |
| `OpenArmHW` 的 414 行 | **当模板改**：把 `OpenArm` 的 arm 组件换成我们的 6 关节，kp/kd 从 YAML 读 |
| `openarm_can` | **直接依赖**（已编译好，`OpenArmCAN::openarm_can`） |
| 控制器 YAML 的 4 控制器分组 | 直接抄（JTC + forward pos/vel + gripper） |
| SRDF 的 group/end_effector/group_state 写法 | 直接抄（我们有 `upright` 姿态要加进去） |
| KDL 配置 | 先用 KDL（够用），以后换 pick_ik |

## 三个坑

1. **版本不匹配**：`openarm_bimanual.urdf.xacro` 里写的是
   `$(find openarm_description)/urdf/robot/v20/v20.urdf.xacro`，
   但我们手上的 `openarm_description-main` 快照**没有 `urdf/` 目录**（只有 `assets/robot/openarm_v2.0/`）。
   → 直接拿来编译会找不到文件，要改路径或对齐两边的 commit。
2. **750 Hz 依赖 CANFD**：经典 CAN 1 Mbps 跑不到，必须 `dbitrate 5000000 fd on`。
3. **夹爪映射是 TODO**：速度/力矩都写死 0，只有位置做了线性换算；我们做夹爪时要补全。
