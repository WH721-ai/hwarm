# `openarm_bimanual_moveit_config` 详解：config/ 与 launch/ 分别是什么

> 路径：`/mnt/hgfs/VMshare/OpenArm/openarm_ros2-main/openarm_bimanual_moveit_config`
> 性质：**MoveIt Setup Assistant 生成 + 手工改造**的 MoveIt2 配置包（`.setup_assistant` 文件就是生成记录）

## 一句话

- **`config/` = 数据**：告诉 MoveIt2「这台机器人怎么动（运动学）、能怎么动（限位）、怎么控（控制器）、怎么显示（RViz）」
- **`launch/` = 程序**：把这些数据加载成运行中的节点（rsp → controller_manager → move_group → rviz）

MoveIt2 的架构天然是"**参数文件 + 启动文件**"分离，所以这个包的形状就是 Setup Assistant 的标准产物。

---

## `config/`：9 个文件，按硬件版本分两套

```
config/
├── openarm_v1.0/     ← v1.0 硬件（7 轴，关节名 openarm_*_joint1..7）
└── openarm_v2.0/     ← v2.0 硬件
     └── 同样是这 9 个文件
```

**为什么分两套**：两代硬件的关节名、限位、控制器名都不同；`arm_type:=v10|v20` 决定加载哪一套。

| 文件 | 谁读它 | 作用 |
|---|---|---|
| `openarm_bimanual.urdf.xacro` | robot_state_publisher / move_group | **包一层**：include `openarm_description` 的 xacro，加 `world → base_link` 固定关节，并注入 ros2_control 段 |
| `openarm_bimanual.srdf` | move_group | **语义描述**：4 个 group（左右臂各 7 关节 + 左右夹爪）、2 个 end_effector、`home`/`hands_up` 命名姿态、自碰撞禁用对 |
| `kinematics.yaml` | move_group | **IK 求解器**：KDL，`search_resolution 0.005`、`timeout 0.005 s` |
| `joint_limits.yaml` | move_group | 速度/加速度上限（从 URDF 覆盖）+ **默认缩放 0.1**（新手保护） |
| `moveit_controllers.yaml` | move_group | **MoveIt → ros2_control 的桥**：4 个控制器，走 `FollowJointTrajectory` action |
| `pilz_cartesian_limits.yaml` | Pilz 插件 | 直线/圆弧规划的速度/加速度限制（`max_trans_vel 1.0` 等） |
| `sensors_3d.yaml` | occupancy map monitor | 深度相机 → Octomap 参数（**当前是占位**，topic 还写着 head_mount_kinect） |
| `ros2_controllers.yaml` | controller_manager | 控制器实现与参数（与 `openarm_bringup` 里那份对应） |
| `moveit.rviz` | RViz | 布局：MotionPlanning 面板、轨迹、场景 |

### 关键内容举例

**SRDF**（决定 MoveIt 眼里机器人是"什么形状"）：

```xml
<group name="left_arm">      openarm_left_joint1..7  </group>
<group name="left_gripper">  openarm_left_finger_joint1 </group>
<group_state name="home"     group="left_arm"> 全 0 </group_state>
<group_state name="hands_up" group="left_arm"> ... </group_state>
<end_effector name="left_ee" parent_link="openarm_left_ee_base_link" group="left_gripper"/>
<disable_collisions link1="openarm_body_link0" link2="openarm_right_link1" reason="Adjacent"/>
```

**moveit_controllers.yaml**（决定 MoveIt 把轨迹发给谁）：

```yaml
moveit_simple_controller_manager:
  controller_names:
    - left_joint_trajectory_controller
    - right_joint_trajectory_controller
    - left_gripper_controller
    - right_gripper_controller
  left_joint_trajectory_controller:
    type: FollowJointTrajectory
    joints: [openarm_left_joint1 ... openarm_left_joint7]
    action_ns: follow_joint_trajectory
    default: true
```

---

## `launch/`：6 个文件，分"一键"和"零件"

| 文件 | 行数 | 起什么 |
|---|---|---|
| `demo.launch.py` | **307** | **一键全栈**：robot_state_publisher + `ros2_control_node` + 若干 `spawner` + move_group + rviz。参数 `arm_type:=v20`、`use_fake_hardware:=true/false` |
| `move_group.launch.py` | 40 | 只起 `move_group`：用 `MoveItConfigsBuilder` 按 `arm_type` 挑 `config/<ver>/` 下的 srdf/limits/kinematics |
| `moveit_rviz.launch.py` | 40 | 只起 RViz + 加载 `config/<ver>/moveit.rviz` |
| `spawn_controllers.launch.py` | 40 | 只调 `controller_manager` 的 `spawner` 拉起控制器（可单独重启控制器，不重启整栈） |
| `static_virtual_joint_tfs.launch.py` | 40 | 发布 `world → base` 的静态 TF（MoveIt 需要机器人在世界里的位姿） |
| `setup_assistant.launch.py` | 8 | 重新打开 Setup Assistant 编辑这套配置 |

### 为什么 `demo.launch.py` 有 307 行

因为标准 MoveIt 的 launch 是一个 include 链（demo → move_group + moveit_rviz + spawn_controllers + static_virtual_joint_tfs），
OpenArm 把它**摊平成一个文件**，好处是"一条命令全起"，代价是参数和节点都堆在一起。

**`arm_type` 的魔法**在这几行：

```python
MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
    .robot_description_semantic(file_path=f"config/{arm_type_str}/openarm_bimanual.srdf")
    .joint_limits(file_path=f"config/{arm_type_str}/joint_limits.yaml")
    .robot_description_kinematics(file_path=f"config/{arm_type_str}/kinematics.yaml")
```

---

## 运行时它们怎么串起来

```
launch/demo.launch.py
  ├─ 读 config/<ver>/openarm_bimanual.urdf.xacro  → robot_state_publisher（TF + 模型）
  ├─ 读 config/<ver>/ros2_controllers.yaml        → ros2_control_node（controller_manager）
  ├─ spawner 拉起 4 个控制器
  ├─ move_group 读 srdf + kinematics + joint_limits + moveit_controllers
  └─ rviz2 读 moveit.rviz
```

`config/` 里任何文件改动都要**重启对应节点**才生效（除了 joint_limits 可以被 MoveIt 动态重载）。

---

## 我们 `hw_arm` 该抄成什么样

```text
hw_arm_moveit_config/
├── config/                 # 只需要一套（我们没有 v1/v2 两代硬件）
│   ├── hw_arm.urdf.xacro
│   ├── hw_arm.srdf         # ← 我们已经生成了（含 home/upright 两个姿态）
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── moveit_controllers.yaml
│   ├── pilz_cartesian_limits.yaml
│   ├── sensors_3d.yaml
│   ├── ros2_controllers.yaml
│   └── moveit.rviz
└── launch/
    ├── demo.launch.py
    ├── move_group.launch.py
    ├── moveit_rviz.launch.py
    ├── spawn_controllers.launch.py
    └── static_virtual_joint_tfs.launch.py
```

**我们已经有的**：URDF（`hw_arm_description`）、SRDF（`srdf/hw_arm.srdf`，含 `home` / `upright`）。
**还缺的**：kinematics / joint_limits / moveit_controllers / ros2_controllers / rviz + 5 个 launch。
