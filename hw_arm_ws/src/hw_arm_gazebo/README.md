# hw_arm_gazebo

hw_arm 的 Gazebo（gz sim / Harmonic）仿真包。与 `hw_arm_mujoco` 对称：**一个仿真器 = 一个包**。

```
hw_arm_gazebo/
├── worlds/hw_arm.sdf                 ← 对应 mjcf/scene.xml（地面/光照/物理步长）
├── launch/hw_arm_gazebo.launch.py    ← gz sim + spawn + controller spawner
├── config/hw_arm_gazebo_controllers.yaml  ← effort 控制器（验证算法用）
└── src/                              ← 以后放算法节点（CMakeLists 里有注释块）
```

---

## 1. 现在能做什么

```bash
colcon build --packages-select hw_arm_gazebo
source install/setup.bash

# 只起 gz + 机器人（控制器不起，因为还差 description 的 gz 插件）
ros2 launch hw_arm_gazebo hw_arm_gazebo.launch.py
```

这一步能看到：gz 打开、地面和机器人被 spawn 出来、`/robot_description` 和 TF 正常。
机器人**不会动**（没有 controller_manager），这是预期行为。

## 2. 还差一件事：description 的 gz 硬件插件

`hw_arm_description/urdf/ros2_control/hw_arm.ros2_control.xacro` 现在只有两个分支：

```xml
<xacro:if value="${use_fake_hardware}">   <plugin>mock_components/GenericSystem</plugin> </xacro:if>
<xacro:unless value="${use_fake_hardware}"> <plugin>hw_arm_hardware/HwArmHW</plugin> </xacro:unless>
```

需要在 `<ros2_control>` 的 `<hardware>` 里加第三个分支（**只有这个文件要改**）：

```xml
<xacro:arg name="use_gazebo" default="false"/>   <!-- 文件顶部 args 区 -->

<!-- hardware 段内，放在 use_fake_hardware 判断之前 -->
<xacro:if value="${use_gazebo}">
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</xacro:if>
<xacro:unless value="${use_gazebo}">
  <!-- 原来的两个分支保持不动 -->
</xacro:unless>
```

再加 gz 插件本身的配置（同一文件末尾，`</ros2_control>` 之后）：

```xml
<xacro:if value="${use_gazebo}">
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find hw_arm_gazebo)/config/hw_arm_gazebo_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</xacro:if>
```

> `use_gazebo` 这个参数现在传进去会被 xacro 忽略（已验证），所以补分支之前 launch 也不会报错。

## 3. 补完之后：起控制器

```bash
ros2 launch hw_arm_gazebo hw_arm_gazebo.launch.py spawn_controllers:=true
```

应该看到：

```
[spawner] loaded 'joint_state_broadcaster'
[spawner] loaded 'hw_arm_effort_controller'
```

然后手动测一下力矩通路：

```bash
# 给 joint2 一个 2 N·m（其余 0），机械臂应该抬起来
ros2 topic pub -1 /hw_arm_effort_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 2.0, 0.0, 0.0, 0.0, 0.0]}"

# 看关节状态
ros2 topic echo /joint_states --once
```

## 4. 下一步：算法节点（对应 launch_mujoco.py 的控制回路）

```
订阅 /joint_states  →  q, qd
hw_arm_algorithms::computeGravity(q)  →  g        ← 同进程直接 link，不用 c_api
tau = kp*(q_des - q) - kd*qd + g
发布 /hw_arm_effort_controller/commands (Float64MultiArray)
```

在 `CMakeLists.txt` 里取消注释那一段，写 `src/gravity_node.cpp` 即可。

## 5. 和 MuJoCo 那边的一一对应

| MuJoCo（hw_arm_mujoco） | Gazebo（hw_arm_gazebo） |
|---|---|
| `mjcf/scene.xml` | `worlds/hw_arm.sdf` |
| `scripts/launch_mujoco.py` 的控制回路 | `src/gravity_node.cpp`（待写） |
| `data.ctrl = tau` | `/hw_arm_effort_controller/commands` |
| `control_gains.yaml`（python 读） | 节点里读同一份 YAML |
| `computeGravity(q)`（ctypes） | `computeGravity(q)`（直接 link）|

**同一个 `hw_arm_algorithms`、同一份 URDF、同样的 1 ms 物理步长**——两边数字才可比。

## 6. 已知差异（对比实验时要注意）

| 项 | MuJoCo | Gazebo |
|---|---|---|
| 控制频率 | 无锁，可 1 kHz | 250 Hz（DDS 调度） |
| 重力真值 | `data.qfrc_bias` 可直接对比 | **没有等价量**，只能看行为 |
| GUI | 硬件 GL 正常 | 本机 VMware 需软件渲染（见 `start_gazebo.sh`） |
| 接触/摩擦模型 | 自己的模型 | ODE/DART 默认，`worlds/hw_arm.sdf` 里 `mu=0.8` 可调 |
