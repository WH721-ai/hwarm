# mini_manip_bt — hw_arm 的 BT 决策层（最小框架）

## 它在系统里的位置

```
用户指令 / 视觉  →  【BT（本包，10 Hz）】  →  Action Goal  →  JTC  →  硬件
                      ↑ 只做"何时做什么、失败怎么办"
                      ✗ 不含 IK、轨迹点、控制增益（那是算法层/控制层的事）
```

## 三个概念，一次讲完

| 概念 | 是什么 | 在哪里 |
|---|---|---|
| **XML** | 树的拓扑（谁先谁后、失败怎么办） | `trees/*.xml` |
| **节点** | 叶子节点 = 一个动作或一个判断 | `src/*_node.cpp` |
| **tick** | 每 100 ms 让整棵树"走一步" | `src/bt_executor.cpp` |

**节点的返回值只有三种：SUCCESS / FAILURE / RUNNING。**
`RUNNING` 表示"没做完，下次 tick 再问我" —— 这是 BT 和普通函数调用的本质区别。

## 跑起来

```bash
# 本包依赖 underlay（BT.CPP 在另一个工作空间）
source /opt/ros/jazzy/setup.bash
source ~/BehaviorTree/BC_ws/install/setup.bash     # ← 必须先 source
source ~/hw_arm/hw_arm_ws/install/setup.bash

colcon build --packages-select mini_manip_bt
```

然后：

```bash
# 终端 1：起机械臂（先用 mock 最安全）
ros2 launch hw_arm_bringup hw_arm.launch.py use_fake_hardware:=true

# 终端 2：跑树
ros2 launch mini_manip_bt bt_demo.launch.py
```

预期输出：

```
[INFO] 已加载行为树: .../trees/goto_home.xml
[1795355535.933]: Sequence              IDLE -> RUNNING
[1795355535.933]: FollowJointTrajectory IDLE -> RUNNING
[1795355540.234]: FollowJointTrajectory RUNNING -> SUCCESS
[1795355540.234]: Sequence              RUNNING -> SUCCESS
[INFO] 行为树结束: SUCCESS
```

## 改树（不用重编译）

编辑 `trees/goto_home.xml`：

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="GotoHome">
    <Sequence>
      <FollowJointTrajectory target="0;0.8;0.4;0;0.6;0" duration="4.0"/>
    </Sequence>
  </BehaviorTree>
</root>
```

* `target` 是 6 个关节角，**用分号分隔**（BT.CPP 的 `vector<double>` 语法）
* `duration` 单位秒
* `action_name` 不给就用默认的 `/hw_arm_controller/follow_joint_trajectory`

改完直接重跑，**不需要 colcon build**（XML 是运行时加载的）。

## 加一个新节点（需要重编译）

在 `include/mini_manip_bt/bt_nodes.hpp` 里声明，在 `src/*.cpp` 实现，
然后在 `bt_executor.cpp` 里注册：

```cpp
factory.registerNodeType<mini_manip_bt::MyNode>("MyNode");
```

XML 里写 `<MyNode/>` —— **注册名必须和 XML 标签一模一样**（大小写敏感）。

## 常用节点写法

| 想做 | 继承 | 要点 |
|---|---|---|
| 发一个 Action | `BT::RosActionNode<ActionT>` | 实现 `setGoal` + `onResultReceived`；**halt 时基类会自动 cancel** |
| 瞬间判断（限位/急停） | `BT::SyncActionNode` | 只返回 SUCCESS/FAILURE，**绝不返回 RUNNING** |
| 需要多 tick 的动作 | `BT::StatefulActionNode` | `onStart`/`onRunning`/`onHalted` |
| 修饰（重试/超时） | 用内置 `RetryUntilSuccessful` / `Timeout` | 不用自己写 |

## 安全相关（照抄教程 M13 的教训）

1. **长动作套安全监控必须用 `ReactiveSequence`**，不是 `Sequence`
   —— `Sequence` 会跳过已 SUCCESS 的子节点，条件只查一次。
2. **动作节点的 `halt` 必须真的取消**（`RosActionNode` 的基类已经做了这件事，
   自己写 `StatefulActionNode` 时要在 `onHalted()` 里 `cancel_goal_async()`）。
3. **BT 不进 1 kHz 伺服环** —— 它跑 10 Hz，急停/超力必须由固件兜底。
