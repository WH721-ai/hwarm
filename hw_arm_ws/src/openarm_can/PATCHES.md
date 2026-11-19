# 本地补丁（相对上游 enactic/openarm_can）

本目录是 `https://github.com/enactic/openarm_can` 的源码，**带一处本地修改**，
直接随 hw_arm 仓库一起提交（不再用 git submodule），这样 `git clone` 之后
`colcon build` 直接就能过，不需要额外的打补丁步骤。

## 补丁：新增两种电机型号

上游的 `MotorType` 枚举里没有 **DM6216P** 和 **DM4310P**，但 hw_arm 的
joint2 / joint4 用的就是这两款。

改动文件：`include/openarm/damiao_motor/dm_motor_constants.hpp`

```cpp
 enum class MotorType : uint8_t {
     ...
     DMG6220 = 12,
+    DM6216P = 13,   // joint2   ← 本地新增
+    DM4310P = 14,   // joint4   ← 本地新增
-    COUNT = 13
+    COUNT = 15
 };

 inline constexpr std::array<LimitParam, ...> MOTOR_LIMIT_PARAMS = {{
     ...
+    {12.5, 20, 120},  // DM6216P
+    {12.5, 10, 30}    // DM4310P
 }};
```

## ⚠️ 量程是**占位值**

| 型号 | pMax | vMax | tMax | 来源 |
|---|---|---|---|---|
| DM6216P | 12.5 | 20 | **120** | 暂借 DM6248P |
| DM4310P | 12.5 | 10 | **30** | 估计值 |

MIT 协议的 12 bit 力矩字段按 `tMax` 归一化，**`tMax` 错了扭矩就不准**：

```
实际下发 = 请求值 × (真实 tMax / 表里的 tMax)
```

- `tMax` 填**大**了 → 实际出力**偏小**（偏安全，但重力补偿会不够）
- `tMax` 填**小**了 → 实际出力**偏大**（危险，可能过流/撞限位）

拿到达妙手册后请把这两个值改成真值，并同步修改：

- `hw_arm_description/scripts/dm_probe.py` 的 `LIMITS`
- `hw_arm_mujoco/scripts/urdf_to_mjcf.py` 的 `MOTORS[...].tmax`

## 上游更新时怎么合

```bash
# 在上游仓库拉最新，然后：
git diff <上游基线> -- include/ > /tmp/upstream.patch
# 把 PATCHES.md 里那 4 行改动重新贴到 dm_motor_constants.hpp
```
