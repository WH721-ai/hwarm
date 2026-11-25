# C-05 静态重力辨识 —— 运行手册

> 2026-11-25 · 机械臂已上电

## 0. 上电后的通路检查（已完成）

| 项 | 结果 |
|---|---|
| `can0` | ✅ UP，0 错误 |
| USB-CAN | ✅ `1d50:606f` |
| ID 1~6（joint1~6） | ✅ 全部在线，**ENABLED** |
| ID 7（夹爪） | ✅ 在线，DISABLED |
| 保持稳定性 | ✅ 3 秒内漂移 0.00 mrad（joint5 0.3 mrad） |

**当前电机原始位置（= 关节坐标 × direction，j2/j3 为 −1）：**

```
joint1  0.0315    joint4  0.0547
joint2  0.0086    joint5 -0.0154
joint3  0.0128    joint6  6.2556   ← 见下
```

⚠️ **joint6 读到 6.2556 rad（358°）**，而之前是 −0.03 rad，差**正好 2π**。
- 物理姿态相同 ⇒ **对重力无影响**（绕工具轴整圈）
- URDF 限位是 ±6.2832（±360°），所以数值合法
- 但如果是**真的转了一整圈**，线束可能被拧了一圈 —— 请肉眼确认一下

## 1. 启动（用 forward_position_controller，不要用 JTC）

```bash
ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:=can0 rviz:=false \
    controllers:=joint_state_broadcaster,hw_arm_forward_position_controller
```

**为什么不用 JTC**：JTC 的到位容差是 0.09 rad，
而重力补偿关闭时 joint2 会下垂 g/kp ≈ 8/70 = 0.11 rad > 0.09 → 会 abort。

**安全前提（都已默认满足）：**

| 参数 | 默认 | 说明 |
|---|---|---|
| `gravity_compensation` | **false** | 必须关！否则 τ_ff 抵消重力，测到的是残差 |
| `gravity_dry_run` | true | |
| `zero_gravity` | false | |
| `return_to_zero` | **false** | 上电不自动回零 |
| `pos_commands_` 初始化 | = 当前实测位置 | 不会猛冲（插件里已修过这个 bug） |

## 2. 验证通路（不动机械臂）

```bash
# ① 位形表 + 力矩预判
python3 ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/collect_static_gravity.py --dry-run

# ② 确认 joint_states 是**真机数据**（不是仿真）
ros2 topic echo --once /joint_states | head -20
#    position 应该接近 [0.03, -0.009, -0.013, 0.055, -0.015, 6.256]
#    如果看到 [0.0, 0.8, 0.4, 0.0, 0.6, 0.0] → 是假的，有别的发布器在抢话题

# ③ 确认 effort 非零
ros2 topic echo --once /joint_states | grep -A6 effort
```

## 3. 保守试跑（只走 2 个位形）

```bash
python3 ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/collect_static_gravity.py \
    --poses 2 --out /tmp/c05_smoke.yaml
```

看三件事：
- 能动、不猛冲
- `wait_settle` 能触发（不是一直等）
- 采到的 τ 量级合理（joint2 应该在 0.2~8 N·m 之间）

## 4. 全量采集

```bash
python3 ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/collect_static_gravity.py \
    --poses 24 --out ~/hw_arm/algo/data/static_gravity.yaml
```

预计 23 个位形 × 2 方向 × (2 s 运动 + ~2 s 稳定) ≈ 3~4 分钟。

## 5. 拟合

```bash
~/hw_arm/hw_arm_ws/build/hw_arm_algorithms/gravity_id_cli \
  --urdf ~/hw_arm/hw_arm_ws/install/hw_arm_description/share/hw_arm_description/urdf/hw_arm.urdf \
  --samples ~/hw_arm/algo/data/static_gravity.yaml \
  --out ~/hw_arm/algo/data/gravity_theta.yaml
```

**验收标准：**

| 指标 | 目标 |
|---|---|
| `rank` | 10（本臂实测有效秩） |
| `rms` | < 0.2 N·m（上一版是 0.198，只比标称好 1.1 倍 —— 不合格） |
| `holdout_rms` | 与 `rms` 同量级（差太多说明过拟合） |

## 6. 排错顺序

拟合残差大时，**按这个顺序查**（不要一上来就怀疑算法）：

1. **位形覆盖不足** —— `joint2/joint3` 是否只在几个值上（Halton 序列已解决）
2. **摩擦没抵消** —— 是否双向采集；看日志里"A/B 两组 τ 的差"是否明显
3. **力矩零点偏移** —— 某个关节在所有位形上残差都偏一个常数
4. **direction 错了** —— j2/j3 残差符号异常
5. **采集时有运动** —— `wait_settle` 没触发就到了采样
