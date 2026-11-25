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

---

# ⚠️ 2026-11-25 试采失败记录（必读）

## 事故经过

```
① bringup 启动成功：6 台电机使能、命令初始化为当前位置、控制器激活
② µ 动诊断：joint2 +15° → τ2 从 0.09 变到 2.03 N·m —— **力矩传感器是好的**
③ 第一次扫 joint2（0 → 0.50 rad，6 个点）：数据与标称模型吻合 0.03~0.12 N·m
④ 第二次扫 → **[ERROR] joint2 报错 0x9（欠压）**
        ↓
   驱动器退出使能 → 机械臂**失去支撑，掉了下来**
        ↓
⑤ 随后 7 台电机全部无响应（电源整个断了）
```

## 根因

`0x9 = 欠压`（DM-J6216P 说明书 line 330：`8——超压；9——欠压`；
line 188：欠压会**退出使能模式**；line 1008：默认阈值 20 V）。

**不是代码 bug，是供电问题**，但暴露了两个设计缺陷：

| 缺陷 | 修正 |
|---|---|
| 一次把 joint2 抬 0.5 rad，6 台电机同时出力，电流峰值大 | 位形按**最近邻排序**，每步只动几度；`--max-j2-torque` 从 8 降到 **6 N·m** |
| 失能/掉电时脚本不知道，还在等 | 加**掉电检测**：运动后位置偏差 > 0.6 rad、或力矩突然全 0、或关节状态中断 → 立即停止 |

## 现在的采集模式：**逐步人工确认**

默认就是交互模式（`--yes` 才自动跑）：

```
[位形 3/20] j2=72.5° j3=51.7° ...  预判 |τ_j2|=3.86 N·m
  ▶ 托好机械臂后按 Enter 开始运动（s 跳过 / q 退出）:
  运动…
  A: τ = [...]
  ▶ Enter 记录 A 并继续；s 丢弃这一条:
  ▶ Enter 执行反向接近（s 跳过 B / q 退出）:
```

**每次物理运动前都要按 Enter**，这样有时间先托住机械臂。

## 最终 20 个位形（τ_j2 ≤ 6 N·m）

| # | j2 | j3 | j4 | j5 | j6 | 预判 τ_j2 |
|---|---|---|---|---|---|---|
| 1 | 72.5 | 51.7 | −5.3 | −13.2 | −22.2 | 3.86 |
| 2 | 43.8 | 93.3 | 9.5 | 37.4 | 10.5 | 1.44 |
| 3 | 101.2 | 23.9 | 32.9 | −19.6 | 23.0 | 4.30 |
| 4 | 29.4 | 65.6 | −33.6 | 58.1 | −60.7 | 2.06 |
| 5 | 86.9 | 107.2 | 15.3 | 11.5 | −32.2 | 4.88 |
| 6 | 58.1 | 37.8 | −22.9 | −12.3 | 35.4 | 2.32 |
| 7 | 79.7 | 14.6 | 28.0 | −27.9 | 37.1 | 2.75 |
| 8 | 50.9 | 56.3 | −14.0 | 64.0 | −52.4 | 1.08 |
| 9 | 36.6 | 28.5 | −5.4 | −25.0 | 34.3 | 0.26 |
| 10 | 65.3 | 111.9 | −10.1 | 29.7 | −16.0 | 1.03 |
| 11 | 122.8 | 42.4 | 29.7 | −36.4 | 25.3 | 5.78 |
| 12 | 18.6 | 84.1 | −29.8 | 39.8 | −42.2 | 4.50 |
| 13 | 76.1 | 125.7 | 18.3 | 8.6 | −8.4 | 2.22 |
| 14 | 47.3 | 19.3 | −23.4 | −13.5 | 48.2 | 1.06 |
| 15 | 33.0 | 102.6 | 11.3 | 47.2 | −1.8 | 3.86 |
| 16 | 90.5 | 33.1 | 22.9 | −27.8 | 58.9 | 4.34 |
| 17 | 61.7 | 74.8 | −17.5 | 57.6 | −26.0 | 2.04 |
| 18 | 25.8 | 47.0 | −6.0 | −3.4 | 49.6 | 1.12 |
| 19 | 83.3 | 88.7 | 10.8 | 11.2 | 14.8 | 4.96 |
| 20 | 54.5 | 130.4 | −6.1 | 43.0 | 19.4 | 2.23 |

（单位：度。运行时按最近邻重新排序，让相邻两步的关节空间距离最小。）

## 命令

```bash
# 启动
ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:=can0 rviz:=false \
    controllers:=joint_state_broadcaster,hw_arm_forward_position_controller

# 采集（默认交互确认；20 位形 × 双向 = 40 个样本）
python3 ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/collect_static_gravity.py \
    --poses 40 --keep 20 --max-j2-torque 6.0 \
    --out ~/hw_arm/algo/data/static_gravity.yaml
```

## 另一个待确认的发现

试采数据里，实测 τ2 匹配的是**裸臂**模型（差 0.03~0.12 N·m），
而不是"含夹爪"模型（后者预测明显更低）。

```
实测   2.205  1.932  1.692  1.487
含夹爪 1.762  1.488  1.199  0.900   ← 差 0.4~0.6 N·m
裸臂   2.132  1.896  1.641  1.371   ← 差 0.03~0.12 N·m
```

⇒ 需要确认：**夹爪是否真的装上了？安装方向是否与假设一致？**
（这不影响辨识本身——辨识会自己吸收夹爪质量，只是影响力矩预判的保守程度。）
