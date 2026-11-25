# 零重力拖拽示教 —— 调试手册

> 2026-11-26 · 从 C-05 辨识转入 hw_arm_algorithms 实测

## 0. 前置条件

| 项 | 要求 |
|---|---|
| 辨识参数 | `~/hw_arm/algo/data/gravity_theta.yaml`（rms **0.184** N·m，holdout 0.221） |
| 环境 | 每个终端都 `source src/hw_arm_description/scripts/env.sh` |
| CAN | `can0` UP，6 台电机在线 |
| **只启动 `joint_state_broadcaster`** | 零重力下 kp=0，轨迹控制器没有意义且会打架 |

```bash
source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh
bash ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/preflight.sh   # 五项必须全 ✅
```

---

## 1. 三个模式的区别（**别搞混**）

| 模式 | kp | tau_ff | 手感 |
|---|---|---|---|
| **位置控制**（默认） | 70 / 70 / 60… | 0 | 很硬，静态有 0.0126 rad 下垂 |
| **重力补偿** | 70 / 70 / 60… | g(q) | 还是硬的，但下垂消失 |
| **零重力** | **0** | g(q) + kd 阻尼 | 失重，手可以随意推动，松手停在原地 |

**零重力模式 kp=0，位置命令完全失效** —— 所以不能同时跑 JTC。

---

## 2. 四道安全机制（已在内核里实现）

| # | 机制 | 参数 | 默认 |
|---|---|---|---|
| ① | **辨识参数**：用 θ̂（0.18 N·m）而不是标称 URDF（0.88 N·m） | `gravity_params_path` | 空 → 标称 |
| ② | **斜坡**：激活后 tau_ff 从 0 线性升到满额 | `gravity_ramp_time` | **3.0 s** |
| ③ | **力矩限幅**：逐轴按连续额定限幅 | `tau_limit` | `40 10 12 3.5 3 3` |
| ④ | **速度守护**：任一轴 > 2 rad/s 就把 kd 提到 4 倍当刹车 | `zero_gravity_vel_limit` | 2.0 |

**⚠️ ④ 只加大阻尼，不去激活** —— 去激活会变成自由落体，更危险。

---

## 3. 分三阶段上（**不要一步跳到零重力**）

### 阶段 1：只算不动（`gravity_dry_run`）

**目的**：确认 θ̂ 在真机上算出的 g(q) 和实际需要的力矩对得上，但**不施加**。

```bash
ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:=can0 rviz:=true \
    controllers:=joint_state_broadcaster,hw_arm_forward_position_controller \
    gravity_compensation:=true \
    gravity_dry_run:=true \
    gravity_params_path:=$HOME/hw_arm/algo/data/gravity_theta.yaml
```

**看日志：**

```
[HwArmHW] 重力参数: .../gravity_theta.yaml
  rank=22  rms=0.184  holdout=0.221  偏移 6 个 / 摩擦 6 个
[HwArmHW] [重力dry-run] g(q)=[...] scale=1.00 → tau_ff=[...]
```

**核对**：`tau_ff` 和 `read()` 里的实测 `effort` 应该**很接近**（差 ≤ 0.2 N·m）。

**这一阶段机械臂行为和现在完全一样**（位置控制，kp=70）。

---

### 阶段 2：重力补偿（`gravity_compensation`，kp 照常）

**目的**：位置控制不变，只是把重力前馈加上，看静态下垂是否变小。

```bash
ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:=can0 rviz:=false \
    controllers:=joint_state_broadcaster,hw_arm_forward_position_controller \
    gravity_compensation:=true \
    gravity_dry_run:=false \
    gravity_scale:=0.5 \
    gravity_ramp_time:=3.0 \
    gravity_params_path:=$HOME/hw_arm/algo/data/gravity_theta.yaml
```

**先 `gravity_scale:=0.5`（半额），确认没问题再改 1.0。**

**验证指标**：把机械臂命令到几个位形，量 `q_cmd − q_meas` 的稳态误差：

| | 无补偿 | 预期（scale=1.0） |
|---|---|---|
| joint2 稳态下垂 | 0.884/70 = **12.6 mrad** | 0.184/70 = **2.6 mrad** |

**这一阶段机械臂仍然是硬的**（kp 没变），掉不下来。

---

### 阶段 3：零重力拖拽（`zero_gravity`）

**⚠️ 这一步机械臂会"失重"。请先用手扶住末端。**

```bash
ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:=can0 rviz:=false \
    controllers:=joint_state_broadcaster \
    zero_gravity:=true \
    zero_gravity_kd:=1.0 \
    gravity_compensation:=true \
    gravity_ramp_time:=5.0 \
    gravity_params_path:=$HOME/hw_arm/algo/data/gravity_theta.yaml
```

**注意：**

```
· controllers 只有 joint_state_broadcaster（不要 trajectory / forward_position）
· gravity_ramp_time:=5.0 —— 给你 5 秒适应，不要一使能就撒手
· 启动后日志会出现：
    ★ 零重力（拖动示教）模式已开启：kp=0 / kd=1.00 / tau_ff=1.00·g(q)
      机械臂会进入'失重'状态 —— 请扶着它，或确认它能自己站稳！
```

**判据（按顺序）：**

| 步骤 | 期望 |
|---|---|
| 1. 使能后 5 秒内 | tau_ff 从 0 升到满额，机械臂**不动**（如果你扶着） |
| 2. 松手 | 机械臂**停住**，不往下溜 |
| 3. 手推末端 | 能推动，阻力均匀，没有"卡住再突然跳" |
| 4. 松手 | 停在原地（允许 ±几度，因为库仑摩擦） |
| 5. 快推一下再松手 | 有阻尼，不会来回振荡 |

**如果 2 失败（往下溜）：** 立刻 Ctrl+C，回到阶段 1 查 tau_ff 和实测 effort 的差。

---

## 4. kd 怎么调

| kd | 手感 |
|---|---|
| 0.5 | 很自由，但推快了会飘、松手会晃 |
| **1.0** | 起步推荐 |
| 2.0 | 明显发黏，适合精细定位 |
| 4.0 | 像泡在蜂蜜里，几乎推不动 |

**MIT 协议 kd 范围 0~5。**

---

## 5. 已知限制（**必须知道**）

| 限制 | 后果 | 缓解 |
|---|---|---|
| 辨识只覆盖 12 个位形 | 范围外 tau_ff 外推不准 | 先在你关心的区域试，超范围时**扶住** |
| 残差 0.184 N·m | 静止时有轻微溜动 | kd ≥ 1.0 |
| 没有重力以外的力补偿 | 推的时候感觉"惯性" | 正常，M(q) 未辨识 |
| 库仑摩擦只辨识了幅值 | 抖动时可能有轻微顿挫 | `tanh(dq/0.05)` 已平滑 |
| **抓手里的东西会变** | 夹了物体后 tau_ff 少算 | 夹物体后**不要**用零重力，或手动加 `gravity_scale` |

---

## 6. 退出

```
Ctrl+C  →  硬件 on_deactivate 会把 kp 恢复成 70 并保持在当前位置
```

**不会掉**：`on_deactivate` 里会先用位置环把当前位形锁住。

---

## 7. 出问题时的排查顺序

| 现象 | 先查 |
|---|---|
| 使能就猛地下垂 | `gravity_params_path` 有没有加载成功（看日志 rank/rms） |
| 逐个位形溜得不一样 | 该位形是否在辨识覆盖范围内 |
| 推起来一顿一顿 | kd 太大或库仑摩擦补偿方向反了 |
| 松手后慢慢漂 | 静摩擦死区（正常，≤ 0.184/70 = 2.6 mrad/s 量级不算问题） |
| 某一轴明显比其他轴差 | 看该轴的 `per_joint_rms`，joint3 是 0.346（最大） |

---

## 8. 退出时的那些 WARN —— **正常，不是故障**

Ctrl+C 之后会出现一屏：

```
[WARN] (hardware 'hw_arm_hardware_interface'): 'joint3/effort' state interface
       not in available list. This should not happen (hint: multiple cleanup calls).
...
[INFO] Shutting down the controller manager.
[ERROR] Exception in publisher thread: context cannot be slept with because it's invalid!
[INFO] process has finished cleanly [pid 230143]
```

**这些都是有序退出的产物：**

| 消息 | 原因 | 要不要管 |
|---|---|---|
| `state interface not in available list ... multiple cleanup calls` | ros2_control 在 deactivate→cleanup 链上重复清理 interface | ❌ 不用管（它自己写了 hint） |
| `context cannot be slept with because it's invalid` | pal_statistics 的发布线程在 rclcpp context 销毁后才 join，上游已知噪音 | ❌ 不用管 |
| `Shutting down the controller manager` | 正在有序关闭 | ✅ 正常 |
| **`process has finished cleanly`** | **判断正常的依据** | ✅ |

**判断标准：**

```
"process has finished cleanly"         → 正常退出
"process has died [pid ... exit code -11/-6]"  → 真的崩了
运行期（不是退出时）出现上面那些 WARN   → 那才是问题
```
