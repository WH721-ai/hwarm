# m30_sysid —— 激励轨迹设计与物理参数辨识

> 离线工具链：**设计激励轨迹 → 安全检查 → 上机采集 → 辨识惯性参数**。
> 不依赖 IPOPT / FIGAROH，只需要 `numpy + scipy + pinocchio`。

---

## 1. 快速开始

```bash
cd ~/hw_arm/algo
source /opt/ros/jazzy/setup.bash

# ① 设计激励轨迹（默认 K=5, T=6s, 2 个周期）
python3 scripts/design_excitation.py \
    --urdf /tmp/hw_arm.urdf \
    --out data/excitation \
    --harmonics 5 --duration 6 --periods 2

# ② 输出
#   data/excitation/excitation.csv          轨迹（t, q, dq, ddq）
#   data/excitation/excitation_report.txt   安全检查 + 条件数
#   data/excitation/excitation_params.npz   Fourier 系数

# ③ 上机执行后（250 Hz 记录 q/dq/tau）→ 辨识
python3 scripts/identify_params.py \
    --urdf /tmp/hw_arm.urdf \
    --data recorded.csv \
    --out data/sysid

# ④ 离线自检（不需要真机）
python3 scripts/identify_params.py --urdf /tmp/hw_arm.urdf \
    --data data/excitation/excitation.csv --simulate
```

---

## 2. 模块结构

```
src/m30_sysid/
├── config.py     电机规格、安全边界、设计与辨识配置
├── fourier.py    Fourier 轨迹 + 五次多项式过渡段 + Trajectory 容器
├── safety.py     七项安全检查（**执行前的守门员**）
├── regressor.py  回归矩阵 + 基参数约简
├── design.py     激励轨迹优化（SLSQP）+ 幅值缩放修复
└── identify.py   参数辨识（加权最小二乘 + 物理一致性）
```

---

## 3. 数学基础

### 3.1 辨识方程

```
τ = Y(q, q̇, q̈) · θ + ε
θ̂ = (YᵀY)⁻¹ Yᵀ τ
Cov(θ̂) = σ² (YᵀY)⁻¹          ← 轨迹决定误差！
```

**"设计激励轨迹" = "最小化 (YᵀY)⁻¹" = "最大化信息矩阵"**。

### 3.2 Fourier 形式（Swevers et al. 1997）

```
q_i(t) = q_i0 + Σ_{k=1}^{K} [ a_ik/(kω) · sin(kωt) − b_ik/(kω) · cos(kωt) ]
q̇_i(t) = Σ_{k=1}^{K} [ a_ik · cos(kωt) + b_ik · sin(kωt) ]
q̈_i(t) = Σ_{k=1}^{K} kω · [ −a_ik · sin(kωt) + b_ik · cos(kωt) ]
```

**除以 (kω) 的作用**：各阶谐波速度幅值相等 → 速度天然有界。

**优点**：导数全解析（无数值微分噪声）、周期性（可多周期平均降噪）、光滑 C^∞。

### 3.3 基参数约简（**必须做**）

`Y` 永远秩亏。实测我们的臂：

| 数据 | 秩 |
|---|---|
| 只有重力（q̇ = q̈ = 0） | **10 / 60** |
| 完整动力学 | **36 / 60** |

用 SVD 找可辨识子空间 V（P×b），只在 b 维里求解：

```python
alpha, *_ = np.linalg.lstsq(Y @ V, tau)      # b 个参数，无偏
theta = theta_nominal + V @ (alpha - V.T @ theta_nominal)
```

**不要**直接对 60 个参数做 Tikhonov —— 会把不可辨识方向压向 0，得到有偏解。

---

## 4. 安全检查（七项）

| # | 检查 | 依据 |
|---|---|---|
| ① | 位置越界 | URDF 限位 + 5% 边距 |
| ② | 速度超限 | `min(电机 vMax, 运行限速)` |
| ③ | 加速度超限 | `vMax/0.1 × scale` |
| ④ | **力矩超限** | 标称模型 ×1.5 安全系数 vs `τ_rated/τ_warn/τ_hard` |
| ⑤ | 奇异接近 | 末端雅可比 `σ_min > 0.01` |
| ⑥ | **力矩变化率** | `|Δτ|/Δt < τ_rated/0.1`（防跳变） |
| ⑦ | 端点静止 + 连续性 | `|dq(0)|, |dq(T)| < 1e-6`，单步无跳变 |

**三条力矩线**：

```
τ_rated = τ_peak × 0.35    ← 期望工作区（优化目标）
τ_warn  = τ_peak × 0.70    ← 黄线（警告）
τ_hard  = τ_peak × 0.80    ← 红线（绝不允许）
```

**关键实现细节**：
- 检查内部按 **1 kHz 重采样**（控制周期 250 Hz 的 4 倍），防止漏掉采样点之间的峰值
- 重采样时 q/dq/ddq **各自独立**做样条插值（不从 q 微分）

---

## 5. 实测踩过的四个坑

### 坑 1：零幅值起点 → 优化器不动

条件数对整体幅值缩放**不变**，在 A=B=0 处梯度为零，SLSQP 立刻停（nit=1）。
→ **必须给非零随机初值**。

### 坑 2：q0 漂移 → 过渡段速度爆炸

优化器把中心 q0 挪远后，从起点到 Fourier 起点的过渡段变长，速度超限（实测 joint4 达 1.52 rad/s）。
→ **q0 限制在中心 ±0.05 rad**，且修复时评估**完整轨迹**（含过渡段）。

### 坑 3：joint5 穿过 0 → 腕奇异

实测：轨迹经过 `q5 ≈ 0` 时 `σ_min` 掉到 **8e-4**（阈值 0.01）。
→ 设计时把 joint5 中心推到 **±0.30 rad**，轨迹不再穿越奇异点。

### 坑 4：约束贴着边界 → 检查失败

优化器会把速度推到恰好等于上限，1 kHz 检查时略超（1.001 > 1.000）。
→ 优化约束留 **5% 余量**，修复目标 **0.98**。

---

## 6. 基线数据（我们的臂，4.2688 kg）

| 轨迹类型 | 条件数 | 秩 |
|---|---|---|
| 单关节轮流正弦（K=1） | 8,806,241 | 33 |
| 全关节同频同相（K=1） | 8,806,241 | 33 |
| 全关节同频不同相 | 48,727 | 36 |
| 多频 Fourier（K=3，随机） | 166.8 | 36 |
| **本工具（K=3 + 优化 + 修复）** | **~100** | **36** |
| **本工具（K=5 + 优化 + 修复）** | **~108** | **36** |

**辨识质量**（仿真自检，注入 0.05 N·m 噪声）：

```
回归矩阵秩    : 36 / 60
条件数        : 116.6
训练残差 RMS  : 0.0502 N·m   ← 等于噪声水平
留出验证 RMS  : 0.0501 N·m   ← 无过拟合
总质量        : 4.2578 kg（标称 4.2688，差 0.3%）
```

---

## 7. 已知限制

| 限制 | 说明 | 缓解 |
|---|---|---|
| **单连杆惯量不唯一** | 基参数解只确定 36 个组合，单连杆惯量矩阵可能非正定 | 用于**控制**没问题（预测准确）；写回 URDF 时建议**保留标称惯量张量，只更新质量和质心** |
| **关节限位窄** | joint5 只有 ±0.785 rad，激励空间受限 | 条件数下限比一般机器人高 |
| **力矩观测有摩擦污染** | 电机回报的 τ 含摩擦（0.1~0.56 N·m） | 高速段摩擦占比小；或先做摩擦辨识 |
| **需要真实 dq/ddq** | 实测只有 q 时要用 SG 滤波求导 | `--smooth-window` 可调；窗口过小噪声大，过大削峰 |

---

## 8. 上机流程（务必按顺序）

```
1. 用 jog.py 移动到设计时用的起始位姿
2. 以 30% 幅值试跑（改 excitation.csv 的 q/dq/ddq 列乘 0.3？不要！
   正确做法：用 --amplitude 0.05 重新设计，或用 repair 后的 α）
3. 观察：跟踪误差 < 0.05 rad、无异常声音、温升 < 2℃/min
4. 全幅值执行，250 Hz 记录 q/dq/tau
5. 辨识 → 检查留出验证 RMS 是否接近噪声水平
6. 用辨识参数替换 URDF，重新验证重力补偿
```

---

## 9. 参考文献

1. **Swevers et al.**, "Optimal Robot Excitation and Identification",
   IEEE T-RA 13(5), 1997 —— Fourier 形式 + D-optimality
2. **Khalil & Dombre**, *Modeling, Identification and Control of Robots*, Ch. 9
   —— 回归矩阵与基参数
3. **Gautier & Khalil**, "Direct calculation of minimum set of inertial
   parameters", IEEE T-RA, 1990 —— 基参数计算
4. **Pukelsheim**, *Optimal Design of Experiments* —— A/D/E 最优性
