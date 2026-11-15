"""激励轨迹优化（Fourier 参数 + SLSQP）。

目标：在安全约束下最大化可辨识性（最小化基参数回归矩阵的条件数）。

优化变量::

    x = [q0 (nv), A (nv*K), B (nv*K)]        共 nv*(2K+1) 个

约束（每个采样点）::

    q_min ≤ q(t) ≤ q_max
    |q̇(t)| ≤ v_max
    |q̈(t)| ≤ a_max
    |τ(t)|·安全系数 ≤ τ_design      ← 用标称模型预估

由于优化器只看条件数，**最终轨迹必须再过一遍 safety.check_trajectory**。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    pin = None

from .config import ExcitationConfig, SafetyLimits
from .fourier import FourierParams, build_excitation_trajectory
from .regressor import condition_number


@dataclass
class DesignResult:
    params: FourierParams
    condition_number: float
    rank: int
    initial_condition: float
    success: bool
    n_iter: int
    message: str


def design_excitation(
    model,
    data,
    limits: SafetyLimits,
    cfg: ExcitationConfig,
    q_center: Optional[np.ndarray] = None,
    log: Optional[Callable[[str], None]] = None,
    frame_id: Optional[int] = None,
) -> DesignResult:
    """设计 Fourier 激励轨迹。

    Args:
        model, data: pinocchio 模型
        limits: 安全边界（q/v/a/τ）
        cfg: 设计配置
        q_center: 中心位置（默认取限位中点）
        log: 日志回调

    Returns:
        DesignResult
    """
    if pin is None:
        raise RuntimeError("需要 pinocchio")
    say = log or (lambda s: None)

    nv = model.nv
    K = cfg.n_harmonics
    T = cfg.duration
    omega = 2.0 * np.pi / T
    nv_free = nv * (2 * K + 1)
    q_center = (limits.q_min + limits.q_max) / 2.0 if q_center is None else q_center

    # 优化用的粗采样网格
    t_opt = np.linspace(0.0, T, cfg.n_samples_opt, endpoint=False)

    def unpack(x: np.ndarray) -> FourierParams:
        return FourierParams.from_vector(x, nv, K, omega)

    def evaluate(x: np.ndarray):
        return unpack(x).evaluate(t_opt)

    def torques(x: np.ndarray) -> np.ndarray:
        q, dq, ddq = evaluate(x)
        tau = np.empty_like(q)
        for i in range(q.shape[0]):
            pin.rnea(model, data, q[i], dq[i], ddq[i])
            tau[i] = data.tau
        return tau

    def objective(x: np.ndarray) -> float:
        """条件数（越小越好），基于基参数子空间。"""
        q, dq, ddq = evaluate(x)
        Ys = []
        step = max(len(t_opt) // 40, 1)
        for i in range(0, len(t_opt), step):
            Yi = pin.computeJointTorqueRegressor(model, data, q[i], dq[i], ddq[i])
            Ys.append(np.asarray(Yi))
        Y = np.vstack(Ys)
        c, _ = condition_number(Y)
        if not np.isfinite(c) or c > 1e12:
            return 1e12
        return float(np.log(c))

    def constraints(x: np.ndarray) -> np.ndarray:
        """全部约束写成 g(x) ≤ 0 的形式。"""
        q, dq, ddq = evaluate(x)
        tau = torques(x) * limits.tau_safety_factor
        out = [
            q.max(axis=0) - limits.q_max,
            limits.q_min - q.min(axis=0),
            np.abs(dq).max(axis=0) - limits.v_max * 0.95,
            np.abs(ddq).max(axis=0) - limits.a_max * 0.95,
            np.abs(tau).max(axis=0) - limits.tau_rated * 0.95,
        ]
        # 奇异约束（粗网格，末端可操作度）
        if cfg.enforce_singularity:
            sig = _min_sigma(model, data, q, frame_id, stride=5)
            out.append(np.array([limits.sigma_min * 1.2 - sig]))
        return np.concatenate(out)

    def f_scalar(x):
        return objective(x)

    def f_constraint(x):
        return constraints(x)

    # ---- 多起点 ----
    from scipy.optimize import minimize

    best = None
    best_val = np.inf
    rng = np.random.default_rng(cfg.random_seed)

    # 幅值上下界：初始化幅值 + 位置余量
    amp = np.minimum(cfg.amplitude_init,
                     np.minimum(q_center - limits.q_min,
                                limits.q_max - q_center).min() * 0.5)
    amp = max(float(amp), 0.02)

    for start in range(cfg.n_starts):
        x0 = np.zeros(nv_free)
        x0[:nv] = q_center
        # 注意：**不能用零幅值起点**。
        # 目标函数（条件数）对整体幅值缩放不变，在 A=B=0 处梯度为零，
        # 优化器会立刻停在原地（实测 nit=1）。必须给一个非零随机幅值。
        x0[nv:] = amp * rng.standard_normal(nv * 2 * K) / np.sqrt(K)

        # q0 只允许在中心附近小范围移动：
        # 否则过渡段（当前位姿 → Fourier 起点）会变得很长，速度超限。
        q0_pad = min(0.05, amp)
        bounds = ([(max(limits.q_min[i], q_center[i] - q0_pad),
                    min(limits.q_max[i], q_center[i] + q0_pad)) for i in range(nv)]
                  + [(-amp * 3, amp * 3)] * (nv * 2 * K))

        try:
            res = minimize(
                f_scalar, x0, method=cfg.optimizer, bounds=bounds,
                constraints=[{"type": "ineq", "fun": lambda x: -f_constraint(x)}],
                options={"maxiter": cfg.max_iter, "ftol": cfg.ftol},
            )
        except Exception as exc:      # noqa: BLE001
            say(f"  起点 {start+1} 失败: {exc}")
            continue

        val = objective(res.x)
        say(f"  起点 {start+1}/{cfg.n_starts}: log(cond)={val:.3f} "
            f"(cond={np.exp(val):.1f}) success={res.success} nit={res.nit}")
        if val < best_val:
            best_val, best = val, res

    if best is None:
        raise RuntimeError("所有起点都失败")

    params = unpack(best.x)
    Y = _stack_regressor(model, data, params, t_opt)
    c, r = condition_number(Y)
    c0 = float(np.exp(objective(np.concatenate([q_center, np.zeros(nv * 2 * K)]))))

    return DesignResult(
        params=params, condition_number=c, rank=r,
        initial_condition=c0, success=bool(best.success),
        n_iter=int(best.nit), message=str(best.message),
    )


def _stack_regressor(model, data, params: FourierParams, t: np.ndarray,
                     max_points: int = 60) -> np.ndarray:
    q, dq, ddq = params.evaluate(t)
    step = max(len(t) // max_points, 1)
    Ys = []
    for i in range(0, len(t), step):
        Yi = pin.computeJointTorqueRegressor(model, data, q[i], dq[i], ddq[i])
        Ys.append(np.asarray(Yi))
    return np.vstack(Ys)


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def _min_sigma(model, data, qs: np.ndarray, frame_id: Optional[int],
               stride: int = 5) -> float:
    """采样点上的最小奇异值（越大越好）。"""
    if frame_id is None:
        return 1e9
    best = np.inf
    for i in range(0, len(qs), stride):
        pin.computeJointJacobians(model, data, qs[i])
        pin.updateFramePlacement(model, data, frame_id)
        J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        best = min(best, float(np.linalg.svd(J, compute_uv=False)[-1]))
    return best


def repair_by_scale(
    params: FourierParams,
    limits: SafetyLimits,
    model,
    data,
    frame_id: Optional[int] = None,
    q_start: Optional[np.ndarray] = None,
    transition_time: float = 1.5,
    n_samples: int = 400,
    target: float = 0.98,
    max_iter: int = 22,
) -> Tuple[FourierParams, float]:
    """按比例缩小幅值，直到**所有**约束在 target 比例内满足。

    为什么需要：优化器（SLSQP）有迭代上限，可能未完全收敛；而**安全不能妥协**。
    缩小幅值会同时降低速度、加速度、力矩和位置偏移，是最稳妥的修复方式。

    注意：奇异度不是幅值的线性函数，但缩小幅值会让轨迹向中心靠拢，
    如果中心位形是良态的，最终能满足（否则说明中心选得不好）。

    Args:
        target: 目标利用率（0.98 = 留 2% 余量）

    Returns:
        (修复后的参数, 缩放系数 α)
    """
    T = 2.0 * np.pi / params.omega
    span = np.maximum(limits.q_max - limits.q_min, 1e-6)

    def utilization(p: FourierParams) -> float:
        """最大利用率；> 1 表示违反约束。

        注意：这里评估的是**完整轨迹**（含起止过渡段）。
        否则优化器把 q0 挪远后，过渡段速度会超限而检查不到。
        """
        if q_start is not None:
            traj = build_excitation_trajectory(
                p, q_start=q_start, v_start=None, a_start=None,
                n_periods=1, transition_time=transition_time,
                rate=max(n_samples / T, 50.0))
            q, dq, ddq = traj.q, traj.dq, traj.ddq
        else:
            t = np.linspace(0.0, T, n_samples, endpoint=False)
            q, dq, ddq = p.evaluate(t)
        tau = np.empty_like(q)
        for i in range(q.shape[0]):
            pin.rnea(model, data, q[i], dq[i], ddq[i])
            tau[i] = data.tau
        u = 0.0
        u = max(u, float(np.max((q - limits.q_max) / span)))
        u = max(u, float(np.max((limits.q_min - q) / span)))
        u = max(u, float(np.max(np.abs(dq) / limits.v_max)))
        u = max(u, float(np.max(np.abs(ddq) / limits.a_max)))
        u = max(u, float(np.max(np.abs(tau) * limits.tau_safety_factor
                               / limits.tau_rated)))
        if frame_id is not None:
            sig = _min_sigma(model, data, q, frame_id, stride=max(len(q) // 250, 1))
            if sig < 1e18:
                u = max(u, float(limits.sigma_min / max(sig, 1e-12)))
        return u

    cur = FourierParams(params.q0.copy(), params.A.copy(), params.B.copy(),
                        params.omega)
    if utilization(cur) <= target:
        return cur, 1.0

    lo, hi = 0.0, 1.0
    best_alpha, best_p = 0.0, cur
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        cand = FourierParams(params.q0.copy(), params.A * mid,
                             params.B * mid, params.omega)
        if utilization(cand) <= target:
            best_alpha, best_p = mid, cand
            lo = mid
        else:
            hi = mid
    if best_alpha <= 1e-9:
        best_alpha = 1e-3
        best_p = FourierParams(params.q0.copy(), params.A * best_alpha,
                               params.B * best_alpha, params.omega)
    return best_p, best_alpha
