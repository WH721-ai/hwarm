"""激励轨迹的安全检查。

**这是"守门员"，任何轨迹在执行前必须通过这里的全部检查。**

七项检查
    ① 位置越界      ② 速度超限      ③ 加速度超限
    ④ 力矩超限（标称模型 ×安全系数）
    ⑤ 奇异接近      ⑥ 力矩变化率（防跳变）
    ⑦ 端点静止 + 连续性

外加：
    ⑧ 起止衔接（起点是否等于当前位姿）
    ⑨ 重采样检查（内部按 check_rate 重采样，防漏峰）
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

try:
    import pinocchio as pin
except ImportError:  # 允许在没有 pinocchio 时做纯几何检查
    pin = None

from .config import SafetyLimits
from .fourier import Trajectory


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
@dataclass
class SafetyReport:
    ok: bool = True
    violations: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    peaks: Dict[str, np.ndarray] = field(default_factory=dict)
    margins: Dict[str, float] = field(default_factory=dict)
    duration: float = 0.0

    def add_violation(self, msg: str) -> None:
        self.ok = False
        self.violations.append(msg)

    def add_warning(self, msg: str) -> None:
        self.warnings.append(msg)

    def summary(self) -> str:
        lines = []
        status = "✅ PASS" if self.ok else "❌ FAIL"
        lines.append(f"安全检查: {status}   时长 {self.duration:.2f} s")
        for k, v in self.peaks.items():
            arr = np.asarray(v)
            if arr.ndim == 1 and arr.size > 1:
                lines.append(f"  {k:14s} 峰值 {np.array2string(arr, precision=3, suppress_small=True)}")
            else:
                lines.append(f"  {k:14s} {float(arr):.4g}")
        for m in self.warnings:
            lines.append(f"  ⚠️  {m}")
        for v in self.violations:
            lines.append(f"  ❌  {v}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def resample(traj: Trajectory, rate: float) -> Trajectory:
    """把轨迹重采样到指定率。

    注意：q / dq / ddq **各自独立**做三次样条插值，而不是由 q 微分得到。
    原因：对实测数据或人工构造的轨迹，dq/ddq 与 q 的时间导数可能不严格一致，
    若重新微分会丢掉我们要检查的信号（例如恒速段的 dq）。
    """
    from scipy.interpolate import CubicSpline

    t_new = _arange(traj.t[0], traj.t[-1], rate)
    nv = traj.nv
    q = np.empty((t_new.size, nv))
    dq = np.empty_like(q)
    ddq = np.empty_like(q)
    for i in range(nv):
        q[:, i] = CubicSpline(traj.t, traj.q[:, i])(t_new)
        dq[:, i] = CubicSpline(traj.t, traj.dq[:, i])(t_new)
        ddq[:, i] = CubicSpline(traj.t, traj.ddq[:, i])(t_new)
    return Trajectory(t_new, q, dq, ddq, dict(traj.meta))


def _arange(t0: float, t1: float, rate: float) -> np.ndarray:
    n = max(int(round((t1 - t0) * rate)), 2)
    return np.linspace(t0, t1, n)


def compute_joint_torques(model, data, traj: Trajectory,
                          stride: int = 1) -> np.ndarray:
    """用 *标称* 模型算逆动力学力矩 τ(t)。"""
    if pin is None:
        raise RuntimeError("需要 pinocchio")
    idx = np.arange(0, len(traj.t), stride)
    tau = np.empty((idx.size, model.nv))
    for k, i in enumerate(idx):
        pin.rnea(model, data, traj.q[i], traj.dq[i], traj.ddq[i])
        tau[k] = data.tau
    return tau


def compute_singularity(model, data, frame_id, traj: Trajectory,
                        stride: int = 10) -> np.ndarray:
    """逐点计算末端雅可比最小奇异值。"""
    if pin is None:
        raise RuntimeError("需要 pinocchio")
    idx = np.arange(0, len(traj.t), stride)
    sig = np.empty(idx.size)
    for k, i in enumerate(idx):
        pin.computeJointJacobians(model, data, traj.q[i])
        pin.updateFramePlacement(model, data, frame_id)
        J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        sig[k] = np.linalg.svd(J, compute_uv=False)[-1]
    return sig


# ---------------------------------------------------------------------------
# 主检查函数
# ---------------------------------------------------------------------------
def check_trajectory(
    traj: Trajectory,
    limits: SafetyLimits,
    model=None,
    data=None,
    frame_id: Optional[int] = None,
    tau_safety_factor: Optional[float] = None,
    check_rate: Optional[float] = None,
) -> SafetyReport:
    """执行全部安全检查。

    Args:
        traj: 待检查轨迹
        limits: 安全边界
        model, data, frame_id: 提供则可做力矩/奇异检查
        tau_safety_factor: 力矩预估安全系数（默认取 limits 里的）
        check_rate: 重采样率（默认 limits.check_rate）

    Returns:
        SafetyReport（`ok=False` 时**禁止执行**）
    """
    rep = SafetyReport(duration=traj.duration)
    nv = traj.nv

    if nv != limits.q_min.size:
        rep.add_violation(f"自由度不匹配: 轨迹 {nv}, 限制 {limits.q_min.size}")
        return rep

    # ---- ⑨ 重采样（防漏峰）----
    rate = check_rate if check_rate is not None else limits.check_rate
    dt_orig = np.median(np.diff(traj.t))
    if 1.0 / dt_orig < rate * 0.99:
        traj = resample(traj, rate)
        rep.warnings.append(f"已重采样到 {rate:.0f} Hz 做检查（原 {1/dt_orig:.0f} Hz）")

    q, dq, ddq, t = traj.q, traj.dq, traj.ddq, traj.t

    # ---- ① 位置 ----
    q_lo, q_hi = q.min(axis=0), q.max(axis=0)
    rep.peaks["q_min"] = q_lo
    rep.peaks["q_max"] = q_hi
    bad = np.where((q_lo < limits.q_min) | (q_hi > limits.q_max))[0]
    for j in bad:
        rep.add_violation(
            f"joint{j+1} 位置越界: 实际 [{q_lo[j]:.3f}, {q_hi[j]:.3f}], "
            f"允许 [{limits.q_min[j]:.3f}, {limits.q_max[j]:.3f}]")

    # ---- ② 速度 ----
    vpk = np.abs(dq).max(axis=0)
    rep.peaks["dq_max"] = vpk
    rep.margins["v"] = float(np.min(limits.v_max / np.maximum(vpk, 1e-9)))
    bad = np.where(vpk > limits.v_max)[0]
    for j in bad:
        rep.add_violation(
            f"joint{j+1} 速度超限: {vpk[j]:.3f} > {limits.v_max[j]:.3f} rad/s")

    # ---- ③ 加速度 ----
    apk = np.abs(ddq).max(axis=0)
    rep.peaks["ddq_max"] = apk
    bad = np.where(apk > limits.a_max)[0]
    for j in bad:
        rep.add_violation(
            f"joint{j+1} 加速度超限: {apk[j]:.3f} > {limits.a_max[j]:.3f} rad/s²")

    # ---- ④ 力矩（需要模型）----
    if model is not None and data is not None:
        sf = tau_safety_factor if tau_safety_factor is not None else limits.tau_safety_factor
        stride = max(int(round(1.0 / (rate * np.median(np.diff(t))))) if len(t) > 1 else 1, 1)
        tau = compute_joint_torques(model, data, traj, stride=stride)
        tpk = np.abs(tau).max(axis=0) * sf
        rep.peaks["tau_max"] = np.abs(tau).max(axis=0)
        rep.margins["tau"] = float(np.min(limits.tau_hard / np.maximum(tpk, 1e-9)))

        bad = np.where(tpk > limits.tau_hard)[0]
        for j in bad:
            rep.add_violation(
                f"joint{j+1} 力矩超红线: {tpk[j]:.2f} > {limits.tau_hard[j]:.2f} N·m"
                f"（已含安全系数 {sf}）")

        warn = np.where((tpk > limits.tau_warn) & (tpk <= limits.tau_hard))[0]
        for j in warn:
            rep.add_warning(
                f"joint{j+1} 力矩进入黄区: {tpk[j]:.2f} N·m"
                f"（额定 {limits.tau_rated[j]:.2f}, 红线 {limits.tau_hard[j]:.2f}）")

        over_rated = np.where(tpk > limits.tau_rated)[0]
        if over_rated.size and not warn.size:
            rep.add_warning(
                f"力矩超过额定值（关节 {[int(j)+1 for j in over_rated]}），"
                "长时间运行会发热")

        # ---- ⑥ 力矩变化率（防跳变）----
        dt = np.median(np.diff(t))
        tau_rate = np.abs(np.diff(tau, axis=0)).max(axis=0) / dt
        rep.peaks["tau_rate_max"] = tau_rate
        bad = np.where(tau_rate > limits.tau_rate_max)[0]
        for j in bad:
            rep.add_violation(
                f"joint{j+1} 力矩变化率过大: {tau_rate[j]:.1f} > "
                f"{limits.tau_rate_max[j]:.1f} N·m/s（存在跳变）")

        # ---- ⑤ 奇异 ----
        if frame_id is not None:
            sig = compute_singularity(model, data, frame_id, traj,
                                      stride=max(len(t) // 300, 1))
            rep.peaks["sigma_min"] = np.array([sig.min()])
            if sig.min() < limits.sigma_min:
                i = int(np.argmin(sig))
                rep.add_violation(
                    f"接近奇异: σ_min={sig.min():.4f} < {limits.sigma_min}"
                    f"（t≈{t[min(i*max(len(t)//300,1), len(t)-1)]:.2f}s）")

    # ---- ⑦ 端点静止 ----
    if np.abs(dq[0]).max() > 1e-6:
        rep.add_violation(f"起点速度非零: |dq(0)|max={np.abs(dq[0]).max():.3e}")
    if np.abs(dq[-1]).max() > 1e-6:
        rep.add_violation(f"终点速度非零: |dq(-1)|max={np.abs(dq[-1]).max():.3e}")

    # ---- ⑦b 连续性（位置/速度无跳变）----
    dt = np.median(np.diff(t))
    dq_step = np.abs(np.diff(dq, axis=0)).max()
    if dq_step > np.max(limits.a_max) * dt * 3.0:
        rep.add_violation(f"速度存在跳变: 单步最大变化 {dq_step:.4f} rad/s")

    return rep


# ---------------------------------------------------------------------------
# 起点衔接检查
# ---------------------------------------------------------------------------
def check_start_pose(traj: Trajectory, q_current: np.ndarray,
                     tol: float = 0.05) -> SafetyReport:
    """检查轨迹起点是否与机器人当前位姿一致。"""
    rep = SafetyReport(duration=traj.duration)
    diff = np.abs(traj.q[0] - q_current)
    rep.peaks["start_pose_error"] = diff
    if diff.max() > tol:
        rep.add_violation(
            f"起点与当前位姿偏差过大: max={diff.max():.3f} rad > {tol}"
            "（应插入过渡段）")
    return rep
