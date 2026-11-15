"""物理参数辨识：从实测数据估计惯性参数。

流程::

    ① 用实测 q, q̇, q̈ 构建回归矩阵 Y
    ② SVD 求基参数子空间（秩 b ≪ 10n）
    ③ 加权最小二乘（权重 = 1/τ_peak，平衡各关节量级）
    ④ 基参数 → 全参数（最小范数或保标称值）
    ⑤ 物理一致性检查 + 留出集验证

关键点
    · **必须做基参数约简**：直接对 10n 个参数做 Tikhonov 会得到有偏解
    · **必须用实测 q/dq/ddq**：用参考轨迹等于假设完美跟踪
    · **必须做留出验证**：训练残差小不代表参数对
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    pin = None

from .config import DEFAULT_MOTORS, IdentificationConfig
from .fourier import Trajectory
from .regressor import (BaseParameterBasis, base_parameter_basis,
                        build_regressor_from_arrays, nominal_parameters)


# ---------------------------------------------------------------------------
# 结果
# ---------------------------------------------------------------------------
@dataclass
class IdentificationResult:
    theta: np.ndarray                      # (P,) 全参数
    theta_base: np.ndarray                 # (b,) 基参数
    basis: BaseParameterBasis
    rank: int
    condition_number: float
    residual_rms: float                    # 训练集 RMS N·m
    residual_max: float                    # 训练集最大残差 N·m
    validation_rms: Optional[float] = None
    validation_max: Optional[float] = None
    per_joint_rms: Optional[np.ndarray] = None
    physical_ok: bool = True
    physical_issues: List[str] = field(default_factory=list)
    link_parameters: Optional[List[Dict]] = None

    def summary(self) -> str:
        lines = [
            "参数辨识结果",
            f"  回归矩阵秩      : {self.rank} / {self.basis.n_full}",
            f"  条件数          : {self.condition_number:.1f}",
            f"  训练残差 RMS    : {self.residual_rms:.4f} N·m",
            f"  训练残差 最大   : {self.residual_max:.4f} N·m",
        ]
        if self.validation_rms is not None:
            lines.append(f"  留出验证 RMS    : {self.validation_rms:.4f} N·m")
            lines.append(f"  留出验证 最大   : {self.validation_max:.4f} N·m")
        if self.per_joint_rms is not None:
            arr = np.array2string(self.per_joint_rms, precision=3, suppress_small=True)
            lines.append(f"  分关节 RMS      : {arr}")
        lines.append(f"  物理一致性      : {'✅ 通过' if self.physical_ok else '❌ 有问题'}")
        for it in self.physical_issues:
            lines.append(f"     · {it}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# 辨识
# ---------------------------------------------------------------------------
def identify(
    model,
    data,
    traj: Trajectory,
    tau_measured: np.ndarray,
    cfg: Optional[IdentificationConfig] = None,
    validation: Optional[tuple] = None,
    motors=DEFAULT_MOTORS,
) -> IdentificationResult:
    """从实测数据辨识惯性参数。

    Args:
        model, data: pinocchio 模型
        traj: 实测轨迹（**必须用实测 q/dq/ddq**）
        tau_measured: (N, nv) 实测关节力矩
        cfg: 辨识配置
        validation: 可选 (traj_val, tau_val) 用于留出验证
        motors: 电机表（用于加权）

    Returns:
        IdentificationResult
    """
    if pin is None:
        raise RuntimeError("需要 pinocchio")
    cfg = cfg or IdentificationConfig()

    Y = build_regressor_from_arrays(model, data, traj.q, traj.dq, traj.ddq)
    tau = np.asarray(tau_measured, float).reshape(-1)
    if Y.shape[0] != tau.size:
        raise ValueError(f"回归矩阵行数 {Y.shape[0]} != 力矩长度 {tau.size}")

    theta_nom = nominal_parameters(model)
    basis = base_parameter_basis(Y, theta_nominal=theta_nom, rtol=cfg.svd_rtol)

    # ---- 加权 ----
    nv = model.nv
    if cfg.use_weighting:
        w = 1.0 / np.array([m.tau_peak for m in motors])
        W = np.tile(w, Y.shape[0] // nv)
    else:
        W = np.ones(Y.shape[0])

    Yw = Y * W[:, None]
    tw = tau * W

    # ---- 基参数求解 ----
    Yb = Yw @ basis.V
    if cfg.regularization > 0:
        n = Yb.shape[1]
        alpha = np.linalg.solve(Yb.T @ Yb + cfg.regularization * np.eye(n),
                                Yb.T @ tw)
    else:
        alpha, *_ = np.linalg.lstsq(Yb, tw, rcond=None)

    theta = basis.from_base(alpha, theta_nominal=theta_nom)

    # ---- 训练残差 ----
    pred = Y @ theta
    err = pred - tau
    rms = float(np.sqrt(np.mean(err**2)))
    max_err = float(np.max(np.abs(err)))
    per_joint = np.sqrt(np.mean(err.reshape(-1, nv)**2, axis=0))

    # ---- 条件数 ----
    s = basis.singular_values
    cond = float(s[0] / s[basis.rank - 1])

    # ---- 留出验证 ----
    val_rms = val_max = None
    if validation is not None:
        traj_v, tau_v = validation
        Yv = build_regressor_from_arrays(model, data, traj_v.q, traj_v.dq, traj_v.ddq)
        tv = np.asarray(tau_v, float).reshape(-1)
        ev = Yv @ theta - tv
        val_rms = float(np.sqrt(np.mean(ev**2)))
        val_max = float(np.max(np.abs(ev)))

    # ---- 物理一致性 ----
    physical_ok, issues, links = (True, [], None)
    if cfg.check_physical_consistency:
        physical_ok, issues, links = check_physical_consistency(
            model, theta, cfg)

    return IdentificationResult(
        theta=theta, theta_base=alpha, basis=basis, rank=basis.rank,
        condition_number=cond, residual_rms=rms, residual_max=max_err,
        validation_rms=val_rms, validation_max=val_max,
        per_joint_rms=per_joint, physical_ok=physical_ok,
        physical_issues=issues, link_parameters=links,
    )


# ---------------------------------------------------------------------------
# 物理一致性
# ---------------------------------------------------------------------------
def check_physical_consistency(model, theta: np.ndarray,
                               cfg: IdentificationConfig):
    """检查辨识出的惯性参数是否物理可行。

    注意：基参数解**不唯一**（零空间里的分量任意），所以这里检查的是
    我们选取的那个代表解（最小范数/保标称）。严重违反说明基参数本身有问题。

    检查项：
        · 质量 > 0
        · 惯量矩阵正定
        · 三角不等式 I1 + I2 ≥ I3
    """
    issues: List[str] = []
    links: List[Dict] = []
    n_bodies = model.njoints - 1

    for i in range(n_bodies):
        blk = theta[10 * i:10 * (i + 1)]
        m = float(blk[0])
        h = blk[1:4]                     # 一阶矩 m·c
        I = np.array([                   # [Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
            [blk[4], blk[5], blk[7]],
            [blk[5], blk[6], blk[8]],
            [blk[7], blk[8], blk[9]],
        ])
        name = model.names[i + 1] if hasattr(model, "names") else f"link{i+1}"
        links.append({"name": name, "mass": m,
                      "com": (h / m) if abs(m) > 1e-12 else np.zeros(3),
                      "inertia": I.copy()})

        if m < cfg.min_mass:
            issues.append(f"{name}: 质量 {m:.4f} kg < {cfg.min_mass}")
        elif m > cfg.max_mass:
            issues.append(f"{name}: 质量 {m:.1f} kg > {cfg.max_mass}（不合理）")

        eig = np.linalg.eigvalsh(I)
        if eig.min() <= 0:
            issues.append(f"{name}: 惯量矩阵非正定（最小特征值 {eig.min():.3e}）")

        # 三角不等式（主轴近似）
        Ixx, Iyy, Izz = I[0, 0], I[1, 1], I[2, 2]
        if Ixx > 1e-9 and Iyy > 1e-9 and Izz > 1e-9:
            if Ixx + Iyy < Izz * (1 - 1e-6) or \
               Iyy + Izz < Ixx * (1 - 1e-6) or \
               Izz + Ixx < Iyy * (1 - 1e-6):
                issues.append(f"{name}: 违反三角不等式 "
                              f"(Ixx={Ixx:.2e}, Iyy={Iyy:.2e}, Izz={Izz:.2e})")

    return (len(issues) == 0), issues, links


# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------
def export_parameter_report(result: IdentificationResult, path: str) -> None:
    """把辨识结果写成可读报告（含每连杆参数）。"""
    lines = [result.summary(), "", "每连杆参数:"]
    if result.link_parameters:
        for lk in result.link_parameters:
            lines.append(
                f"  {lk['name']:12s} m={lk['mass']:8.4f} kg  "
                f"c=[{lk['com'][0]:+.4f}, {lk['com'][1]:+.4f}, {lk['com'][2]:+.4f}]")
    lines.append("")
    lines.append("全参数向量 θ (10/连杆):")
    for i in range(len(result.theta) // 10):
        blk = result.theta[10 * i:10 * (i + 1)]
        lines.append("  [" + ", ".join(f"{v:+.6e}" for v in blk) + "]")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def theta_to_urdf_inertials(model, theta: np.ndarray) -> List[Dict]:
    """把 θ 转成可写回 URDF 的 (mass, com, inertia) 列表。"""
    out = []
    for i in range(model.njoints - 1):
        blk = theta[10 * i:10 * (i + 1)]
        m = float(blk[0])
        c = blk[1:4] / m if abs(m) > 1e-12 else np.zeros(3)
        I = np.array([
            [blk[4], blk[5], blk[7]],
            [blk[5], blk[6], blk[8]],
            [blk[7], blk[8], blk[9]],
        ])
        out.append({"link": model.names[i + 1], "mass": m,
                    "com": c, "inertia": I})
    return out
