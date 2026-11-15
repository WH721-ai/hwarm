"""回归矩阵与基参数约简。

辨识方程::

    τ = Y(q, q̇, q̈) · θ

Y 的列数 = 10 × 连杆数，但**永远秩亏**，只有"基参数"可辨识：

    rank(Y) = b < 10 n

本模块负责：
    1. 逐点构建回归矩阵
    2. SVD 求可辨识子空间 V_b (10n × b)
    3. 把参数投影到基参数空间
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    pin = None

from .fourier import Trajectory


@dataclass
class BaseParameterBasis:
    """基参数子空间的描述。

    Attributes:
        V:      (P, b) 正交基，P = 10×连杆数
        rank:   b
        singular_values: SVD 奇异值（降序）
        theta_nominal:  标称参数（用于把基参数映射回全参数）
    """

    V: np.ndarray
    rank: int
    singular_values: np.ndarray
    theta_nominal: Optional[np.ndarray] = None

    @property
    def n_full(self) -> int:
        return self.V.shape[0]

    def to_base(self, theta: np.ndarray) -> np.ndarray:
        return self.V.T @ theta

    def from_base(self, alpha: np.ndarray,
                  theta_nominal: Optional[np.ndarray] = None) -> np.ndarray:
        """基参数 → 全参数。默认取最小范数解（以标称值为参考点）。"""
        ref = theta_nominal if theta_nominal is not None else self.theta_nominal
        if ref is None:
            return self.V @ alpha
        return ref + self.V @ (alpha - self.V.T @ ref)


def build_regressor(model, data, traj: Trajectory,
                    stride: int = 1) -> np.ndarray:
    """构建堆叠回归矩阵 Y，形状 (N·nv, P)。"""
    if pin is None:
        raise RuntimeError("需要 pinocchio")
    idx = np.arange(0, len(traj.t), stride)
    Y = np.empty((idx.size * model.nv, 10 * (model.njoints - 1)))
    for k, i in enumerate(idx):
        Yi = pin.computeJointTorqueRegressor(model, data,
                                             traj.q[i], traj.dq[i], traj.ddq[i])
        Y[k * model.nv:(k + 1) * model.nv, :] = np.asarray(Yi)
    return Y


def build_regressor_from_arrays(model, data, q: np.ndarray, dq: np.ndarray,
                               ddq: np.ndarray) -> np.ndarray:
    """从原始数组构建回归矩阵（用于实测数据）。"""
    if pin is None:
        raise RuntimeError("需要 pinocchio")
    N = q.shape[0]
    P = 10 * (model.njoints - 1)
    Y = np.empty((N * model.nv, P))
    for i in range(N):
        Yi = pin.computeJointTorqueRegressor(model, data, q[i], dq[i], ddq[i])
        Y[i * model.nv:(i + 1) * model.nv, :] = np.asarray(Yi)
    return Y


def nominal_parameters(model) -> np.ndarray:
    """从 URDF 读取标称惯性参数向量（10 个/连杆，跳过 universe）。"""
    return np.concatenate([
        np.asarray(model.inertias[i].toDynamicParameters())
        for i in range(1, model.njoints)
    ])


def base_parameter_basis(Y: np.ndarray, theta_nominal: Optional[np.ndarray] = None,
                         rtol: float = 1e-10) -> BaseParameterBasis:
    """SVD 求基参数子空间。

    Args:
        Y: (M, P) 回归矩阵
        theta_nominal: 标称参数（可选，用于 from_base 的参考点）
        rtol: 相对阈值，奇异值小于 rtol·σ_max 视为零

    Returns:
        BaseParameterBasis
    """
    U, s, Vt = np.linalg.svd(Y, full_matrices=False)
    if s.size == 0:
        raise ValueError("回归矩阵为空")
    tol = max(s[0] * rtol, np.finfo(float).eps)
    rank = int((s > tol).sum())
    V = Vt[:rank, :].T          # (P, rank)
    return BaseParameterBasis(V=V, rank=rank, singular_values=s,
                              theta_nominal=theta_nominal)


def condition_number(Y: np.ndarray, rtol: float = 1e-10) -> Tuple[float, int]:
    """可辨识子空间上的条件数（忽略零奇异值）。"""
    s = np.linalg.svd(Y, compute_uv=False)
    tol = max(s[0] * rtol, np.finfo(float).eps)
    s_pos = s[s > tol]
    return float(s_pos[0] / s_pos[-1]), int(s_pos.size)
