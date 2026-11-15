"""Fourier 系列激励轨迹 + 五次多项式过渡段。

数学形式（Swevers et al. 1997）::

    q_i(t) = q_i0 + Σ_{k=1}^{K} [ a_ik/(k·ω) · sin(k·ω·t)
                                − b_ik/(k·ω) · cos(k·ω·t) ]

    q̇_i(t) = Σ_{k=1}^{K} [ a_ik · cos(k·ω·t) + b_ik · sin(k·ω·t) ]
    q̈_i(t) = Σ_{k=1}^{K} k·ω · [ −a_ik · sin(k·ω·t) + b_ik · cos(k·ω·t) ]

其中 ω = 2π/T 是基本角频率。

**为什么除以 (k·ω)**：使各阶谐波的速度幅值都等于 a_ik，速度天然有界，
不会因为高频项而放大——这是 Swevers 形式的关键设计。

**导数全部解析**，不需要数值微分（避免噪声放大）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# 轨迹容器
# ---------------------------------------------------------------------------
@dataclass
class Trajectory:
    """采样后的轨迹（时间序列）。

    这是本模块唯一的"输出类型"，所有检查/执行/存盘都围绕它。
    """

    t: np.ndarray            # (N,)      时间 s
    q: np.ndarray            # (N, nv)   位置 rad
    dq: np.ndarray           # (N, nv)   速度 rad/s
    ddq: np.ndarray          # (N, nv)   加速度 rad/s²
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        n = len(self.t)
        for name in ("q", "dq", "ddq"):
            arr = getattr(self, name)
            if arr.shape[0] != n:
                raise ValueError(f"{name} 长度 {arr.shape[0]} != t 长度 {n}")

    @property
    def nv(self) -> int:
        return self.q.shape[1]

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    def slice(self, i0: int, i1: int) -> "Trajectory":
        return Trajectory(self.t[i0:i1].copy(), self.q[i0:i1].copy(),
                          self.dq[i0:i1].copy(), self.ddq[i0:i1].copy(),
                          dict(self.meta))

    def to_csv(self, path: str) -> None:
        nv = self.nv
        header = ["t"]
        header += [f"q{i+1}" for i in range(nv)]
        header += [f"dq{i+1}" for i in range(nv)]
        header += [f"ddq{i+1}" for i in range(nv)]
        data = np.column_stack([self.t, self.q, self.dq, self.ddq])
        np.savetxt(path, data, delimiter=",", header=",".join(header),
                   comments="", fmt="%.9g")

    @staticmethod
    def from_csv(path: str, nv: int) -> "Trajectory":
        raw = np.loadtxt(path, delimiter=",", skiprows=1)
        t = raw[:, 0]
        q = raw[:, 1:1 + nv]
        dq = raw[:, 1 + nv:1 + 2 * nv]
        ddq = raw[:, 1 + 2 * nv:1 + 3 * nv]
        return Trajectory(t, q, dq, ddq)


# ---------------------------------------------------------------------------
# Fourier 参数
# ---------------------------------------------------------------------------
@dataclass
class FourierParams:
    """Fourier 级数系数。

    Attributes:
        q0:  (nv,)       中心位置
        A:   (nv, K)     sin 系数
        B:   (nv, K)     cos 系数
        omega: float     基本角频率
        phases: (nv, K)  可选的相位偏移（内部把相位吸收进 A/B，这里仅记录）
    """

    q0: np.ndarray
    A: np.ndarray
    B: np.ndarray
    omega: float

    @property
    def nv(self) -> int:
        return self.q0.shape[0]

    @property
    def n_harmonics(self) -> int:
        return self.A.shape[1]

    @property
    def n_params(self) -> int:
        return self.q0.size + self.A.size + self.B.size

    def to_vector(self) -> np.ndarray:
        return np.concatenate([self.q0, self.A.ravel(), self.B.ravel()])

    @classmethod
    def from_vector(cls, x: np.ndarray, nv: int, K: int, omega: float) -> "FourierParams":
        q0 = x[:nv].copy()
        A = x[nv:nv + nv * K].reshape(nv, K).copy()
        B = x[nv + nv * K:nv + 2 * nv * K].reshape(nv, K).copy()
        return cls(q0, A, B, omega)

    def evaluate(self, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """在时刻 t 求值，返回 (q, dq, ddq)。

        Args:
            t: (N,) 时间
        Returns:
            q, dq, ddq 均为 (N, nv)
        """
        t = np.asarray(t, dtype=float).reshape(-1)
        nv, K = self.nv, self.n_harmonics
        q = np.tile(self.q0, (t.size, 1))
        dq = np.zeros((t.size, nv))
        ddq = np.zeros((t.size, nv))

        for k in range(1, K + 1):
            kw = k * self.omega
            s = np.sin(kw * t)[:, None]      # (N,1)
            c = np.cos(kw * t)[:, None]
            a = self.A[:, k - 1][None, :]    # (1,nv)
            b = self.B[:, k - 1][None, :]
            q += (a * s - b * c) / kw
            dq += a * c + b * s
            ddq += kw * (-a * s + b * c)
        return q, dq, ddq


# ---------------------------------------------------------------------------
# 五次多项式（过渡段）
# ---------------------------------------------------------------------------
def quintic_coefficients(q0: np.ndarray, v0: np.ndarray, a0: np.ndarray,
                         q1: np.ndarray, v1: np.ndarray, a1: np.ndarray,
                         T: float) -> np.ndarray:
    """求解五次多项式系数，满足两端的位置/速度/加速度共 6 个边界条件。

    Returns:
        C: (6, nv)，q(t) = C[0] + C[1] t + ... + C[5] t^5
    """
    nv = q0.size
    C = np.zeros((6, nv))
    C[0] = q0
    C[1] = v0
    C[2] = a0 / 2.0

    # 解 3x3 线性系统求 c3, c4, c5
    M = np.array([
        [T**3,      T**4,      T**5],
        [3 * T**2,  4 * T**3,  5 * T**4],
        [6 * T,     12 * T**2, 20 * T**3],
    ])
    rhs = np.vstack([
        q1 - (q0 + v0 * T + a0 * T**2 / 2.0),
        v1 - (v0 + a0 * T),
        a1 - a0,
    ])                                   # (3, nv)
    sol = np.linalg.solve(M, rhs)        # (3, nv)
    C[3:6] = sol
    return C


def quintic_evaluate(C: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """求值五次多项式及其一二阶导。"""
    t = np.asarray(t, dtype=float).reshape(-1, 1)   # (N,1)
    q = sum(C[i][None, :] * t**i for i in range(6))
    dq = sum(i * C[i][None, :] * t**(i - 1) for i in range(1, 6))
    ddq = sum(i * (i - 1) * C[i][None, :] * t**(i - 2) for i in range(2, 6))
    return q, dq, ddq


# ---------------------------------------------------------------------------
# 组装完整激励轨迹
# ---------------------------------------------------------------------------
def build_excitation_trajectory(
    params: FourierParams,
    q_start: np.ndarray,
    v_start: Optional[np.ndarray],
    a_start: Optional[np.ndarray],
    n_periods: int,
    transition_time: float,
    rate: float = 1000.0,
    end_at_rest: bool = True,
) -> Trajectory:
    """组装「过渡段 → N 个周期 Fourier → 过渡段」的完整轨迹。

    设计要点：
        · 过渡段用五次多项式，**位置/速度/加速度三阶连续**（C²）
        · 起止速度为 0（可由调用方指定）
        · Fourier 段与过渡段在拼接点处 q/dq/ddq 完全一致
          → 力矩不会在拼接点出现跳变

    Args:
        params: Fourier 系数
        q_start: 机器人当前位姿（过渡段起点）
        v_start, a_start: 起始速度/加速度（None → 0）
        n_periods: Fourier 周期数
        transition_time: 每段过渡时长
        rate: 采样率 Hz
        end_at_rest: True 时结尾停在最后位置（v=0, a=0）

    Returns:
        Trajectory（含 meta 记录各段边界）
    """
    nv = params.nv
    v_start = np.zeros(nv) if v_start is None else np.asarray(v_start, float)
    a_start = np.zeros(nv) if a_start is None else np.asarray(a_start, float)

    T = 2.0 * np.pi / params.omega
    T_fourier = T * n_periods

    # Fourier 段在两个端点的状态（周期性 → 两端相同）
    q_f0, v_f0, a_f0 = params.evaluate(np.array([0.0]))
    q_f0, v_f0, a_f0 = q_f0[0], v_f0[0], a_f0[0]

    pieces = []
    t_offset = 0.0

    # ---- 过渡段 1：当前位姿 → Fourier 起点 ----
    if transition_time > 0:
        C_in = quintic_coefficients(q_start, v_start, a_start,
                                    q_f0, v_f0, a_f0, transition_time)
        t_in = _arange(0.0, transition_time, rate, include_end=False)
        q, dq, ddq = quintic_evaluate(C_in, t_in)
        pieces.append(Trajectory(t_in + t_offset, q, dq, ddq))
        t_offset += transition_time

    # ---- Fourier 段 ----
    t_f = _arange(0.0, T_fourier, rate, include_end=True)
    q, dq, ddq = params.evaluate(t_f)
    pieces.append(Trajectory(t_f + t_offset, q, dq, ddq,
                             {"segment": "fourier", "n_periods": n_periods}))
    t_offset += T_fourier

    # ---- 过渡段 2：Fourier 终点 → 静止 ----
    if end_at_rest and transition_time > 0:
        q_end_target = q_f0.copy()       # 停在 Fourier 起点（一个完整周期）
        C_out = quintic_coefficients(q_f0, v_f0, a_f0,
                                     q_end_target, np.zeros(nv), np.zeros(nv),
                                     transition_time)
        t_out = _arange(0.0, transition_time, rate, include_end=True)
        q, dq, ddq = quintic_evaluate(C_out, t_out)
        pieces.append(Trajectory(t_out + t_offset, q, dq, ddq,
                                 {"segment": "transition_out"}))

    traj = _concat(pieces)
    traj.meta.update({
        "n_harmonics": params.n_harmonics,
        "fundamental_period": T,
        "n_periods": n_periods,
        "transition_time": transition_time,
        "rate": rate,
    })
    return traj


def _arange(t0: float, t1: float, rate: float, include_end: bool) -> np.ndarray:
    n = max(int(round((t1 - t0) * rate)), 1)
    if include_end:
        return np.linspace(t0, t1, n + 1)
    return t0 + np.arange(n) / rate


def _concat(pieces) -> Trajectory:
    t = np.concatenate([p.t for p in pieces])
    q = np.vstack([p.q for p in pieces])
    dq = np.vstack([p.dq for p in pieces])
    ddq = np.vstack([p.ddq for p in pieces])
    meta = {}
    for p in pieces:
        meta.update({k: v for k, v in p.meta.items() if k not in meta})
    return Trajectory(t, q, dq, ddq, meta)
