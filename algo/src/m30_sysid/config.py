"""系统辨识的配置定义。

所有安全边界集中在这里，**不允许**在其它模块里写死数值。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


# ---------------------------------------------------------------------------
# 电机规格
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MotorSpec:
    """单个电机的规格。

    注意：tau_peak 是**峰值**扭矩（达妙手册的 TMAX），不是额定扭矩。
    长时间在峰值附近运行会显著发热，因此实际约束用 tau_rated。
    """

    name: str
    model: str
    tau_peak: float          # 峰值扭矩 N·m（硬上限，绝不允许超）
    v_max: float             # 最大角速度 rad/s
    rated_ratio: float = 0.35  # 额定扭矩 = tau_peak * rated_ratio

    @property
    def tau_rated(self) -> float:
        return self.tau_peak * self.rated_ratio


#: 实测硬件配置（2026-11 标定）
DEFAULT_MOTORS: List[MotorSpec] = [
    MotorSpec("joint1", "DM10010L", tau_peak=200.0, v_max=20.0),
    MotorSpec("joint2", "DM6216P",  tau_peak=45.0,  v_max=20.0),  # vMax 待确认
    MotorSpec("joint3", "DM4340",   tau_peak=27.0,  v_max=10.0),
    MotorSpec("joint4", "DM4310P",  tau_peak=11.0,  v_max=10.0),
    MotorSpec("joint5", "DM4310",   tau_peak=7.0,   v_max=10.0),
    MotorSpec("joint6", "DM4310",   tau_peak=7.0,   v_max=10.0),
]


# ---------------------------------------------------------------------------
# 安全限制
# ---------------------------------------------------------------------------
@dataclass
class SafetyLimits:
    """激励轨迹的安全边界。

    三条线（力矩）：
        tau_rated  —— 期望工作区（优化目标）
        tau_warn   —— 黄线（允许，但报告警告）
        tau_hard   —— 红线（**绝不接受**）
    """

    # 位置：需要外部注入实际 URDF 限位
    q_min: np.ndarray
    q_max: np.ndarray

    # 速度 / 加速度
    v_max: np.ndarray
    a_max: np.ndarray

    # 力矩
    tau_peak: np.ndarray
    tau_rated: np.ndarray
    tau_warn: np.ndarray
    tau_hard: np.ndarray

    # 力矩预估安全系数：真实参数可能比标称差 20~30%
    tau_safety_factor: float = 1.5

    # 力矩变化率上限 N·m/s（防跳变）
    tau_rate_max: np.ndarray = field(default_factory=lambda: np.array([]))

    # 奇异阈值
    sigma_min: float = 0.01

    # 位置安全边距（占量程的比例）
    position_margin: float = 0.05

    # 检查用的重采样率（**必须高于控制周期**）
    check_rate: float = 1000.0

    @classmethod
    def from_model(cls, model, motors: Optional[List[MotorSpec]] = None,
                   position_margin: float = 0.05, rated_ratio: float = 0.35,
                   v_scale: float = 1.0, a_scale: float = 1.0,
                   tau_ratio_for_design: float = 1.0) -> "SafetyLimits":
        """从 pinocchio Model + 电机表构造限制。

        Args:
            position_margin: 两端预留的比例（0.05 = 5%）
            rated_ratio: 额定/峰值比例
            v_scale: 速度限制的缩放（<1 更保守）
            a_scale: 加速度限制的缩放
            tau_ratio_for_design: 设计约束用 tau_peak 的多少倍
        """
        motors = motors or DEFAULT_MOTORS
        nv = model.nv
        if len(motors) != nv:
            raise ValueError(f"电机数量 {len(motors)} != nv {nv}")

        span = model.upperPositionLimit - model.lowerPositionLimit
        m = position_margin * span

        tau_peak = np.array([mt.tau_peak for mt in motors])
        tau_rated = np.array([mt.tau_peak * (rated_ratio if rated_ratio is not None
                                             else mt.rated_ratio) for mt in motors])

        return cls(
            q_min=model.lowerPositionLimit + m,
            q_max=model.upperPositionLimit - m,
            v_max=np.array([mt.v_max for mt in motors]) * v_scale,
            # 加速度限值：取"能在 0.1 s 内加到额定速度"的量级，再缩放
            a_max=np.array([mt.v_max for mt in motors]) / 0.1 * a_scale,
            tau_peak=tau_peak,
            tau_rated=tau_rated * tau_ratio_for_design,
            tau_warn=tau_peak * 0.7,
            tau_hard=tau_peak * 0.8,
            # 力矩变化率：允许 100 ms 内从 0 到额定扭矩
            tau_rate_max=tau_rated / 0.1,
            position_margin=position_margin,
        )

    def with_conservative_speed(self, v_limit: float) -> "SafetyLimits":
        """所有关节速度统一限制到 v_limit（取较小者）。"""
        import copy
        new = copy.deepcopy(self)
        new.v_max = np.minimum(new.v_max, v_limit)
        return new


# ---------------------------------------------------------------------------
# 激励轨迹设计参数
# ---------------------------------------------------------------------------
@dataclass
class ExcitationConfig:
    """Fourier 激励轨迹的设计参数。"""

    n_harmonics: int = 5          # K：每轴谐波数
    duration: float = 6.0         # T：基本周期 s
    n_periods: int = 2            # 重复周期数（便于平均降噪）
    transition_time: float = 1.5  # 起止过渡段时长 s

    # 优化
    optimizer: str = "SLSQP"      # SLSQP / L-BFGS-B
    max_iter: int = 200
    ftol: float = 1e-4
    n_starts: int = 3             # 多起点次数（取最优）
    random_seed: int = 0

    # 优化时的采样点（粗网格，只为评估约束）
    n_samples_opt: int = 200

    # 幅值初始化
    amplitude_init: float = 0.15  # rad

    # 目标函数："cond"（条件数）或 "dopt"（-log det）
    objective: str = "cond"

    # 是否在优化中强制奇异约束（需要 frame_id）
    enforce_singularity: bool = True

    # 优化后再按比例缩放修复（强烈建议开启）
    repair_by_scaling: bool = True


@dataclass
class IdentificationConfig:
    """参数辨识配置。"""

    # 正则化：0 = 纯基参数最小二乘（推荐）
    regularization: float = 0.0

    # 是否按 tMax 加权（推荐 True，平衡各关节量级）
    use_weighting: bool = True

    # SVD 截断相对阈值
    svd_rtol: float = 1e-10

    # 物理一致性检查
    check_physical_consistency: bool = True
    min_mass: float = 1e-4        # kg
    max_mass: float = 100.0       # kg
