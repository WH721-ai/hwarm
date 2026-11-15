"""m30_sysid —— 激励轨迹设计、安全检查和物理参数辨识。

模块
    config      配置（电机规格、安全边界、设计/辨识参数）
    fourier     Fourier 激励轨迹 + 五次多项式过渡段
    safety      七项安全检查（执行前的守门员）
    regressor   回归矩阵 + 基参数约简
    design      激励轨迹优化（SLSQP）
    identify    参数辨识（加权最小二乘 + 物理一致性）

典型用法::

    from m30_sysid import (SafetyLimits, ExcitationConfig, IdentificationConfig,
                           design_excitation, build_excitation_trajectory,
                           check_trajectory, identify)

    limits = SafetyLimits.from_model(model)
    res = design_excitation(model, data, limits, ExcitationConfig())
    traj = build_excitation_trajectory(res.params, q_current, None, None,
                                       n_periods=2, transition_time=1.5)
    report = check_trajectory(traj, limits, model, data, frame_id)
    if not report.ok:
        raise SystemExit(report.summary())
    traj.to_csv("excitation.csv")
"""

from .config import (DEFAULT_MOTORS, ExcitationConfig, IdentificationConfig,
                     MotorSpec, SafetyLimits)
from .design import DesignResult, design_excitation
from .fourier import (FourierParams, Trajectory, build_excitation_trajectory,
                      quintic_coefficients, quintic_evaluate)
from .identify import (IdentificationResult, check_physical_consistency,
                       export_parameter_report, identify, theta_to_urdf_inertials)
from .regressor import (BaseParameterBasis, base_parameter_basis,
                        build_regressor, build_regressor_from_arrays,
                        condition_number, nominal_parameters)
from .safety import SafetyReport, check_start_pose, check_trajectory, resample

__all__ = [
    "DEFAULT_MOTORS", "MotorSpec", "SafetyLimits",
    "ExcitationConfig", "IdentificationConfig",
    "FourierParams", "Trajectory",
    "build_excitation_trajectory", "quintic_coefficients", "quintic_evaluate",
    "SafetyReport", "check_trajectory", "check_start_pose", "resample",
    "BaseParameterBasis", "base_parameter_basis", "build_regressor",
    "build_regressor_from_arrays", "condition_number", "nominal_parameters",
    "DesignResult", "design_excitation",
    "IdentificationResult", "identify", "check_physical_consistency",
    "export_parameter_report", "theta_to_urdf_inertials",
]
