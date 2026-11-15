"""m30_sysid 单元测试。

运行::

    cd ~/hw_arm/algo
    PYTHONPATH=src python3 -m pytest tests/test_sysid.py -v
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

pin = pytest.importorskip("pinocchio")

from m30_sysid import (ExcitationConfig, FourierParams, SafetyLimits,  # noqa: E402
                       Trajectory, base_parameter_basis,
                       build_excitation_trajectory, check_trajectory,
                       condition_number, nominal_parameters,
                       quintic_coefficients, quintic_evaluate)
from m30_sysid.design import repair_by_scale  # noqa: E402

URDF_CANDIDATES = [
    os.environ.get("HW_ARM_URDF", ""),
    "/tmp/hw_arm.urdf",
    os.path.expanduser("~/hw_arm/hw_arm_ws/install/hw_arm_description/share/"
                       "hw_arm_description/urdf/hw_arm.urdf"),
]


def _urdf() -> str:
    for p in URDF_CANDIDATES:
        if p and os.path.isfile(p):
            return p
    pytest.skip("找不到 URDF（设置 HW_ARM_URDF 或生成 /tmp/hw_arm.urdf）")


@pytest.fixture(scope="module")
def model():
    return pin.buildModelFromUrdf(_urdf())


@pytest.fixture(scope="module")
def data(model):
    return model.createData()


# ---------------------------------------------------------------------------
# 五次多项式
# ---------------------------------------------------------------------------
def test_quintic_boundary_conditions():
    nv = 3
    rng = np.random.default_rng(0)
    q0, v0, a0 = rng.standard_normal((3, nv))
    q1, v1, a1 = rng.standard_normal((3, nv))
    T = 1.7

    C = quintic_coefficients(q0, v0, a0, q1, v1, a1, T)
    q, dq, ddq = quintic_evaluate(C, np.array([0.0]))
    assert np.allclose(q[0], q0, atol=1e-12)
    assert np.allclose(dq[0], v0, atol=1e-12)
    assert np.allclose(ddq[0], a0, atol=1e-12)

    q, dq, ddq = quintic_evaluate(C, np.array([T]))
    assert np.allclose(q[0], q1, atol=1e-10)
    assert np.allclose(dq[0], v1, atol=1e-10)
    assert np.allclose(ddq[0], a1, atol=1e-10)


# ---------------------------------------------------------------------------
# Fourier
# ---------------------------------------------------------------------------
def test_fourier_analytic_derivatives():
    """解析导数必须和数值微分一致。"""
    nv, K = 3, 3
    rng = np.random.default_rng(1)
    p = FourierParams(q0=rng.standard_normal(nv),
                      A=rng.standard_normal((nv, K)) * 0.1,
                      B=rng.standard_normal((nv, K)) * 0.1,
                      omega=2 * np.pi / 5.0)
    t = np.linspace(0, 5, 500)
    q, dq, ddq = p.evaluate(t)

    dq_num = np.gradient(q, t, axis=0)
    ddq_num = np.gradient(dq, t, axis=0)
    assert np.allclose(dq, dq_num, atol=2e-3)
    assert np.allclose(ddq, ddq_num, atol=2e-2)


def test_fourier_velocity_amplitude_is_bounded_by_coefficients():
    """Swevers 形式：速度幅值 ≈ 各阶系数幅值之和，不随 k 放大。"""
    nv, K = 1, 5
    A = np.full((nv, K), 0.1)
    B = np.zeros((nv, K))
    p = FourierParams(q0=np.zeros(nv), A=A, B=B, omega=2 * np.pi / 4.0)
    t = np.linspace(0, 4, 2000)
    _, dq, _ = p.evaluate(t)
    # 速度峰值 ≤ Σ|a_k| = 0.5
    assert np.abs(dq).max() <= 0.5 + 1e-9


# ---------------------------------------------------------------------------
# 完整轨迹组装
# ---------------------------------------------------------------------------
def test_trajectory_starts_and_ends_at_rest(model):
    nv = model.nv
    K = 2
    rng = np.random.default_rng(2)
    p = FourierParams(q0=np.zeros(nv),
                      A=rng.standard_normal((nv, K)) * 0.05,
                      B=rng.standard_normal((nv, K)) * 0.05,
                      omega=2 * np.pi / 5.0)
    traj = build_excitation_trajectory(p, q_start=np.zeros(nv), v_start=None,
                                       a_start=None, n_periods=1,
                                       transition_time=1.0, rate=500.0)
    assert np.abs(traj.dq[0]).max() < 1e-9, "起点速度必须为 0"
    assert np.abs(traj.dq[-1]).max() < 1e-9, "终点速度必须为 0"
    assert traj.duration == pytest.approx(1.0 + 5.0 + 1.0, abs=1e-6)


def test_trajectory_is_c2_at_splice(model):
    """过渡段与 Fourier 段拼接处必须 C² 连续（力矩不跳变）。"""
    nv = model.nv
    K = 2
    rng = np.random.default_rng(3)
    p = FourierParams(q0=rng.standard_normal(nv) * 0.1,
                      A=rng.standard_normal((nv, K)) * 0.05,
                      B=rng.standard_normal((nv, K)) * 0.05,
                      omega=2 * np.pi / 4.0)
    traj = build_excitation_trajectory(p, q_start=np.zeros(nv), v_start=None,
                                       a_start=None, n_periods=1,
                                       transition_time=1.0, rate=1000.0)
    t_trans = 1.0
    i = int(np.argmin(np.abs(traj.t - t_trans)))
    # 拼接点两侧的加速度应连续
    assert np.abs(traj.ddq[i] - traj.ddq[i - 1]).max() < 0.05
    # 位置连续
    assert np.abs(traj.q[i] - traj.q[i - 1]).max() < 1e-3


# ---------------------------------------------------------------------------
# 回归矩阵与基参数
# ---------------------------------------------------------------------------
def test_regressor_reproduces_inverse_dynamics(model, data):
    nv = model.nv
    rng = np.random.default_rng(4)
    q = rng.uniform(-0.5, 0.5, nv)
    dq = rng.uniform(-0.3, 0.3, nv)
    ddq = rng.uniform(-0.5, 0.5, nv)

    Y = np.asarray(pin.computeJointTorqueRegressor(model, data, q, dq, ddq))
    theta = nominal_parameters(model)
    pin.rnea(model, data, q, dq, ddq)
    assert np.allclose(Y @ theta, data.tau, atol=1e-9)


def test_base_parameter_projection(model, data):
    """V·Vᵀ 应是到可辨识子空间的正交投影。"""
    nv = model.nv
    rng = np.random.default_rng(5)
    Ys = []
    for _ in range(40):
        q = rng.uniform(-0.5, 0.5, nv)
        dq = rng.uniform(-0.3, 0.3, nv)
        ddq = rng.uniform(-0.5, 0.5, nv)
        Ys.append(np.asarray(pin.computeJointTorqueRegressor(model, data, q, dq, ddq)))
    Y = np.vstack(Ys)

    basis = base_parameter_basis(Y)
    assert basis.rank < basis.n_full, "必然秩亏"
    # 投影后应基本还原 Y
    Y_proj = Y @ basis.V @ basis.V.T
    assert np.allclose(Y_proj, Y, atol=1e-8 * max(1.0, np.abs(Y).max()))


def test_condition_number_ignores_null_space(model, data):
    nv = model.nv
    rng = np.random.default_rng(6)
    Ys = []
    for _ in range(30):
        q = rng.uniform(-0.5, 0.5, nv)
        dq = rng.uniform(-0.3, 0.3, nv)
        ddq = rng.uniform(-0.5, 0.5, nv)
        Ys.append(np.asarray(pin.computeJointTorqueRegressor(model, data, q, dq, ddq)))
    Y = np.vstack(Ys)
    c, r = condition_number(Y)
    assert np.isfinite(c) and c > 0
    assert r == base_parameter_basis(Y).rank


# ---------------------------------------------------------------------------
# 安全检查
# ---------------------------------------------------------------------------
def test_safety_detects_position_violation(model):
    nv = model.nv
    limits = SafetyLimits.from_model(model)
    t = np.linspace(0, 1, 101)
    q = np.zeros((t.size, nv))
    q[:, 0] = np.linspace(0, limits.q_max[0] + 1.0, t.size)   # 故意越界
    traj = Trajectory(t, q, np.zeros_like(q), np.zeros_like(q))
    rep = check_trajectory(traj, limits, check_rate=200.0)
    assert not rep.ok
    assert any("位置越界" in v for v in rep.violations)


def test_safety_detects_velocity_violation(model):
    nv = model.nv
    limits = SafetyLimits.from_model(model)
    t = np.linspace(0, 1, 101)
    q = np.zeros((t.size, nv))
    dq = np.zeros_like(q)
    dq[:, 1] = limits.v_max[1] * 3.0
    traj = Trajectory(t, q, dq, np.zeros_like(q))
    rep = check_trajectory(traj, limits, check_rate=200.0)
    assert not rep.ok
    assert any("速度超限" in v for v in rep.violations)


def test_safety_detects_torque_spike(model, data):
    """构造一个加速度突变 → 力矩变化率超限。"""
    nv = model.nv
    limits = SafetyLimits.from_model(model)
    t = np.linspace(0, 2, 801)
    q = np.zeros((t.size, nv))
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)
    # 加速度方波
    ddq[t.size // 2:, 1] = 50.0
    traj = Trajectory(t, q, dq, ddq)
    rep = check_trajectory(traj, limits, model=model, data=data, check_rate=400.0)
    assert not rep.ok
    assert any(("力矩变化率" in v) or ("力矩超红线" in v) for v in rep.violations)


def test_safety_passes_on_designed_trajectory(model, data):
    """一个温和的轨迹应能通过检查。"""
    nv = model.nv
    limits = SafetyLimits.from_model(model).with_conservative_speed(0.5)
    K = 2
    rng = np.random.default_rng(7)
    center = (limits.q_min + limits.q_max) / 2
    p = FourierParams(q0=center,
                      A=rng.standard_normal((nv, K)) * 0.03,
                      B=rng.standard_normal((nv, K)) * 0.03,
                      omega=2 * np.pi / 4.0)
    frame_id = model.getFrameId("tool0") if model.existFrame("tool0") else None
    traj = build_excitation_trajectory(p, q_start=center, v_start=None,
                                       a_start=None, n_periods=1,
                                       transition_time=1.0, rate=500.0)
    rep = check_trajectory(traj, limits, model=model, data=data,
                           frame_id=frame_id, check_rate=500.0)
    assert rep.ok, rep.summary()


# ---------------------------------------------------------------------------
# 缩放修复
# ---------------------------------------------------------------------------
def test_repair_by_scale_reduces_utilization(model, data):
    nv = model.nv
    limits = SafetyLimits.from_model(model).with_conservative_speed(0.4)
    K = 2
    rng = np.random.default_rng(8)
    center = (limits.q_min + limits.q_max) / 2
    # 故意给一个很大的幅值 → 必然越界
    p = FourierParams(q0=center,
                      A=rng.standard_normal((nv, K)) * 0.8,
                      B=rng.standard_normal((nv, K)) * 0.8,
                      omega=2 * np.pi / 3.0)

    repaired, alpha = repair_by_scale(p, limits, model, data,
                                      q_start=center, transition_time=1.0)
    assert 0.0 < alpha < 1.0, "必然需要缩小"
    assert np.allclose(repaired.A, p.A * alpha)
    assert np.allclose(repaired.B, p.B * alpha)


# ---------------------------------------------------------------------------
# 辨识（仿真数据）
# ---------------------------------------------------------------------------
def test_identify_recovers_torques_on_synthetic_data(model, data):
    """用标称参数生成数据 → 辨识后训练/验证残差都应接近 0。"""
    from m30_sysid import IdentificationConfig, identify

    nv = model.nv
    limits = SafetyLimits.from_model(model).with_conservative_speed(0.6)
    K = 3
    rng = np.random.default_rng(9)
    center = (limits.q_min + limits.q_max) / 2
    p = FourierParams(q0=center,
                      A=rng.standard_normal((nv, K)) * 0.15,
                      B=rng.standard_normal((nv, K)) * 0.15,
                      omega=2 * np.pi / 4.0)
    traj = build_excitation_trajectory(p, q_start=center, v_start=None,
                                       a_start=None, n_periods=2,
                                       transition_time=1.0, rate=500.0)

    theta_true = nominal_parameters(model)
    tau = np.empty_like(traj.q)
    for i in range(len(traj.t)):
        Yi = pin.computeJointTorqueRegressor(model, data,
                                             traj.q[i], traj.dq[i], traj.ddq[i])
        tau[i] = np.asarray(Yi) @ theta_true
    tau += 0.01 * rng.standard_normal(tau.shape)

    n = len(traj.t)
    n_val = n // 5
    res = identify(model, data, traj.slice(0, n - n_val), tau[:n - n_val],
                   cfg=IdentificationConfig(), validation=(traj.slice(n - n_val, n),
                                                           tau[n - n_val:]))
    assert res.rank > 0 and res.rank < res.basis.n_full
    assert res.residual_rms < 0.05          # 噪声 0.01，留足余量
    assert res.validation_rms < 0.05
