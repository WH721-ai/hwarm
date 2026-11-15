#!/usr/bin/env python3
"""参数辨识：从实测数据估计惯性参数。

输入 CSV 两种格式（自动识别）::

    a) 完整:  t, q1..q6, dq1..dq6, ddq1..ddq6, tau1..tau6
    b) 常见:  t, q1..q6, dq1..dq6, tau1..tau6        ← 自动用 SG 滤波求 ddq

用法::

    # 真机数据
    python3 scripts/identify_params.py --urdf ... --data recorded.csv

    # 仿真自检（用设计好的轨迹 + URDF 标称参数生成"假数据"）
    python3 scripts/identify_params.py --urdf ... --data excitation.csv --simulate
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import pinocchio as pin  # noqa: E402

from m30_sysid import (DEFAULT_MOTORS, IdentificationConfig,  # noqa: E402
                       export_parameter_report, identify,
                       nominal_parameters, theta_to_urdf_inertials)
from m30_sysid.fourier import Trajectory  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="惯性参数辨识")
    p.add_argument("--urdf", required=True)
    p.add_argument("--data", required=True, help="实测/仿真数据 CSV")
    p.add_argument("--out", default="data/sysid", help="输出目录")
    p.add_argument("--simulate", action="store_true",
                   help="用 URDF 标称参数生成假数据（自检用）")
    p.add_argument("--noise", type=float, default=0.05,
                   help="--simulate 时注入的力矩噪声 std (N·m)")
    p.add_argument("--smooth-window", type=int, default=21,
                   help="SG 滤波窗口（求 ddq 用，奇数）")
    p.add_argument("--no-weighting", action="store_true",
                   help="关闭按 tMax 加权")
    p.add_argument("--regularization", type=float, default=0.0,
                   help="基参数上的 Tikhonov 正则（0=纯最小二乘）")
    p.add_argument("--val-ratio", type=float, default=0.2,
                   help="留出验证集比例")
    return p.parse_args()


def load_data(path: str, nv: int) -> dict:
    raw = np.loadtxt(path, delimiter=",", skiprows=1)
    n_cols = raw.shape[1]
    t = raw[:, 0]
    q = raw[:, 1:1 + nv]
    dq = raw[:, 1 + nv:1 + 2 * nv]
    if n_cols >= 1 + 3 * nv + nv:      # 有 ddq
        ddq = raw[:, 1 + 2 * nv:1 + 3 * nv]
        tau = raw[:, 1 + 3 * nv:1 + 4 * nv]
        has_ddq = True
    else:
        ddq = None
        tau = raw[:, 1 + 2 * nv:1 + 3 * nv]
        has_ddq = False
    return {"t": t, "q": q, "dq": dq, "ddq": ddq, "tau": tau, "has_ddq": has_ddq}


def savgol_derivative(y: np.ndarray, t: np.ndarray, window: int = 21,
                      polyorder: int = 3) -> np.ndarray:
    """Savitzky-Golay 一阶导数（逐列）。"""
    from scipy.signal import savgol_filter
    dt = np.median(np.diff(t))
    w = min(window, (len(t) // 2) * 2 - 1)
    w = max(w, polyorder + 2 + (polyorder % 2 == 0))
    if w % 2 == 0:
        w += 1
    return savgol_filter(y, w, polyorder, deriv=1, delta=dt, axis=0)


def main() -> int:
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    nv = model.nv

    d = load_data(args.data, nv)
    print(f"数据: {len(d['t'])} 点, {len(d['t'])/np.median(np.diff(d['t'])):.1f} 采样")

    # ---- 加速度 ----
    if d["has_ddq"]:
        ddq = d["ddq"]
        print("  使用数据中的 ddq")
    else:
        ddq = savgol_derivative(d["dq"], d["t"], args.smooth_window)
        print(f"  用 SG 滤波（窗口 {args.smooth_window}）对 dq 求导得到 ddq")

    # ---- 仿真模式：用标称参数生成力矩 ----
    if args.simulate:
        theta_true = nominal_parameters(model)
        tau = np.empty((len(d["t"]), nv))
        for i in range(len(d["t"])):
            pin.rnea(model, data, d["q"][i], d["dq"][i], ddq[i])
            tau[i] = data.tau
        rng = np.random.default_rng(0)
        tau += args.noise * rng.standard_normal(tau.shape)
        print(f"  仿真模式：用标称参数 + {args.noise} N·m 噪声生成力矩")
        # 稍微扰动"真实"参数，使辨识有挑战性
        scale = 1.0 + 0.05 * rng.standard_normal(theta_true.size)
        theta_true = theta_true * scale
        tau = np.empty((len(d["t"]), nv))
        for i in range(len(d["t"])):
            Yi = pin.computeJointTorqueRegressor(
                model, data, d["q"][i], d["dq"][i], ddq[i])
            tau[i] = np.asarray(Yi) @ theta_true
        tau += args.noise * rng.standard_normal(tau.shape)
        print(f"  真值参数扰动 ±5%")
    else:
        tau = d["tau"]

    # ---- 训练 / 验证切分（按时间连续切，不用随机，避免泄漏）----
    n = len(d["t"])
    n_val = int(n * args.val_ratio)
    traj_all = Trajectory(d["t"], d["q"], d["dq"], ddq)
    if n_val > 10:
        tr_tr = traj_all.slice(0, n - n_val)
        tr_va = traj_all.slice(n - n_val, n)
        val = (tr_va, tau[n - n_val:])
        print(f"  训练 {n-n_val} 点 / 验证 {n_val} 点")
    else:
        tr_tr = traj_all
        val = None

    cfg = IdentificationConfig(
        regularization=args.regularization,
        use_weighting=not args.no_weighting,
    )

    print("\n" + "=" * 64)
    print("参数辨识")
    print("=" * 64)
    result = identify(model, data, tr_tr, tau[:n - n_val] if val else tau,
                      cfg=cfg, validation=val, motors=DEFAULT_MOTORS)
    print(result.summary())

    # ---- 输出 ----
    np.savez(os.path.join(args.out, "theta.npz"),
             theta=result.theta, theta_base=result.theta_base,
             rank=result.rank, cond=result.condition_number)
    export_parameter_report(result, os.path.join(args.out, "identify_report.txt"))

    print("\n每连杆（可写回 URDF）:")
    for lk in theta_to_urdf_inertials(model, result.theta):
        print(f"  {lk['link']:12s} m={lk['mass']:7.4f} kg  "
              f"c=[{lk['com'][0]:+.4f} {lk['com'][1]:+.4f} {lk['com'][2]:+.4f}]")

    print(f"\n已输出:\n  {os.path.join(args.out,'theta.npz')}"
          f"\n  {os.path.join(args.out,'identify_report.txt')}")

    # ---- 质量守恒自检（仿真模式下真值已知）----
    if args.simulate:
        m_nom = float(np.sum([model.inertias[i].mass for i in range(1, model.njoints)]))
        m_hat = float(sum(lk["mass"] for lk in theta_to_urdf_inertials(model, result.theta)))
        print(f"\n总质量: 标称 {m_nom:.4f} kg / 辨识 {m_hat:.4f} kg "
              f"(差 {abs(m_hat-m_nom)/m_nom*100:.1f}%)")

    if not result.physical_ok:
        print("\n⚠️ 物理一致性未通过（基参数解不唯一，属常见现象；"
              "可用保标称值的解改善）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
