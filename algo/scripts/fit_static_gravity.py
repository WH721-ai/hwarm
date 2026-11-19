#!/usr/bin/env python3
"""C-05 静态重力辨识 —— 参数拟合（离线）

把 collect_static_gravity.py 采到的 (q, tau) 解最小二乘：

    tau_ij = Y_g(q_i)[j, :] · theta + b_j

其中 `Y_g` 是 Pinocchio 的**关节力矩回归矩阵**（静止时 dq=ddq=0，
它就退化成重力回归矩阵），`theta` 是每连杆 10 个惯性参数，
`b_j` 是每关节一个常值（吸收静摩擦 + 零位/装配偏差）。

⚠️ 【重要】本工具**不是用来测每根连杆的质心的**。
   静态数据只能辨识出「基参数」—— 若干惯性参数的线性组合，共 10 维
   （60 个参数里）。具体到每根连杆的 m·c，18 个分量里只有 4 个能独立辨识。

   它真正的产出是：**一个能准确复现真实重力矩的模型** Y(q)·θ
   —— 重力补偿需要的正是这个，而不是每根连杆的物理参数。

   想拿到每根连杆的真实质心，得靠 CAD 几何，或者动态辨识（C-07/C-08，
   需要激励轨迹让机械臂动起来）。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    python3 algo/scripts/fit_static_gravity.py algo/data/static_gravity_xxx.yaml
    # 加 --synthetic 做自检：用标称参数造数据再拟合回去，验证管线正确
    python3 algo/scripts/fit_static_gravity.py --synthetic

输出
    · 可辨识基参数的个数 / 条件数
    · 残差 RMS（验收线 0.2 N·m）
    · 辨识模型 vs 标称模型的验证残差（这才是验收标准）
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    print("需要 pinocchio：source /opt/ros/jazzy/setup.bash")
    raise

# 允许直接 import algo/src
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.dirname(_HERE)),
    "hw_arm_ws", "install", "hw_arm_description", "share",
    "hw_arm_description", "urdf", "hw_arm.urdf")


def load_model(urdf: str):
    if not os.path.isfile(urdf):
        raise SystemExit(f"找不到 URDF: {urdf}\n先 colcon build --packages-select hw_arm_description")
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    return model, data


def static_regressor(model, data, q_samples: np.ndarray) -> np.ndarray:
    """静止姿态下的回归矩阵：Y[i*nv:(i+1)*nv, :] = computeJointTorqueRegressor(q, 0, 0)"""
    N, nq = q_samples.shape
    P = 10 * (model.njoints - 1)
    Y = np.zeros((N * model.nv, P))
    zero = np.zeros(model.nv)
    for i in range(N):
        Yi = pin.computeJointTorqueRegressor(model, data, q_samples[i], zero, zero)
        Y[i * model.nv:(i + 1) * model.nv, :] = np.asarray(Yi)
    return Y


def fit(model, data, q: np.ndarray, tau: np.ndarray, use_bias: bool = True,
        rtol: float = 1e-6, fix_mass: bool = False):
    """解 tau = Y·theta + bias。返回 dict。

    fix_mass=True：把质量列的贡献从 tau 里减掉（质量固定为模型里的称重值），
    只辨识剩下的参数（一阶矩 + 惯量）。这样 m 与 c 就不会互相抵消。
    """
    Y = static_regressor(model, data, q)
    t = tau.reshape(-1)
    nv = model.nv
    N = q.shape[0]

    if fix_mass:
        n_links = model.njoints - 1
        m_cols = [10 * i for i in range(n_links)]      # 每连杆第 0 个参数是质量
        m_fixed = np.array([model.inertias[i + 1].mass for i in range(n_links)])
        t = t - Y[:, m_cols] @ m_fixed
        keep = [c for c in range(Y.shape[1]) if c not in m_cols]
        Y = Y[:, keep]

    # 附加每关节常值列（静摩擦/零位偏差）
    if use_bias:
        B = np.zeros((N * nv, nv))
        for j in range(nv):
            B[j::nv, j] = 1.0
        A = np.hstack([Y, B])
    else:
        A = Y
        B = None

    # SVD 求可辨识子空间
    U, s, Vt = np.linalg.svd(A, full_matrices=False)
    # 秩判定：1e-6 相对容差。用 1e-10 会把"SVD 数值噪声级"的方向也算进去
    # （实测静态重力回归矩阵只有 10 个真实可辨识方向，剩两个奇异值 ~1e-6·σmax）
    tol = max(s[0] * rtol, np.finfo(float).eps)
    rank = int((s > tol).sum())

    # 最小范数解
    x = Vt[:rank, :].T @ ((U[:, :rank].T @ t) / s[:rank])

    pred = A @ x
    resid = t - pred
    rms = float(np.sqrt(np.mean(resid ** 2)))

    theta_raw = x[:Y.shape[1]] if use_bias else x
    bias = x[Y.shape[1]:] if use_bias else None
    if fix_mass:
        n_links = model.njoints - 1
        theta = np.zeros(10 * n_links)
        keep = [c for c in range(10 * n_links) if c % 10 != 0]
        theta[keep] = theta_raw[:len(keep)]
        for i in range(n_links):
            theta[10 * i] = model.inertias[i + 1].mass
    else:
        theta = theta_raw

    # 参数不确定度：cov = σ² (AᵀA)⁺（用伪逆，天然只覆盖可辨识子空间）
    dof = max(1, A.shape[0] - rank)
    sigma2 = float(np.sum(resid ** 2) / dof)
    ATA_pinv = Vt[:rank, :].T @ np.diag(1.0 / s[:rank] ** 2) @ Vt[:rank, :]
    cov = sigma2 * ATA_pinv
    std_full = np.sqrt(np.clip(np.diag(cov), 0, None))
    if fix_mass:
        n_links = model.njoints - 1
        std = np.zeros(10 * n_links)
        keep = [c for c in range(10 * n_links) if c % 10 != 0]
        std[keep] = std_full[:len(keep)]
    else:
        std = std_full[:Y.shape[1]] if bias is None else std_full[:Y.shape[1]]

    return dict(Y=Y, A=A, theta=theta, bias=bias, rank=rank, std=std, sigma2=sigma2,
                singular_values=s, rms=rms, resid=resid, pred=pred,
                cond=float(s[0] / s[rank - 1]) if rank else float("inf"))


def report_first_moments(model, theta: np.ndarray, data):
    """把 theta 拆回每连杆的 (m, m·c, I)，并算质心。"""
    rows = []
    for i in range(1, model.njoints):
        p = theta[10 * (i - 1):10 * i]
        m, mc = p[0], p[1:4]
        # Pinocchio 的 toDynamicParameters 顺序：
        #   [m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
        Ixx, Ixy, Iyy, Ixz, Iyz, Izz = p[4], p[5], p[6], p[7], p[8], p[9]
        I = np.array([[Ixx, Ixy, Ixz],
                      [Ixy, Iyy, Iyz],
                      [Ixz, Iyz, Izz]])
        c = mc / m if abs(m) > 1e-9 else np.full(3, np.nan)
        rows.append(dict(name=model.names[i], m=m, mc=mc, c=c, I=I))
    return rows


def synthetic_check(model, data):
    """用标称参数造一批静止数据，再拟合回去，验证管线。"""
    rng = np.random.default_rng(0)
    N = 30
    lo = np.array([-0.8, -0.1, 0.1, -1.0, -0.6, -1.0])
    hi = np.array([2.0, 2.8, 2.0, 1.0, 0.6, 1.0])
    q = rng.uniform(lo, hi, (N, model.nq))

    theta_true = np.concatenate([
        np.asarray(model.inertias[i].toDynamicParameters())
        for i in range(1, model.njoints)])

    Y = static_regressor(model, data, q)
    tau = (Y @ theta_true).reshape(N, model.nv)
    tau += rng.normal(0, 0.02, tau.shape)      # 20 mN·m 噪声

    res = fit(model, data, q, tau, use_bias=False)
    err = res["theta"] - theta_true
    # 只比较可辨识子空间里的投影
    U, s, Vt = np.linalg.svd(res["A"], full_matrices=False)
    rank = res["rank"]
    basis = Vt[:rank, :].T
    proj_err = np.linalg.norm(basis.T @ err)
    print("=" * 74)
    print("  合成数据自检（用标称参数造数据 → 拟合 → 比对）")
    print("=" * 74)
    print(f"  样本数      : {N} 个静止姿态")
    print(f"  真值参数数  : {len(theta_true)}")
    print(f"  可辨识基参数: {res['rank']}  "
          f"（剩下的不可辨识，符合静态重力辨识的理论预期）")
    print(f"  条件数      : {res['cond']:.2e}")
    print(f"  残差 RMS    : {res['rms']*1000:.2f} mN·m  （噪声 σ=20 mN·m）")
    print(f"  参数投影误差: {proj_err:.2e}  "
          f"{'✅ 管线正确' if proj_err < 1e-2 else '❌ 有问题'}")
    print("=" * 74)
    return 0 if proj_err < 1e-2 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="C-05 静态重力辨识拟合")
    ap.add_argument("data", nargs="?", help="采集到的 yaml")
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--synthetic", action="store_true", help="合成数据自检")
    ap.add_argument("--no-bias", action="store_true", help="不拟合每关节常值")
    ap.add_argument("--fix-mass", action="store_true",
                    help="质量固定为 URDF 里的称重值（减少未知数，结果更稳）")
    ap.add_argument("--out", default=None, help="把辨识结果写成 yaml")
    args = ap.parse_args()

    model, data = load_model(args.urdf)
    print(f"  模型: {args.urdf}")
    print(f"  nq={model.nq} nv={model.nv} joints={model.njoints - 1}")

    if args.synthetic:
        return synthetic_check(model, data)

    if not args.data:
        ap.error("要么给数据文件，要么 --synthetic")

    import yaml
    raw = yaml.safe_load(open(args.data, encoding="utf-8"))
    samples = raw["samples"]
    q = np.array([s["q"] for s in samples])
    tau = np.array([s["tau"] for s in samples])
    meta = raw.get("meta", {})

    print(f"  数据: {args.data}  {len(samples)} 个姿态")
    if meta.get("note"):
        print(f"  备注: {meta['note']}")

    res = fit(model, data, q, tau, use_bias=not args.no_bias,
              fix_mass=args.fix_mass)

    print()
    print("=" * 74)
    print(f"  可辨识基参数: {res['rank']} / {res['Y'].shape[1] + (0 if args.no_bias else model.nv)}")
    print(f"  条件数      : {res['cond']:.2e}")
    print(f"  残差 RMS    : {res['rms']:.4f} N·m   "
          f"{'✅ < 0.2 达标' if res['rms'] < 0.2 else '⚠️ > 0.2（姿态不够多样？或力矩标度不对）'}")
    print("=" * 74)

    if res["bias"] is not None:
        print("\n  每关节常值偏差（摩擦/零位）：")
        for i in range(model.nv):
            print(f"    joint{i+1}: {res['bias'][i]:+.3f} N·m")

    # ── 验收：辨识参数 vs 标称参数（在留出样本上比）──
    print("\n  " + "-" * 70)
    print("  验收：用辨识参数做重力补偿，比标称参数好多少？")
    print("  " + "-" * 70)
    n = len(samples)
    idx_val = np.arange(0, n, max(2, n // 5))          # 每 k 个留一个做验证
    idx_tr = np.setdiff1d(np.arange(n), idx_val)
    if len(idx_val) >= 2 and len(idx_tr) >= 6:
        res_tr = fit(model, data, q[idx_tr], tau[idx_tr],
                     use_bias=not args.no_bias, fix_mass=args.fix_mass)
        Yv = static_regressor(model, data, q[idx_val])
        tv = tau[idx_val].reshape(-1)

        theta_nom = np.concatenate(
            [np.asarray(model.inertias[i].toDynamicParameters())
             for i in range(1, model.njoints)])
        rms_nom = float(np.sqrt(np.mean((Yv @ theta_nom - tv) ** 2)))

        pred_id = Yv @ res_tr["theta"]
        if res_tr["bias"] is not None and not args.no_bias:
            Bv = np.zeros((len(idx_val) * model.nv, model.nv))
            for j in range(model.nv):
                Bv[j::model.nv, j] = 1.0
            pred_id = pred_id + Bv @ res_tr["bias"]
        rms_id = float(np.sqrt(np.mean((pred_id - tv) ** 2)))

        print(f"  训练样本 {len(idx_tr)} / 验证样本 {len(idx_val)}")
        print(f"  可辨识基参数 : {res_tr['rank']}（含 {0 if args.no_bias else model.nv} 个偏置）")
        print(f"  条件数       : {res_tr['cond']:.2f}")
        print(f"  训练残差 RMS : {res_tr['rms']:.4f} N·m")
        print()
        print(f"  验证残差 RMS  标称参数 : {rms_nom:.4f} N·m")
        print(f"              辨识参数 : {rms_id:.4f} N·m   "
              f"{'✅ 优于标称 %.1f 倍' % (rms_nom / max(rms_id, 1e-9)) if rms_id < rms_nom else '⚠️ 没有改善'}")
        print()
        print(f"  验收线 0.2 N·m：{'✅ 达标' if rms_id < 0.2 else '❌ 未达标'}")
    else:
        print("  样本太少，跳过验证")

    print("\n  ⚠️ 静态辨识得到的是**基参数**（可辨识的线性组合），不是每根连杆的真实")
    print("     质量/质心 —— 单靠静态数据两者分不开（m 与 c 同时缩放得到同样的力矩）。")
    print("     补偿只需要 Y(q)·θ 数值正确，不需要 θ 的每个分量都有物理意义。")
    if not args.fix_mass:
        print("     若想让结果更接近物理参数，加 --fix-mass（质量固定为称重值）。")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            yaml.safe_dump({
                "meta": {"source": args.data, "rank": res["rank"],
                         "rms": res["rms"], "cond": res["cond"]},
                "links": [{"name": r["name"],
                           "mass": float(r["m"]),
                           "com": [float(x) for x in r["c"]]} for r in rows],
            }, f, allow_unicode=True, sort_keys=False)
        print(f"\n  → 已写出 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
