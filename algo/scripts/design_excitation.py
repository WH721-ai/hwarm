#!/usr/bin/env python3
"""设计 Fourier 激励轨迹 → 安全检查 → 输出 CSV。

用法::

    python3 scripts/design_excitation.py \
        --urdf /tmp/hw_arm.urdf \
        --out data/excitation/ \
        --harmonics 5 --duration 6 --periods 2 \
        --q-start 0,1.5,0.6,0,-0.3,0

输出::

    <out>/excitation.csv        轨迹（t, q, dq, ddq）
    <out>/excitation_report.txt 安全检查 + 条件数报告
    <out>/excitation_params.npz Fourier 系数（可复用）
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import pinocchio as pin  # noqa: E402

from m30_sysid import (ExcitationConfig, SafetyLimits,  # noqa: E402
                       build_excitation_trajectory, check_start_pose,
                       check_trajectory, design_excitation)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="激励轨迹设计与安全检查")
    p.add_argument("--urdf", required=True, help="URDF 路径")
    p.add_argument("--out", default="data/excitation", help="输出目录")
    p.add_argument("--frame", default="tool0", help="末端帧名")
    p.add_argument("--harmonics", type=int, default=5, help="谐波数 K")
    p.add_argument("--duration", type=float, default=6.0, help="基本周期 T (s)")
    p.add_argument("--periods", type=int, default=2, help="重复周期数")
    p.add_argument("--transition", type=float, default=1.5, help="过渡段时长 (s)")
    p.add_argument("--amplitude", type=float, default=0.15, help="初始幅值 (rad)")
    p.add_argument("--q-start", type=str, default=None,
                   help="当前关节角（逗号分隔）；省略则用限位中点")
    p.add_argument("--v-limit", type=float, default=1.0, help="速度上限 rad/s")
    p.add_argument("--a-limit", type=float, default=5.0, help="加速度上限 rad/s²")
    p.add_argument("--tau-ratio", type=float, default=0.35,
                   help="设计力矩上限 = 峰值 × 该比例（0.35≈额定）")
    p.add_argument("--starts", type=int, default=3, help="优化起点数")
    p.add_argument("--max-iter", type=int, default=200)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--q5-offset", type=float, default=0.30,
                   help="joint5 中心与腕奇异(0)的最小距离；<0 表示不调整")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    frame_id = model.getFrameId(args.frame)
    if frame_id >= len(model.frames):
        print(f"❌ 找不到末端帧 {args.frame}")
        return 1
    nv = model.nv

    # ---- 安全边界 ----
    limits = SafetyLimits.from_model(model).with_conservative_speed(args.v_limit)
    limits.a_max = np.minimum(limits.a_max, args.a_limit)
    limits.tau_rated = limits.tau_peak * args.tau_ratio

    print("=" * 68)
    print("安全边界")
    print("=" * 68)
    names = [f"j{i+1}" for i in range(nv)]
    print(f"{'':6s} {'q_min':>8s} {'q_max':>8s} {'v_max':>7s} "
          f"{'a_max':>7s} {'τ_rated':>8s} {'τ_hard':>8s}")
    for i in range(nv):
        print(f"{names[i]:6s} {limits.q_min[i]:8.3f} {limits.q_max[i]:8.3f} "
              f"{limits.v_max[i]:7.2f} {limits.a_max[i]:7.1f} "
              f"{limits.tau_rated[i]:8.1f} {limits.tau_hard[i]:8.1f}")

    if args.q_start:
        q_start = np.array([float(v) for v in args.q_start.split(",")])
        if q_start.size != nv:
            print(f"❌ --q-start 需要 {nv} 个数")
            return 1
    else:
        q_start = (limits.q_min + limits.q_max) / 2.0
    print(f"\n起始位姿: {np.array2string(q_start, precision=3)}")

    # 激励轨迹的中心：默认与起点相同，但 joint5 必须避开腕奇异(q5=0)。
    # 实测：轨迹穿过 q5≈0 时 σ_min 会掉到 1e-3 量级（腕奇异）。
    q_center = q_start.copy()
    if args.q5_offset > 0 and nv >= 5:
        if abs(q_center[4]) < args.q5_offset:
            sign = 1.0 if q_center[4] >= 0 else -1.0
            new_q5 = sign * args.q5_offset
            new_q5 = float(np.clip(new_q5, limits.q_min[4], limits.q_max[4]))
            print(f"⚠️  joint5 起点 {q_center[4]:+.3f} 太靠近腕奇异，"
                  f"中心调整到 {new_q5:+.3f}")
            q_center[4] = new_q5

    # ---- 设计 ----
    cfg = ExcitationConfig(
        n_harmonics=args.harmonics, duration=args.duration,
        n_periods=args.periods, transition_time=args.transition,
        amplitude_init=args.amplitude, n_starts=args.starts,
        max_iter=args.max_iter, random_seed=args.seed,
    )
    print("\n" + "=" * 68)
    print(f"激励轨迹优化（K={cfg.n_harmonics}, T={cfg.duration}s, "
          f"起点数={cfg.n_starts}）")
    print("=" * 68)
    result = design_excitation(model, data, limits, cfg, q_center=q_center,
                               log=lambda s: print(s), frame_id=frame_id)
    print(f"\n优化后条件数: {result.condition_number:.1f} "
          f"(初值 {result.initial_condition:.1f}, 秩 {result.rank})")

    # ---- 修复：按比例缩放，保证一定满足约束 ----
    params = result.params
    if cfg.repair_by_scaling:
        from m30_sysid.design import repair_by_scale
        params, alpha = repair_by_scale(
            params, limits, model, data, frame_id=frame_id,
            q_start=q_start, transition_time=cfg.transition_time)
        if alpha < 0.999:
            print(f"⚠️  优化结果仍违反约束，已按 α={alpha:.3f} 缩放幅值修复")
            result.params = params
            # 重新计算条件数
            t_chk = np.linspace(0, 2*np.pi/params.omega, 60, endpoint=False)
            from m30_sysid.regressor import build_regressor, condition_number
            tmp = build_excitation_trajectory(
                params, q_start, None, None, 1, 0.0, rate=200.0)
            Yc = build_regressor(model, data, tmp)
            c2, r2 = condition_number(Yc)
            print(f"   修复后条件数: {c2:.1f}（秩 {r2}）")
            result.condition_number = c2
            result.rank = r2

    # ---- 组装完整轨迹（含过渡段）----
    traj = build_excitation_trajectory(
        result.params, q_start=q_start, v_start=None, a_start=None,
        n_periods=cfg.n_periods, transition_time=cfg.transition_time,
        rate=1000.0,
    )
    print(f"\n完整轨迹: {traj.duration:.2f} s, {len(traj.t)} 点 "
          f"({1/np.median(np.diff(traj.t)):.0f} Hz)")

    # ---- 安全检查 ----
    print("\n" + "=" * 68)
    print("安全检查")
    print("=" * 68)
    rep = check_trajectory(traj, limits, model=model, data=data,
                           frame_id=frame_id)
    rep_start = check_start_pose(traj, q_start, tol=1e-3)
    print(rep.summary())
    if not rep_start.ok:
        rep.ok = False
        rep.violations.extend(rep_start.violations)
        print(rep_start.summary())

    # ---- 输出 ----
    csv = os.path.join(args.out, "excitation.csv")
    traj.to_csv(csv)
    np.savez(os.path.join(args.out, "excitation_params.npz"),
             q0=result.params.q0, A=result.params.A, B=result.params.B,
             omega=result.params.omega, q_start=q_start,
             n_periods=cfg.n_periods, transition_time=cfg.transition_time)

    report_path = os.path.join(args.out, "excitation_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(f"URDF: {args.urdf}\n")
        f.write(f"末端帧: {args.frame}\n")
        f.write(f"K={cfg.n_harmonics}, T={cfg.duration}s, "
                f"周期数={cfg.n_periods}, 过渡={cfg.transition_time}s\n")
        f.write(f"优化后条件数: {result.condition_number:.2f} "
                f"(初值 {result.initial_condition:.2f})\n")
        f.write(f"秩: {result.rank}\n\n")
        f.write(rep.summary() + "\n")

    print(f"\n已输出:\n  {csv}\n  {report_path}")

    if not rep.ok:
        print("\n❌ 安全检查未通过，**禁止执行**。请调整参数（降低幅值/速度）后重试。")
        return 2

    print("\n✅ 安全检查通过。")
    print("\n下一步:")
    print("  1) 用 jog.py 移动到起始位姿（或直接用当前位姿重新设计）")
    print("  2) 以 30% 幅值试跑，确认无异常")
    print("  3) 执行完整轨迹，250 Hz 记录 q/dq/tau")
    print("  4) python3 scripts/identify_params.py --data <recorded.csv>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
