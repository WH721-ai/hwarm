#!/usr/bin/env python3
"""重力模型现场验证 —— 在**训练范围之内**的位形上核对 tau_ff 与实测 effort

══════════════════════════════════════════════════════════════════════
为什么需要这个脚本（2026-11-26）
══════════════════════════════════════════════════════════════════════
零位（所有关节 0°）是**机械结构自己托住**的位形 —— 电机只需要 <0.01 N·m
就能保持，所以在那儿根本测不到重力，没法验证模型。

而 C-05 的辨识数据只覆盖：
    q2 ∈ [+24°, +90°]     q3 ∈ [+14°, +124°]
零位（q2≈0、q3≈0）**在范围之外**，属于外推区。

所以正确的验证地点是**训练范围里面** —— 那儿机械臂是自由站立的，
effort 才真的等于重力。

══════════════════════════════════════════════════════════════════════
它做什么
══════════════════════════════════════════════════════════════════════
  1. 从已采样本里挑 N 个位形（默认 6 个，尽量分散）
  2. 逐个用位置控制走过去，等静止
  3. 读实测 effort
  4. 用 Pinocchio 回归矩阵 + θ̂ 独立算一遍 g(q)
  5. 打印每轴残差

⚠️ 脚本**自己独立算 g(q)**，不读控制器日志 —— 这样同时也交叉验证了
   C++ 侧的 computeGravityFromParamsAndOffset 实现。

用法：
  source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh
  python3 gravity_verify.py                       # 6 个位形
  python3 gravity_verify.py --poses 8 --tol 0.4   # 8 个，容差 0.4 N·m
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
except ImportError:
    print("❌ 找不到 rclpy —— 先 source env.sh")
    sys.exit(2)

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CMD_TOPIC = "/hw_arm_forward_position_controller/commands"


def load_theta(path):
    import yaml

    with open(path) as f:
        d = yaml.safe_load(f)
    theta = np.array(d["theta"], dtype=float)
    off = np.array(d.get("offsets", np.zeros(6)), dtype=float)
    return theta, off, d.get("meta", {})


def make_gravity_fn(urdf, theta, off):
    """返回 g(q) = Y(q)·θ̂ + offset（与 C++ 侧 computeGravityFromParamsAndOffset 等价）"""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()

    def g(q):
        Y = np.asarray(
            pin.computeJointTorqueRegressor(model, data, np.asarray(q, dtype=float),
                                            np.zeros(model.nv), np.zeros(model.nv))
        ).copy()
        return Y @ theta + off

    return g, model


def pick_poses(samples_path, n):
    """从已采样本里挑 n 个分散的位形"""
    import yaml

    with open(samples_path) as f:
        S = yaml.safe_load(f)["samples"]
    Q = np.array([s["q"] for s in S], dtype=float)
    Q = np.unique(np.round(Q, 4), axis=0)
    if len(Q) <= n:
        return Q
    # 按"离已选集合的最大最小距离"贪心挑，保证分散
    idx = [int(np.argmax(Q[:, 1]))]  # 先取 q2 最大的（离零位最远，最自由）
    while len(idx) < n:
        d = np.min(np.linalg.norm(Q[:, None, :] - Q[None, idx, :], axis=2), axis=1)
        idx.append(int(np.argmax(d)))
    return Q[sorted(idx)]


class Verifier(Node):
    def __init__(self):
        super().__init__("gravity_verify")
        self.q = None
        self.dq = None
        self.tau = None
        self.last_stamp = None
        self.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)

    def _cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.q = np.array([msg.position[idx[j]] for j in JOINTS])
        self.dq = (np.array([msg.velocity[idx[j]] for j in JOINTS])
                   if len(msg.velocity) == len(msg.name) else np.zeros(6))
        self.tau = (np.array([msg.effort[idx[j]] for j in JOINTS])
                    if len(msg.effort) == len(msg.name) else np.zeros(6))
        self.last_stamp = time.monotonic()

    def wait_state(self, timeout=20.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.q is not None and (time.monotonic() - self.last_stamp) < 0.3:
                return True
        return False

    def move_to(self, goal, duration=3.0):
        """墙钟 + 五次多项式缓动（和采集脚本一致）"""
        rclpy.spin_once(self, timeout_sec=0.05)
        q0 = self.q.copy()
        rate = 100.0
        period = 1.0 / rate
        t0 = time.monotonic()
        k = 0
        while True:
            a = min(1.0, (time.monotonic() - t0) / max(duration, 1e-3))
            s = 10 * a**3 - 15 * a**4 + 6 * a**5
            m = Float64MultiArray()
            m.data = [float(x) for x in q0 + (goal - q0) * s]
            self.pub.publish(m)
            k += 1
            if a >= 1.0:
                break
            tn = t0 + k * period
            rem = tn - time.monotonic()
            rclpy.spin_once(self, timeout_sec=max(1e-3, min(rem, period)) if rem > 0 else 1e-3)

    def settle(self, vel_tol=0.03, need=8, timeout=4.0):
        t0 = time.monotonic()
        ok = 0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.dq is None:
                continue
            if np.max(np.abs(self.dq)) < vel_tol:
                ok += 1
                if ok >= need:
                    t1 = time.monotonic()
                    while time.monotonic() - t1 < 0.25:
                        rclpy.spin_once(self, timeout_sec=0.02)
                    return True
            else:
                ok = 0
        return False

    def average_tau(self, n=8):
        acc = []
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tau is not None:
                acc.append(self.tau.copy())
        return np.mean(acc, axis=0) if acc else np.zeros(6)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=os.environ.get("HW_ARM_URDF"))
    ap.add_argument("--theta", default=os.path.expanduser("~/hw_arm/algo/data/gravity_theta.yaml"))
    ap.add_argument("--samples", default=os.path.expanduser(
        "~/hw_arm/algo/data/static_gravity_dither.yaml"))
    ap.add_argument("--poses", type=int, default=6, help="验证几个位形（默认 6）")
    ap.add_argument("--tol", type=float, default=0.4, help="每轴容差 N·m（默认 0.4）")
    ap.add_argument("--move-time", type=float, default=3.0, help="每段运动时长 s")
    ap.add_argument("--dry-run", action="store_true", help="只打印位形，不动机械臂")
    ap.add_argument("--return-home", dest="return_home", action="store_true", default=True,
                    help="验证完回到起始位形（默认开）")
    ap.add_argument("--no-return-home", dest="return_home", action="store_false",
                    help="验证完停在最后一个位形")
    args = ap.parse_args()

    if not args.urdf or not os.path.isfile(args.urdf):
        print(f"❌ URDF 不存在: {args.urdf}")
        return 2
    if not os.path.isfile(args.theta):
        print(f"❌ θ̂ 不存在: {args.theta}")
        return 2
    if not os.path.isfile(args.samples):
        print(f"❌ 样本不存在: {args.samples}")
        return 2

    theta, off, meta = load_theta(args.theta)
    g_fn, model = make_gravity_fn(args.urdf, theta, off)
    print(f"θ̂ : {args.theta}")
    print(f"    rank={meta.get('rank')}  rms={meta.get('rms', 0):.4f}  "
          f"holdout={meta.get('holdout_rms', 0):.4f}")

    poses = pick_poses(args.samples, args.poses)
    print(f"\n验证位形（取自已采样本，落在辨识范围内）：")
    for i, q in enumerate(poses):
        g = g_fn(q)
        print(f"  #{i+1}  q(deg)={np.round(np.degrees(q),1)}   "
              f"预测 |g|max={np.abs(g).max():.2f} N·m")
    if args.dry_run:
        print("\n（--dry-run：不动机械臂）")
        return 0

    rclpy.init()
    node = Verifier()
    try:
        if not node.wait_state(20.0):
            print("❌ 收不到 /joint_states —— 确认 bringup 已启动且控制器 active")
            return 1

        q_start = node.q.copy()
        print(f"\n起始 q = {np.round(q_start, 4)}")
        print("开始验证……\n")
        print(f"  {'#':>2} {'轴':>7}{'实测τ':>9}{'预测g':>9}{'残差':>9}   ")
        print("  " + "-" * 52)

        rows = []
        for i, q_goal in enumerate(poses):
            node.move_to(q_goal, args.move_time)
            if not node.settle():
                print(f"  #{i+1} ⚠️ 没等到静止，跳过")
                continue
            q_meas = node.q.copy()
            tau_meas = node.average_tau()
            g_pred = g_fn(q_meas)
            res = tau_meas - g_pred
            rows.append((i, q_meas, tau_meas, g_pred, res))
            for j in range(6):
                flag = "" if abs(res[j]) <= args.tol else "  ⚠️"
                print(f"  {i+1:>2} {'joint'+str(j+1):>7}{tau_meas[j]:9.3f}"
                      f"{g_pred[j]:9.3f}{res[j]:9.3f}{flag}")
            print()

        if args.return_home:
            print("\n回到起始位形……")
            node.move_to(q_start, args.move_time * 1.5)
            node.settle(timeout=5.0)
            print(f"  已回到 {np.round(node.q, 4)}")

        if not rows:
            print("❌ 一个位形都没测成")
            return 1

        R = np.array([r[4] for r in rows])
        print("══ 汇总 ══")
        print(f"  位形数: {len(rows)}")
        print(f"  整体 rms : {np.sqrt((R**2).mean()):.3f} N·m")
        print(f"  每轴 rms : {np.round(np.sqrt((R**2).mean(0)), 3)}")
        print(f"  每轴 |max|: {np.round(np.abs(R).max(0), 3)}")
        worst = int(np.argmax(np.abs(R).max(0)))
        print(f"  最差的轴  : joint{worst+1}")

        bad = np.abs(R) > args.tol
        nbad = int(bad.sum())
        print(f"\n  超出 {args.tol} N·m 容差的点数: {nbad} / {R.size}")
        if nbad == 0:
            print("  ✅ 模型在现场位形上通过验证")
            return 0
        print("  ⚠️ 有超差点 —— 看是哪些轴、哪些位形")
        for j in range(6):
            if (np.abs(R[:, j]) > args.tol).any():
                print(f"     joint{j+1}: 残差 {np.round(R[:, j], 2)}")
        return 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
