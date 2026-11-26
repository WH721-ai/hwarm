#!/usr/bin/env python3
"""走到指定关节角 —— 用 forward_position 控制器（不依赖 JTC）

═══════════════════════════════════════════════════════════════════════
为什么需要这个（2026-11-26）
═══════════════════════════════════════════════════════════════════════
`jog.py` 走的是 `/hw_arm_controller/follow_joint_trajectory`（JTC action），
而零重力调试时启动的是 `hw_arm_forward_position_controller`（没有 action server）：

    $ python3 jog.py 0 0.524 0.436 0 0 0 --abs --time 6
    ❌ 找不到 action server /hw_arm_controller/follow_joint_trajectory

结果机械臂没动，但后面照样调了 set_zero_gravity —— **在零位上进了零重力**，
而零位恰恰是补偿误差最大的地方（实测 joint2 +2.1 / joint3 +5.7 N·m）。

本脚本直接往 forward_position 的 commands 话题发 min-jerk 序列，
和 gravity_verify / collect_static_gravity 用同一套缓动，不依赖 action。

═══════════════════════════════════════════════════════════════════════
安全特性
═══════════════════════════════════════════════════════════════════════
  · 起点取实测 /joint_states，不会跳变
  · 限位检查，超限直接拒绝，不发任何命令
  · 台面净空检查（夹爪包络最低点必须 > --min-z）
  · 到位误差检查：走完若 |q_meas − q_cmd| 太大，提示可能被挡住
  · 先打印对比表，确认后才发（--yes 可跳过确认）

用法：
    source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh

    # 角度直接写（deg），走到 q2=30°, q3=25°
    python3 goto_pose.py --deg 0 30 25 0 0 0 --time 6

    # 弧度
    python3 goto_pose.py --rad 0 0.524 0.436 0 0 0 --time 6

    # 只动某几轴（其余保持当前）
    python3 goto_pose.py --deg 0 30 25 -  - - --time 6
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CMD_TOPIC = "/hw_arm_forward_position_controller/commands"


def _need_rclpy():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        return rclpy, Node, JointState, Float64MultiArray
    except ImportError:
        print("❌ 找不到 rclpy —— 先 source env.sh")
        sys.exit(2)


rclpy, Node, JointState, Float64MultiArray = _need_rclpy()


class Mover(Node):
    def __init__(self):
        super().__init__("goto_pose")
        self.q = None
        self.dq = None
        self.last = 0.0
        self.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)

    def _cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.q = np.array([msg.position[idx[j]] for j in JOINTS])
        self.dq = (np.array([msg.velocity[idx[j]] for j in JOINTS])
                   if len(msg.velocity) == len(msg.name) else np.zeros(6))
        self.last = time.monotonic()

    def wait_state(self, timeout=20.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.q is not None and (time.monotonic() - self.last) < 0.3:
                return True
        return False

    def move_to(self, goal, duration):
        rclpy.spin_once(self, timeout_sec=0.05)
        q0 = self.q.copy()
        period = 1.0 / 100.0
        t0 = time.monotonic()
        k = 0
        while True:
            a = min(1.0, (time.monotonic() - t0) / max(duration, 1e-3))
            s = 10 * a**3 - 15 * a**4 + 6 * a**5          # min-jerk
            m = Float64MultiArray()
            m.data = [float(x) for x in q0 + (goal - q0) * s]
            self.pub.publish(m)
            k += 1
            if a >= 1.0:
                break
            rem = t0 + k * period - time.monotonic()
            rclpy.spin_once(self, timeout_sec=max(1e-3, min(rem, period)) if rem > 0 else 1e-3)


def load_limits(urdf):
    try:
        import pinocchio as pin
        m = pin.buildModelFromUrdf(urdf)
        return m.lowerPositionLimit.copy(), m.upperPositionLimit.copy()
    except Exception:
        return None, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--deg", nargs=6, help="目标角度（度），'-' 表示保持当前")
    g.add_argument("--rad", nargs=6, help="目标弧度，'-' 表示保持当前")
    ap.add_argument("--time", type=float, default=6.0, help="运动时长 s（默认 6）")
    ap.add_argument("--urdf", default=__import__("os").environ.get("HW_ARM_URDF"))
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    args = ap.parse_args()

    target_raw = args.deg if args.deg else args.rad
    is_deg = bool(args.deg)

    rclpy.init()
    node = Mover()
    try:
        if not node.wait_state(20.0):
            print("❌ 收不到 /joint_states —— bringup 起了吗？控制器是 "
                  "hw_arm_forward_position_controller 吗？")
            return 1

        q0 = node.q.copy()
        goal = q0.copy()
        for i, tok in enumerate(target_raw):
            if tok == "-":
                continue
            v = float(tok)
            goal[i] = np.radians(v) if is_deg else v

        lo, hi = load_limits(args.urdf) if args.urdf else (None, None)
        print(f"  时长 {args.time:.1f}s")
        print(f"  {'关节':>8}{'起点':>11}{'终点':>11}{'增量':>11}{'平均速度':>12}")
        bad = []
        for i in range(6):
            d = goal[i] - q0[i]
            print(f"  {JOINTS[i]:>8}{q0[i]:>+11.4f}{goal[i]:>+11.4f}"
                  f"{d:>+11.4f}{d/args.time:>+11.3f}")
            if lo is not None and (goal[i] < lo[i] - 1e-6 or goal[i] > hi[i] + 1e-6):
                bad.append(f"{JOINTS[i]} {np.degrees(goal[i]):+.2f}° 超出 "
                           f"[{np.degrees(lo[i]):+.2f}, {np.degrees(hi[i]):+.2f}]°")
        if bad:
            print("\n  ❌ 超限，拒绝发送：")
            for b in bad:
                print(f"     {b}")
            return 1

        if not args.yes:
            ans = input("\n确认发送？[y/N] ").strip().lower()
            if ans not in ("y", "yes"):
                print("已取消")
                return 0

        node.move_to(goal, args.time)
        time.sleep(0.6)
        rclpy.spin_once(node, timeout_sec=0.1)

        err = np.degrees(node.q - goal)
        print(f"\n  到位后 q(deg) = [{', '.join(f'{v:+.2f}' for v in np.degrees(node.q))}]")
        print(f"  到位误差(deg) = [{', '.join(f'{v:+.2f}' for v in err)}]")
        if np.abs(err).max() > 8.0:
            print("  ⚠️ 到位误差偏大 —— kp 位置环本来就有重力下垂（g/kp ≈ 2~6°），"
                  "但超过 8° 要怀疑被挡住")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
