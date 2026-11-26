#!/usr/bin/env python3
"""读一次当前关节角，打印成便于比对的格式（用于零位漂移检查）

用法：
    source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh
    python3 read_pose.py                # 读一次
    python3 read_pose.py --avg 20       # 取 20 帧平均（更稳）

典型用途：**断电 → 手动拨动 joint → 上电 → 再读一次**，
对比两次读数是否一致，判断编码器零位有没有漂。
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:
    print("❌ 找不到 rclpy —— 先 source env.sh")
    sys.exit(2)

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Reader(Node):
    def __init__(self):
        super().__init__("read_pose")
        self.q = None
        self.tau = None
        self.dq = None
        self.n = 0
        self.create_subscription(JointState, "/joint_states", self._cb, 50)

    def _cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.q = np.array([msg.position[idx[j]] for j in JOINTS])
        self.dq = np.array([msg.velocity[idx[j]] for j in JOINTS])
        self.tau = np.array([msg.effort[idx[j]] for j in JOINTS])
        self.n += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--avg", type=int, default=10, help="取多少帧平均（默认 10）")
    ap.add_argument("--tag", default="", help="这次读数的标签，便于记录")
    args = ap.parse_args()

    rclpy.init()
    node = Reader()
    try:
        t0 = time.monotonic()
        while node.n < args.avg and time.monotonic() - t0 < 20:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.n == 0:
            print("❌ 收不到 /joint_states —— bringup 起了吗？")
            return 1

        acc = []
        for _ in range(args.avg):
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.q is not None:
                acc.append(node.q.copy())
        q = np.mean(acc, axis=0)

        tag = f"[{args.tag}] " if args.tag else ""
        print(f"{tag}q(rad) = [" + ", ".join(f"{v:+.5f}" for v in q) + "]")
        print(f"{tag}q(deg) = [" + ", ".join(f"{v:+8.2f}" for v in np.degrees(q)) + "]")
        if node.tau is not None:
            print(f"{tag}tau    = [" + ", ".join(f"{v:+.3f}" for v in node.tau) + "] N·m")
        if node.dq is not None:
            print(f"{tag}|dq|max = {np.abs(node.dq).max():.4f} rad/s")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
