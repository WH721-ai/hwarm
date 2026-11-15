#!/usr/bin/env python3
"""点动机械臂：以**当前位置**为起点，按给定的增量（或绝对角度）发一条轨迹。

相比手写 ros2 action send_goal 的 YAML 串，这个脚本：
    · 自动读当前 /joint_states 作为轨迹起点（不会跳变）
    · 自动做限位检查，超限直接拒绝，不会把命令发出去
    · 支持增量（默认）和绝对位置（--abs）
    · 打印每个关节 起点 → 终点 和估算的最大速度

用法：
    # 每个关节 +0.1 rad（约 5.7°），3 秒走完
    python3 scripts/jog.py 0.1 0.1 0.1 0.05 0.05 0.05

    # 只动 joint1
    python3 scripts/jog.py 0.2 --only 1

    # 绝对位置（rad）
    python3 scripts/jog.py 0.9 0.1 0.1 0.2 0.4 0.28 --abs

    # 指定 5 秒
    python3 scripts/jog.py 0.3 0.3 0.3 0.2 0.2 0.2 --time 5
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# URDF 限位（与 config/joint/joint_limits.yaml 一致）
LIMITS = {
    "joint1": (-1.3963, 3.4907),
    "joint2": (-0.17453, 3.3161),
    "joint3": (0.0, 2.4435),
    "joint4": (-1.5708, 1.5708),
    "joint5": (-0.7854, 0.7854),
    "joint6": (-1.5708, 1.5708),
}


class Jog(Node):
    def __init__(self):
        super().__init__("jog")
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   "/hw_arm_controller/follow_joint_trajectory")

    def _cb(self, msg: JointState):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.q = [d[j] for j in JOINTS]

    def wait_current(self, timeout=5.0) -> list[float]:
        import time
        t0 = time.time()
        while self.q is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.q is None:
            raise RuntimeError("5 秒内读不到 /joint_states")
        return self.q


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("values", nargs="+", type=float,
                    help="6 个值（增量或绝对位置，rad）")
    ap.add_argument("--time", type=float, default=3.0, help="轨迹时长（秒）")
    ap.add_argument("--abs", action="store_true", help="按绝对位置解释 values")
    ap.add_argument("--only", default=None, help="只动这些关节（1-based，逗号分隔）")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    args = ap.parse_args()

    rclpy.init()
    node = Jog()
    try:
        cur = node.wait_current()
    except RuntimeError as e:
        print(f"❌ {e}")
        print("   栈起来了吗？ros2 launch ... 之后再跑本脚本")
        return 1

    only = None
    if args.only:
        only = {int(x) - 1 for x in args.only.split(",") if x.strip()}
        vals = [0.0] * 6
        for i, v in enumerate(args.values[: len(only) or 6]):
            pass
        # 只取给出的前 N 个值分配给指定关节
        idx = sorted(only)
        vals = [0.0] * 6
        for k, i in enumerate(idx):
            vals[i] = args.values[k] if k < len(args.values) else 0.0
    else:
        vals = list(args.values)
        if len(vals) != 6:
            print(f"❌ 需要 6 个值（或配合 --only），收到 {len(vals)} 个")
            return 1

    target = [v if args.abs else cur[i] + v for i, v in enumerate(vals)]

    # 限位检查
    bad = []
    for i, j in enumerate(JOINTS):
        lo, hi = LIMITS[j]
        if only is not None and i not in only:
            continue
        if not (lo <= target[i] <= hi):
            bad.append(f"  {j}: 目标 {target[i]:+.4f} 超出限位 [{lo:+.4f}, {hi:+.4f}]")
    if bad:
        print("❌ 目标超限，拒绝发送：")
        print("\n".join(bad))
        return 1

    print(f"轨迹时长: {args.time:.1f}s")
    print(f"{'关节':8s} {'起点':>9s} {'终点':>9s} {'增量':>9s} {'平均速度':>12s}")
    for i, j in enumerate(JOINTS):
        if only is not None and i not in only:
            print(f"{j:8s} {cur[i]:+9.4f} {'（不动）':>9s}")
            continue
        d = target[i] - cur[i]
        v = abs(d) / args.time if args.time > 0 else 0
        print(f"{j:8s} {cur[i]:+9.4f} {target[i]:+9.4f} {d:+9.4f} {v:9.3f} rad/s")

    # 速度合理性提醒
    fast = [j for i, j in enumerate(JOINTS)
            if (only is None or i in only)
            and abs(target[i] - cur[i]) / max(args.time, 1e-3) > 3.0]
    if fast:
        print(f"\n⚠️  这些关节平均速度超过 3 rad/s，可能过冲或被电机限速拦下：{', '.join(fast)}")

    if not args.yes:
        try:
            if input("\n确认发送？[y/N] ").strip().lower() not in ("y", "yes"):
                print("已取消")
                return 0
        except EOFError:
            pass

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    p0 = JointTrajectoryPoint()
    p0.positions = [float(x) for x in cur]
    p0.velocities = [0.0] * 6
    p0.time_from_start.sec = 0
    p1 = JointTrajectoryPoint()
    p1.positions = [float(x) for x in target]
    p1.velocities = [0.0] * 6
    p1.time_from_start.sec = int(args.time)
    p1.time_from_start.nanosec = int((args.time % 1) * 1e9)
    goal.trajectory.points = [p0, p1]

    if not node.client.wait_for_server(timeout_sec=5.0):
        print("❌ 找不到 action server /hw_arm_controller/follow_joint_trajectory")
        return 1

    print("发送中…")
    send_future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        print("❌ 目标被拒绝（可能被另一个目标占用，或限位检查失败）")
        return 1

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.time + 15)
    res = result_future.result()
    if res is None:
        print("⚠️ 等待结果超时")
        return 1
    code = res.result.error_code
    names = {0: "SUCCEEDED", 1: "INVALID_GOAL", 2: "INVALID_JOINTS",
             3: "OLD_HEADER_TIMESTAMP", 4: "PATH_TOLERANCE_VIOLATED",
             5: "GOAL_TOLERANCE_VIOLATED"}
    print(f"结果: {names.get(code, code)}  {res.result.error_string}")

    # 反馈实际到位情况
    import time
    time.sleep(0.5)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.q:
        print(f"\n{'关节':8s} {'目标':>9s} {'实际':>9s} {'误差':>9s}")
        for i, j in enumerate(JOINTS):
            e = node.q[i] - target[i]
            print(f"{j:8s} {target[i]:+9.4f} {node.q[i]:+9.4f} {e:+9.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
