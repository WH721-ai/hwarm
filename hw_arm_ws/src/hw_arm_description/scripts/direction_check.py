#!/usr/bin/env python3
"""**一个电机一个电机**地确认「电机 +q 方向」与「URDF 轴方向」是否一致。

用途：待办 C-02（direction ±1）

═══════════════════════════════════════════════════════════════════
怎么用
═══════════════════════════════════════════════════════════════════
    终端 A（RViz）：
        ros2 launch hw_arm_mujoco display.launch.py

    终端 B：
        python3 scripts/direction_check.py --joint 1
        python3 scripts/direction_check.py --joint 2
        ...

脚本会按顺序做两件事：

    阶段 1（RViz，只看不动真机）
        把 /joint_states 里的 joint{N} 从 0 平滑转到 +30°
        → RViz 里能明显看到这根 link 往哪边转

    阶段 2（真机，只动一点点）
        只使能**这一个**电机，命令它转 +0.05°（默认），保持 1 秒，再失能
        → 读编码器，算出实际 Δq 的正负

判断：
    Δq > 0  →  电机的 +q 和 URDF 的 +q **同向**  →  direction = +1
    Δq < 0  →  反的                              →  direction = −1

═══════════════════════════════════════════════════════════════════
⚠️ 0.05° 可能根本不动 —— 这是物理限制，不是 bug
═══════════════════════════════════════════════════════════════════
想让电机克服静摩擦转 Δq，需要  kp · Δq > fc。
我们有实测 fc：

    joint1 0.43   joint2 0.23   joint3 0.56
    joint4 0.10   joint5 ?      joint6 0.09   （N·m）

0.05° = 0.000873 rad，在 kp 上限 500 下最多给 0.44 N·m，
**只有 joint2/joint4/joint6 勉强能动，joint1/joint3 动不了。**

所以脚本会先算给你看，如果不动，用 --deg 加大：

    python3 scripts/direction_check.py --joint 1 --deg 2.0

2° 在 0.3 m 臂展上是 10 mm，肉眼可见，且绝对安全。

═══════════════════════════════════════════════════════════════════
安全
═══════════════════════════════════════════════════════════════════
    · 每次只使能**一个**电机，其余保持失能
    · kp 默认自动取「刚好能克服摩擦」的值（力矩 ≤ ~1 N·m）
    · 结束后一定失能（Ctrl-C 也会失能）
    · 位移量由 --deg 限定，ramp 上去不会有过冲

前置：can0 已配置为经典 CAN 1 Mbps 且 UP。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

# 复用 dm_probe 的 CAN 层（同一个包里的脚本）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dm_probe import Bus  # noqa: E402

# 实测静摩擦（N·m），来自真机测试
FRICTION = {1: 0.43, 2: 0.23, 3: 0.56, 4: 0.10, 5: 0.10, 6: 0.09}
# 电机型号（决定 pMax/vMax/tMax 的编码范围）
MOTOR_TYPE = {1: "10010L", 2: "6216P", 3: "4340",
              4: "4310P", 5: "4310", 6: "4310"}


# ──────────────────────────────────────────────────────────────
# /joint_states 发布器（让 RViz 动）
# ──────────────────────────────────────────────────────────────
class RvizDriver(threading.Thread):
    """后台线程：持续发布 /joint_states，joint{N} 可被实时改。"""

    def __init__(self, n_joints: int = 6, rate_hz: float = 50.0):
        super().__init__(daemon=True)
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = Node("direction_check_rviz")
        self.pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.msg = JointState()
        self.msg.name = [f"joint{i+1}" for i in range(n_joints)]
        self.msg.position = [0.0] * n_joints
        self.rate = rate_hz
        self._stop_event = threading.Event()
        self.lock = threading.Lock()

    def set_joint(self, idx: int, q: float):
        with self.lock:
            self.msg.position[idx] = q

    def run(self):
        dt = 1.0 / self.rate
        while not self._stop_event.is_set():
            with self.lock:
                self.msg.header.stamp = self.node.get_clock().now().to_msg()
                self.pub.publish(self.msg)
            time.sleep(dt)

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)
        self.node.destroy_node()


def ramp_rviz(rviz: RvizDriver, idx: int, q_from: float, q_to: float,
              seconds: float, verbose_label: str = ""):
    """把 RViz 里的某个关节从 q_from 平滑转到 q_to。"""
    n = max(1, int(seconds * 50))
    for k in range(n + 1):
        s = k / n
        # 平滑（smoothstep），避免看着突兀
        w = s * s * (3 - 2 * s)
        rviz.set_joint(idx, q_from + (q_to - q_from) * w)
        time.sleep(seconds / n)
    if verbose_label:
        print(f"    {verbose_label}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="单电机方向确认：RViz 转 +30°，真机转 +0.05°")
    ap.add_argument("--joint", type=int, required=True, choices=range(1, 7),
                    help="要测的关节号 1~6")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--deg", type=float, default=5.0,
                    help="真机要转多少度（默认 5°，肉眼可见的最小量）")
    ap.add_argument("--rviz-deg", type=float, default=30.0,
                    help="RViz 里转动角度（度），默认 30")
    ap.add_argument("--kp", type=float, default=None,
                    help="刚度，默认自动（刚好能克服摩擦）")
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--no-rviz", action="store_true", help="只动真机，不发布 joint_states")
    args = ap.parse_args()

    j = args.joint
    idx = j - 1
    deg = args.deg
    dq_target = math.radians(deg)
    fc = FRICTION[j]

    # ── kp 选择 ────────────────────────────────────────────────
    # 想克服摩擦：kp * Δq ≳ 1.2 * fc
    kp_need = 1.2 * fc / max(abs(dq_target), 1e-9)
    kp = args.kp if args.kp is not None else min(500.0, max(2.0, kp_need))
    kd = args.kd if args.kd is not None else max(0.1, kp * 0.02)
    tau_max = kp * abs(dq_target)

    print("=" * 66)
    print(f"  关节        : joint{j}（电机 ID {j}，{MOTOR_TYPE[j]}）")
    print(f"  实测静摩擦  : fc = {fc:.2f} N·m")
    print(f"  真机目标角  : +{deg:.3f}° = {dq_target:+.6f} rad")
    print(f"  所需 kp     : {kp_need:.0f}（1.2·fc/Δq）")
    print(f"  实际 kp/kd  : {kp:.1f} / {kd:.2f}")
    print(f"  最大力矩    : {tau_max:.3f} N·m")
    if tau_max < 1.5 * fc:
        print(f"  ⚠️  {tau_max:.3f} < 1.5·fc={1.5*fc:.2f} → **电机大概率不动**"
              f"（余量不足，实测 1.0·fc 时纹丝不动）")
        print(f"     建议：--deg {math.degrees(1.5*fc/max(kp,1)):.1f} 或更大，"
              f"例如 --deg 2")
    else:
        print(f"  ✅ {tau_max:.3f} > 1.5·fc={1.5*fc:.2f} → 应该能动")
    print("=" * 66)

    # ── RViz ──────────────────────────────────────────────────
    rviz = None
    if not args.no_rviz:
        try:
            rviz = RvizDriver()
            rviz.start()
            time.sleep(0.3)
            print(f"\n[阶段 1] RViz：joint{j} 从 0° 平滑转到 +{args.rviz_deg:.0f}°")
            print("         ⇒ 盯着 RViz 看这根 link 往哪边转，记住方向")
            ramp_rviz(rviz, idx, 0.0, math.radians(args.rviz_deg), 2.0)
            print(f"         已到 +{args.rviz_deg:.0f}°，保持 2 秒…")
            time.sleep(2.0)
        except Exception as e:  # noqa: BLE001
            print(f"[warn] RViz 发布器起不来（{e}），跳过阶段 1")
            rviz = None
    else:
        print("\n[阶段 1] 跳过（--no-rviz）")

    # ── 真机 ──────────────────────────────────────────────────
    # ⚠️ 实测结论：**位置模式靠 kp 顶不动**
    #   kp·Δq 只有在超过「静摩擦 + 重力」时才会动，而 0.05°/2° 时
    #   kp 上限给不出那么大扭矩。实测 joint1：位置模式 2° 完全不动，
    #   而纯力矩 +1.0 N·m × 0.5 s 直接转了 11°。
    #   所以这里改成**力矩脉冲 + 运动检测**：从小力矩开始逐级加大，
    #   一旦检测到 |Δq| 超过阈值立刻停，只动那么一点点。
    print(f"\n[阶段 2] 真机：只使能 joint{j}（电机 {j}），力矩脉冲探测")
    bus = Bus(args.interface, MOTOR_TYPE[j], [j], fd=False)

    bus.wait_first_state(0.6)
    q0 = bus.state[j]["q"]
    t_mos = bus.state[j]["t_mos"]
    print(f"         初始 q = {q0:+.6f} rad   温度 MOS={t_mos}℃")

    # 阈值：至少要超过编码器 3 个 LSB（16 bit / pMax=12.5 → LSB≈0.0004 rad）
    target = math.radians(args.deg)          # 目标：真的转这么多
    min_move = 3 * 0.0004                    # 至少 3 个编码器 LSB 才算"动了"
    print(f"         目标转角 {args.deg:.2f}° = {target:.5f} rad")

    TAU_STEPS = [0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0]   # N·m，逐级加大
    dq = 0.0
    used_tau = 0.0
    moved = False            # 是否检测到运动
    detected_dq = 0.0        # 检测到运动那一刻的 Δq（失能后会回弹，用它判定）
    try:
        bus.enable()
        time.sleep(0.15)
        for tau in TAU_STEPS:
            used_tau = tau
            t_step = time.monotonic()
            moved = False
            while time.monotonic() - t_step < 0.8:
                bus.mit({j: (0.0, 0.3, 0.0, 0.0, tau)})   # kp=0 → 纯力矩
                bus.poll(0.001)
                if bus.state[j].get("seen"):
                    dq = bus.state[j]["q"] - q0
                    if abs(dq) >= target:      # 转到目标就停
                        moved = True
                        detected_dq = dq
                        break
                time.sleep(0.001)
            if moved:
                print(f"         {tau:.1f} N·m → 动了（Δq={dq:+.5f} rad）")
                break
            print(f"         {tau:.1f} N·m → 还没动，加大")
    finally:
        # 先停力矩，再失能
        for _ in range(5):
            bus.mit({j: (0.0, 0.5, 0.0, 0.0, 0.0)})
            time.sleep(0.002)
        bus.disable()
        time.sleep(0.2)

    bus.state[j]["seen"] = False
    bus.wait_first_state(0.6)
    q1 = bus.state[j]["q"]
    dq = q1 - q0

    print(f"         最终 q = {q1:+.6f} rad")
    print(f"         Δq     = {dq:+.6f} rad = {math.degrees(dq):+.4f}°"
          f"   （最大用到 {used_tau:.1f} N·m）")

    # ── 判断 ──────────────────────────────────────────────────
    judge = detected_dq if moved else dq
    print("\n" + "=" * 66)
    if abs(judge) < min_move:
        print(f"  ⚠️ |Δq| < {min_move:.5f} rad —— 力矩加到 {used_tau:.1f} N·m 仍没动")
        print(f"     检查：机械卡死 / 电机未使能 / 关节在硬限位")
    else:
        sign = +1 if judge > 0 else -1
        print(f"  检测到运动时的 Δq = {detected_dq:+.6f} rad "
              f"（{math.degrees(detected_dq):+.3f}°）")
        print(f"  失能后回落至 Δq = {dq:+.6f} rad（回弹正常，力矩撤掉后应力释放）")
        print(f"  ⇒ 电机 {'+' if sign>0 else '−'}q 方向 …")
        print()
        print(f"  ⚠️ 本测试**只能证明电机能动、能动多少**，")
        print(f"     不能判定 direction 的正负 —— 因为"
              f"「+tau → +q」是电机内部的同义反复，")
        print(f"     跟它怎么装在机械臂上无关。")
        print()
        print(f"     判 direction 请用 mirror_check.py（实时镜像）：")
        print(f"         python3 scripts/mirror_check.py")
    print("=" * 66)

    if rviz is not None:
        # 收尾：回零
        ramp_rviz(rviz, idx, math.radians(args.rviz_deg), 0.0, 0.8)
        rviz.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
