#!/usr/bin/env python3
"""C-05 静态重力辨识 —— 数据采集

════════════════════════════════════════════════════════════════════
原理
════════════════════════════════════════════════════════════════════
让机械臂在若干姿态下**静止**，测各关节力矩 τ。
零速零加速度时 τ = g(q) + 摩擦，而 g(q) = Y_g(q)·θ 对 θ 线性 ⇒ 最小二乘解 θ。

⚠️ 摩擦是主要干扰：稳态静摩擦的方向取决于"从哪个方向接近这个位形"。
   ⇒ 每个位形**从两个方向各采一次**，取平均消掉摩擦；
      差值 (τ_上 − τ_下)/2 顺便给出库仑摩擦估计。

════════════════════════════════════════════════════════════════════
安全设计（这个脚本会真的动机械臂）
════════════════════════════════════════════════════════════════════
① **绝不直接跳变位置命令**
   ForwardCommandController 是直接下发位置，kp=70，跳跃会造成力矩尖峰。
   本脚本用**余弦缓动**在 --move-time 秒内插值到目标（默认 2.0 s）。

② **每个位形先做力矩预判**
   用标称模型 + 夹爪质量估出 |τ_j2|，超过 --max-j2-torque（默认 8 N·m，
   即连续额定 10 的 80%）就跳过该位形。
   实测：夹爪 0.3185 kg 时 joint2 峰值 10.4 N·m（104% 连续额定）。

③ **运行中监控**
   每次采样前检查实测 |τ_j2|；超过 --abort-j2-torque（默认 9.5 N·m）立即
   停止并保持当前位置（不继续运动）。

④ **静止判据**
   |dq| < --vel-tol 连续 --settle-count 个周期才算静止（默认 0.01 rad/s × 10）。
   不静止就不采 —— 运动中的 τ 含惯量项，不是纯重力。

════════════════════════════════════════════════════════════════════
用法
════════════════════════════════════════════════════════════════════
    # ① 先规划位形（不动机械臂，只打印）
    python3 collect_static_gravity.py --dry-run

    # ② 真机采集
    python3 collect_static_gravity.py --out ~/hw_arm/algo/data/static_gravity.yaml

    # ③ 拟合
    gravity_id_cli --urdf <urdf> --samples <yaml> --out <theta.yaml>

════════════════════════════════════════════════════════════════════
前置
════════════════════════════════════════════════════════════════════
  · 用 forward_position_controller 启动（不要用 JTC，它的容差会因下垂而 abort）：
      ros2 launch hw_arm_bringup hw_arm.launch.py use_fake_hardware:=false \
          controllers:=joint_state_broadcaster,hw_arm_forward_position_controller
  · **重力补偿必须关闭**（gravity_compensation:=false 是默认值），
    否则 tau_ff 会把重力抵消掉，测出来的是残差而不是重力。
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
    print("❌ 需要 rclpy", file=sys.stderr)
    raise

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# 额定（用于安全判断）：min(t_peak, 协议 TMAX)。见 docs/TODO.md 的电机表
T_RATED = np.array([40.0, 10.0, 12.0, 3.5, 3.0, 3.0])
T_PEAK = np.array([120.0, 45.0, 28.0, 10.0, 7.0, 7.0])


def _halton(i: int, base: int) -> float:
    """Halton 低差异序列：让 (j2, j3) 在矩形里分布均匀，而不是挤在几行上"""
    f, r = 1.0, 0.0
    while i > 0:
        f /= base
        r += f * (i % base)
        i //= base
    return r


def order_by_nearest(poses: list[np.ndarray], start: np.ndarray) -> list[np.ndarray]:
    """最近邻排序：让相邻两步的关节空间距离最小

    为什么重要：2026-11-25 真机试采时，一次把 joint2 从 0 抬到 0.5 rad，
    6 台电机同时出力，电源欠压（电机报 0x9）→ 失能 → 机械臂掉下来。
    排序后每步只动几度，持续电流和峰值都小很多。
    """
    left = list(poses)
    out = []
    cur = start
    while left:
        d = [np.linalg.norm(p - cur) for p in left]
        i = int(np.argmin(d))
        out.append(left.pop(i))
        cur = out[-1]
    return out


def plan_poses(n_main: int, seed: int) -> list[np.ndarray]:
    """规划采集位形

    覆盖策略：
      · joint2 ∈ [15°, 130°]、joint3 ∈ [10°, 135°]
        避开 joint2>150° 的高力矩区（实测夹爪 0.3185 kg 时 joint2 在 150°/150°
        附近达 10.4 N·m = 104% 连续额定）
      · (j2, j3) 用 **Halton 低差异序列**铺开 —— 早先按网格二重循环取前 N 个，
        结果 24 个位形只覆盖了 3 个 joint2 值，回归矩阵条件数很差
      · joint4/5/6 轮换几个离散组合 + 随机扰动，让各轴都动起来
    """
    rng = np.random.default_rng(seed)
    others = [
        (0.0, 0.0, 0.0),
        (0.0, 0.6, 0.0),
        (0.4, -0.5, 0.8),
        (-0.4, 0.9, -0.8),
        (0.25, 0.3, -0.5),
        (-0.25, -0.3, 0.5),
    ]
    j2_lo, j2_hi = math.radians(15), math.radians(130)
    j3_lo, j3_hi = math.radians(10), math.radians(135)

    poses = []
    for i in range(1, n_main + 1):
        u2 = _halton(i, 2)
        u3 = _halton(i, 3)
        q = np.zeros(6)
        q[0] = rng.uniform(-0.35, 0.35)                       # joint1 也动一动
        q[1] = j2_lo + (j2_hi - j2_lo) * u2
        q[2] = j3_lo + (j3_hi - j3_lo) * u3
        o = others[(i - 1) % len(others)]
        q[3] = o[0] + rng.uniform(-0.2, 0.2)
        q[4] = o[1] + rng.uniform(-0.25, 0.25)
        q[5] = o[2] + rng.uniform(-0.4, 0.4)
        poses.append(q)
    return poses


def nominal_gravity(q: np.ndarray, urdf: str,
                    gripper_mass: float = 0.3185,
                    gripper_com=(0.00812, -0.00173, 0.04161)) -> np.ndarray | None:
    """用 Pinocchio + 标称 URDF 估重力矩，**含夹爪质点**，用于安全预判

    ⚠️ 不含夹爪会把 joint2 低估：实测 0.3185 kg @ 质心 41.6 mm 让 joint2
       峰值从 8.2 涨到 10.4 N·m（104% 连续额定）。
    """
    try:
        import pinocchio as pin
    except ImportError:
        return None
    key = (urdf, round(gripper_mass, 6))
    if not hasattr(nominal_gravity, "_cache"):
        nominal_gravity._cache = {}
    if key not in nominal_gravity._cache:
        m = pin.buildModelFromUrdf(urdf)
        if gripper_mass > 0:
            m.appendBodyToJoint(
                m.getJointId("joint6"),
                pin.Inertia(gripper_mass, np.array(gripper_com), np.zeros((3, 3))),
                pin.SE3.Identity())
        nominal_gravity._cache[key] = (m, m.createData())
    m, d = nominal_gravity._cache[key]
    return np.asarray(pin.computeGeneralizedGravity(m, d, q)).copy()


class Collector(Node):
    def __init__(self, args):
        super().__init__("collect_static_gravity")
        self.args = args
        self.q = None
        self.dq = None
        self.tau = None
        self.last_stamp = None
        self.n_msg = 0

        self.sub = self.create_subscription(JointState, "/joint_states", self.on_state, 50)
        self.pub = self.create_publisher(
            Float64MultiArray, "/hw_arm_forward_position_controller/commands", 10)

    def on_state(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.q = np.array([msg.position[idx[j]] for j in JOINTS])
        self.dq = (np.array([msg.velocity[idx[j]] for j in JOINTS])
                   if len(msg.velocity) == len(msg.name) else np.zeros(6))
        self.tau = (np.array([msg.effort[idx[j]] for j in JOINTS])
                    if len(msg.effort) == len(msg.name) else np.zeros(6))
        self.last_stamp = time.monotonic()
        self.n_msg += 1

    def refresh(self, seconds: float = 0.3):
        """主动 spin 一小段，刷新缓存（交互阻塞后必须调用）"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_state(self, timeout=5.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.q is not None and (time.monotonic() - self.last_stamp) < 0.3:
                return True
        return False

    def send(self, q_cmd: np.ndarray):
        m = Float64MultiArray()
        m.data = [float(x) for x in q_cmd]
        self.pub.publish(m)

    def move_to(self, q_goal: np.ndarray, duration: float) -> bool:
        """余弦缓动，绝不跳变"""
        if self.q is None:
            return False
        q0 = self.q.copy()
        rate = 50.0
        n = max(1, int(duration * rate))
        for k in range(1, n + 1):
            a = k / n
            s = 0.5 * (1.0 - math.cos(math.pi * a))     # 0→1，端点速度为 0
            self.send(q0 + (q_goal - q0) * s)
            rclpy.spin_once(self, timeout_sec=1.0 / rate)
            # 运行中力矩监控
            if self.tau is not None and abs(self.tau[1]) > self.args.abort_j2_torque:
                self.get_logger().error(
                    f"❌ joint2 实测力矩 {self.tau[1]:.2f} N·m 超过中止阈值 "
                    f"{self.args.abort_j2_torque} —— 停止运动并保持")
                return False
        return True

    def wait_settle(self, vel_tol: float, need: int, timeout: float) -> tuple[bool, np.ndarray]:
        """等静止：|dq| 连续 need 个周期低于阈值"""
        t0 = time.monotonic()
        ok_cnt = 0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.dq is None:
                continue
            if np.max(np.abs(self.dq)) < vel_tol:
                ok_cnt += 1
                if ok_cnt >= need:
                    # 再稳一小会儿，让静摩擦进入粘滞态
                    t1 = time.monotonic()
                    while time.monotonic() - t1 < 0.25:
                        rclpy.spin_once(self, timeout_sec=0.02)
                    return True, self.q.copy()
            else:
                ok_cnt = 0
        return False, self.q.copy() if self.q is not None else np.zeros(6)


def ask(prompt: str, allow: str = "s q") -> str:
    """读一行输入；空 = 继续。返回 'go' / 'skip' / 'quit'"""
    while True:
        try:
            ans = input(prompt).strip().lower()
        except EOFError:
            return "quit"
        if ans == "":
            return "go"
        if ans and ans[0] in allow:
            return {"s": "skip", "q": "quit"}.get(ans[0], "go")
        print(f"  请输入 Enter（继续）/{allow.replace(' ', '（跳过）')}（退出）")


def main() -> int:
    ap = argparse.ArgumentParser(description="C-05 静态重力辨识采集")
    ap.add_argument("--urdf", default=None,
                    help="标称 URDF（用于力矩预判；默认自动找 hw_arm_description）")
    ap.add_argument("--out", default="static_gravity.yaml")
    ap.add_argument("--poses", type=int, default=24, help="位形数（默认 24）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--move-time", type=float, default=2.0, help="每段运动时间 s")
    ap.add_argument("--settle-time", type=float, default=4.0, help="等待静止上限 s")
    ap.add_argument("--vel-tol", type=float, default=0.01, help="静止速度阈值 rad/s")
    ap.add_argument("--settle-count", type=int, default=10)
    ap.add_argument("--max-j2-torque", type=float, default=8.0,
                    help="joint2 预判超过它的位形直接跳过（N·m）")
    ap.add_argument("--max-torque-frac", type=float, default=0.6,
                    help="其余轴的上限 = 连续额定 × 该比例（默认 0.6）")
    ap.add_argument("--abort-j2-torque", type=float, default=9.5,
                    help="实测超过它立即中止（N·m）")
    ap.add_argument("--gripper-mass", type=float, default=0.3185,
                    help="夹爪质量 kg（实测 318.5 g）；0 = 按裸臂预判")
    ap.add_argument("--no-bidirectional", action="store_true",
                    help="不做双向采集（不推荐：摩擦无法抵消）")
    ap.add_argument("--dry-run", action="store_true", help="只规划并打印，不动机械臂")
    ap.add_argument("--keep", type=int, default=0,
                    help="过滤后最多保留多少个位形（0=全保留）")
    ap.add_argument("--yes", action="store_true",
                    help="自动模式（不逐步确认）。默认**每步都要按 Enter**，"
                         "方便先托住机械臂")
    ap.add_argument("--confirm-each", choices=["move", "pose"], default="move",
                    help="确认粒度：move=每次运动前确认（最安全）；pose=每个位形确认一次")
    args = ap.parse_args()

    poses = plan_poses(args.poses, args.seed)

    # ── 力矩预判 ──
    urdf = args.urdf
    if urdf is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            urdf = os.path.join(get_package_share_directory("hw_arm_description"),
                                "urdf", "hw_arm.urdf")
        except Exception:
            urdf = None

    # 各轴力矩上限：joint2 用 --max-j2-torque（默认 6），
    # 其余轴用连续额定的一定比例。
    # ⚠️ 只看 joint2 不够：有的位形 joint3 才是最重的
    #    （实测某位形 joint2 只有 −0.26，joint3 却到 4.05）。
    FRAC = args.max_torque_frac
    t_lim = np.array([T_RATED[0] * FRAC, args.max_j2_torque, T_RATED[2] * FRAC,
                      T_RATED[3] * FRAC, T_RATED[4] * FRAC, T_RATED[5] * FRAC])

    def check_tau(g):
        """返回 (ok, 超限的轴号 或 -1, 该轴预判值)"""
        if g is None:
            return True, -1, float("nan")
        over = np.where(np.abs(g) > t_lim)[0]
        if over.size == 0:
            return True, -1, abs(g[1])
        j = int(over[np.argmax(np.abs(g[over]) / t_lim[over])])
        return False, j, abs(g[j])

    safe, skipped = [], []
    rows = []
    for i, q in enumerate(poses):
        g = nominal_gravity(q, urdf, args.gripper_mass) if urdf else None
        ok, over_j, val = check_tau(g)
        rows.append((q, abs(g[1]) if g is not None else float("nan"), ok, over_j, val))
        (safe if ok else skipped).append(q)

    # 先截取，再打印 —— 否则表格里会出现"✅ 但其实被 --keep 丢掉的"位形
    if args.keep > 0 and len(safe) > args.keep:
        keep_ids = {id(q) for q in safe[:args.keep]}
        print(f"\n--keep {args.keep}：从 {len(safe)} 个合格位形里取前 {args.keep} 个")
        safe = safe[:args.keep]
    else:
        keep_ids = {id(q) for q in safe}

    print(f"\n{'#':>3}  {'j2':>7} {'j3':>7} {'j4':>7} {'j5':>7} {'j6':>7}   "
          f"{'τ_j2 预判':>10}  结论")
    for i, (q, j2_tau, ok, over_j, val) in enumerate(rows):
        if not ok:
            mark = "⏭️ 跳过（超力矩）"
        elif id(q) in keep_ids:
            mark = "✅ 采用"
        else:
            mark = "➖ 合格但未选（--keep）"
        extra = "" if ok else f"（joint{over_j+1} 预判 {val:.2f} > 限 {t_lim[over_j]:.1f}）"
        print(f"{i:3d}  {math.degrees(q[1]):7.1f} {math.degrees(q[2]):7.1f} "
              f"{math.degrees(q[3]):7.1f} {math.degrees(q[4]):7.1f} {math.degrees(q[5]):7.1f}   "
              f"{j2_tau:10.2f}  {mark}{extra}")

    print(f"\n规划 {len(poses)} 个候选：合格 {len(rows)-len(skipped)} 个，"
          f"超力矩 {len(skipped)} 个，**最终采用 {len(safe)} 个**")
    if not safe:
        print("❌ 没有可用位形")
        return 1
    if args.dry_run:
        print("\n（--dry-run：不接触机械臂，退出）")
        return 0

    # ── 真机采集（默认**逐步人工确认**）──
    interactive = not args.yes

    rclpy.init()
    node = Collector(args)
    try:
        if not node.wait_state(10.0):
            print("❌ 收不到 /joint_states —— 请确认 bringup 已启动")
            return 1

        q_now = np.array(node.q)
        print(f"\n起始位形 q = {np.round(q_now, 4)}")
        print(f"起始实测 τ  = {np.round(node.tau, 3)}")

        # 按最近邻排序，减少每步的运动量
        safe = order_by_nearest(safe, q_now)
        print(f"\n已按最近邻排序（每步运动量最小）。共 {len(safe)} 个位形。")

        if interactive:
            print("""
┌──────────────────────────────────────────────────────────────┐
│ 交互模式：每一步都会停下来等你按 Enter                        │
│   Enter = 继续          s = 跳过这一步        q = 退出采集     │
│ ⚠️ 按 Enter 前请先托住机械臂 —— 发生过欠压失能导致掉落          │
└──────────────────────────────────────────────────────────────┘""")

        samples = []
        aborted = False
        for i, q_goal in enumerate(safe):
            print(f"\n══ 位形 {i+1}/{len(safe)} ══  "
                  f"j2={math.degrees(q_goal[1]):6.1f}°  j3={math.degrees(q_goal[2]):6.1f}°  "
                  f"j4={math.degrees(q_goal[3]):6.1f}°  j5={math.degrees(q_goal[4]):6.1f}°  "
                  f"j6={math.degrees(q_goal[5]):6.1f}°")
            # 力矩预判再显示一次（用于判断要不要托）
            g_pre = nominal_gravity(q_goal, urdf, args.gripper_mass)
            if g_pre is not None:
                print(f"   预判 τ = {np.round(g_pre, 2)}   （|τ_j2|={abs(g_pre[1]):.2f} N·m）")

            if interactive:
                act = ask("  ▶ 托好机械臂后按 Enter 开始运动（s 跳过 / q 退出）: ")
                if act == "quit":
                    print("  已退出"); aborted = True; break
                if act == "skip":
                    print("  跳过"); continue

            # ── 运动前检查：控制器是否还活着 ──
            #
            # ⚠️ 必须先 spin 几次再判新鲜度。
            #    交互模式下 ask() 阻塞在 input()，期间没有任何 spin_once，
            #    last_stamp 不会更新 —— 用户只要考虑超过 1 秒，
            #    这里就会误报"关节状态中断"。2026-11-25 真机踩过一次。
            node.refresh(0.5)
            if node.q is None or node.last_stamp is None or \
                    (time.monotonic() - node.last_stamp) > 1.0:
                print("  ❌ 关节状态中断（控制器可能被 deactivate）—— 停止采集")
                print(f"     最后收到状态: "
                      f"{(time.monotonic()-node.last_stamp):.1f} s 前" if node.last_stamp else "     从未收到")
                aborted = True; break

            if not node.move_to(q_goal, args.move_time):
                print("  ❌ 运动被中止（力矩超限）")
                aborted = True; break

            # ── 掉电检测：运动后位置应接近目标 ──
            time.sleep(0.4)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.q is not None:
                dev = np.max(np.abs(np.array(node.q) - q_goal))
                if dev > 0.6:
                    print(f"  ❌ 实际位置离目标差 {dev:.2f} rad —— **疑似掉电/失能**，立即停止")
                    print(f"     实际 q = {np.round(node.q, 3)}")
                    aborted = True; break
                if np.all(np.abs(np.array(node.tau)) < 1e-6):
                    print("  ❌ 力矩突然全为 0 —— **疑似掉电**，立即停止")
                    aborted = True; break

            ok, qa = node.wait_settle(args.vel_tol, args.settle_count, args.settle_time)
            if not ok:
                print("  ⚠️ 没等到静止 —— 可能是支撑不足或被卡住")
                if interactive:
                    act = ask("  仍要记录吗？（Enter 记录 / s 跳过 / q 退出）: ")
                    if act == "quit": aborted = True; break
                    if act == "skip": continue
                else:
                    continue

            if abs(node.tau[1]) > args.abort_j2_torque:
                print(f"  ❌ 实测 joint2 力矩 {node.tau[1]:.2f} 超限，停止收集")
                aborted = True; break

            samples.append({"q": list(qa), "tau": node.tau.copy().tolist(), "tag": "A"})
            print(f"  A: τ = {np.round(node.tau, 3)}")

            if interactive:
                act = ask("  ▶ Enter 记录 A 并继续；s 丢弃这一条：")
                if act == "quit": aborted = False; break     # 记录完A直接收工
                if act == "skip": samples.pop()

            if args.no_bidirectional:
                continue

            # ── B：先偏一点再回来，让静摩擦方向相反 ──
            off = q_goal.copy()
            off[1] -= math.radians(4.0)     # 幅度从 6° 降到 4°，减小电流
            off[2] += math.radians(3.0)
            if interactive and args.confirm_each == "move":
                act = ask("  ▶ Enter 执行反向接近（s 跳过 B / q 退出）: ")
                if act == "quit": aborted = False; break
                if act == "skip": continue
            if not node.move_to(off, args.move_time * 0.6):
                aborted = True; break
            node.wait_settle(args.vel_tol, args.settle_count, args.settle_time)
            if not node.move_to(q_goal, args.move_time * 0.6):
                aborted = True; break
            ok, qb = node.wait_settle(args.vel_tol, args.settle_count, args.settle_time)
            if not ok:
                print("  ⚠️ B 方向没等到静止，跳过")
                continue
            if abs(node.tau[1]) > args.abort_j2_torque:
                print(f"  ❌ 实测 joint2 力矩 {node.tau[1]:.2f} 超限，停止收集")
                aborted = True; break
            samples.append({"q": list(qb), "tau": node.tau.copy().tolist(), "tag": "B"})
            print(f"  B: τ = {np.round(node.tau, 3)}")
            fr = (np.array(samples[-2]["tau"]) - np.array(samples[-1]["tau"])) / 2.0
            print(f"  库仑摩擦估计 = {np.round(fr, 3)}")

        # ── 写文件 ──
        if not samples:
            print("\n❌ 没采到任何样本")
            return 1
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        try:
            import yaml
            with open(args.out, "w") as f:
                yaml.safe_dump({"samples": samples,
                                "meta": {"urdf": urdf, "n": len(samples),
                                         "aborted": aborted,
                                         "bidirectional": not args.no_bidirectional}},
                               f, allow_unicode=True, sort_keys=False)
        except ImportError:
            import json
            with open(args.out, "w") as f:
                json.dump({"samples": samples}, f)
        tag = "（中途停止）" if aborted else ""
        print(f"\n✅ 采集 {len(samples)} 个样本 → {args.out} {tag}")
        print(f"   下一步：gravity_id_cli --urdf {urdf} --samples {args.out}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
