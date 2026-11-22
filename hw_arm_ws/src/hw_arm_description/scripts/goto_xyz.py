#!/usr/bin/env python3
"""笛卡尔直线运动：终端给末端位姿 → 机械臂沿**直线**走过去

═══════════════════════════════════════════════════════════════════
它做什么
═══════════════════════════════════════════════════════════════════
    ① 读 /joint_states 得到当前关节角 q_now
    ② 用 plan_line_cli（hw_arm_algorithms 里的 RMRC 实现）算出
       "q_now → 目标位姿" 的**笛卡尔直线**轨迹（末端走直线，姿态走定轴旋转）
    ③ 把路点打包成 FollowJointTrajectory 发给 JTC
    ④ JTC 内部 250 Hz 插值 → 电机位置环

⚠️ 和"关节空间点动"（jog.py）的区别：
    jog.py      末端走曲线（关节空间插值）
    goto_xyz.py 末端走**直线**（笛卡尔空间插值）

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    # ① 最常用：只给位置，姿态保持不变
    python3 goto_xyz.py 0.15 0.05 0.30

    # ② 相对当前位置的增量（+10 mm x）
    python3 goto_xyz.py --rel 0.010 0 0

    # ③ 连姿态一起给（roll pitch yaw，单位度）
    python3 goto_xyz.py 0.15 0.05 0.30 --rpy 0 90 0

    # ④ 走慢一点 / 快一点
    python3 goto_xyz.py 0.15 0.05 0.30 --duration 5

    # ⑤ 只看规划结果，不发命令
    python3 goto_xyz.py 0.15 0.05 0.30 --dry-run

═══════════════════════════════════════════════════════════════════
坐标约定
═══════════════════════════════════════════════════════════════════
    · 位置 (x, y, z)：**世界系**，单位 m
    · 姿态 (rpy)：世界系下的固定轴 roll-pitch-yaw（度），对应 R = Rz(yaw)·Ry(pitch)·Rx(roll)
    · 末端帧：默认 tool0（可用 --frame 改）
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys

import numpy as np

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    """固定轴 RPY → 四元数 (x, y, z, w)，R = Rz(yaw)·Ry(pitch)·Rx(roll)"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def find_cli() -> str:
    """找到 plan_line_cli（优先 install，其次 build）"""
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.abspath(os.path.join(here, "..", "..", ".."))
    cands = [
        os.path.join(ws, "build", "hw_arm_algorithms", "plan_line_cli"),
        os.path.join(ws, "install", "hw_arm_algorithms", "lib",
                     "hw_arm_algorithms", "plan_line_cli"),
    ]
    p = shutil.which("plan_line_cli")
    if p:
        return p
    for c in cands:
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    raise FileNotFoundError(
        "找不到 plan_line_cli。先编译：\n"
        "    colcon build --symlink-install --packages-select hw_arm_algorithms")


def load_limits() -> tuple[str, str]:
    """从 motor_specs.yaml 读每轴速度/加速度上限，返回 (vmax_csv, amax_csv)

    vmax = min(RID22 协议量程, 24V 空载转速×2π/60)
    amax = (t_rated − |g|max) / M_jj   ← 已算好写在文件里，这里只算 vmax
    """
    import yaml
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.abspath(os.path.join(here, "..", "..", ".."))
    p = os.path.join(ws, "src", "hw_arm_description", "config", "control",
                     "motor_specs.yaml")
    if not os.path.isfile(p):
        p = os.path.join(ws, "install", "hw_arm_description", "share",
                         "hw_arm_description", "config", "control", "motor_specs.yaml")
    if not os.path.isfile(p):
        raise FileNotFoundError("找不到 motor_specs.yaml")
    d = yaml.safe_load(open(p))["motors"]
    vmax, amax = [], []
    for i in range(1, 7):
        m = d[f"joint{i}"]
        phys = m["no_load_speed_rpm"] * 2.0 * math.pi / 60.0
        vmax.append(min(phys, m["v_max"]))
        # 加速度上限：文件里没存就直接用"额定扭矩 − 重力"的安全估计
        amax.append(m.get("a_max_rated", AMAX_DEFAULT[i - 1]))
    return (",".join(f"{v:.4f}" for v in vmax),
            ",".join(f"{v:.2f}" for v in amax))


# 兜底加速度上限（来自说明书额定扭矩 + 实读转子惯量算出的值）
AMAX_DEFAULT = [84.8, 4.0, 80.8, 785.0, 852.0, 1700.0]


def find_urdf() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.abspath(os.path.join(here, "..", "..", ".."))
    p = os.path.join(ws, "install", "hw_arm_description", "share",
                     "hw_arm_description", "urdf", "hw_arm.urdf")
    if os.path.isfile(p):
        return p
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("hw_arm_description"),
                            "urdf", "hw_arm.urdf")
    except Exception:
        raise FileNotFoundError(f"找不到 hw_arm.urdf（试过 {p}）")


def fresh_state(node, state, joints, timeout=1.0, need=2):
    """等一份**新鲜的**六轴状态：要求连续 need 帧都齐备，并用 header.stamp 判年龄"""
    import time as _t
    t0 = _t.time()
    seen = 0
    while _t.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        st = state.get("stamp")
        if state.get("q") is None or st is None:
            continue
        age = (_t.time() - st)
        if age > 0.5:          # 状态超过 0.5 s 没更新 → 不算新鲜
            continue
        seen += 1
        if seen >= need:
            return list(state["q"])
    return None


def parse_cli_output(text: str):
    """解析 plan_line_cli 的 YAML 输出"""
    meta = {}
    waypoints = []
    cur = None
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("ok:"):
            meta["ok"] = s.split(":", 1)[1].strip() == "true"
        elif ":" in s and not s.startswith("-") and not s.startswith("q") \
                and not s.startswith("qd"):
            k, v = s.split(":", 1)
            meta[k.strip()] = v.strip()
        elif s.startswith("- t:"):
            if cur is not None:
                waypoints.append(cur)
            cur = {"t": float(s.split(":", 1)[1])}
        elif s.startswith("q:") and cur is not None:
            cur["q"] = [float(x) for x in s.split("[", 1)[1].rstrip("]").split(",")]
        elif s.startswith("qd:") and cur is not None:
            cur["qd"] = [float(x) for x in s.split("[", 1)[1].rstrip("]").split(",")]
        elif s.startswith("q:") and "[" in s:
            # ik_cli 的顶层字段：q: [..]
            meta["q"] = [float(x) for x in s.split("[", 1)[1].rstrip("]").split(",")]
    if cur is not None:
        waypoints.append(cur)
    return meta, waypoints


def main() -> int:
    ap = argparse.ArgumentParser(description="笛卡尔直线运动到目标位姿")
    ap.add_argument("xyz", nargs=3, type=float,
                    help="目标位置 X Y Z (m)；--rel 时是增量")
    ap.add_argument("--rel", action="store_true", help="xyz 按当前位置的增量解释")
    ap.add_argument("--rpy", nargs=3, type=float, default=None,
                    help="目标姿态 ROLL PITCH YAW（度，固定轴）；不给则保持当前姿态")
    ap.add_argument("--frame", default="tool0")
    ap.add_argument("--duration", type=float, default=3.0, help="轨迹时长 (s)")
    ap.add_argument("--sample", type=float, default=0.05,
                    help="路点间隔 (s)，默认 0.05")
    ap.add_argument("--duration-per-meter", type=float, default=None,
                    help="按距离给时长（秒/米），给了就覆盖 --duration")
    ap.add_argument("--engine", choices=["ik", "rmrc"], default="ik",
                    help="ik=点到点（IK 一次，JTC 关节插值，默认）；"
                         "rmrc=笛卡尔直线（走 plan_line_cli）")
    ap.add_argument("--tol", type=float, default=1e-6,
                    help="IK 收敛阈值 (m/rad)，默认 1e-6")
    ap.add_argument("--max-joint-jump", type=float, default=0.5,
                    help="单轴关节跳变上限 (rad)，超过判为解跳分支")
    ap.add_argument("--max-joint-norm", type=float, default=1.0,
                    help="关节行程 2-范数上限 (rad)")
    ap.add_argument("--start-tolerance", type=float, default=0.02,
                    help="确认后起点允许的漂移 (rad)，默认 0.02（约 1.1°）")
    ap.add_argument("--dry-run", action="store_true", help="只规划，不发命令")
    ap.add_argument("--yes", "-y", action="store_true", help="跳过确认")
    args = ap.parse_args()

    try:
        cli = find_cli()
        urdf = find_urdf()
    except FileNotFoundError as e:
        print(f"❌ {e}")
        return 2

    # ── 读当前关节角 ──
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
    except ImportError:
        print("❌ 需要 ROS：source /opt/ros/jazzy/setup.bash")
        return 2

    import time

    rclpy.init()
    node = Node("goto_xyz")
    state = {"q": None, "stamp": None}

    def cb(msg: JointState):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            state["q"] = [d[j] for j in JOINTS]
            state["stamp"] = time.time()          # ★ R3：记录到达时刻

    node.create_subscription(JointState, "/joint_states", cb, 10)
    t0 = time.time()
    while state["q"] is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["q"] is None:
        print("❌ 5 秒内读不到 /joint_states（机械臂没启动？）")
        return 2
    q_now = list(state["q"])

    # ── 用 FK 求当前位姿（pinocchio）──
    try:
        import pinocchio as pin
    except ImportError:
        print("❌ 需要 pinocchio（python3 -c 'import pinocchio'）")
        return 2
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    fid = model.getFrameId(args.frame)
    if fid >= len(model.frames):
        print(f"❌ 找不到帧 {args.frame}")
        return 2
    pin.forwardKinematics(model, data, np.array(q_now))
    pin.updateFramePlacements(model, data)
    T_now = data.oMf[fid]
    p_now = T_now.translation.copy()

    # ── 目标位姿 ──
    if args.rel:
        p_target = p_now + np.array(args.xyz)
    else:
        p_target = np.array(args.xyz)
    if args.rpy is not None:
        r, pi, y = [math.radians(a) for a in args.rpy]
        quat = rpy_to_quat(r, pi, y)                      # (x, y, z, w)
        # Eigen 的 Quaternion 构造顺序是 (w, x, y, z)
        R_target = pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).toRotationMatrix()
    else:
        # 保持当前姿态：从旋转矩阵反解四元数
        R_target = np.array(T_now.rotation, dtype=float)
        qq = pin.Quaternion(R_target)
        quat = (qq.x, qq.y, qq.z, qq.w)

    dist = float(np.linalg.norm(p_target - p_now))
    duration = args.duration
    if args.duration_per_meter is not None:
        duration = max(1.0, dist * args.duration_per_meter)

    print(f"  当前末端 ({args.frame}) = {np.round(p_now, 4)} m")
    print(f"  目标位置            = {np.round(p_target, 4)} m   距离 {dist*1000:.1f} mm")
    print(f"  轨迹时长            = {duration:.2f} s   "
          f"平均速度 {dist/max(duration,1e-3)*1000:.1f} mm/s")

    if dist < 1e-4 and args.rpy is None:
        print("  ⚠️ 目标与当前位置几乎重合，无需运动")
        return 0

    # ── 生成关节路点 ──
    wps = []
    if args.engine == "ik":
        # ① 点到点：只调一次 IK（复用 kinematics.cpp 的 solveIK），
        #    然后发两点轨迹，段内由 JTC 做关节空间插值。
        ik_cli = find_cli().replace("plan_line_cli", "ik_cli")
        if not os.path.isfile(ik_cli):
            print(f"❌ 找不到 ik_cli：{ik_cli}\n"
                  f"   先编译：colcon build --symlink-install --packages-select hw_arm_algorithms")
            return 2
        # ★ R2：duration 必须与真正下发的轨迹一致；
        #        速度/加速度上限必须真的传下去（否则 ik_selection 里那两条判据
        #        因为向量为空而**静默跳过**）。
        try:
            vmax_csv, amax_csv = load_limits()
        except Exception as e:
            print(f"❌ 读不到电机上限：{e}")
            return 2

        # 用五次多项式的峰值系数反推最短时长（1.875·Δq/T ≤ v, 5.77·Δq/T² ≤ a）
        # 注意：IK 解还没算出来，这里先按"关节跳变上限"粗估
        dq_est = np.full(6, args.max_joint_jump)
        vmax_v = np.array([float(x) for x in vmax_csv.split(",")])
        amax_v = np.array([float(x) for x in amax_csv.split(",")])
        T_min = max(1.875 * dq_est / vmax_v) if np.all(vmax_v > 0) else 0.0
        T_min = max(T_min, float(np.sqrt(5.77 * dq_est.min() / amax_v.min())))
        if duration < T_min:
            print(f"❌ 时长 {duration:.3f}s 太短：慢轴至少要 {T_min:.3f}s")
            print(f"   建议加 --duration {math.ceil(T_min * 10) / 10:.1f}")
            return 3

        cmd = [ik_cli, "--urdf", urdf, "--frame", args.frame,
               "--seed", ",".join(f"{v:.9f}" for v in q_now),
               "--target", " ".join(f"{v:.9f}" for v in (
                   p_target[0], p_target[1], p_target[2],
                   quat[0], quat[1], quat[2], quat[3])),
               "--tol", f"{args.tol:.1e}",
               "--duration", f"{duration:.6f}",
               "--max-jump", f"{args.max_joint_jump:.6f}",
               "--max-norm", f"{args.max_joint_norm:.6f}",
               "--vmax", vmax_csv,
               "--amax", amax_csv]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        meta_ik, _ = parse_cli_output(proc.stdout)
        if meta_ik.get("ok") is not True:
            print("❌ IK 失败（目标可能不可达 / 超限位 / 起始就在奇异点）")
            print(f"   position_error={meta_ik.get('position_error')}  "
                  f"orientation_error={meta_ik.get('orientation_error')}  "
                  f"limited={meta_ik.get('limited')}")
            if proc.stderr:
                print("   " + proc.stderr.strip()[:300])
            return 3
        q_sol = np.array([float(x) for x in meta_ik["q"]])
        dq = q_sol - np.array(q_now)
        if np.abs(dq).max() > args.max_joint_jump:
            k = int(np.argmax(np.abs(dq)))
            print(f"❌ IK 解与当前位形差太多（joint{k+1} 要动 "
                  f"{math.degrees(dq[k]):+.1f}° > "
                  f"{math.degrees(args.max_joint_jump):.0f}° 上限）")
            print("   常见原因：解跳到了另一个构型分支（肘上/肘下）。")
            print("   建议：先用 jog.py 分几步接近，或用更小的位移")
            return 3
        sig = float(meta_ik.get("sigma_ratio", 0) or 0)
        psig = float(meta_ik.get("path_sigma_ratio", 0) or 0)
        print(f"  ✅ 选解成功：候选 {meta_ik.get('n_candidates')} 个 → "
              f"通过 {meta_ik.get('n_accepted')} 个，score="
              f"{float(meta_ik.get('score', 0) or 0):.3f}")
        print(f"     残差 {float(meta_ik.get('position_error', 0))*1000:.4f} mm / "
              f"{float(meta_ik.get('orientation_error', 0)):.1e} rad")
        print(f"     目标关节角 = {np.round(np.degrees(q_sol), 1)} 度")
        print(f"     关节跳变   = {np.round(np.degrees(dq), 1)} 度（最大 "
              f"{np.degrees(np.abs(dq).max()):.1f}°）")
        print(f"     奇异度     = 终点 {sig:.4f} / 路径 {psig:.4f}"
              f"（阈值 0.002）")
        if psig and psig < 2e-3:
            print(f"     ⚠️ 运动路径穿过奇异区（路径 σ={psig:.5f}），执行时可能不跟手")
        wps = [{"t": 0.0, "q": [float(x) for x in q_now], "qd": [0.0] * 6},
               {"t": float(duration), "q": [float(x) for x in q_sol], "qd": [0.0] * 6}]
    else:
        # ② 笛卡尔直线：走 plan_line_cli（RMRC），保留为扩展
        cmd = [cli, "--urdf", urdf, "--start", ",".join(f"{v:.9f}" for v in q_now),
               "--target", " ".join(f"{v:.9f}" for v in (
                   p_target[0], p_target[1], p_target[2],
                   quat[0], quat[1], quat[2], quat[3])),
               "--frame", args.frame,
               "--duration", f"{duration:.4f}", "--sample", f"{args.sample:.4f}"]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        if proc.returncode != 0 and not proc.stdout:
            print("❌ plan_line_cli 失败：")
            print(proc.stderr[:800])
            return 3
        meta, wps = parse_cli_output(proc.stdout)
        if meta.get("ok") is not True:
            err = meta.get("error", "未知原因")
            print(f"❌ 规划失败：{err}")
            print(f"   （position_error={meta.get('position_error')}, "
                  f"sigma_ratio={meta.get('min_sigma_ratio')}）")
            if meta.get("failure_q_deg"):
                print(f"   失败的位形 q(度) = {meta['failure_q_deg']}")
            if "奇异" in err:
                print()
                print("   ⚠️ 这是【真实的奇异/病态位形】，不是误报：")
                print("      · 本臂 j2/j3/j5 三轴平行，q5 接近 0° 时腕部奇异")
                print("      · 当前位形里 joint4 又贴着限位，笛卡尔解会把它推出去")
                print("   ✅ 解法：先用关节空间点动挪到「舒适位形」，再走直线：")
                print("        python3 jog.py 0 0.8 0.4 0 0.6 0 --abs --time 8")
                print("      该位形 σ_min/σ_max ≈ 0.06（现在的约 30 倍）")
            elif "限位" in err:
                print()
                print("   ⚠️ 轨迹中途撞到关节限位：目标可能超出工作空间，或需要换构型")
            return 3
        print(f"  ✅ 规划成功：{len(wps)} 个路点，"
              f"终点残差 {float(meta.get('position_error', 0))*1000:.3f} mm / "
              f"{float(meta.get('orientation_error', 0)):.2e} rad")
        print(f"     最小 σ_min/σ_max = {meta.get('min_sigma_ratio')}")

    if args.dry_run:
        print("\n  （--dry-run：不发送命令）")
        return 0

    if not args.yes:
        try:
            if input("\n发送这条轨迹？[y/N] ").strip().lower() not in ("y", "yes"):
                print("已取消")
                return 0
        except EOFError:
            # ★ R4：stdin 关闭 ≠ 用户同意。旧代码这里是 pass → 直接发送。
            print("未确认（stdin 已关闭），已取消")
            return 0

    # ── ★ R3：确认期间机器人可能动过，发送前必须重新确认起点 ──
    fresh = fresh_state(node, state, JOINTS, timeout=1.5, need=2)
    if fresh is None:
        print("❌ 确认后读不到新鲜 /joint_states，拒绝发送")
        return 3
    dq_start = np.array(fresh) - np.array(q_now)
    if np.abs(dq_start).max() > args.start_tolerance:
        k = int(np.argmax(np.abs(dq_start)))
        print(f"❌ 确认期间关节动了 {np.degrees(dq_start[k]):+.1f}°（joint{k+1}）"
              f" > {np.degrees(args.start_tolerance):.1f}° 上限")
        print("   轨迹的起点已经过期，请重新规划")
        return 3
    # 起点微动：把首路点对齐到**最新**实测值，避免 t=0 就跳变
    if wps and abs(dq_start).max() > 1e-9:
        print(f"   ⚠️ 起点微动 {np.degrees(np.abs(dq_start).max()):.2f}°，"
              f"首路点对齐到实测值")
        wps[0]["q"] = [float(x) for x in fresh]

    # ── 发给 JTC ──
    client = ActionClient(node, FollowJointTrajectory,
                          "/hw_arm_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=5.0):
        print("❌ 找不到 action server /hw_arm_controller/follow_joint_trajectory")
        return 1

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    pts = []
    for w in wps:
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in w["q"]]
        p.velocities = [float(x) for x in w.get("qd", [0.0] * 6)]
        tt = float(w["t"])
        p.time_from_start.sec = int(tt)
        p.time_from_start.nanosec = int((tt - int(tt)) * 1e9)
        pts.append(p)
    goal.trajectory.points = pts

    print("发送中…")
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if not fut.done():
        # ★ R5：响应迟到 ≠ 被拒绝。可能已被接受，这里不能当"拒绝"处理，
        #        也不能继续等结果（没有 handle）；明确失败退出。
        print("❌ 目标接受响应超时（10 s 无回应）")
        print("   目标可能已被接受，请检查控制器状态：ros2 control list_controllers")
        return 1
    handle = fut.result()
    if handle is None or not handle.accepted:
        print("❌ 目标被拒绝（已有目标在执行 / 轨迹不合法）")
        return 1

    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=duration + 20)
    if not res_fut.done():
        # ★ R5：超时必须**主动取消**，否则轨迹还在跑，脚本却已经退出了
        print("⚠️ 等结果超时，请求取消…")
        cancel_fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_fut, timeout_sec=5.0)
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=3.0)
        print("❌ 执行超时（已请求取消）")
        return 1

    res = res_fut.result()
    if res is None:
        print("❌ 拿不到执行结果")
        return 1

    # ★ R5：必须同时满足"终态 SUCCEEDED"和"结果码正常"。
    #   取消/中止时 error_code 常常还是 0，只看 error_code 会误报成功。
    from action_msgs.msg import GoalStatus
    ok_status = (res.status == GoalStatus.STATUS_SUCCEEDED)
    code = res.result.error_code
    if not ok_status or code != 0:
        names = {GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
                 GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
                 GoalStatus.STATUS_EXECUTING: "EXECUTING",
                 GoalStatus.STATUS_CANCELING: "CANCELING",
                 GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                 GoalStatus.STATUS_CANCELED: "CANCELED",
                 GoalStatus.STATUS_ABORTED: "ABORTED"}
        print(f"❌ 执行未成功：status={names.get(res.status, res.status)} "
              f"error_code={code}  {res.result.error_string}")
        return 1
    print("✅ 完成")

    # 到位后核对
    t_end = time.time() + 1.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["q"] is not None:
        pin.forwardKinematics(model, data, np.array(state["q"]))
        pin.updateFramePlacements(model, data)
        p_end = data.oMf[fid].translation
        print(f"  实际到位 {np.round(p_end, 4)} m，"
              f"与目标差 {float(np.linalg.norm(p_target - p_end))*1000:.2f} mm")

    node.destroy_node()
    return 0


if __name__ == "__main__":
    sys.exit(main())
