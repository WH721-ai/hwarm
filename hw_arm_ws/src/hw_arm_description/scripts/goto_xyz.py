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

    rclpy.init()
    node = Node("goto_xyz")
    state = {"q": None}

    def cb(msg: JointState):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            state["q"] = [d[j] for j in JOINTS]

    node.create_subscription(JointState, "/joint_states", cb, 10)
    import time
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
        cmd = [ik_cli, "--urdf", urdf, "--frame", args.frame,
               "--seed", ",".join(f"{v:.9f}" for v in q_now),
               "--target", " ".join(f"{v:.9f}" for v in (
                   p_target[0], p_target[1], p_target[2],
                   quat[0], quat[1], quat[2], quat[3])),
               "--tol", f"{args.tol:.1e}"]
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
            pass

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
    handle = fut.result()
    if handle is None or not handle.accepted:
        print("❌ 目标被拒绝（可能已有目标在执行）")
        return 1
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=duration + 20)
    res = res_fut.result()
    if res is None:
        print("⚠️ 等结果超时（轨迹可能仍在执行）")
        return 1
    code = res.result.error_code
    if code != 0:
        print(f"❌ 执行失败，error_code={code}  {res.result.error_string}")
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
