#!/usr/bin/env python3
"""量化重力补偿的效果 —— 测量"使能后下垂了多少"

═══════════════════════════════════════════════════════════════════
为什么需要这个
═══════════════════════════════════════════════════════════════════
打开重力补偿后，机械臂**不会动起来** —— 它依然是"保持位置"。
补偿的作用是让**静态下垂变小**：

    没有补偿：  kp·下垂 = 重力矩  →  joint2 下垂 2.285/70 ≈ 1.87°
    30% 补偿：  残差 0.7·g        →  下垂 ≈ 1.31°
    100% 补偿： 残差只剩模型误差   →  下垂 ≈ 0.1°

30% 和 0% 只差 0.5°，肉眼几乎看不出来。所以要**测**，不要看。

═══════════════════════════════════════════════════════════════════
原理
═══════════════════════════════════════════════════════════════════
使能瞬间，控制器把命令初始化成"当前位置"；随后机械臂在重力下慢慢沉到
平衡点。所以：

    下垂量 = (使能后稳定位置) − (使能瞬间的位置)

只订阅 /joint_states，不发任何控制帧。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    # 终端 A：先启动测量
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/measure_droop.py --seconds 8

    # 终端 B：再启动机械臂
    ros2 launch hw_arm_bringup hw_arm.launch.py use_fake_hardware:=false

    # 8 秒后终端 A 会打印每个关节的下垂量

然后改 gravity_scale 再跑一次，对比两次的结果。
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="测量使能后的静态下垂")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--window", type=float, default=0.4,
                    help="取首尾各多少秒的平均值（默认 0.4s）")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        print("需要 ROS：source /opt/ros/jazzy/setup.bash")
        return 2

    rclpy.init()
    node = Node("measure_droop")
    samples = []      # (t, q array, names)

    def cb(msg: JointState):
        names = list(msg.name)
        if not all(f"joint{i}" in names for i in range(1, 7)):
            return
        q = np.array([msg.position[names.index(f"joint{i}")] for i in range(1, 7)])
        samples.append((node.get_clock().now().nanoseconds * 1e-9, q))

    node.create_subscription(JointState, args.topic, cb, 50)
    print(f"  等待 {args.topic} …（最多 {args.seconds:.0f} 秒）")
    t_end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while node.get_clock().now().nanoseconds * 1e-9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 只保留"有变化"的那一段：从第一个偏离起始值超过 0.05° 的采样开始
    if len(samples) < 20:
        print(f"  ❌ 只收到 {len(samples)} 帧，机械臂是不是没启动？")
        node.destroy_node()
        return 2

    q_all = np.array([s[1] for s in samples])
    q0 = q_all[0]
    moved = np.abs(np.degrees(q_all - q0)).max(axis=1)
    idx = np.where(moved > 0.05)[0]
    if idx.size == 0:
        print("  ⚠️ 位置全程没有变化（机械臂可能一直没使能，或本来就稳如磐石）")
        start, end = 0, len(samples) - 1
    else:
        start = max(0, idx[0] - 5)
        end = len(samples) - 1

    nwin = max(3, int(args.window * 20))
    q_before = q_all[start:start + nwin].mean(axis=0)
    q_after = q_all[-nwin:].mean(axis=0)
    d = q_after - q_before

    print()
    print("=" * 70)
    print("  使能后的静态下垂")
    print("=" * 70)
    print(f"  {'关节':8s}{'使能瞬间(°)':>14s}{'稳定后(°)':>13s}{'下垂(°)':>11s}")
    for i in range(6):
        print(f"  joint{i+1} {math.degrees(q_before[i]):13.2f}"
              f"{math.degrees(q_after[i]):13.2f}{math.degrees(d[i]):10.2f}")
    print()
    worst = int(np.argmax(np.abs(d)))
    print(f"  下垂最大的关节：joint{worst+1}  {math.degrees(d[worst]):+.2f}°")
    print(f"  全部下垂的绝对值之和：{np.abs(np.degrees(d)).sum():.2f}°")
    # ── 反推真实重力矩：g_true = kp × (q_des − q_actual) = −kp × 下垂 ──
    # 这只在 gravity_scale = 0（纯 PD）时成立，但也可以用来核对模型。
    try:
        import os, yaml
        here = os.path.dirname(os.path.abspath(__file__))
        pkg = os.path.dirname(here)
        gains = yaml.safe_load(open(os.path.join(pkg, "config", "control",
                                                 "control_gains.yaml"), encoding="utf-8"))
        kp = np.array([gains[f"joint{i}"]["kp"] for i in range(1, 7)])
        urdf = os.path.join(os.path.dirname(pkg), "..", "install", "hw_arm_description",
                            "share", "hw_arm_description", "urdf", "hw_arm.urdf")
        urdf = os.path.normpath(urdf)
        g_model = None
        if os.path.isfile(urdf):
            import pinocchio as pin
            m = pin.buildModelFromUrdf(urdf)
            dd = m.createData()
            g_model = np.asarray(pin.rnea(m, dd, q_after, np.zeros(m.nv), np.zeros(m.nv)))

        g_true = -kp * d          # d = 稳定后 − 使能瞬间，下垂时 d<0
        print()
        print("  反推真实重力矩（只在下垂不受补偿影响时才准）")
        print(f"  {'关节':8s}{'kappa·下垂':>12s}{'模型 g':>11s}{'比值':>8s}   结论")
        for i in range(6):
            if abs(g_model[i]) < 0.05 and abs(g_true[i]) < 0.05:
                continue
            ratio = (g_true[i] / g_model[i]) if abs(g_model[i]) > 1e-6 else float("nan")
            tag = ""
            if abs(ratio - 1.0) < 0.10:
                tag = "✅ 模型准（差 <10%）"
            elif ratio > 1.0:
                tag = f"❌ 模型偏小 {ratio:.2f} 倍"
            else:
                tag = f"❌ 模型偏大 {1/ratio:.2f} 倍"
            print(f"  joint{i+1} {g_true[i]:11.3f}{g_model[i]:11.3f}{ratio:8.2f}   {tag}")
        print()
        print("  ⚠️ 这个反推假设 kp 准、且没有摩擦/其它外力，只能做量级判断。")
        print("     要精确辨识请用 C-05：algo/scripts/collect_static_gravity.py")
    except Exception as e:  # noqa: BLE001
        print(f"  （反推重力矩失败：{e}）")

    print()
    print("  对比方法：改 gravity_scale（0 → 0.3 → 1.0）各跑一次，")
    print("            『下垂(°)』越小说明补偿越有效。")
    print("=" * 70)

    node.destroy_node()
    return 0


if __name__ == "__main__":
    sys.exit(main())
