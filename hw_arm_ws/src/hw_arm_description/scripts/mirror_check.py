#!/usr/bin/env python3
"""实时镜像：**手拖真机，RViz 里的手臂跟着动** —— 用来判定 direction ±1。

═══════════════════════════════════════════════════════════════════
为什么用这个方法
═══════════════════════════════════════════════════════════════════
判定「电机 +q 是否等于 URDF +q」只有一条可靠途径：
    **看真机的物理转动方向，和 URDF 的转动方向是不是同一个。**

之前试过的两种做法都不靠谱：

  ✗ 命令 +tau，看编码器变大还是变小
    → 同义反复。电机内部的 q 和 tau 用同一套符号，正力矩必然产生正读数，
      跟它怎么装在机械臂上无关。

  ✗ RViz 转 30°（2 秒动画），真机转 5°（0.2 秒），靠回忆对比
    → 幅度差 6 倍、视角不同、隔了十几秒，极易看错。

本方法的做法：

    电机全部 DISABLED（零力矩，绝对安全）
            ↓
    本脚本 30 Hz 读六个编码器 → 直接发布 /joint_states
            ↓
    你用手晃真机的任意一根 link
            ↓
    RViz 里的"影子手臂"跟着动 → direction = +1
    反着动                     → direction = −1

判断是**实时、同幅度、同视角**的 —— 只有"跟不跟"这一个二元问题。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    # 终端 A：RViz（robot_state_publisher + rviz2）
    ros2 launch hw_arm_mujoco display.launch.py

    # 终端 B：镜像
    python3 scripts/mirror_check.py

    # 发现某个关节反着动，就取反再试
    python3 scripts/mirror_check.py --flip 2
    python3 scripts/mirror_check.py --flip 2,5

    # 只镜像一个关节（其他保持 0），减少干扰
    python3 scripts/mirror_check.py --joint 1

    # 调零位偏置（如果电机零点不等于 URDF 零点）
    python3 scripts/mirror_check.py --offset 0,0,0,0,0,0

═══════════════════════════════════════════════════════════════════
安全
═══════════════════════════════════════════════════════════════════
    · **只发广播查询帧（0x7FF），不使能、不给力矩、不写 Flash**
    · 电机全程 DISABLED，你是唯一让它动的人
    · Ctrl-C 退出，不改变任何硬件状态

前置：can0 已配置为经典 CAN 1 Mbps 且 UP。
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import time

P_MAX = 12.5          # 全系统一，位置解码与型号无关
BROADCAST_ID = 0x7FF
N_JOINTS = 6


def link_state(ifname: str = "can0"):
    """返回 (是否 UP, tx_queue_len, bitrate)。读 sysfs，不需要 root。"""
    try:
        flags = int(open(f"/sys/class/net/{ifname}/flags").read().strip(), 16)
        up = bool(flags & 0x1)                       # IFF_UP
        qlen = int(open(f"/sys/class/net/{ifname}/tx_queue_len").read().strip())
        return up, qlen
    except Exception:  # noqa: BLE001
        return None, 0


def self_qlen() -> int:
    """读 can0 的 txqueuelen（不依赖 root）。"""
    try:
        txt = open("/sys/class/net/can0/tx_queue_len").read().strip()
        return int(txt)
    except Exception:  # noqa: BLE001
        return 0


def uint_to_float(u: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return u * span / ((1 << bits) - 1) + x_min


# ──────────────────────────────────────────────────────────────
# CAN 只读层
# ──────────────────────────────────────────────────────────────
class MirrorBus:
    """只做两件事：发广播查询帧、收状态帧。**不发任何控制帧。**"""

    def __init__(self, ifname: str, ids: list[int]):
        self.ids = ids
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        # 非阻塞：发送队列满时立刻返回 EAGAIN/ENOBUFS，而不是等 2 ms 后抛
        # TimeoutError 把整个脚本打断
        self.sock.setblocking(False)
        self.q = {i: 0.0 for i in ids}
        self.seen = {i: False for i in ids}
        self.other = 0
        self.dropped = 0

    def query(self):
        """广播刷新帧：CAN ID=0x7FF, dlc=4, [id_lo, id_hi, 0xCC, 0x00]

        发到广播地址，任何电机都 ACK，探不存在的 ID 也不会把总线拖进 bus-off。
        """
        for i in self.ids:
            frame = struct.pack("<IB3x8s", BROADCAST_ID, 4,
                                bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0x00])
                                .ljust(8, b"\x00"))
            try:
                self.sock.send(frame)
            except (BlockingIOError, TimeoutError):
                self.dropped += 1       # 队列满，下一轮重发
            except OSError as e:
                if e.errno in (105, 11, 100):   # ENOBUFS / EAGAIN / ENETDOWN
                    self.dropped += 1
                else:
                    raise

    def drain(self) -> bool:
        """把已到达的帧全部收完并解码。返回是否收到新的有效帧。"""
        got = False
        while True:
            try:
                raw = self.sock.recv(16)
            except (BlockingIOError, TimeoutError, socket.timeout):
                break
            if len(raw) < 16:
                continue
            can_id, dlc = struct.unpack_from("<IB", raw, 0)
            data = raw[8:8 + dlc]
            if len(data) < 8:
                continue
            mid = data[0] & 0x0F
            if mid in self.q:
                self.q[mid] = uint_to_float((data[1] << 8) | data[2],
                                            -P_MAX, P_MAX, 16)
                self.seen[mid] = True
                got = True
            else:
                self.other += 1
        return got

    def close(self):
        self.sock.close()


# ──────────────────────────────────────────────────────────────
# 主流程
# ──────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(
        description="实时镜像：手拖真机，RViz 跟随（电机全程失能）")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--rate", type=float, default=30.0, help="发布频率 Hz")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--flip", default="", help="取反的关节号，如 2 或 2,5")
    ap.add_argument("--joint", type=int, default=0,
                    help="只镜像这一个关节（1~6），其它保持 0；0=全部")
    ap.add_argument("--offset", default="", help="每关节零位偏置(rad)，逗号分隔")
    ap.add_argument("--no-ros", action="store_true", help="不发布，只在终端看数字")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    flips = {int(x) for x in args.flip.split(",") if x.strip()}
    sign = {i: (-1.0 if i in flips else 1.0) for i in ids}
    offset = {}
    if args.offset:
        vals = [float(x) for x in args.offset.split(",")]
        offset = {i: vals[k] for k, i in enumerate(ids) if k < len(vals)}
    offset = {i: offset.get(i, 0.0) for i in ids}

    # ── ROS ──────────────────────────────────────────────────
    rclpy = np_msg = None
    node = pub = None
    if not args.no_ros:
        try:
            import rclpy as _rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState as _JS
            rclpy, np_msg = _rclpy, _JS
            if not rclpy.ok():
                rclpy.init()
            node = Node("mirror_check")
            pub = node.create_publisher(np_msg, "/joint_states", 10)
        except Exception as e:  # noqa: BLE001
            print(f"❌ ROS 起不来（{e}）\n"
                  f"   先 source /opt/ros/jazzy/setup.bash 和 install/setup.bash")
            return 2

    up, qlen = link_state(args.interface)
    if up is False:
        print("=" * 72)
        print(f"  ❌ {args.interface} 现在是 **DOWN**，收发都做不了")
        print("=" * 72)
        print("  先跑这三条（需要密码）：")
        print()
        print(f"    sudo ip link set {args.interface} type can "
              f"bitrate 1000000 sample-point 0.75 restart-ms 0 fd off")
        print(f"    sudo ip link set {args.interface} txqueuelen 1000")
        print(f"    sudo ip link set {args.interface} up")
        print()
        print(f"  然后确认：  ip -br link show {args.interface}")
        print("  期望：      can0  UP  <NOARP,UP,LOWER_UP,ECHO>")
        return 3

    bus = MirrorBus(args.interface, ids)

    print("=" * 72)
    print("  实时镜像 —— 电机全程 DISABLED，只读编码器，不发任何控制帧")
    print("=" * 72)
    print(f"  接口     : {args.interface}")
    print(f"  发布频率 : {args.rate:.0f} Hz → /joint_states")
    print(f"  方向系数 : " + "  ".join(f"j{i}:{sign[i]:+.0f}" for i in ids))
    if any(abs(v) > 1e-9 for v in offset.values()):
        print(f"  零位偏置 : " + "  ".join(f"j{i}:{offset[i]:+.4f}" for i in ids))
    if args.joint:
        print(f"  只镜像   : joint{args.joint}（其它固定 0）")
    print()
    print("  ➜ 现在用手慢慢晃真机的任意一根 link，看 RViz 里的手臂：")
    print("      跟着动 → 该关节 direction = +1")
    print("      反着动 → 该关节 direction = −1（用 --flip 取反再确认）")
    print()
    print("  Ctrl-C 退出")
    print("=" * 72)
    header = f"{'t(s)':>6s} |" + "".join(f"{f'j{i}':>9s}" for i in ids)
    print(header)

    cmd_names = [f"joint{i}" for i in range(1, N_JOINTS + 1)]
    msg = None
    if pub is not None:
        msg = np_msg()
        msg.name = cmd_names
        msg.position = [0.0] * N_JOINTS

    dt = 1.0 / args.rate
    q_pub = {i: 0.0 for i in ids}
    t0 = time.monotonic()
    next_print = 0.0
    next_query = 0.0
    warned = False

    try:
        while True:
            now = time.monotonic()

            # 查询限速：rate 的一半，避免灌满 txqueuelen
            if now >= next_query:
                bus.query()
                next_query = now + max(dt, 0.02)
            bus.drain()

            for i in ids:
                q_pub[i] = sign[i] * (bus.q[i] - offset[i])

            if pub is not None:
                msg.header.stamp = node.get_clock().now().to_msg()
                full = [0.0] * N_JOINTS
                if args.joint:
                    full[args.joint - 1] = q_pub.get(args.joint, 0.0)
                else:
                    for i in ids:
                        if 1 <= i <= N_JOINTS:
                            full[i - 1] = q_pub[i]
                msg.position = full
                pub.publish(msg)

            # 1.5 s 还一台都没读到 → 总线有问题，明确报出来
            if not warned and now - t0 > 1.5 and not any(bus.seen.values()):
                warned = True
                print()
                print("  ⚠️  1.5 秒内一台电机都没读到！可能原因：")
                up_now, qlen = link_state(args.interface)
                print(f"     本轮丢帧 {bus.dropped}，接口 UP={up_now} qlen={qlen}")
                if up_now is False:
                    print("     ⛔ can0 掉线了（DOWN）→ 重新配置：")
                    print("        sudo ip link set can0 type can bitrate 1000000 "
                          "sample-point 0.75 restart-ms 0 fd off")
                    print("        sudo ip link set can0 txqueuelen 1000")
                    print("        sudo ip link set can0 up")
                elif qlen and qlen >= 1000:
                    print("        ⚠️ qlen 已经够大，仍发不出去"
                          " → **USB-CAN 适配器卡死了**")
                    print("        修复（二选一）：")
                    print("          a) 拔掉 USB-CAN 适配器，等 3 秒，插回去")
                    print("          b) sudo ip link set can0 down && "
                          "sudo modprobe -r gs_usb && sudo modprobe gs_usb")
                else:
                    print("        修复： sudo ip link set can0 txqueuelen 1000")
                    print("              sudo ip link set can0 down")
                    print("              sudo ip link set can0 up")
                print("     ② can0 不是 UP：  ip -br link show can0")
                print("     ③ 电机没上电 / 接线松 / 终端电阻缺失")
                print()

            if now >= next_print:
                t = now - t0
                cells = "".join(f"{math.degrees(q_pub[i]):+9.2f}" for i in ids)
                print(f"{t:6.1f} |{cells}   (度)")
                next_print = now + 0.25

            time.sleep(max(0.0, dt - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        if node is not None:
            node.destroy_node()

    print("\n退出。看到的实际方向请记下来，最终填进：")
    print("  hw_arm.ros2_control.xacro  →  direction: ["
          + ", ".join(f"{'+1' if i not in flips else '-1'}"
                      for i in range(1, N_JOINTS + 1)) + "]")
    print(f"\n统计：其他帧 {bus.other}，查询帧丢弃 {bus.dropped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
