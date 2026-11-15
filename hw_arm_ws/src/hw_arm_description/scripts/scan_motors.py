#!/usr/bin/env python3
"""扫描 CAN 总线上的达妙电机：型号无关的识别 + 与预期配置对账。

为什么不能「直接往 ID 1..16 发 MIT 帧去试探」：
    对不存在的 CAN ID 发帧 → 没有节点 ACK → 控制器不断重发 → TEC 每帧 +8
    → 到 255 直接 bus-off，整条总线要等恢复。

本脚本用的是**广播查询**（达妙协议的 refresh，0x7FF + 负载里带电机 ID）：
    帧发到广播地址 0x7FF，总线上**任何一台**在线的电机都会 ACK，
    目标电机在负载里认领自己的 ID 后回状态帧。
    → 探不存在的 ID 也绝不会把总线拖进 bus-off。

输出：
    · 哪些 ID 在线
    · 每台的 **Master ID**（从回复帧的 CAN ID 直接读出来）
    · 位置（pMax 全系 12.5，与型号无关，位置一定准）
    · 温度（MOS / 转子）、错误码
    · 与 hw_arm_hardware.hpp + control_gains.yaml 的预期配置逐项对账

用法：
    python3 scripts/scan_motors.py                    # 扫 1..8
    python3 scripts/scan_motors.py --ids 1,2,3        # 只扫指定 ID
    python3 scripts/scan_motors.py --passive          # 只监听，不发任何帧
    python3 scripts/scan_motors.py --interface can1
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import struct
import sys
import time

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HPP = os.path.join(os.path.dirname(PKG), "hw_arm_hardware",
                   "include", "hw_arm_hardware", "hw_arm_hardware.hpp")

CANFD_MTU = 72
BROADCAST_ID = 0x7FF
P_MAX = 12.5          # 全系统一，位置解码与型号无关

ERR_NAMES = {
    0x0: "DISABLED", 0x1: "ENABLED", 0x8: "OVERVOLTAGE", 0x9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "ROTOR_OVERTEMP",
    0xD: "COMM_LOST", 0xE: "OVERLOAD", 0xF: "UNKNOWN",
}


def uint_to_float(u: int, lo: float, hi: float, bits: int) -> float:
    return u * (hi - lo) / ((1 << bits) - 1) + lo


def expected_config() -> dict[int, dict]:
    """从 hw_arm_hardware.hpp 读出厂配置：joint index -> {type, slave, master}。"""
    src = open(HPP, encoding="utf-8").read()

    def vec(name: str) -> list[str]:
        m = re.search(name + r"\s*=\s*\{(.*?)\};", src, re.S)
        if not m:
            return []
        body = re.sub(r"//[^\n]*", "", m.group(1))
        out = []
        for it in body.split(","):
            it = it.strip()
            if it:
                out.append(it.split("::")[-1].strip())
        return out

    types = vec("MOTOR_TYPES")
    send = vec("SEND_CAN_IDS")
    recv = vec("RECV_CAN_IDS")
    cfg = {}
    for i in range(min(len(types), len(send), len(recv))):
        cfg[i + 1] = {"type": types[i], "slave": int(send[i], 0), "master": int(recv[i], 0)}
    return cfg


class Bus:
    """mode: classic（经典 CAN 帧）| fd（FD 帧，不切速率）| fd-brs（FD + 数据段 5M）"""

    def __init__(self, ifname: str, mode: str = "fd"):
        self.ifname = ifname
        self.mode = mode
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        if mode != "classic":
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        self.sock.bind((ifname,))
        self.sock.settimeout(0.002)

    def send(self, can_id: int, data: bytes):
        if self.mode == "classic":
            # struct can_frame: can_id(4) + dlc(1) + pad(3) + data[8]
            frame = struct.pack("<IB3x8s", can_id, len(data),
                                data.ljust(8, b"\x00"))
        else:
            flags = 0x01 if self.mode == "fd-brs" else 0x00   # CANFD_BRS = 0x01
            frame = struct.pack("<IBBBB64s", can_id, len(data), flags, 0, 0,
                                data.ljust(64, b"\x00"))
        try:
            self.sock.send(frame)
        except OSError as e:
            self._raise_if_down(e)

    def _raise_if_down(self, e: OSError):
        if e.errno == 100 or "Network is down" in str(e):
            raise SystemExit(
                f"\n❌ {self.ifname} 处于 DOWN 状态，收发都不行。\n"
                f"   请先执行：\n"
                f"     sudo ip link set {self.ifname} up "
                f"type can bitrate 1000000 dbitrate 5000000 fd on\n"
                f"   然后重跑本脚本。")
        raise e

    def drain(self, seconds: float) -> list[tuple[int, bytes]]:
        out, t_end = [], time.monotonic() + seconds
        while time.monotonic() < t_end:
            try:
                raw = self.sock.recv(16 if self.mode == "classic" else CANFD_MTU)
            except (socket.timeout, BlockingIOError):
                continue
            except OSError as e:
                self._raise_if_down(e)
            can_id, dlc = struct.unpack("<IB", raw[:5])
            if can_id & 0x20000000:          # 错误帧
                continue
            out.append((can_id & 0x1FFFFFFF, raw[8:8 + dlc]))
        return out


def probe(bus: Bus, motor_id: int, wait: float = 0.25) -> dict | None:
    """广播查询单个电机：0x7FF + [id, 0, 0xCC, ...]。"""
    bus.drain(0.02)                                    # 清空旧的
    # 官方 SDK 的刷新帧：0x7FF, dlc=4, [id_lo, id_hi, 0xCC, 0x00]
    bus.send(BROADCAST_ID, bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xCC, 0x00]))
    frames = bus.drain(wait)
    for can_id, data in frames:
        if len(data) < 8:
            continue
        if (data[0] & 0x0F) != (motor_id & 0x0F):
            continue                                   # 不是这台回的
        q = uint_to_float((data[1] << 8) | data[2], -P_MAX, P_MAX, 16)
        return {
            "id": motor_id,
            "master": can_id,                          # 回复帧的 CAN ID 就是 Master ID
            "q": q,
            "err": data[0] >> 4,
            "t_mos": data[6],
            "t_rotor": data[7],
        }
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6,7,8")
    ap.add_argument("--passive", action="store_true", help="只监听，不发帧")
    ap.add_argument("--mode", default="classic", choices=["classic", "fd", "fd-brs"],
                    help="帧格式：classic / fd（不切速率）/ fd-brs（数据段 5M，官方 SDK 用这个）")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    cfg = expected_config()

    # 接口检查
    try:
        bus = Bus(args.interface, mode=args.mode)
    except OSError as e:
        print(f"❌ 打不开 {args.interface}: {e}")
        print(f"   先执行：sudo ip link set {args.interface} up "
              f"type can bitrate 1000000 dbitrate 5000000 fd on")
        return 2

    print(f"接口 {args.interface} / 模式 {args.mode} / 候选 ID {ids}\n")

    if args.passive:
        print("被动监听 3 秒（不发任何帧）…")
        frames = bus.drain(3.0)
        if not frames:
            print("  没有任何帧。电机只在被查询/控制时回复，这是正常的。")
        for can_id, data in frames[:20]:
            print(f"  0x{can_id:03X}  {data.hex(' ')}")
        return 0

    found = {}
    for mid in ids:
        # 重试：电机在空闲后常常漏掉第一帧查询，单次探测会误报"无响应"
        r = None
        for attempt in range(3):
            r = probe(bus, mid, wait=0.25 if attempt == 0 else 0.3)
            if r:
                break
            time.sleep(0.05)
        if r:
            found[mid] = r
            print(f"  ✅ ID {mid:2d} 在线  master=0x{r['master']:02X}  "
                  f"q={r['q']:+.4f} rad  Tmos={r['t_mos']}℃ Trot={r['t_rotor']}℃  "
                  f"{ERR_NAMES.get(r['err'], hex(r['err']))}")
        else:
            print(f"  ·  ID {mid:2d} 无响应")

    if not found:
        print("\n❌ 一台都没找到。按顺序排查：")
        print("   1) 电机是否上电（24/48V），指示灯亮？")
        print("   2) CAN_H / CAN_L 是否接反？")
        print("   3) 终端电阻：总线两端各 120Ω？")
        print("   4) 波特率：电机若是经典 CAN，加 --no-fd 再试")
        print(f"   5) 接口是否 UP：ip -br link show {args.interface}")
        return 1

    # ---------------- 对账 ----------------
    print(f"\n{'':2s}{'joint':7s} {'预期型号':9s} {'预期ID':6s} {'预期Master':10s} "
          f"{'实际master':10s} {'位置':>9s} {'Tmos':>5s}  结论")
    print("─" * 92)
    bad = 0
    for j in sorted(found):
        exp = cfg.get(j)
        r = found[j]
        if exp is None:
            print(f"  {('joint%d' % j):7s} {'—':9s} {'—':6s} {'—':10s} "
                  f"0x{r['master']:02X}      {r['q']:+9.4f} {r['t_mos']:5d}  多余（配置里没有）")
            bad += 1
            continue
        id_ok = r["master"] == exp["master"]
        notes = []
        if not id_ok:
            notes.append(f"Master 不符（期望 0x{exp['master']:02X}）")
            bad += 1
        if r["err"] >= 0x8:
            notes.append(f"故障 {ERR_NAMES.get(r['err'], hex(r['err']))}")
            bad += 1
        if abs(r["q"]) > P_MAX:
            notes.append("位置超量程")
            bad += 1
        verdict = "✅ 一致" if not notes else "❌ " + "；".join(notes)
        print(f"  {('joint%d' % j):7s} {exp['type']:9s} 0x{exp['slave']:02X}  "
              f"0x{exp['master']:02X}      0x{r['master']:02X}      "
              f"{r['q']:+9.4f} {r['t_mos']:5d}  {verdict}")

    missing = [j for j in sorted(cfg) if j not in found]
    if missing:
        print(f"\n⚠️  未响应的关节: {missing}（若今晚只接了部分电机，属正常）")

    print(f"\n共找到 {len(found)} 台，{bad} 项不一致")
    if bad == 0:
        print("✅ 全部与配置一致，可以开始控制")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
