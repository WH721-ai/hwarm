#!/usr/bin/env python3
"""读取达妙电机内部寄存器（经典 CAN，不需要 CANFD，也不需要 root）。

为什么自己写而不用 `openarm-can-cli show_param`：
    那个命令在源码里写死了 `OpenArm(interface, true)` —— **CANFD 模式**，
    而我们的总线是经典 CAN，所以它永远读不到任何电机。
    （见 openarm_can/setup/cli/commands/motor_read_param_commands.cpp）

═══════════════════════════════════════════════════════════════════
重点寄存器
═══════════════════════════════════════════════════════════════════
    RID 21  PMAX   位置量程  (rad)
    RID 22  VMAX   速度量程  (rad/s)
    RID 23  TMAX   力矩量程  (N·m)   ← 就是 MIT 协议 12 bit 归一化用的那个！
    RID  1  KT     力矩常数
    RID 20  Gr     减速比
    RID 55  dir    电机内部方向
    RID 13/14      硬件/软件版本
    RID 15  SN     序列号

**TMAX 直接从电机里读出来**，不用猜、不用标定 —— 前提是固件支持这个寄存器
（达妙部分型号会返回 NaN 或 0，脚本会标出来）。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    python3 scripts/read_motor_registers.py                  # 读 1..6，关键寄存器
    python3 scripts/read_motor_registers.py --ids 2,4        # 只读两台
    python3 scripts/read_motor_registers.py --all            # 全部 82 个寄存器
    python3 scripts/read_motor_registers.py --rid 23         # 只读 TMAX

⚠️ 只读：发的全是 0x33 查询帧（0x7FF 广播），不改任何参数、不使能电机。
"""
from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

BROADCAST_ID = 0x7FF

# 整数寄存器（其余按 float32 解析）—— 与 openarm_can 的 is_in_ranges() 一致
INT_RIDS = set(range(7, 11)) | set(range(13, 17)) | set(range(35, 37))

RID_NAMES = {
    0: "UV_Value", 1: "KT_Value", 2: "OT_Value", 3: "OC_Value",
    4: "ACC", 5: "DEC", 6: "MAX_SPD", 7: "MST_ID", 8: "ESC_ID",
    9: "TIMEOUT", 10: "CTRL_MODE", 11: "Damp", 12: "Inertia",
    13: "hw_ver", 14: "sw_ver", 15: "SN", 16: "NPP", 17: "Rs",
    18: "LS", 19: "Flux", 20: "Gr", 21: "PMAX", 22: "VMAX",
    23: "TMAX", 24: "I_BW", 25: "KP_ASR", 26: "KI_ASR",
    27: "KP_APR", 28: "KI_APR", 29: "OV_Value", 30: "GREF",
    31: "Deta", 32: "V_BW", 33: "IQ_c1", 34: "VL_c1",
    35: "can_br", 36: "sub_ver", 50: "u_off", 51: "v_off",
    52: "k1", 53: "k2", 54: "m_off", 55: "dir", 80: "p_m", 81: "xout",
}

# 默认关注：量程 + 型号线索 + 版本
KEY_RIDS = [23, 21, 22, 20, 1, 3, 13, 14, 10, 55]


class ParamBus:
    def __init__(self, ifname: str, ids: list[int]):
        self.ids = ids
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.setblocking(False)
        self.results = {i: {} for i in ids}
        self.dropped = 0

    def _send(self, can_id: int, data: bytes):
        frame = struct.pack("<IB3x8s", can_id, len(data), data.ljust(8, b"\x00"))
        try:
            self.sock.send(frame)
        except (BlockingIOError, TimeoutError):
            self.dropped += 1
        except OSError as e:
            if e.errno in (105, 11):
                self.dropped += 1
            else:
                raise

    def query(self, mid: int, rid: int):
        """0x7FF + [id_lo, id_hi, 0x33, RID, 0,0,0,0]"""
        self._send(BROADCAST_ID,
                   bytes([mid & 0xFF, (mid >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0]))

    def poll(self) -> int:
        n = 0
        while True:
            try:
                raw = self.sock.recv(16)
            except (BlockingIOError, TimeoutError):
                break
            if len(raw) < 16:
                continue
            _can_id, dlc = struct.unpack_from("<IB", raw, 0)
            data = raw[8:8 + dlc]
            if len(data) < 8 or data[2] != 0x33:
                continue
            rid = data[3]
            mid = data[0] & 0x0F
            if mid not in self.results:
                continue
            b = data[4:8]
            if rid in INT_RIDS:
                val = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)
            else:
                val = struct.unpack("<f", bytes(b))[0]
            self.results[mid][rid] = val
            n += 1
        return n

    def read(self, rids: list[int], settle: float = 0.10, retries: int = 4):
        """逐 RID 发查询。**未收到的一律重试** —— 实测丢帧率约 10~20%，
        只发一次会有随机空洞（第一版就漏了 ID2 的 VMAX 和 ID3 的 PMAX）。"""
        for rid in rids:
            for attempt in range(retries):
                missing = [m for m in self.ids if rid not in self.results[m]]
                if not missing:
                    break
                for mid in missing:
                    self.query(mid, rid)
                t_end = time.monotonic() + settle
                while time.monotonic() < t_end:
                    self.poll()
                    time.sleep(0.002)

    def close(self):
        self.sock.close()


def fmt(rid: int, v) -> str:
    if v is None:
        return "—"
    if isinstance(v, int):
        return f"{v}"
    if isinstance(v, float):
        if math.isnan(v):
            return "NaN（不支持）"
        if abs(v) < 1e-6 and v != 0:
            return f"{v:.3e}"
        return f"{v:.4f}"
    return str(v)


def main() -> int:
    ap = argparse.ArgumentParser(description="读达妙电机内部寄存器（经典 CAN）")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--all", action="store_true", help="读全部 82 个寄存器")
    ap.add_argument("--rid", default="", help="只读指定 RID，逗号分隔")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    if args.rid:
        rids = [int(x) for x in args.rid.split(",")]
    elif args.all:
        rids = sorted(RID_NAMES)
    else:
        rids = KEY_RIDS

    bus = ParamBus(args.interface, ids)
    print("=" * 78)
    print(f"  读电机寄存器  接口={args.interface}  电机={ids}")
    print("  只发 0x33 查询帧（0x7FF 广播），不使能、不改参数")
    print("=" * 78)

    try:
        bus.read(rids)
    finally:
        bus.close()

    for mid in ids:
        r = bus.results[mid]
        alive = bool(r)
        print(f"\n  ── 电机 ID {mid} (0x{mid:02X}) ──"
              f"{'' if alive else '   ❌ 无响应'}")
        if not alive:
            continue
        for rid in rids:
            name = RID_NAMES.get(rid, f"RID{rid}")
            mark = ""
            if rid == 23:            # TMAX
                mark = "   ★ MIT 12bit 力矩归一化用的量程"
            elif rid == 21:
                mark = "   ★ 位置量程"
            elif rid == 22:
                mark = "   ★ 速度量程"
            print(f"    [{rid:2d}] {name:10s} = {fmt(rid, r.get(rid)):>14s}{mark}")

    # 汇总 TMAX 对比表
    have = {m: bus.results[m].get(23) for m in ids}
    if any(v is not None and not (isinstance(v, float) and math.isnan(v))
           for v in have.values()):
        print("\n" + "=" * 78)
        print("  TMAX 汇总（电机固件里读到的 vs 我们代码里假设的）")
        print("=" * 78)
        # 2026-11-19 起代码里的值已改成实测值，这里用来做「回归检查」
        assumed = {1: 200.0, 2: 70.0, 3: 28.0, 4: 10.0, 5: 10.0, 6: 10.0}
        print(f"  {'ID':>4s}{'读到的 TMAX':>14s}{'代码里假设':>14s}{'差异':>12s}   结论")
        for m in ids:
            got = have.get(m)
            a = assumed.get(m)
            if got is None or (isinstance(got, float) and math.isnan(got)):
                print(f"  {m:4d}{'不受支持':>14s}{a:14.1f}{'—':>12s}   ⚠️ 只能是零/NaN，需标定")
                continue
            g = float(got)
            ratio = (a / g) if g else float("inf")
            ok = "✅ 一致" if abs(ratio - 1) < 0.05 else f"❌ 差 {ratio:.2f} 倍"
            print(f"  {m:4d}{g:14.2f}{a:14.1f}{ratio:12.2f}   {ok}")
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
