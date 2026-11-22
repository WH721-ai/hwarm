#!/usr/bin/env python3
"""按**全精度**读电机寄存器（read_motor_registers.py 只显示 4 位小数）

为什么需要这个脚本：
    RID 12「电机转动惯量」的量级是 1e-5 ~ 1e-3 kg·m²，
    `%.4f` 会把 1e-5 显示成 0.0000 —— 看起来像"没数据"，其实是精度不够。
    这里直接打印 float32 的完整有效数字（%.9g）和原始 4 字节。

用法：
    source /opt/ros/jazzy/setup.bash
    python3 dump_register_f32.py --ids 1,2,3,4,5,6 --rid 12
    python3 dump_register_f32.py --ids 1 --rid 11          # 粘滞系数
    python3 dump_register_f32.py --ids 1 --rid 12 --raw    # 附带十六进制

协议：
    请求  0x7FF + [id_lo, id_hi, 0x33, RID, 0,0,0,0]
    应答  CAN ID = master_id(=id+0x10)，data[0]=RID，data[1:5]=float LE
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

CAN_EFF_FLAG = 0x80000000


def request_frame(motor_id: int, rid: int) -> bytes:
    return bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])


def query(sock: socket.socket, motor_id: int, rid: int, tries: int = 6,
          timeout: float = 0.05):
    """返回 (float32 值, 原始 4 字节) 或 (None, None)"""
    payload = request_frame(motor_id, rid)
    can_id = 0x7FF | CAN_EFF_FLAG
    frame = struct.pack("<IB3x8s", can_id, 8, payload)

    for _ in range(tries):
        sock.send(frame)
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                raw = sock.recv(16)
            except socket.timeout:
                break
            cid, dlc = struct.unpack_from("<IB", raw, 0)
            # 应答来自 master_id = motor_id + 0x10
            if (cid & 0x1FFFFFFF) != (0x10 + motor_id) or dlc < 5:
                continue
            data = raw[8:8 + dlc]
            if data[0] != rid:
                continue
            return struct.unpack("<f", bytes(data[1:5]))[0], bytes(data[1:5])
    return None, None


def main() -> int:
    ap = argparse.ArgumentParser(description="按全精度读电机寄存器")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6", help="电机 ID，逗号分隔")
    ap.add_argument("--rid", type=int, required=True, help="寄存器号（12=转动惯量，11=粘滞）")
    ap.add_argument("--raw", action="store_true", help="附带打印原始 4 字节（十六进制）")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.interface,))
    except OSError as e:
        print(f"❌ 打不开 {args.interface}：{e}")
        print("   先确认接口在：ip -br link show can0")
        return 2

    sock.settimeout(0.02)

    print(f"  读 RID {args.rid}（接口 {args.interface}，全精度 float32）")
    print(f"  {'ID':>4s}{'值 (%.9g)':>22s}{'float32 repr':>18s}   原始字节")
    print("  " + "-" * 66)

    miss = []
    for mid in ids:
        val, raw = query(sock, mid, args.rid)
        if val is None:
            miss.append(mid)
            print(f"  {mid:4d}{'无响应':>22s}")
            continue
        hexs = " ".join(f"{b:02x}" for b in raw) if args.raw else ""
        print(f"  {mid:4d}{val:22.9g}{hexs:>20s}")

    sock.close()

    if miss:
        print(f"\n  ⚠️ 无响应：{miss}")
        print("     常见原因：电机没上电 / 该 RID 只写不读 / 总线被占用")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
