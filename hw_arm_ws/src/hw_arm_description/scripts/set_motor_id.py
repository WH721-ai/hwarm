#!/usr/bin/env python3
"""修改 DM 电机的 CAN ID（ESC_ID / MST_ID）并写入 flash

═══════════════════════════════════════════════════════════════════
协议依据
═══════════════════════════════════════════════════════════════════
DM-J6216P 说明书 V1.5（2026-10-26）第 655~690 行：

  写入参数   0x7FF(STD) + [CANID_L, CANID_H, 0x55, RID, D4..D7]
             成功返回帧：MST_ID + 同样负载
             ⚠️ 写寄存器**立即生效但掉电丢失**

  存储参数   0x7FF(STD) + [CANID_L, CANID_H, 0xAA, 0x01]      ← 只有 4 字节
             成功返回帧：MST_ID + 同样负载
             ⚠️ 只在**失能模式**下生效
             ⚠️ 一次性保存全部参数，操作最长 30 ms

  RID 7 = MST_ID（回复 ID）    RID 8 = ESC_ID（接收 ID）

═══════════════════════════════════════════════════════════════════
⚠️ 使用前提（必须满足，否则会改错电机）
═══════════════════════════════════════════════════════════════════
  **总线上只能有一台电机在 --old-id 上。**

  两台同 ID 时，写 ESC_ID 会把两台一起改掉 —— 比不改更糟。
  本脚本会先连续多次读 SN 校验；不一致直接中止。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    # ① 先体检（不写，只看 old-id 上到底是什么电机）
    python3 set_motor_id.py --old-id 6

    # ② 确认无误后真的改：ID 6 → 7，Master 0x16 → 0x17
    python3 set_motor_id.py --old-id 6 --new-id 7 --new-master 0x17 --confirm

    # ③ 改完自己验一遍
    python3 set_motor_id.py --old-id 7

═══════════════════════════════════════════════════════════════════
操作顺序（脚本内部）
═══════════════════════════════════════════════════════════════════
    断电 → 只接要改的那台 → 上电 → 跑本脚本 → 断电 → 接回全部
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

BROADCAST_ID = 0x7FF          # 标准 11 位帧（不要加 EFF 标志）
RID_MST_ID = 7
RID_ESC_ID = 8
RID_SN = 15
RID_GR = 20
RID_TMAX = 23

INT_RIDS = {7, 8, 15, 16, 35, 36}


def open_bus(ifname: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((ifname,))
    s.settimeout(0.02)
    return s


def drain(s: socket.socket, sec: float = 0.02) -> None:
    t0 = time.time()
    while time.time() - t0 < sec:
        try:
            s.recv(16)
        except socket.timeout:
            break


def send(s: socket.socket, data: bytes) -> None:
    s.send(struct.pack("<IB3x8s", BROADCAST_ID, len(data), data.ljust(8, b"\x00")))


def query(s: socket.socket, motor_id: int, rid: int, deadline_ms: int = 400):
    """读单个寄存器；返回 (值, 原始 4 字节) 或 (None, None)

    ⚠️ 实测：电机空闲后**头 1~2 帧查询常常不回**（不只是丢帧 10~20%，
       而是"唤醒"需要几次）。所以这里按**截止时间**重试，不用固定次数。
    """
    payload = bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
    deadline = time.time() + deadline_ms / 1000.0
    while time.time() < deadline:
        send(s, payload)
        t0 = time.time()
        while time.time() - t0 < 0.03:
            try:
                raw = s.recv(16)
            except socket.timeout:
                break
            cid, dlc = struct.unpack_from("<IB", raw, 0)
            if cid == BROADCAST_ID or cid < 0x10:
                continue                     # 自己的 loopback / 无效
            data = raw[8:8 + dlc]
            if len(data) < 8 or data[2] != 0x33 or data[3] != rid:
                continue
            if (data[0] & 0x0F) != (motor_id & 0x0F):
                continue
            b = data[4:8]
            val = (b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)) if rid in INT_RIDS \
                else struct.unpack("<f", bytes(b))[0]
            return val, bytes(b)
    return None, None


def write_param(s: socket.socket, motor_id: int, rid: int, value: int,
                tries: int = 4) -> bool:
    """写一个 uint32 寄存器，**用回读验证**而不是靠回显

    ⚠️ 实测（DM-H3510）：写参数的成功回显**经常收不到**（发出去的帧有效、
       但回复帧丢了），可寄存器其实已经改了。
       如果只看回显判断，会误判成"写入失败" —— 我第一次改 ESC_ID 就是这样，
       工具报"没有回显，中止"，但电机已经跑到 ID 7 去了，状态不一致。
       所以这里写完一律**回读寄存器比对**。
    """
    payload = bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x55, rid]) + \
        struct.pack("<I", value & 0xFFFFFFFF)

    for _ in range(tries):
        send(s, payload)
        time.sleep(0.05)                     # 给电机时间处理
        got, _raw = query_target(s, motor_id, rid, value)
        if got:
            return True
        # 回读没对上，再来一次
    return False


def query_target(s: socket.socket, motor_id: int, rid: int, expect: int,
                 deadline_ms: int = 300) -> tuple[bool, int | None]:
    """回读某寄存器并与 expect 比对（只对 uint32 寄存器有效）"""
    deadline = time.time() + deadline_ms / 1000.0
    payload = bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
    while time.time() < deadline:
        send(s, payload)
        t0 = time.time()
        while time.time() - t0 < 0.03:
            try:
                raw = s.recv(16)
            except socket.timeout:
                break
            cid, dlc = struct.unpack_from("<IB", raw, 0)
            if cid == BROADCAST_ID or cid < 0x10:
                continue
            data = raw[8:8 + dlc]
            if len(data) < 8 or data[2] != 0x33 or data[3] != rid:
                continue
            if (data[0] & 0x0F) != (motor_id & 0x0F):
                continue
            b = data[4:8]
            val = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)
            return (val == (expect & 0xFFFFFFFF)), val
        time.sleep(0.01)
    return False, None


def store_params(s: socket.socket, motor_id: int, tries: int = 2) -> bool:
    """存储参数到 flash（4 字节帧，最长 30 ms）

    ⚠️ 说明书：flash 擦写次数约 1 万次，**不要频繁发送存储指令**。
       所以这里只重试 2 次，不要调大。
    """
    payload = bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xAA, 0x01])
    for _ in range(tries):
        send(s, payload)
        t0 = time.time()
        while time.time() - t0 < 0.10:      # 说明书说最长 30 ms，给足
            try:
                raw = s.recv(16)
            except socket.timeout:
                break
            cid, dlc = struct.unpack_from("<IB", raw, 0)
            if cid == BROADCAST_ID or cid < 0x10:
                continue
            data = raw[8:8 + dlc]
            if len(data) >= 4 and data[2] == 0xAA and data[3] == 0x01:
                if (data[0] & 0x0F) == (motor_id & 0x0F):
                    return True
        time.sleep(0.05)
    return False


def identify(s: socket.socket, motor_id: int, tag: str) -> dict | None:
    """连续读 SN，确认 old-id 上**只有一台**电机（一致性检查）"""
    sns, grs, tm = [], [], []
    for attempt in range(6):
        sn, _ = query(s, motor_id, RID_SN)
        gr, _ = query(s, motor_id, RID_GR)
        tx, _ = query(s, motor_id, RID_TMAX)
        if sn is None and gr is None and tx is None:
            time.sleep(0.05)          # 可能是唤醒中，再来一轮
            continue
        sns.append(sn)
        grs.append(gr)
        tm.append(tx)
        time.sleep(0.03)
        if len(sns) >= 2:
            break
    if not sns:
        return None

    unique = {v for v in sns if v is not None}
    if len(unique) > 1:
        print(f"  ❌ {tag}: ID {motor_id} 上读到 {unique} 个不同 SN —— **ID 冲突**！")
        print(f"     必须物理断开其中一台再改，否则会两台一起改。")
        return None

    esc, _ = query(s, motor_id, RID_ESC_ID)
    mst, _ = query(s, motor_id, RID_MST_ID)
    info = {
        "sn": sns[0] if sns else None,
        "gr": grs[0] if grs else None,
        "tmax": tm[0] if tm else None,
        "esc_id": esc,
        "mst_id": mst,
    }
    print(f"  {tag}: ID {motor_id}")
    print(f"       SN={info['sn']}  Gr={info['gr']}  TMAX={info['tmax']}  "
          f"ESC_ID={info['esc_id']}  MST_ID={info['mst_id']}")
    return info


def main() -> int:
    ap = argparse.ArgumentParser(description="修改 DM 电机 CAN ID（含 flash 存储）")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--old-id", type=int, required=True)
    ap.add_argument("--new-id", type=int, default=None,
                    help="新的 ESC_ID（接收 ID）")
    ap.add_argument("--new-master", default=None, type=lambda x: int(x, 0),
                    help="新的 MST_ID（回复 ID），支持 0x17 写法")
    ap.add_argument("--confirm", action="store_true",
                    help="真的写。不加只做体检")
    ap.add_argument("--skip-master", action="store_true",
                    help="只改 ESC_ID，不动 MST_ID")
    args = ap.parse_args()

    try:
        s = open_bus(args.interface)
    except OSError as e:
        print(f"❌ 打不开 {args.interface}: {e}")
        print(f"   先确认：ip -br link show {args.interface}")
        return 2

    print(f"════ 体检：看 ID {args.old_id} 上到底是什么 ════")
    drain(s, 0.05)
    info = identify(s, args.old_id, "当前")
    if info is None:
        print(f"\n  ❌ ID {args.old_id} 上读不到稳定的单台电机，中止。")
        s.close()
        return 1

    if not args.confirm:
        print("\n  （体检模式：没有写入。确认无误后加 --confirm）")
        s.close()
        return 0

    # ── 参数校验 ──
    if args.new_id is None:
        print("❌ --confirm 时必须给 --new-id")
        s.close()
        return 2
    if not (0 < args.new_id < 0x10):
        print(f"❌ new-id 必须在 1~15（实际 {args.new_id}）")
        s.close()
        return 2
    new_master = args.old_id  # 占位
    if not args.skip_master:
        if args.new_master is None:
            print("❌ 必须给 --new-master（如 0x17），或用 --skip-master")
            s.close()
            return 2
        new_master = args.new_master
        if not (0x10 <= new_master <= 0x7FE):
            print(f"❌ new-master 不合理：0x{new_master:X}")
            s.close()
            return 2

    print(f"\n════ 开始修改 ════")
    print(f"  ESC_ID: {args.old_id} → {args.new_id}")
    if not args.skip_master:
        print(f"  MST_ID: {info['mst_id']} → 0x{new_master:X}")

    # ── ① 写 ESC_ID（此时电机还在 old_id 上监听）──
    print(f"\n  ① 写 ESC_ID = {args.new_id}（发到 ID {args.old_id}）")
    if not write_param(s, args.old_id, RID_ESC_ID, args.new_id):
        print("     ❌ 回读校验没过，中止")
        print(f"        请先体检：python3 set_motor_id.py --old-id {args.old_id}")
        print(f"                   python3 set_motor_id.py --old-id {args.new_id}")
        s.close()
        return 1
    print("     ✅ 回读确认：ESC_ID =", args.new_id)
    time.sleep(0.05)
    drain(s, 0.05)

    # ── ② 写 MST_ID（★ 必须发到**新** ID）──
    if not args.skip_master:
        print(f"  ② 写 MST_ID = 0x{new_master:X}（发到新 ID {args.new_id}）")
        if not write_param(s, args.new_id, RID_MST_ID, new_master):
            print("     ❌ 回读校验没过")
            print(f"        电机现在在 ID {args.new_id} 上，可重试：")
            print(f"        python3 set_motor_id.py --old-id {args.new_id} "
                  f"--new-id {args.new_id} --new-master 0x{new_master:X} --confirm")
            s.close()
            return 1
        print(f"     ✅ 回读确认：MST_ID = 0x{new_master:X}")
        time.sleep(0.05)
        drain(s, 0.05)

    # ── ③ 存储到 flash（★ 必须发到新 ID）──
    print(f"  ③ 存储参数到 flash（发到 ID {args.new_id}）")
    if not store_params(s, args.new_id):
        print("     ⚠️ 没收到存储回显 —— **参数可能只存在 RAM，掉电会丢**")
        print("        电机必须是失能状态才能存储")
        rc = 1
    else:
        print("     ✅ 已写入 flash")
        rc = 0

    # ── ④ 验证 ──
    time.sleep(0.15)
    drain(s, 0.05)
    print(f"\n════ 验证 ════")
    after = identify(s, args.new_id, "改后")
    if after is None:
        print(f"  ❌ 新 ID {args.new_id} 读不到 —— 需要人工排查")
        s.close()
        return 1
    if info["sn"] is not None and after["sn"] is not None and after["sn"] != info["sn"]:
        print(f"  ❌ SN 变了（{info['sn']} → {after['sn']}）—— 改错了电机？")
        rc = 1

    old_still = identify(s, args.old_id, "旧 ID 残留检查")
    if old_still is not None and args.old_id != args.new_id:
        print(f"  ⚠️ 旧 ID {args.old_id} 上仍有电机应答（可能是另一台，也可能是没改成功）")

    s.close()
    print(f"\n{'✅ 完成' if rc == 0 else '⚠️ 完成但有问题'}")
    print(f"   断电 → 接回全部电机 → 扫描确认：")
    print(f"   python3 scan_motors.py --ids 1,2,3,4,5,6,7,8")
    return rc


if __name__ == "__main__":
    sys.exit(main())
