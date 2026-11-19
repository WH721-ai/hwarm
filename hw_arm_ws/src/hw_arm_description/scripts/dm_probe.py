#!/usr/bin/env python3
"""达妙电机 PC 侧快速自检工具（纯 Python，无 ROS、无第三方依赖）

用途：5 颗电机到手后，10 分钟内确认「线通了 / ID 对了 / 能转了」。

协议依据（与 openarm_can / 官方 dmbot_serial 一致）：
    发送：CAN ID = 电机 ID，8 字节
        MIT   : q(16) | dq(12) | kp(12) | kd(12) | tau(12)   —— 共 64 bit
        使能   : FF FF FF FF FF FF FF FC
        失能   : FF FF FF FF FF FF FF FD
        置零   : FF FF FF FF FF FF FF FE
    反馈：CAN ID = Master ID，8 字节
        D0 = 电机ID(低4位) | 错误码(高4位)
        q   = (D1<<8)|D2                    → [-pMax, pMax]
        dq  = (D3<<4)|(D4>>4)               → [-vMax, vMax]
        tau = ((D4&0xF)<<8)|D5              → [-tMax, tMax]
        D6 = MOS 温度, D7 = 转子温度

用法：
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

    python3 dm_probe.py --ids 1,2,3,4,5 scan
    python3 dm_probe.py --ids 1,2,3,4,5 enable
    python3 dm_probe.py --ids 1,2,3,4,5 hold  --kp 20 --kd 1.0        # 保持当前位置
    python3 dm_probe.py --ids 1,2,3,4,5 step  --joint 1 --target 0.5  # 单电机阶跃
    python3 dm_probe.py --ids 1,2,3,4,5 monitor                        # 只读不控
    python3 dm_probe.py --ids 1,2,3,4,5 disable

安全提示：第一次测试请把电机夹在台钳上、脱开负载；先 monitor 再 enable。
"""
from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

# 每种电机的量程 —— 必须与实际型号一致，否则解出来的数是错的
LIMITS = {
    # 型号: (pMax, vMax, tMax) —— 与 openarm_can / 达妙官方 SDK 的量程表一致
    "3507": (12.5, 50.0, 5.0),
    "4310": (12.5, 30.0, 10.0),
    "4340": (12.5, 10.0, 28.0),
    "6006": (12.5, 45.0, 20.0),
    "8006": (12.5, 45.0, 40.0),
    "8009": (12.5, 45.0, 54.0),
    "10010L": (12.5, 25.0, 200.0),   # joint1（hw_arm）
    "10010": (12.5, 20.0, 200.0),
    "3510": (12.5, 280.0, 1.0),
    "6215": (12.5, 45.0, 10.0),
    # ---- 以下两项 openarm_can 上游没有；量程**从电机固件直接读出**（2026-11-19）----
    # 读法：scripts/read_motor_registers.py（RID 21 PMAX / 22 VMAX / 23 TMAX）
    "6216P": (12.566, 40.0, 70.0),   # joint2 实测
    "4310P": (12.5, 30.0, 10.0),     # joint4 实测
}

ERR_NAMES = {
    0x0: "DISABLED", 0x1: "ENABLED", 0x8: "OVERVOLTAGE", 0x9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "ROTOR_OVERTEMP",
    0xD: "COMM_LOST", 0xE: "OVERLOAD", 0xF: "UNKNOWN",
}

CANFD_MTU = 72


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = min(max(x, x_min), x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(u: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return u * span / ((1 << bits) - 1) + x_min


class CanFD:
    """CANFD 原始套接字封装（canfd_frame: <IBBBB64s, 72 字节）。"""

    def __init__(self, ifname: str = "can0", fd: bool = True):
        self.fd = fd
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        if fd:
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        self.sock.bind((ifname,))
        self.sock.settimeout(0.002)

    def send(self, can_id: int, data: bytes):
        # ⚠️ 经典 CAN 必须发 16 字节的 can_frame，不能发 72 字节的 canfd_frame
        #    —— 往经典模式接口发 canfd_frame 会 OSError: [Errno 22] Invalid argument
        if self.fd:
            frame = struct.pack("<IBBBB64s", can_id, len(data), 0, 0, 0,
                                data.ljust(64, b"\x00"))
        else:
            # struct can_frame: can_id(4) + dlc(1) + pad(3) + data[8]
            frame = struct.pack("<IB3x8s", can_id, len(data),
                                data.ljust(8, b"\x00"))
        self.sock.send(frame)

    def recv_all(self, timeout: float = 0.005):
        """收一波帧，返回 [(can_id, data_bytes), ...]。"""
        out, t_end = [], time.monotonic() + timeout
        while time.monotonic() < t_end:
            try:
                raw = self.sock.recv(CANFD_MTU if self.fd else 16)
            except (socket.timeout, BlockingIOError):
                break
            can_id, length = struct.unpack("<IB", raw[:5])
            out.append((can_id & 0x1FFFFFFF, raw[8:8 + length]))
        return out


def pack_mit(kp, kd, q, dq, tau, limits) -> bytes:
    p_max, v_max, t_max = limits
    q_u = float_to_uint(q, -p_max, p_max, 16)
    dq_u = float_to_uint(dq, -v_max, v_max, 12)
    kp_u = float_to_uint(kp, 0.0, 500.0, 12)
    kd_u = float_to_uint(kd, 0.0, 5.0, 12)
    tau_u = float_to_uint(tau, -t_max, t_max, 12)
    return bytes([
        (q_u >> 8) & 0xFF,
        q_u & 0xFF,
        (dq_u >> 4) & 0xFF,
        ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
        kp_u & 0xFF,
        (kd_u >> 4) & 0xFF,
        ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
        tau_u & 0xFF,
    ])


def decode_feedback(data: bytes, limits):
    err = data[0] >> 4
    q_u = (data[1] << 8) | data[2]
    dq_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0xF) << 8) | data[5]
    p_max, v_max, t_max = limits
    return {
        "err": err,
        "q": uint_to_float(q_u, -p_max, p_max, 16),
        "dq": uint_to_float(dq_u, -v_max, v_max, 12),
        "tau": uint_to_float(tau_u, -t_max, t_max, 12),
        "t_mos": data[6],
        "t_rotor": data[7],
    }


class Bus:
    def __init__(self, ifname, motor_type, ids, master_offset=0x10, fd=True):
        self.can = CanFD(ifname, fd)
        self.limits = LIMITS[motor_type]
        self.ids = ids
        self.master = {i: i + master_offset for i in ids}
        self.state = {i: {"q": 0.0, "dq": 0.0, "tau": 0.0, "err": 0,
                          "t_mos": 0, "t_rotor": 0} for i in ids}
        self.other_frames = []
        self.dropped = 0        # 因 TX 队列满而丢弃的查询帧数

    def _special(self, cmd: int):
        for i in self.ids:
            self.can.send(i, b"\xff" * 7 + bytes([cmd]))

    def enable(self):
        self._special(0xFC)

    def disable(self):
        self._special(0xFD)

    def set_zero(self):
        self._special(0xFE)

    def mit(self, cmd: dict):
        """cmd[motor_id] = (kp, kd, q, dq, tau)"""
        for i in self.ids:
            kp, kd, q, dq, tau = cmd[i]
            self.can.send(i, pack_mit(kp, kd, q, dq, tau, self.limits))

    def poll(self, timeout=0.005):
        """收帧并按电机归属解码。

        归属判定有两条路径，任一命中即可：
          1) CAN ID == 该电机的 Master ID（OpenArm 约定：slave+0x10）
          2) 反馈帧 D0 低 4 位 == 电机 ID（达妙协议里 D0 = ID | ERR<<4）

        路径 2 是为了兼容「电机 Master ID 还是出厂默认 0x00（广播）」的情况——
        第一次接线时不用先去改 Master ID 也能看到反馈。ID 1~6 都小于 16，安全。
        """
        for can_id, data in self.can.recv_all(timeout):
            if len(data) < 8:
                continue
            owner = data[0] & 0x0F
            target = None
            for i in self.ids:
                if can_id == self.master[i]:
                    target = i
                    break
            if target is None and owner in self.ids:
                target = owner
            if target is None:
                self.other_frames.append((can_id, bytes(data)))
                continue
            st = decode_feedback(data, self.limits)
            st["seen"] = True
            st["can_id"] = can_id
            self.state[target] = st

    def query(self):
        """广播刷新帧 —— 让电机主动回一帧状态。

        ⚠️ 达妙电机是**被动**的：不被查询就不发帧。
           所以 monitor/scan 必须先发一帧查询，否则永远收到 0 台。
        帧格式（官方 SDK）：CAN ID = 0x7FF，dlc=4，[id_lo, id_hi, 0xCC, 0x00]
        发到广播地址，总线上任何电机都会 ACK；目标电机认领 ID 后回状态。
        """
        for i in self.ids:
            try:
                self.can.send(0x7FF,
                              bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0x00]))
            except OSError as e:
                # ENOBUFS(105)：CAN 发送队列满（默认 txqueuelen=10）。
                # 查询帧丢了就丢了，下一轮会重发，不要让整个脚本崩掉。
                if e.errno == 105:
                    self.dropped += 1
                    continue
                raise

    def wait_first_state(self, timeout=1.0, resend=0.1) -> int:
        t_end = time.monotonic() + timeout
        next_q = 0.0
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= next_q:          # 限速：每 100 ms 发一轮查询
                self.query()
                next_q = now + resend
            self.poll(0.02)
            n = sum(1 for i in self.ids if self.state[i].get("seen"))
            if n == len(self.ids):
                break
        return sum(1 for i in self.ids if self.state[i].get("seen"))

    def print_table(self):
        print(f"{'id':>3s} {'CANID':>6s} {'pos(rad)':>9s} {'vel(r/s)':>9s} {'tau(Nm)':>8s} "
              f"{'Tmos':>5s} {'Trot':>5s}  state")
        for i in self.ids:
            s = self.state[i]
            mark = "" if s.get("seen") else "  <-- 无反馈"
            cid = f"0x{s['can_id']:02X}" if "can_id" in s else "-"
            print(f"{i:3d} {cid:>6s} {s['q']:9.3f} {s['dq']:9.3f} {s['tau']:8.3f} "
                  f"{s['t_mos']:5d} {s['t_rotor']:5d}  "
                  f"{ERR_NAMES.get(s['err'], hex(s['err']))}{mark}")
        if self.other_frames:
            seen = sorted({cid for cid, _ in self.other_frames})
            print(f"  （另有 {len(self.other_frames)} 帧来自未知 CAN ID: "
                  f"{', '.join(f'0x{c:02X}' for c in seen[:6])}）")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5", help="电机 ID，逗号分隔")
    ap.add_argument("--type", default="4310", choices=sorted(LIMITS), help="电机型号")
    ap.add_argument("--no-fd", action="store_true", help="经典 CAN（默认 CANFD）")
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--joint", type=int, default=None, help="step 模式的目标电机 ID")
    ap.add_argument("--target", type=float, default=0.3, help="step 目标位置 (rad)")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("cmd", choices=["scan", "enable", "disable", "zero", "monitor", "hold", "step"])
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    try:
        bus = Bus(args.interface, args.type, ids, fd=not args.no_fd)
    except OSError as e:
        print(f"打不开 {args.interface}: {e}")
        print("先确认接口存在并已启动：")
        print(f"  sudo ip link set {args.interface} up type can bitrate 1000000 "
              f"dbitrate 5000000 fd on")
        return 1

    print(f"接口 {args.interface} / 型号 DM{args.type} / ID {ids} / "
          f"{'CANFD' if not args.no_fd else '经典CAN'}")

    if args.cmd in ("scan", "monitor"):
        n = bus.wait_first_state(1.5)
        print(f"\n收到 {n}/{len(ids)} 台的反馈")
        bus.print_table()
        return 0 if n else 2

    if args.cmd == "zero":
        bus.set_zero()
        time.sleep(0.2)
        print("已发送置零（0xFE）")
        return 0

    if args.cmd in ("enable", "disable"):
        (bus.enable if args.cmd == "enable" else bus.disable)()
        time.sleep(0.3)
        bus.poll(0.05)
        bus.print_table()
        return 0

    # hold / step：先使能，再用 MIT 保持或阶跃
    bus.enable()
    time.sleep(0.3)
    bus.poll(0.05)
    if not bus.wait_first_state(1.0):
        print("❌ 没有任何电机反馈，检查 ID / 接线 / 波特率")
        return 2
    print("\n使能后状态：")
    bus.print_table()

    hold = {i: bus.state[i]["q"] for i in ids}
    print(f"\n开始 {args.cmd}：kp={args.kp} kd={args.kd}，{args.seconds:.1f} s")
    print("（Ctrl-C 中止，会自动失能）\n")

    t0, period = time.monotonic(), 0.002          # 500 Hz
    try:
        while time.monotonic() - t0 < args.seconds:
            t = time.monotonic() - t0
            cmd = {}
            for i in ids:
                q_des = hold[i]
                if args.cmd == "step" and args.joint == i:
                    # 1 s 内线性到目标位置
                    q_des = hold[i] + (args.target / max(1.0, 1.0)) * min(t, 1.0)
                cmd[i] = (args.kp, args.kd, q_des, 0.0, 0.0)
            bus.mit(cmd)
            bus.poll(0.0015)
            if int(t * 500) % 100 == 0:
                row = "  ".join(f"{i}:{bus.state[i]['q']:+.3f}" for i in ids)
                print(f"\r t={t:4.2f}s  {row}", end="", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0 - t) % period))
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        bus.disable()
        time.sleep(0.2)
        bus.poll(0.05)
        print("已失能，最终状态：")
        bus.print_table()
    return 0


if __name__ == "__main__":
    sys.exit(main())
