#!/usr/bin/env python3
"""C-05 静态重力辨识 —— 数据采集（真机）

═══════════════════════════════════════════════════════════════════
原理
═══════════════════════════════════════════════════════════════════
静止时（dq = ddq = 0）电机的输出力矩**只有一项**：顶住重力。

    tau = Y_g(q) · theta_g

`Y_g(q)` 是 Pinocchio 的重力回归矩阵（只跟姿态有关），
`theta_g` 是惯性参数的线性组合。

**关键优化：不需要激励轨迹。** 只要把机械臂摆到 20~30 个不同姿态，
每个姿态静止保持 1 秒，读「关节角 + 保持力矩」，就能解最小二乘。

  ✓ 不用动起来 → 不用清场地、不用大行程
  ✓ 每个姿态只保持 1 秒 → 电机不发热
  ✓ 姿态可以离得很近（差 5° 也算独立样本）

⚠️ 但静态数据**只能辨识质量与质心（一阶矩 m·c）**，辨识不了惯量张量 ——
   惯量只在有加速度时才体现。这是方法本身的限制，不是 bug。

═══════════════════════════════════════════════════════════════════
安全
═══════════════════════════════════════════════════════════════════
    · 采集时电机会使能（kp 很小，默认 25），**保持当前位置不动**
    · 每个姿态采完立刻失能
    · Ctrl-C 任何时候都安全退出（会先失能）
    · manual 模式：机械臂全程由你用**手**摆位，脚本只在按下回车后
      "扶住"它 1.2 秒

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    # 手动摆位（推荐第一次用）：每个姿态摆好后按回车
    python3 algo/scripts/collect_static_gravity.py --poses 25

    # 自动摆位：从当前姿态出发，每次随机偏 ±5°，自动采 25 个点
    python3 algo/scripts/collect_static_gravity.py --mode auto --poses 25

    # 只采 5 个点先试试流程
    python3 algo/scripts/collect_static_gravity.py --poses 5

输出：algo/data/static_gravity_<时间戳>.yaml

前置：can0 已配置为经典 CAN 1 Mbps 且 UP；六台电机均已置零。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import math
import os
import socket
import struct
import sys
import time

import numpy as np

# ──────────────────────────────────────────────────────────────
# 配置
# ──────────────────────────────────────────────────────────────
P_MAX = 12.5

# 每台电机的 (pMax, vMax, tMax)。tMax 决定 12 bit 力矩字段的归一化，
# 2026-11-19：全部从电机固件 RID21/22/23 读出（scripts/read_motor_registers.py），
#            不再是占位值。DM6216P 的 PMAX 是 12.566（不是 12.5，别忽略）。
LIMITS = {
    1: (12.5, 25.0, 200.0),   # DM10010L
    2: (12.566, 40.0, 70.0),  # DM6216P  ← 固件实测（RID21/22/23）
    3: (12.5, 10.0, 28.0),    # DM4340
    4: (12.5, 30.0, 10.0),    # DM4310P  ← 固件实测
    5: (12.5, 30.0, 10.0),    # DM4310
    6: (12.5, 30.0, 10.0),    # DM4310
}

# 电机转向（C-02 实测）：电机坐标系 → 关节坐标系
DIRECTION = [+1, -1, -1, +1, +1, +1]

# 关节限位（URDF 约定，rad）
LIMITS_Q = {
    1: (-1.3963, 3.4907),
    2: (-0.17453, 3.3161),
    3: (0.0, 2.4435),
    4: (-1.5708, 1.5708),
    5: (-0.7854, 0.7854),
    6: (-1.5708, 1.5708),
}

BROADCAST_ID = 0x7FF


def uint_to_float(u: int, lo: float, hi: float, bits: int) -> float:
    return u * (hi - lo) / ((1 << bits) - 1) + lo


# ──────────────────────────────────────────────────────────────
# CAN
# ──────────────────────────────────────────────────────────────
class ArmBus:
    """只做三件事：发 MIT 指令 / 发广播查询 / 收状态。"""

    def __init__(self, ifname: str, ids: list[int]):
        self.ids = ids
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.setblocking(False)
        self.state = {i: {"q": 0.0, "dq": 0.0, "tau": 0.0, "t_mos": 0,
                          "t_rotor": 0, "err": 0, "seen": False} for i in ids}
        self.dropped = 0

    # ── 发送 ──────────────────────────────────────────────────
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

    def query(self):
        for i in self.ids:
            self._send(BROADCAST_ID, bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0x00]))

    def enable(self):
        for i in self.ids:
            self._send(i, b"\xff" * 7 + b"\xfc")

    def disable(self):
        for i in self.ids:
            self._send(i, b"\xff" * 7 + b"\xfd")

    def mit(self, cmd: dict):
        """cmd[id] = (kp, kd, q, dq, tau) —— 全部在**电机坐标系**"""
        for i, (kp, kd, q, dq, tau) in cmd.items():
            p_max, v_max, t_max = LIMITS[i]

            def f2u(x, lo, hi, bits):
                x = min(max(x, lo), hi)
                return int((x - lo) * ((1 << bits) - 1) / (hi - lo))

            q_u = f2u(q, -p_max, p_max, 16)
            dq_u = f2u(dq, -v_max, v_max, 12)
            kp_u = f2u(kp, 0.0, 500.0, 12)
            kd_u = f2u(kd, 0.0, 5.0, 12)
            tau_u = f2u(tau, -t_max, t_max, 12)
            data = bytes([
                (q_u >> 8) & 0xFF, q_u & 0xFF,
                (dq_u >> 4) & 0xFF, ((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
                kp_u & 0xFF,
                (kd_u >> 4) & 0xFF, ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F),
                tau_u & 0xFF,
            ])
            self._send(i, data)

    # ── 接收 ──────────────────────────────────────────────────
    def poll(self) -> int:
        n = 0
        while True:
            try:
                raw = self.sock.recv(16)
            except (BlockingIOError, TimeoutError):
                break
            if len(raw) < 16:
                continue
            can_id, dlc = struct.unpack_from("<IB", raw, 0)
            data = raw[8:8 + dlc]
            if len(data) < 8:
                continue
            mid = data[0] & 0x0F
            if mid not in LIMITS or mid not in self.ids:
                continue
            p_max, v_max, t_max = LIMITS[mid]
            self.state[mid].update(
                q=uint_to_float((data[1] << 8) | data[2], -p_max, p_max, 16),
                dq=uint_to_float((data[3] << 4) | (data[4] >> 4), -v_max, v_max, 12),
                tau=uint_to_float(((data[4] & 0x0F) << 8) | data[5], -t_max, t_max, 12),
                t_mos=data[6], t_rotor=data[7], err=data[0] >> 4, seen=True,
            )
            n += 1
        return n

    def settle(self, seconds: float):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.query()
            self.poll()
            time.sleep(0.005)

    def close(self):
        self.sock.close()


# ──────────────────────────────────────────────────────────────
def joint_frame(bus: ArmBus, ids: list[int]):
    """电机坐标系 → 关节坐标系（q、tau 都乘 direction）"""
    q = np.array([DIRECTION[i - 1] * bus.state[i]["q"] for i in ids])
    tau = np.array([DIRECTION[i - 1] * bus.state[i]["tau"] for i in ids])
    dq = np.array([DIRECTION[i - 1] * bus.state[i]["dq"] for i in ids])
    return q, dq, tau


def acquire_one(bus: ArmBus, ids: list[int], kp: float, kd: float,
                hold_s: float, settle_s: float):
    """在当前姿态使能 → 等稳定 → 采样 → 失能。返回 (q, tau) 或 None。"""
    q0, _, _ = joint_frame(bus, ids)

    # 目标 = 当前位置（**不动**），只是"扶住"
    cmd = {i: (kp, kd, DIRECTION[i - 1] * q0[k], 0.0, 0.0)
           for k, i in enumerate(ids)}
    bus.enable()
    time.sleep(0.1)

    t_end = time.monotonic() + hold_s
    samples = []
    while time.monotonic() < t_end:
        bus.mit(cmd)
        bus.poll()
        time.sleep(0.002)
        samples.append((time.monotonic(),))

    # 稳定后采样（后半段）
    time.sleep(settle_s)
    qs, taus = [], []
    t_end = time.monotonic() + hold_s
    while time.monotonic() < t_end:
        bus.mit(cmd)
        bus.poll()
        q, dq, tau = joint_frame(bus, ids)
        qs.append(q)
        taus.append(tau)
        time.sleep(0.002)

    bus.disable()
    time.sleep(0.15)

    if len(qs) < 5:
        return None
    qs = np.array(qs)
    taus = np.array(taus)
    return np.median(qs, axis=0), np.median(taus, axis=0)


def main() -> int:
    ap = argparse.ArgumentParser(description="C-05 静态重力辨识数据采集")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--poses", type=int, default=25, help="要采多少个姿态")
    ap.add_argument("--mode", default="manual", choices=["manual", "auto"])
    ap.add_argument("--kp", type=float, default=25.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=1.2, help="每个姿态保持秒数")
    ap.add_argument("--auto-step", type=float, default=5.0,
                    help="auto 模式每次每关节随机偏转的最大角度（度）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    out = args.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "data", f"static_gravity_{_dt.datetime.now():%Y%m%d_%H%M%S}.yaml")

    bus = ArmBus(args.interface, ids)

    print("=" * 74)
    print("  C-05 静态重力辨识 —— 数据采集")
    print("=" * 74)
    print(f"  接口   : {args.interface}   关节: {ids}")
    print(f"  目标   : {args.poses} 个姿态")
    print(f"  模式   : {'手动摆位（每个姿态按回车）' if args.mode == 'manual' else '自动摆位'}")
    print(f"  kp/kd  : {args.kp} / {args.kd}   （很小，只是'扶住'）")
    print(f"  direction: {DIRECTION}")
    print()
    print("  ⚠️ 采集时电机会使能，但**只保持当前位置**，不会主动动")
    print("  ⚠️ Ctrl-C 随时安全退出（会先失能）")
    print("=" * 74)

    records = []
    try:
        if args.mode == "manual":
            print("\n  操作：用手把机械臂摆到一个新姿态 → 按回车 → 脚本扶住 1.2 s 并记录")
            print("  （姿态差别越大越好：大臂抬高/放低、肘部弯/伸、腕部转）\n")
        else:
            print("\n  自动模式：从当前姿态出发，每次随机偏转 ±%.1f°\n" % args.auto_step)

        base_q = None
        for k in range(args.poses):
            if args.mode == "manual":
                input(f"  [{k+1}/{args.poses}] 摆好姿态后按回车…")
            else:
                # 自动：在前一个姿态基础上随机偏一点
                bus.settle(0.05)
                if base_q is None:
                    bus.settle(0.3)
                    base_q, _, _ = joint_frame(bus, ids)
                step = np.radians(args.auto_step)
                target = base_q + np.random.uniform(-step, step, len(ids))
                for m, i in enumerate(ids):
                    lo, hi = LIMITS_Q[i]
                    target[m] = float(np.clip(target[m], lo + 0.02, hi - 0.02))
                cmd = {i: (args.kp, args.kd, DIRECTION[i - 1] * target[m], 0.0, 0.0)
                       for m, i in enumerate(ids)}
                bus.enable()
                t_mv = time.monotonic() + 0.6
                while time.monotonic() < t_mv:
                    bus.mit(cmd)
                    bus.poll()
                    time.sleep(0.002)
                bus.disable()
                time.sleep(0.15)
                base_q = target

            res = acquire_one(bus, ids, args.kp, args.kd, args.hold, 0.10)
            if res is None:
                print("     ⚠️ 采样失败，跳过")
                continue
            q, tau = res
            records.append({"q": q.tolist(), "tau": tau.tolist()})
            print(f"     q(deg) = {np.round(np.degrees(q), 1)}")
            print(f"     tau    = {np.round(tau, 3)} N·m")
    except KeyboardInterrupt:
        print("\n  中断，先失能…")
    finally:
        bus.disable()
        time.sleep(0.2)
        bus.close()

    if len(records) < 6:
        print(f"\n❌ 只采到 {len(records)} 个姿态，至少需要 6 个才能解")
        return 2

    os.makedirs(os.path.dirname(out), exist_ok=True)
    import yaml
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump({
            "meta": {
                "created": _dt.datetime.now().isoformat(timespec="seconds"),
                "interface": args.interface, "ids": ids,
                "direction": DIRECTION, "kp": args.kp, "kd": args.kd,
                "mode": args.mode, "n_poses": len(records),
                "note": "q / tau 均为**关节坐标系**（已乘 direction）。"
                        "DM6216P/DM4310P 的 tMax 是占位值，力矩可能有系统偏差",
            },
            "samples": records,
        }, f, allow_unicode=True, sort_keys=False)

    print(f"\n✅ 采到 {len(records)} 个姿态 → {out}")
    print(f"   下一步： python3 algo/scripts/fit_static_gravity.py {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
