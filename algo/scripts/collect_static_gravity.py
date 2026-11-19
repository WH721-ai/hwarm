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


def can_txF(ifname: str = "can0") -> int:
    """读 /proc/net/can/stats 的真实发送计数（-1 = 读不到）。

    gs_usb 驱动不实时更新 /sys/class/net/*/statistics/tx_packets，
    只有这个计数器是同步累加的。
    """
    try:
        import re
        t = open("/proc/net/can/stats").read()
        return int(re.search(r"(\d+) transmitted frames", t).group(1))
    except Exception:  # noqa: BLE001
        return -1


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

    def fresh_q(self, ids: list[int], timeout: float = 1.0,
                need: int = 6) -> np.ndarray | None:
        """**读一份新鲜的位置**，读不到就返回 None。

        为什么必须有这一步：
            电机是"被动"的，不发查询就不回帧。`self.state` 里存的可能是
            **上一次**收到的值（甚至是初始化的 0）。
            用它当 MIT 的目标位 = 把机械臂往旧位置拉 —— 实测会导致
            "一按回车就以最高速度冲回零点"。
        """
        self.state_reset(ids)
        t_end = time.monotonic() + timeout
        counts = {i: 0 for i in ids}
        while time.monotonic() < t_end:
            self.query()
            self.poll()
            for i in ids:
                if self.state[i]["seen"]:
                    counts[i] += 1
            if all(counts[i] >= need for i in ids):
                return np.array([DIRECTION[i - 1] * self.state[i]["q"] for i in ids])
            time.sleep(0.003)
        return None

    def state_reset(self, ids: list[int]):
        for i in ids:
            self.state[i]["seen"] = False

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
                settle_s: float, sample_s: float, quiet: float = 0.02):
    """在当前姿态使能 → 等稳定 → 采样 → 失能。

    返回 (q, tau, dq_max)：
        q      关节角（关节坐标系，rad）
        tau    保持力矩（关节坐标系，N·m）
        dq_max 采样期间的|dq|最大值，用来判断"是不是真的静止"

    返回 None 表示采样失败（帧太少）。
    """
    # ★ 必须先读一份**新鲜**位置，不能用缓存的旧值（旧值会让机械臂冲回去）
    q0 = bus.fresh_q(ids, timeout=1.0)
    if q0 is None:
        print("     ❌ 读不到新鲜位置（CAN 丢帧太多），跳过这个姿态 —— 不使能")
        return None

    cmd = {i: (kp, kd, DIRECTION[i - 1] * q0[k], 0.0, 0.0)
           for k, i in enumerate(ids)}

    # ★ kp 软启动：0 → kp 用 0.3 s 爬上去，避免"一使能就顶一下"
    bus.enable()
    t_ramp = time.monotonic() + 0.3
    while time.monotonic() < t_ramp:
        a = max(0.0, min(1.0, (0.3 - (t_ramp - time.monotonic())) / 0.3))
        soft = {i: (kp * a, kd, DIRECTION[i - 1] * q0[k], 0.0, 0.0)
                for k, i in enumerate(ids)}
        bus.mit(soft)
        bus.poll()
        time.sleep(0.002)

    # ★ 超速看门狗：使能后 0.4 s 内任何关节 |dq| 超过 1.5 rad/s → 立刻失能
    t_wd = time.monotonic() + 0.4
    while time.monotonic() < t_wd:
        bus.mit(cmd)
        bus.poll()
        _, dq, _ = joint_frame(bus, ids)
        if np.abs(dq).max() > 1.5:
            bus.disable()
            time.sleep(0.1)
            print(f"     ❌ 使能后出现高速运动 |dq|max={np.abs(dq).max():.2f} rad/s，"
                  f"已紧急失能，跳过这个姿态")
            return None
        time.sleep(0.002)

    # ① 稳定阶段：让 PD 收敛（会有轻微下沉，这是正常的）
    t_end = time.monotonic() + settle_s
    while time.monotonic() < t_end:
        bus.mit(cmd)
        bus.poll()
        time.sleep(0.002)

    # ② 采样阶段：取中位数，抗单帧噪声
    qs, taus, dqs = [], [], []
    t_end = time.monotonic() + sample_s
    while time.monotonic() < t_end:
        bus.mit(cmd)
        bus.poll()
        q, dq, tau = joint_frame(bus, ids)
        qs.append(q)
        taus.append(tau)
        dqs.append(dq)
        time.sleep(0.002)

    bus.disable()
    time.sleep(0.10)

    if len(qs) < 10:
        return None
    qs, taus, dqs = np.array(qs), np.array(taus), np.array(dqs)
    return (np.median(qs, axis=0), np.median(taus, axis=0),
            float(np.abs(dqs).max()))


def main() -> int:
    ap = argparse.ArgumentParser(description="C-05 静态重力辨识数据采集")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--poses", type=int, default=25, help="要采多少个姿态")
    ap.add_argument("--mode", default="manual", choices=["manual", "auto"])
    ap.add_argument("--kp", type=float, default=25.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--settle", type=float, default=0.8,
                    help="使能后等 PD 收敛的秒数（默认 0.8）")
    ap.add_argument("--hold", type=float, default=0.8,
                    help="收敛后采样多少秒（默认 0.8）")
    ap.add_argument("--auto-step", type=float, default=5.0,
                    help="auto 模式每次每关节随机偏转的最大角度（度）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    out = args.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "data", f"static_gravity_{_dt.datetime.now():%Y%m%d_%H%M%S}.yaml")

    # ── 启动自检：CAN 到底能不能发出去 ──
    #
    # ⚠️ 必须读 /proc/net/can/stats 的 TXF，**不能**读
    #    /sys/class/net/*/statistics/tx_packets —— gs_usb 驱动的后者是
    #    惰性更新的（实测停在 32 不动，而 TXF 已经 175118）。
    #    之前用错计数器，把正常的总线误判成"适配器卡死"。
    _bus0 = ArmBus(args.interface, ids)
    t0 = can_txF()
    for _ in range(10):
        _bus0.query()
        time.sleep(0.010)
    time.sleep(0.15)
    t1 = can_txF()
    _bus0.close()
    if t0 < 0:
        print(f"  ⚠️ 读不到 /proc/net/can/stats，跳过发送自检")
    elif t1 <= t0:
        print("=" * 74)
        print(f"  ❌ {args.interface} **真的发不出帧**（TXF 停在 {t0}）")
        print("=" * 74)
        print("  脚本已自动**拒绝使能**（不会让机械臂动）。")
        print()
        print("  修复（需要密码）：")
        print(f"    bash hw_arm_ws/src/hw_arm_description/scripts/can_up.sh")
        print("  还不行就拔插 USB-CAN 适配器，再跑上面那条。")
        return 3
    else:
        print(f"  ✅ CAN 发送正常（TXF {t0} → {t1}）")

    bus = ArmBus(args.interface, ids)

    print("=" * 74)
    print("  C-05 静态重力辨识 —— 数据采集")
    print("=" * 74)
    print(f"  接口   : {args.interface}   关节: {ids}")
    print(f"  目标   : {args.poses} 个姿态")
    print(f"  模式   : {'手动摆位（每个姿态按回车）' if args.mode == 'manual' else '自动摆位'}")
    print(f"  kp/kd  : {args.kp} / {args.kd}   （很小，只是'扶住'）")
    print(f"  每个姿态: 稳定 {args.settle}s + 采样 {args.hold}s ≈ "
          f"{args.settle + args.hold + 0.2:.1f}s")
    print(f"  direction: {DIRECTION}")
    print()
    print("  ⚠️ 采集时电机会使能，但**只保持当前位置**，不会主动动")
    print("  ⚠️ Ctrl-C 随时安全退出（会先失能）")
    print("=" * 74)

    records = []
    try:
        if args.mode == "manual":
            print("\n  操作：用手把机械臂摆到一个新姿态 → 按回车 → 脚本扶住约 1.6 s 并记录")
            print("  （姿态差别越大越好：大臂抬高/放低、肘部弯/伸、腕部转）\n")
        else:
            print("\n  自动模式：从当前姿态出发，每次随机偏转 ±%.1f°\n" % args.auto_step)

        base_q = None
        for k in range(args.poses):
            if args.mode == "manual":
                try:
                    input(f"  [{k+1}/{args.poses}] 摆好姿态后按回车…")
                except EOFError:
                    print("\n  输入结束（非交互终端），停止采集")
                    break
            else:
                # 自动：在前一个姿态基础上随机偏一点
                if base_q is None:
                    base_q = bus.fresh_q(ids, timeout=1.0)
                    if base_q is None:
                        print("  ❌ 读不到新鲜位置，退出")
                        break
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

            res = acquire_one(bus, ids, args.kp, args.kd,
                              args.settle, args.hold)
            if res is None:
                print("     ⚠️ 采样失败（帧太少），跳过")
                continue
            q, tau, dq_max = res
            if dq_max > 0.05:
                print(f"     ⚠️ 没完全静止（|dq|max={dq_max:.3f} rad/s），"
                      f"这一条仍记录，但拟合时会体现为残差")

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
