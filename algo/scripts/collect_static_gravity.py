#!/usr/bin/env python3
"""C-05 静态重力辨识 —— 数据采集（重写版）

═══════════════════════════════════════════════════════════════════
这个脚本之前被打了太多补丁，积累了一堆坑，重写一遍把设计讲清楚。
═══════════════════════════════════════════════════════════════════

原理
    静止时 dq = ddq = 0，电机输出力矩只剩一项：顶住重力
        tau = Y_g(q) · θ
    把机械臂摆到 20~30 个不同姿态，每处静止保持一下，读 (q, tau)，
    解最小二乘。**不需要激励轨迹。**

为什么使能时必须加"重力前馈"
    只用位置环（kp·误差）托住，kp=25 时 joint2/3 会下垂 5~6°；
    更糟的是"kp 从 0 软启动"会让机械臂在重力下自由落体
    （实测估算 0.3 s 掉 68°）—— 这就是"一按回车就冲回去"的真正原因。
    → 用标称模型算 tau_ff = g(q0) 一起下发，机械臂几乎不动。

为什么测到的力矩要"扣掉 PD 贡献"
    电机报告的 tau 是**总输出** = tau_ff + kp·误差 + kd·速度 + 外力矩
    所以：  外力矩 = tau_meas − tau_ff − kp·(q_des − q) − kd·(0 − dq)
    扣掉后剩下的才是"重力 + 摩擦"，这才是要拟合的量。

为什么要把"压在支撑上"的样本挑出来
    机械臂靠在桌面上时，重力被结构吃掉，电机力矩接近 0 ——
    这种样本没有信息量，还会把参数带偏。脚本会检测并标记。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    source /opt/ros/jazzy/setup.bash        # 需要 pinocchio

    # ① 干跑：只读不发控制帧，确认读数和模型
    python3 algo/scripts/collect_static_gravity.py --dry-run

    # ② 采 3 个试试（把大臂抬起来，别压在支撑上）
    python3 algo/scripts/collect_static_gravity.py --poses 3 -v

    # ③ 正式采集
    python3 algo/scripts/collect_static_gravity.py --poses 25

═══════════════════════════════════════════════════════════════════
每个姿态的流程
═══════════════════════════════════════════════════════════════════
    1. 读**新鲜**位置 q0（连续查询，读不到就跳过，绝不使能）
    2. 算 tau_ff = g(q0)
    3. 使能，第一帧就发满 kp + 满前馈（**不做 kp 软启动**）
    4. 等真正静止：连续 25 次 |dq| < dq_tol
    5. 采样 sample 秒，取中位数
    6. 失能
    7. 扣掉 PD 贡献 → 外力矩；和标称模型对比，偏差过大就警告

前置：can0 已配置（scripts/can_up.sh），六台电机已置零。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import re
import socket
import struct
import sys
import time

import numpy as np

# ══════════════════════════════════════════════════════════════
# 常量
# ══════════════════════════════════════════════════════════════

# 每台电机 (pMax, vMax, tMax)。tMax 决定 12 bit 力矩字段的归一化。
# 2026-11-19 从固件 RID21/22/23 读出（scripts/read_motor_registers.py）
LIMITS = {
    1: (12.5,   25.0, 200.0),   # DM10010L
    2: (12.566, 40.0,  70.0),   # DM6216P
    3: (12.5,   10.0,  28.0),   # DM4340
    4: (12.5,   30.0,  10.0),   # DM4310P
    5: (12.5,   30.0,  10.0),   # DM4310
    6: (12.5,   30.0,  10.0),   # DM4310
}

# 电机转向（C-02 实测，mirror_check.py 验证过）
DIRECTION = [+1, -1, -1, +1, +1, +1]

# 关节限位（URDF 约定，rad）
LIMITS_Q = {
    1: (-1.3963, 3.4907), 2: (-0.17453, 3.3161), 3: (0.0, 2.4435),
    4: (-1.5708, 1.5708), 5: (-0.7854, 0.7854), 6: (-1.5708, 1.5708),
}

BROADCAST_ID = 0x7FF


# ══════════════════════════════════════════════════════════════
# 小工具
# ══════════════════════════════════════════════════════════════
def can_txF() -> int:
    """真实发送计数（-1 = 读不到）。

    ⚠️ 不能用 /sys/class/net/*/statistics/tx_packets —— gs_usb 惰性更新，
       曾经据此把正常总线误判成"适配器卡死"。
    """
    try:
        t = open("/proc/net/can/stats").read()
        return int(re.search(r"(\d+) transmitted frames", t).group(1))
    except Exception:  # noqa: BLE001
        return -1


def u2f(u: int, lo: float, hi: float, bits: int) -> float:
    return u * (hi - lo) / ((1 << bits) - 1) + lo


def f2u(x: float, lo: float, hi: float, bits: int) -> int:
    x = min(max(x, lo), hi)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def urdf_candidates() -> list[str]:
    here = os.path.dirname(os.path.abspath(__file__))        # .../algo/scripts
    roots = [
        os.path.dirname(os.path.dirname(here)),              # .../hw_arm
        os.path.dirname(os.path.dirname(os.path.dirname(here))),
        os.getcwd(),
        os.path.expanduser("~/hw_arm"),
    ]
    return [os.path.join(r, "hw_arm_ws", "install", "hw_arm_description",
                         "share", "hw_arm_description", "urdf", "hw_arm.urdf")
            for r in roots]


# ══════════════════════════════════════════════════════════════
# CAN
# ══════════════════════════════════════════════════════════════
class ArmBus:
    """MIT 指令 + 广播查询，只做这一件事。"""

    def __init__(self, ifname: str, ids: list[int]):
        self.ids = ids
        if not os.path.exists(f"/sys/class/net/{ifname}"):
            raise FileNotFoundError(
                f"接口 {ifname} 不存在 —— USB-CAN 适配器没插，或者插了但没枚举\n"
                f"  插好后跑： bash hw_arm_ws/src/hw_arm_description/scripts/can_up.sh")
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.setblocking(False)
        self.dropped = 0
        self.other = 0
        self.reset_state()

    def reset_state(self):
        self.q = {i: 0.0 for i in self.ids}
        self.dq = {i: 0.0 for i in self.ids}
        self.tau = {i: 0.0 for i in self.ids}
        self.err = {i: 0 for i in self.ids}
        self.t_mos = {i: 0 for i in self.ids}
        self.stamp = {i: 0.0 for i in self.ids}   # 收到该电机帧的 monotonic 时间

    # ── 发送 ──────────────────────────────────────────────────
    def _send(self, can_id: int, data: bytes) -> bool:
        frame = struct.pack("<IB3x8s", can_id, len(data), data.ljust(8, b"\x00"))
        try:
            self.sock.send(frame)
            return True
        except (BlockingIOError, TimeoutError):
            self.dropped += 1
            return False
        except OSError as e:
            if e.errno in (105, 11):
                self.dropped += 1
                return False
            raise

    def query(self):
        """广播刷新帧：0x7FF + [id_lo, id_hi, 0xCC, 0]"""
        for i in self.ids:
            self._send(BROADCAST_ID,
                       bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0x00]))

    def enable(self):
        for i in self.ids:
            self._send(i, b"\xff" * 7 + b"\xfc")

    def disable(self):
        for i in self.ids:
            self._send(i, b"\xff" * 7 + b"\xfd")

    def mit(self, kp: float, kd: float, q: dict, tau: dict) -> bool:
        """q / tau 都是**电机坐标系**。dq_des 恒为 0。"""
        ok = True
        for i in self.ids:
            p_max, _v, t_max = LIMITS[i]
            q_u = f2u(q[i], -p_max, p_max, 16)
            kp_u = f2u(kp, 0.0, 500.0, 12)
            kd_u = f2u(kd, 0.0, 5.0, 12)
            tau_u = f2u(tau[i], -t_max, t_max, 12)
            dq_u = 0
            data = bytes([
                (q_u >> 8) & 0xFF, q_u & 0xFF,
                (dq_u >> 4) & 0xFF, ((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
                kp_u & 0xFF,
                (kd_u >> 4) & 0xFF, ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F),
                tau_u & 0xFF,
            ])
            ok = self._send(i, data) and ok
        return ok

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
            _cid, dlc = struct.unpack_from("<IB", raw, 0)
            d = raw[8:8 + dlc]
            if len(d) < 8:
                continue
            mid = d[0] & 0x0F
            if mid not in LIMITS or mid not in self.ids:
                self.other += 1
                continue
            p_max, v_max, t_max = LIMITS[mid]
            self.q[mid] = u2f((d[1] << 8) | d[2], -p_max, p_max, 16)
            self.dq[mid] = u2f((d[3] << 4) | (d[4] >> 4), -v_max, v_max, 12)
            self.tau[mid] = u2f(((d[4] & 0x0F) << 8) | d[5], -t_max, t_max, 12)
            self.err[mid] = d[0] >> 4
            self.t_mos[mid] = d[6]
            self.stamp[mid] = time.monotonic()
            n += 1
        return n

    # ── 新鲜读数 ──────────────────────────────────────────────
    def fresh(self, timeout: float = 1.0, max_age: float = 0.03) -> bool:
        """持续查询，直到**每台电机**都在 max_age 秒内回过一帧。"""
        self.reset_state()
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.query()
            self.poll()
            now = time.monotonic()
            if all(self.stamp[i] > 0 and now - self.stamp[i] < max_age
                   for i in self.ids):
                return True
            time.sleep(0.002)
        return False

    def q_joint(self) -> np.ndarray:
        return np.array([DIRECTION[i - 1] * self.q[i] for i in self.ids])

    def dq_joint(self) -> np.ndarray:
        return np.array([DIRECTION[i - 1] * self.dq[i] for i in self.ids])

    def tau_joint(self) -> np.ndarray:
        return np.array([DIRECTION[i - 1] * self.tau[i] for i in self.ids])

    def close(self):
        self.sock.close()


# ══════════════════════════════════════════════════════════════
# 标称重力模型
# ══════════════════════════════════════════════════════════════
class GravityModel:
    def __init__(self, urdf: str | None):
        import pinocchio as pin
        self.pin = pin
        path = urdf
        if path is None:
            for c in urdf_candidates():
                if os.path.isfile(c):
                    path = c
                    break
        if path is None or not os.path.isfile(path):
            raise FileNotFoundError(
                "找不到 hw_arm.urdf，试过：\n  " + "\n  ".join(urdf_candidates()))
        self.path = path
        self.model = pin.buildModelFromUrdf(path)
        self.data = self.model.createData()

    def g(self, q_joint: np.ndarray) -> np.ndarray:
        n = self.model.nv
        return np.asarray(self.pin.rnea(
            self.model, self.data, q_joint[:n], np.zeros(n), np.zeros(n)))


# ══════════════════════════════════════════════════════════════
# 单姿态采集
# ══════════════════════════════════════════════════════════════
def acquire(bus: ArmBus, ids: list[int], gm: GravityModel,
            kp: float, kd: float, dq_tol: float, settle_timeout: float,
            sample_s: float, verbose: bool) -> dict | None:
    """当前姿态使能 → 稳到真正静止 → 采样 → 失能。返回一条样本或 None。"""

    # ① 必须读到新鲜位置
    if not bus.fresh(timeout=1.0):
        print("     ❌ 读不到新鲜位置，跳过（不使能）")
        return None
    q0 = bus.q_joint()

    # ② 标称重力前馈（关节系 → 电机系）
    g_nom = gm.g(q0)
    q_des = {i: DIRECTION[i - 1] * q0[k] for k, i in enumerate(ids)}
    tau_ff = {i: DIRECTION[i - 1] * g_nom[k] for k, i in enumerate(ids)}

    if verbose:
        print(f"       q0    (度) = {np.round(np.degrees(q0), 2)}")
        print(f"       g_nom (N·m)= {np.round(g_nom, 3)}")

    # ③ 使能 + 立刻满 kp / 满前馈（**不做软启动**）
    bus.enable()
    for _ in range(5):
        bus.mit(kp, kd, q_des, tau_ff)
        time.sleep(0.002)
    bus.poll()

    try:
        # ④ 等真正静止
        t_end = time.monotonic() + settle_timeout
        calm = 0
        while time.monotonic() < t_end:
            bus.mit(kp, kd, q_des, tau_ff)
            bus.poll()
            vmax = float(np.abs(bus.dq_joint()).max())
            if vmax > 5.0:
                print(f"     ❌ 高速运动 |dq|max={vmax:.1f} rad/s，紧急失能")
                return None
            calm = calm + 1 if vmax < dq_tol else 0
            if calm >= 25:                       # 连续 25 次 ≈ 50 ms
                break
            time.sleep(0.002)

        # ⑤ 采样
        q_l, tau_l, dq_l = [], [], []
        t_end = time.monotonic() + sample_s
        while time.monotonic() < t_end:
            bus.mit(kp, kd, q_des, tau_ff)
            bus.poll()
            q_l.append(bus.q_joint())
            tau_l.append(bus.tau_joint())
            dq_l.append(bus.dq_joint())
            time.sleep(0.002)
    finally:
        bus.disable()
        time.sleep(0.10)

    if len(q_l) < 10:
        print("     ❌ 采样帧太少，跳过")
        return None

    q = np.median(np.array(q_l), axis=0)
    tau_raw = np.median(np.array(tau_l), axis=0)
    dq_max = float(np.abs(np.array(dq_l)).max())

    # ⑥ 扣掉 PD 贡献 → 外力矩（重力 + 摩擦）
    #    位置误差（关节系）= q_des_joint − q，其中 q_des_joint = q0
    perr = q0 - q
    tau_ext = tau_raw - g_nom + kp * perr      # kd 项：dq≈0，忽略

    return dict(q=q.tolist(), tau=tau_ext.tolist(),
                tau_raw=tau_raw.tolist(), g_nom=g_nom.tolist(),
                dq_max=dq_max, sag_deg=float(np.degrees(np.abs(perr).max())))


# ══════════════════════════════════════════════════════════════
# 主流程
# ══════════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(description="C-05 静态重力辨识 —— 数据采集")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--poses", type=int, default=25)
    ap.add_argument("--mode", default="manual", choices=["manual", "auto"])
    ap.add_argument("--kp", type=float, default=40.0)
    ap.add_argument("--kd", type=float, default=1.5)
    ap.add_argument("--dq-tol", type=float, default=0.02, help="静止判据 rad/s")
    ap.add_argument("--settle-timeout", type=float, default=3.0)
    ap.add_argument("--sample", type=float, default=0.6, help="采样时长 s")
    ap.add_argument("--auto-step", type=float, default=6.0, help="auto 每次偏转(度)")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="只读不发控制帧，确认读数与模型")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    out = args.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "data", f"static_gravity_{_dt.datetime.now():%Y%m%d_%H%M%S}.yaml")

    print("=" * 72)
    print("  C-05 静态重力辨识 —— 数据采集" + ("（干跑）" if args.dry_run else ""))
    print("=" * 72)

    # ── 模型 ──
    try:
        gm = GravityModel(args.urdf)
        print(f"  ✅ 重力模型: {gm.path}")
        print(f"     总质量 {gm.pin.computeTotalMass(gm.model):.4f} kg")
    except Exception as e:  # noqa: BLE001
        print(f"  ❌ {e}")
        print("     重力前馈是必需的（没有它机械臂会下垂/自由落体）")
        print("     先 colcon build --packages-select hw_arm_description")
        return 2

    # ── CAN ──
    try:
        bus = ArmBus(args.interface, ids)
    except FileNotFoundError as e:
        print(f"  ❌ {e}")
        return 3
    t0 = can_txF()
    for _ in range(10):
        bus.query()
        time.sleep(0.01)
    time.sleep(0.15)
    t1 = can_txF()
    if t0 >= 0 and t1 <= t0:
        print(f"  ❌ {args.interface} 发不出帧（TXF 停在 {t0}）")
        print("     跑： bash hw_arm_ws/src/hw_arm_description/scripts/can_up.sh")
        return 3
    print(f"  ✅ CAN 发送正常（TXF {t0} → {t1}）")

    if not bus.fresh(timeout=1.5):
        print("  ❌ 读不到电机反馈（六台都没回帧）")
        print("     查：电机供电 / CAN 线 / 终端电阻")
        return 3
    print(f"  ✅ 六台在线，q(度) = {np.round(np.degrees(bus.q_joint()), 2)}")
    print(f"     温度 MOS = {[bus.t_mos[i] for i in ids]} ℃")

    if args.dry_run:
        g = gm.g(bus.q_joint())
        print(f"  g(q) 标称 = {np.round(g, 3)} N·m")
        print(f"  tau 实测  = {np.round(bus.tau_joint(), 3)} N·m（失能，应≈0）")
        print("\n  ✅ 干跑结束：没有使能、没有发控制帧")
        bus.close()
        return 0

    print(f"\n  kp/kd = {args.kp}/{args.kd}   每姿态最长 "
          f"{args.settle_timeout:.1f}s + 采样 {args.sample:.1f}s")
    print("  ⚠️ 采集时电机会使能（'变硬'），但目标就是当前位置，不应移动")
    print("  ⚠️ 请把机械臂**抬离支撑/桌面**，让电机真正扛着重力\n")

    records, suspicious = [], 0
    try:
        base = None
        for k in range(args.poses):
            if args.mode == "manual":
                try:
                    input(f"  [{k+1}/{args.poses}] 摆好姿态后按回车…")
                except EOFError:
                    print("  （非交互终端，停止）")
                    break
            else:
                if base is None:
                    if not bus.fresh():
                        break
                    base = bus.q_joint()
                step = np.radians(args.auto_step)
                tgt = np.clip(base + np.random.uniform(-step, step, len(ids)),
                              [LIMITS_Q[i][0] + 0.05 for i in ids],
                              [LIMITS_Q[i][1] - 0.05 for i in ids])
                q_des = {i: DIRECTION[i - 1] * tgt[m] for m, i in enumerate(ids)}
                g_t = gm.g(tgt)
                tau_t = {i: DIRECTION[i - 1] * g_t[m] for m, i in enumerate(ids)}
                bus.enable()
                t_end = time.monotonic() + 1.0
                while time.monotonic() < t_end:
                    bus.mit(args.kp, args.kd, q_des, tau_t)
                    bus.poll()
                    time.sleep(0.002)
                bus.disable()
                time.sleep(0.1)
                base = tgt

            rec = acquire(bus, ids, gm, args.kp, args.kd,
                          args.dq_tol, args.settle_timeout, args.sample,
                          args.verbose)
            if rec is None:
                continue

            resid = np.array(rec["tau"]) - np.array(rec["g_nom"])
            worst = float(np.abs(resid).max())
            flag = ""
            if is_supported(rec):
                suspicious += 1
                flag = "  ⚠️ 可能压在支撑上（力矩远小于标称）"
            records.append(rec)

            print(f"     q(度)    = {np.round(np.degrees(rec['q']), 1)}")
            print(f"     tau实测  = {np.round(rec['tau'], 3)} N·m")
            print(f"     标称 g   = {np.round(rec['g_nom'], 3)} N·m")
            print(f"     偏差     = {np.round(resid, 3)} N·m（最大 {worst:.2f}）"
                  f"  下垂 {rec['sag_deg']:.2f}°{flag}")
    except KeyboardInterrupt:
        print("\n  中断")
    finally:
        bus.disable()
        time.sleep(0.2)
        bus.close()

    if len(records) < 6:
        print(f"\n❌ 只采到 {len(records)} 个有效姿态（至少 6 个）")
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
                "urdf": gm.path,
                "note": "q/tau 均为关节坐标系；tau 已扣掉 PD 贡献，"
                        "只剩重力+摩擦。标称 g 一并存下便于对比。",
            },
            "samples": records,
        }, f, allow_unicode=True, sort_keys=False)

    print(f"\n✅ 采到 {len(records)} 个姿态 → {out}")
    if suspicious:
        print(f"   ⚠️ 其中 {suspicious} 个可能压在支撑上，拟合时注意")
    print(f"   下一步： python3 algo/scripts/fit_static_gravity.py {out}")
    return 0


def is_supported(rec: dict) -> bool:
    """粗判：某关节标称重力矩 >1 N·m，但实测不到它的一半。"""
    g = np.array(rec["g_nom"])
    tau = np.array(rec["tau"])
    return any(abs(g[i]) > 1.0 and abs(tau[i]) < 0.5 * abs(g[i])
               for i in range(len(g)))


if __name__ == "__main__":
    sys.exit(main())
