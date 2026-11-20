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

为什么不用"扣掉 PD 贡献"
    电机反馈的 tau 是**Kt × 实测电流**估算出来的真实输出力矩，不是指令回显。
    静止时力矩平衡：  tau_motor = tau_ext（重力 + 摩擦）
    而 tau_motor = kp·(q_des − q) + kd·(0 − dq) + tau_ff
    所以 **tau_ext 就等于反馈的 tau 本身**，任何"再扣一次 PD 贡献"的写法
    都会把 kp·误差 算两遍，得到错误数据。

    ⚠️ 但要注意 tau 里混入了：Kt 标称误差、减速器效率（都是**全局乘性**，
       会被 gravity_scale 吸收）、电流零偏（每关节常值，被拟合的 bias 吸收）、
       静摩擦（随姿态变号，留在残差里，构成精度下限）。

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
LIMITS_Q = {   # 2026-11-20 真机标定的关节限位（见 hw_arm_description/config/joint/joint_limits.yaml）
    1: (-3.1415926536, 3.1415926536), 2: (-0.0017, 3.1034), 3: (0.0105, 3.0364),
    4: (-3.3522, 0.5262), 5: (-1.2037, 1.1896), 6: (-3.1415926536, 3.1415926536),
}

BROADCAST_ID = 0x7FF
MASTER_OFFSET = 0x10      # 电机回复用的 Master CAN ID = 发送 ID + 0x10（0x01..0x06 → 0x11..0x16）
FAULT_LEVEL = 0x8         # 状态字段：0=失能 1=使能 >=8=故障


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
        self.seq = {i: 0 for i in self.ids}       # 收到的**有效状态帧**计数（逐轴）

    # ── 发送 ──────────────────────────────────────────────────
    def _send(self, can_id: int, data: bytes, tries: int = 3) -> bool:
        frame = struct.pack("<IB3x8s", can_id, len(data), data.ljust(8, b"\x00"))
        for k in range(max(1, tries)):
            try:
                self.sock.send(frame)
                return True
            except (BlockingIOError, TimeoutError):
                pass
            except OSError as e:
                if e.errno not in (105, 11):      # ENOBUFS / EAGAIN
                    raise
            # 发送队列短暂拥塞，等一下再试（实测 gs_usb 的可持续速率约 3000 fps）
            time.sleep(0.0005 * (k + 1))
        self.dropped += 1
        return False

    def query(self):
        """广播刷新帧：0x7FF + [id_lo, id_hi, 0xCC, 0]（一次刷全部）"""
        for i in self.ids:
            self._send(BROADCAST_ID,
                       bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0x00]))

    def refresh_one(self, i: int) -> bool:
        """只刷新一台。轮转刷新用，避免一次发太多帧把发送队列打满。"""
        return self._send(BROADCAST_ID,
                          bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0x00]))

    def enable(self):
        for i in self.ids:
            self._send(i, b"\xff" * 7 + b"\xfc")

    def disable(self) -> bool:
        """失能。返回是否**每一帧都发成功**（不代表电机已经停）。"""
        ok = True
        for i in self.ids:
            ok = self._send(i, b"\xff" * 7 + b"\xfd") and ok
        return ok

    def disable_and_verify(self, tries: int = 3, timeout: float = 0.6) -> bool:
        """反复发失能，并**回读状态**确认每台都真的 DISABLED。

        DM 的状态字段：0=失能 1=使能 >=8=故障。
        以前只发一次、且忽略发送失败 —— 发送队列满时脚本会以为停了、
        实际电机仍在保持之前的控制力矩。必须回读确认。
        """
        sent_ok = False
        for _ in range(max(1, tries)):
            sent_ok = self.disable() or sent_ok
            time.sleep(0.05)

        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.query()
            self.poll()
            if all(self.stamp[i] > 0 and self.err[i] == 0 for i in self.ids):
                return True
            time.sleep(0.01)
        return False

    def bus_health(self, max_age: float = 0.05) -> str | None:
        """返回故障描述；一切正常返回 None。

        逐轴检查三件事：收到过帧、帧足够新、状态字不是故障。
        """
        now = time.monotonic()
        never = [i for i in self.ids if self.stamp[i] == 0.0]
        if never:
            return f"从未收到状态帧: {never}"
        stale = [i for i in self.ids if now - self.stamp[i] > max_age]
        if stale:
            age = {i: round(now - self.stamp[i], 3) for i in stale}
            return f"反馈超时(>{max_age * 1000:.0f}ms): {age}"
        fault = [i for i in self.ids if self.err[i] >= FAULT_LEVEL]
        if fault:
            return f"电机故障: { {i: hex(self.err[i]) for i in fault} }"
        return None

    def mit(self, kp: float, kd: float, q: dict, tau: dict) -> bool:
        """q / tau 都是**电机坐标系**。dq_des 恒为 0（语义上是零速）。"""
        ok = True
        for i in self.ids:
            p_max, v_max, t_max = LIMITS[i]
            q_u = f2u(q[i], -p_max, p_max, 16)
            kp_u = f2u(kp, 0.0, 500.0, 12)
            kd_u = f2u(kd, 0.0, 5.0, 12)
            tau_u = f2u(tau[i], -t_max, t_max, 12)
            # ⚠️ P0：MIT 协议的每个字段都是**偏移编码**，字段值 0 表示区间下限。
            #    所以 dq_u = 0 编出去的是 **-v_max**（j2 就是 -40 rad/s），
            #    电机内部 kd·(dq_des − dq) 于是变成 kd·(−v_max)：
            #        kd=1.5 时 j1 −37.5 / j2 −60 / j3 −15 / j4~6 −45 N·m
            #    这是"猛推"的来源，必须按偏移编码写 0，12 bit 下 = 2047。
            dq_u = f2u(0.0, -v_max, v_max, 12)
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
            cid, dlc = struct.unpack_from("<IB", raw, 0)
            # SocketCAN 的错误帧带 CAN_ERR_FLAG，标准帧 ID 只占低 11 位
            if cid & 0x20000000:                       # CAN_ERR_FLAG
                self.other += 1
                continue
            cid &= 0x7FF
            d = raw[8:8 + dlc]
            if dlc != 8 or len(d) < 8:                 # DM 状态帧固定 8 字节
                self.other += 1
                continue
            # ⚠️ 只认 Master CAN ID = 发送 ID + 0x10 的帧
            #
            #    以前只按 d[0] & 0x0F 匹配，而 SocketCAN 默认开了 loopback：
            #      · 广播查询帧  0x7FF + [0x01,0x00,0xCC,0x00] → mid=1 被当成状态帧
            #        （实测过：joint1 显示 master=0x7FF / q=-12.42 / 温度 0）
            #      · MIT 指令帧  CAN ID=0x01..0x06，d[0]=(q_u>>8)&0xFF，
            #        低四位落在 1..6 就有 6/16 概率被误收，还可能记到别的电机头上
            #    后果：fresh() 被击穿（没有电机回也能"新鲜"）、
            #          静止判据失效、采样数据被污染。
            mid = d[0] & 0x0F
            if mid not in LIMITS or mid not in self.ids:
                self.other += 1
                continue
            if cid != mid + MASTER_OFFSET:
                self.other += 1
                continue
            p_max, v_max, t_max = LIMITS[mid]
            self.q[mid] = u2f((d[1] << 8) | d[2], -p_max, p_max, 16)
            self.dq[mid] = u2f((d[3] << 4) | (d[4] >> 4), -v_max, v_max, 12)
            self.tau[mid] = u2f(((d[4] & 0x0F) << 8) | d[5], -t_max, t_max, 12)
            self.err[mid] = d[0] >> 4
            self.t_mos[mid] = d[6]
            self.stamp[mid] = time.monotonic()
            self.seq[mid] += 1
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
    """当前姿态使能 → 稳到真正静止 → 采样 → 失能。返回一条样本或 None。

    安全约束（本次新增）：
      · 每一拍都必须**每台电机都有新状态帧**，否则立即中止
      · 每轴状态帧超时、电机报故障、MIT 帧发送失败 → 立即中止
      · 静止判据基于"有效帧数 + 时间窗内位置/速度都稳"，超时则**拒绝本次采集**
      · 采样期间持续做同样的检查，任何一项不满足都丢弃样本
      · 失能要回读确认，没有确认就明确报错
    """
    calm_frames = 20          # 需要连续多少拍满足"慢 + 不漂"
    settle_window_s = 0.20    # 位置稳定窗口
    pos_tol = 0.01            # 窗口内位置漂移上限 (rad)
    v_hard_limit = 5.0        # 紧急失能阈值 (rad/s)
    min_valid_frames = 20     # 采样期至少要有多少个有效样本

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

    # 每拍刷新的电机数（轮转）与循环周期
    #
    # 为什么要限速：gs_usb 适配器的可持续发送速率实测约 3000 fps，
    #   6 帧 MIT + 6 帧查询 @ 400 Hz = 4800 fps → 直接 ENOBUFS。
    #   这里改成「6 帧 MIT + 2 帧刷新」@ 200 Hz = 1600 fps，留足余量；
    #   每台电机每 3 拍被刷新一次（15 ms），比 100 ms 的健康门槛宽裕得多。
    refresh_per_step = 2
    step_period = 0.005              # 200 Hz
    max_age = 0.10                   # 每轴 100 ms 内必须有过状态帧
    rr = [0]                         # 轮转游标

    def step() -> str | None:
        """发一帧 MIT + 轮转刷新若干台，返回错误描述（None=正常）。

        ⚠️ **电机只在被查询时才回状态帧**（ROS 硬件插件也是每周期
           refresh_one + recv_all；只发 MIT 不会有回包）。
           但也**不能每拍刷全部 6 台** —— 帧数会超过适配器可持续速率。
        """
        t_begin = time.monotonic()
        if not bus.mit(kp, kd, q_des, tau_ff):
            return "MIT 帧发送失败（CAN 发送队列满？）"

        n_before = dict(bus.seq)
        for k in range(refresh_per_step):
            bus.refresh_one(ids[(rr[0] + k) % len(ids)])
        rr[0] = (rr[0] + refresh_per_step) % len(ids)

        # 等这一拍刷新到的电机回帧（最多 4 ms）
        t_end = time.monotonic() + 0.004
        while time.monotonic() < t_end:
            bus.poll()
            time.sleep(0.0002)

        err = bus.bus_health(max_age=max_age)
        # 补足到固定周期，避免循环跑得过快把总线打满
        rest = step_period - (time.monotonic() - t_begin)
        if rest > 0:
            time.sleep(rest)
        return err

    # ③ 使能 + 立刻满 kp / 满前馈（**不做软启动**）
    bus.enable()
    fails = 0
    for _ in range(12):
        err = step()
        if err:
            fails += 1
            if fails >= 4:            # 连续 4 拍都不行才判定失败
                print(f"     ❌ 使能阶段异常（连续 {fails} 拍）：{err}")
                bus.disable_and_verify()
                return None
        else:
            fails = 0

    settled = False
    q_l, tau_l, dq_l = [], [], []

    try:
        # ④ 等真正静止
        #
        # 数据时效由 step() 的 bus_health 保证（每轴 100 ms 内必有状态帧），
        # 所以这里不再要求"每一拍六轴全到"（轮转刷新下本来就做不到），
        # 改用：速度足够小 + 位置在时间窗内不漂。
        t_end = time.monotonic() + settle_timeout
        calm = 0
        q_ref = None
        t_win = None
        while time.monotonic() < t_end:
            err = step()
            if err:
                print(f"     ❌ {err}，中止本次采集")
                return None

            dq = bus.dq_joint()
            q_now = bus.q_joint()
            vmax = float(np.abs(dq).max())
            if vmax > v_hard_limit:
                print(f"     ❌ 高速运动 |dq|max={vmax:.1f} rad/s，紧急失能")
                return None

            if vmax < dq_tol:
                if q_ref is None:
                    q_ref, t_win, calm = q_now, time.monotonic(), 0
                calm += 1
                drift = float(np.abs(q_now - q_ref).max())
                if drift > pos_tol:            # 位置在窗口内漂了，重新计时
                    q_ref, t_win, calm = q_now, time.monotonic(), 0
                elif (calm >= calm_frames
                      and time.monotonic() - t_win >= settle_window_s):
                    settled = True
                    break
            else:
                q_ref, t_win, calm = None, None, 0

        if not settled:
            print(f"     ❌ {settle_timeout:.1f}s 内没有稳定下来"
                  f"（calm={calm}，需要 {calm_frames} 帧 + "
                  f"{settle_window_s * 1000:.0f}ms 窗口）→ 拒绝本次采集")
            return None

        # ⑤ 采样（期间持续做同样的安全检查）
        t_end = time.monotonic() + sample_s
        while time.monotonic() < t_end:
            err = step()          # 内含 100 ms 逐轴时效检查
            if err:
                print(f"     ❌ 采样中异常：{err} → 丢弃本样本")
                return None

            dq = bus.dq_joint()
            if float(np.abs(dq).max()) > v_hard_limit:
                print(f"     ❌ 采样中高速运动 {np.abs(dq).max():.1f} rad/s → 丢弃")
                return None

            q_l.append(bus.q_joint())
            tau_l.append(bus.tau_joint())
            dq_l.append(dq)
    finally:
        # ⑥ 停止：重发 + 回读确认。确认失败必须吼出来
        if not bus.disable_and_verify():
            print("     🚨🚨 失能**未确认**！电机可能仍在输出力矩，"
                  "请立即断 CAN 电源或按急停！")
        else:
            time.sleep(0.05)

    if len(q_l) < min_valid_frames:
        print(f"     ❌ 采样有效帧太少（{len(q_l)} < {min_valid_frames}），跳过")
        return None

    q_arr = np.array(q_l)
    q = np.median(q_arr, axis=0)
    tau_raw = np.median(np.array(tau_l), axis=0)
    dq_max = float(np.abs(np.array(dq_l)).max())
    pos_spread = float(np.abs(q_arr - q).max())

    # 采样期还要够"静"：速度、位置散布都要小
    if dq_max > dq_tol:
        print(f"     ❌ 采样期 |dq|max={dq_max:.3f} > 容差 {dq_tol} → 丢弃")
        return None
    if pos_spread > pos_tol:
        print(f"     ❌ 采样期位置散布 {pos_spread:.4f} rad > {pos_tol} → 丢弃")
        return None

    # ⑦ 静止时，电机反馈的力矩**就是**外力矩（重力 + 摩擦）
    #
    #    平衡条件： tau_motor = tau_ext
    #    而 tau_motor = kp·(q_des − q) + kd·(0 − dq) + tau_ff
    #                 = kp·perr + g_nom(q0)          （dq≈0）
    #    所以 tau_ext = tau_raw，**不需要再做任何加减**。
    #
    #    ⚠️ 前面那个版本写成 tau_ext = tau_raw − g_nom + kp·perr，
    #       把 kp·perr 算了两遍，数据是错的。
    perr = q0 - q
    tau_ext = tau_raw                          # kd 项：dq≈0，忽略

    return dict(q=q.tolist(), tau=tau_ext.tolist(),
                tau_raw=tau_raw.tolist(), g_nom=g_nom.tolist(),
                dq_max=dq_max, pos_spread=pos_spread,
                n_frames=len(q_l),
                sag_deg=float(np.degrees(np.abs(perr).max())))


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
    ap.add_argument("--selftest", action="store_true",
                    help="离线自检：验证速度编码与收包过滤（不需要电机）")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

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
        if not bus.disable_and_verify():
            print("\n  🚨🚨 失能**未确认**！电机可能仍在输出力矩，"
                  "请立即断电或按急停！")
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
                "note": "q/tau 均为关节坐标系；tau 是电机反馈的力矩"
                        "（Kt×电流估算），静止时即外力矩=重力+摩擦，"
                        "未做任何 PD 扣减。标称 g 一并存下便于对比。",
            },
            "samples": records,
        }, f, allow_unicode=True, sort_keys=False)

    print(f"\n✅ 采到 {len(records)} 个姿态 → {out}")
    if suspicious:
        print(f"   ⚠️ 其中 {suspicious} 个可能压在支撑上，拟合时注意")
    print(f"   下一步： python3 algo/scripts/fit_static_gravity.py {out}")
    return 0


def selftest() -> int:
    """离线自检：不需要电机、不需要 CAN 接口。

    验证两件事：
      ① MIT 帧里 dq_des 解码回来必须是 ≈0（P0 那个 bug 的回归测试）
      ② poll() 必须丢掉 SocketCAN 回环帧（广播查询 / MIT 指令回显），
         只接受 Master CAN ID 的状态帧
    """
    print("=" * 72)
    print("  离线自检")
    print("=" * 72)
    fail = 0

    class _FakeSock:
        def __init__(self, frames=None):
            self.frames = list(frames or [])
            self.sent = []

        def recv(self, n):
            if not self.frames:
                raise BlockingIOError
            return self.frames.pop(0)

        def send(self, b):
            self.sent.append(b)
            return len(b)

    def new_bus(frames=None):
        b = ArmBus.__new__(ArmBus)
        b.ids = list(range(1, 7))
        b.sock = _FakeSock(frames)
        b.dropped = 0
        b.other = 0
        b.reset_state()
        return b

    # ── ① 速度编码：直接测 mit() 真正发出的字节 ──
    print("\n① MIT 速度字段编码（解析 mit() 实际发出的帧）")
    kp_t, kd_t = 40.0, 1.5
    q_t = {i: 0.0 for i in range(1, 7)}
    tau_t = {i: 0.0 for i in range(1, 7)}
    bus = new_bus()
    assert bus.mit(kp_t, kd_t, q_t, tau_t), "mit() 发送失败"
    frames = {}
    for raw in bus.sock.sent:
        cid = struct.unpack_from("<I", raw, 0)[0] & 0x7FF
        frames[cid] = raw[8:16]

    print(f"  {'关节':8s}{'v_max':>7s}{'dq_u':>7s}{'解码 dq_des':>13s}"
          f"{'kd·该项(N·m)':>14s}{'判定':>6s}")
    for i in range(1, 7):
        _p, v_max, _t = LIMITS[i]
        d = frames[i]
        dq_u = (d[2] << 4) | (d[3] >> 4)
        back = u2f(dq_u, -v_max, v_max, 12)
        ok = abs(back) <= 2 * v_max / 4095 + 1e-9
        fail += 0 if ok else 1
        print(f"  joint{i} {v_max:6.1f}{dq_u:7d}{back:12.4f}"
              f"{kd_t * back:13.2f}{'✅' if ok else '❌':>6s}")
    print("  （旧代码 dq_u 恒为 0 → 解码 -v_max → kd·该项 达 -37.5~-60 N·m）")

    # ── ② 收包过滤 ──
    print("\n② 收包过滤（模拟 SocketCAN 回环）")

    def mk(can_id, data):
        return struct.pack("<IB3x8s", can_id, 8, bytes(data).ljust(8, b"\x00"))

    # 一条合法的 joint2 状态帧（Master 0x12）
    good = [0x12, 0x80, 0x00, 0x7F, 0xF0, 0x00, 0x1A, 0x14]
    cases = [
        ("合法状态帧 0x12 (joint2)", mk(0x12, good), 2, True),
        ("回环：广播查询 0x7FF", mk(0x7FF, [0x02, 0x00, 0xCC, 0x00]), 2, False),
        ("回环：MIT 指令 0x02", mk(0x02, [0x80, 0x00, 0x7F, 0xF0, 0x00, 0x00, 0x00, 0x00]), 2, False),
        ("别人的状态帧 0x13 (joint3)", mk(0x13, [0x13, 0x80, 0x00, 0x7F, 0xF0, 0x00, 0x1A, 0x14]), 2, False),
        ("CAN 错误帧", mk(0x20000000 | 0x12, good), 2, False),
        ("长度不对 (dlc=4)", struct.pack("<IB3x8s", 0x12, 4, bytes(good[:4]).ljust(8, b"\x00")), 2, False),
    ]
    print(f"  {'用例':30s}{'是否更新 joint2':>16s}{'期望':>8s}{'判定':>6s}")
    for name, frame, jid, expect in cases:
        bus = new_bus([frame])
        bus.poll()
        got = bus.seq[jid] > 0
        ok = got == expect
        fail += 0 if ok else 1
        print(f"  {name:30s}{str(got):>16s}{str(expect):>8s}{'✅' if ok else '❌':>6s}")

    print()
    if fail:
        print(f"  ❌ 自检失败 {fail} 项")
        return 1
    print("  ✅ 自检全部通过")
    return 0


def is_supported(rec: dict) -> bool:
    """粗判：某关节标称重力矩 >1 N·m，但实测不到它的一半。"""
    g = np.array(rec["g_nom"])
    tau = np.array(rec["tau"])
    return any(abs(g[i]) > 1.0 and abs(tau[i]) < 0.5 * abs(g[i])
               for i in range(len(g)))


if __name__ == "__main__":
    sys.exit(main())
