#!/usr/bin/env python3
"""诊断：使能状态下的 CAN 收发到底发生了什么

背景：collect_static_gravity.py 在使能后报 "反馈超时(>100ms)"，
      但机械臂失能时同样的刷新节奏一切正常。所以要看**使能后**的收发统计。

它做的事：
    1. 读当前位置（新鲜）
    2. 用标称重力前馈把机械臂"扶住"（kp=40, kd=1.5，目标就是当前位置）
    3. 以 200 Hz 跑 3 秒：每拍发 6 帧 MIT + 轮转刷新 2 台，逐拍记录统计
    4. 失能并回读确认
    5. 打印：帧龄分布、每轴回帧数、发送失败、被过滤帧数

只用于诊断，不写数据文件。
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time

import numpy as np


def _load_collector():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "collect_static_gravity.py")
    spec = importlib.util.spec_from_file_location("collector", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main() -> int:
    ap = argparse.ArgumentParser(description="使能状态下的 CAN 收发诊断")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--kp", type=float, default=40.0)
    ap.add_argument("--kd", type=float, default=1.5)
    args = ap.parse_args()

    C = _load_collector()
    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    gm = C.GravityModel(None)
    bus = C.ArmBus(args.interface, ids)

    if not bus.fresh(timeout=1.5):
        print("  ❌ 读不到电机（先确认上电/CAN）")
        return 2
    q0 = bus.q_joint()
    g_nom = gm.g(q0)
    q_des = {i: C.DIRECTION[i - 1] * q0[k] for k, i in enumerate(ids)}
    tau_ff = {i: C.DIRECTION[i - 1] * g_nom[k] for k, i in enumerate(ids)}
    print(f"  q0(度) = {np.round(np.degrees(q0), 2)}")
    print(f"  g(q0)  = {np.round(g_nom, 3)} N·m")
    print(f"  开始使能并跑 {args.seconds:.1f}s @ {args.hz:.0f} Hz "
          f"(kp={args.kp}, kd={args.kd})")

    bus.enable()
    period = 1.0 / args.hz
    t_end = time.monotonic() + args.seconds
    rr = 0
    ages, tx, rx, replies = [], 0, 0, {i: 0 for i in ids}
    other0, dropped0 = bus.other, bus.dropped
    steps = 0
    try:
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            tx += 6
            if not bus.mit(args.kp, args.kd, q_des, tau_ff):
                print("     ❌ MIT 发送失败")
                break
            before = dict(bus.seq)
            for k in range(2):
                bus.refresh_one(ids[(rr + k) % len(ids)])
                tx += 1
            rr = (rr + 2) % len(ids)
            t_poll = time.monotonic() + 0.004
            while time.monotonic() < t_poll:
                bus.poll()
                time.sleep(0.0002)
            for i in ids:
                replies[i] += bus.seq[i] - before[i]
            now = time.monotonic()
            ages.append([now - bus.stamp[i] for i in ids])
            steps += 1
            rest = period - (time.monotonic() - t0)
            if rest > 0:
                time.sleep(rest)
    finally:
        ok = bus.disable_and_verify()
        print(f"  失能{'确认成功' if ok else '**未确认**！'}")

    a = np.array(ages) * 1000
    d_other = bus.other - other0
    d_drop = bus.dropped - dropped0
    bus.close()

    print()
    print("=" * 66)
    print(f"  {steps} 拍，发送 {tx} 帧，被过滤 {d_other} 帧，发送失败 {d_drop} 帧")
    print(f"  每轴回帧数: {replies}")
    print(f"  帧龄(ms)  最大值的分布: 中位 {np.median(a.max(axis=1)):.1f}  "
          f"90分位 {np.percentile(a.max(axis=1), 90):.1f}  峰值 {a.max():.1f}")
    print(f"  {'关节':8s}{'帧龄中位':>10s}{'帧龄90分位':>12s}{'帧龄峰值':>10s}{'回帧数':>8s}")
    for k, i in enumerate(ids):
        col = a[:, k]
        print(f"  joint{i} {np.median(col):9.1f}{np.percentile(col, 90):11.1f}"
              f"{col.max():10.1f}{replies[i]:8d}")
    print("=" * 66)
    if d_other > steps * 3:
        print(f"  ⚠️ 被过滤的帧很多（{d_other}），检查是否有别的设备在同一条总线上")
    if replies[min(ids)] < steps * 0.2:
        print("  ⚠️ 回帧数明显少于拍数 —— 电机在使能状态下可能不再应答刷新帧")
    return 0


if __name__ == "__main__":
    sys.exit(main())
