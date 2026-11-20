#!/usr/bin/env python3
"""实时显示六轴角度/速度/力矩（电机必须处于失能或使能均可，只读）

用途：
  · 检查某个关节用手转开后**会不会自己回去**（判断有没有非重力的回复力矩）
  · 摆姿态时看角度够不够

用法：
    source /opt/ros/jazzy/setup.bash
    python3 algo/scripts/monitor_joints.py            # 默认 10 Hz
    python3 algo/scripts/monitor_joints.py --hz 20

配合实验：
    1. 跑起来，确认六轴都是 DISABLED
    2. 用手把 joint1 平转 30~40°，松手
    3. 盯住 j1 那一列：
         缓慢回到 0  → 有非重力的回复力矩（线缆扭转 / 弹簧 / 抱闸）
         停在原地    → 没有回复力矩，之前"回零"是别的原因
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="实时显示六轴状态")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--hz", type=float, default=10.0)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "collector", os.path.join(here, "collect_static_gravity.py"))
    C = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(C)

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    bus = C.ArmBus(args.interface, ids)
    if not bus.fresh(timeout=1.5):
        print("  ❌ 读不到电机")
        return 2

    period = 1.0 / args.hz
    rr = 0
    print(f"\n  每 {1/args.hz*1000:.0f} ms 刷新一次，Ctrl-C 退出")
    print(f"  {'时间':>6s}  " + "".join(f"{f'j{i}':>9s}" for i in ids)
          + "".join(f"{f'v{i}':>7s}" for i in ids) + "  状态")
    t0 = time.monotonic()
    try:
        while True:
            t_cycle = time.monotonic()
            bus.refresh_one(ids[rr % len(ids)])
            bus.refresh_one(ids[(rr + 1) % len(ids)])
            rr = (rr + 2) % len(ids)
            t_end = time.monotonic() + 0.004
            while time.monotonic() < t_end:
                bus.poll()
                time.sleep(0.0002)
            q = bus.q_joint()
            dq = bus.dq_joint()
            st = "".join(
                "E" if bus.err[i] == 1 else ("D" if bus.err[i] == 0 else "F")
                for i in ids)
            print(f"  {time.monotonic()-t0:6.1f}  "
                  + "".join(f"{np.degrees(x):9.2f}" for x in q)
                  + "".join(f"{x:7.2f}" for x in dq)
                  + f"  {st}")
            rest = period - (time.monotonic() - t_cycle)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        print("\n  退出")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
