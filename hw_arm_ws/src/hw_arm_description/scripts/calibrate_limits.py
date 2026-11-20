#!/usr/bin/env python3
"""关节限位标定（真机手动版）

═══════════════════════════════════════════════════════════════════
做什么
═══════════════════════════════════════════════════════════════════
URDF 里的 joint_limits.yaml 是从 CAD 抄的，和真机的机械硬限位不一定一致
（装配误差、线缆/气管阻挡、结构干涉都会让实际行程变小或变大）。

这个脚本让你**用手**把每个关节推到真实的两端，它把你的读数记下来，
最后生成一份可以直接替换 joint_limits.yaml 的数据。

⚠️ 全程电机保持 **DISABLED**（零力矩），脚本只发查询帧，
   一个 MIT / 使能帧都不发 —— 你推它，它绝不动。

═══════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════
    source /opt/ros/jazzy/setup.bash
    python3 scripts/calibrate_limits.py                # 逐个关节标定
    python3 scripts/calibrate_limits.py --joints 2,3   # 只标定部分
    python3 scripts/calibrate_limits.py --write        # 标定完直接改 yaml

    标定完的对比表会打印出来；加 --write 才会真的写文件（会先备份）。

每个关节两次：
    ① 推到**负方向极限** → 回车
    ② 推到**正方向极限** → 回车

═══════════════════════════════════════════════════════════════════
注意
═══════════════════════════════════════════════════════════════════
    · 角度都是**关节坐标系**（已乘 direction），和 URDF 一致
    · 记录的是"推到底并保持住"时的中位数，抖动会被滤掉
    · 如果某关节没有硬限位（比如无限位块），别标它 —— 转过界会扯线
"""
from __future__ import annotations

import argparse
import datetime as _dt
import math
import os
import shutil
import socket
import struct
import sys
import time

import numpy as np

# ── 与 collect_static_gravity.py 保持一致的常量 ──
LIMITS = {
    1: (12.5,   25.0, 200.0),   # DM10010L
    2: (12.566, 40.0,  70.0),   # DM6216P
    3: (12.5,   10.0,  28.0),   # DM4340
    4: (12.5,   30.0,  10.0),   # DM4310P
    5: (12.5,   30.0,  10.0),   # DM4310
    6: (12.5,   30.0,  10.0),   # DM4310
}
DIRECTION = [+1, -1, -1, +1, +1, +1]     # 电机坐标系 → 关节坐标系（C-02 实测）
BROADCAST_ID = 0x7FF


def u2f(u, lo, hi, bits):
    return u * (hi - lo) / ((1 << bits) - 1) + lo


class Reader:
    """只读：广播刷新 + 收状态。绝不使能。"""

    def __init__(self, ifname, ids):
        if not os.path.exists(f"/sys/class/net/{ifname}"):
            raise FileNotFoundError(f"{ifname} 不存在（USB-CAN 没插或没配置）")
        self.ids = ids
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.setblocking(False)
        self.q = {i: None for i in ids}
        self.stamp = {i: 0.0 for i in ids}

    def query(self):
        for i in self.ids:
            try:
                self.sock.send(struct.pack("<IB3x8s", BROADCAST_ID, 4,
                                           bytes([i & 0xFF, (i >> 8) & 0xFF, 0xCC, 0])
                                           .ljust(8, b"\x00")))
            except (BlockingIOError, TimeoutError, OSError):
                pass

    def poll(self):
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
            if mid not in self.q:
                continue
            p = LIMITS[mid][0]
            self.q[mid] = u2f((d[1] << 8) | d[2], -p, p, 16)
            self.stamp[mid] = time.monotonic()

    def read(self, seconds=0.25):
        """采一小段时间的中位数，返回关节坐标系角度。"""
        vals = {i: [] for i in self.ids}
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.query()
            self.poll()
            for i in self.ids:
                if self.q[i] is not None:
                    vals[i].append(DIRECTION[i - 1] * self.q[i])
            time.sleep(0.004)
        out = {}
        for i in self.ids:
            out[i] = float(np.median(vals[i])) if vals[i] else None
        return out

    def wait_all_alive(self, timeout=3.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.query()
            self.poll()
            now = time.monotonic()
            if all(self.q[i] is not None and now - self.stamp[i] < 0.05
                   for i in self.ids):
                return True
            time.sleep(0.01)
        return False

    def close(self):
        self.sock.close()


def fmt_deg(v):
    return f"{math.degrees(v):+8.2f}°" if v is not None else "    —   "


def main() -> int:
    ap = argparse.ArgumentParser(description="关节限位标定（真机手动版）")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--all-ids", default="1,2,3,4,5,6")
    ap.add_argument("--joints", default="1,2,3,4,5,6", help="要标定的关节号")
    ap.add_argument("--write", action="store_true", help="标定完直接更新 joint_limits.yaml")
    ap.add_argument("--yaml", default=None, help="joint_limits.yaml 路径")
    args = ap.parse_args()

    ids = [int(x) for x in args.all_ids.split(",") if x.strip()]
    targets = [int(x) for x in args.joints.split(",") if x.strip()]

    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    yaml_path = args.yaml or os.path.join(pkg, "config", "joint", "joint_limits.yaml")

    print("=" * 74)
    print("  关节限位标定 —— 全程电机 DISABLED，脚本只读不写")
    print("=" * 74)
    try:
        rd = Reader(args.interface, ids)
    except FileNotFoundError as e:
        print(f"  ❌ {e}")
        return 2
    if not rd.wait_all_alive():
        print("  ❌ 读不到全部电机。先跑： bash scripts/can_up.sh")
        rd.close()
        return 2

    cur = rd.read(0.3)
    print("  当前角度（关节坐标系）：")
    print("    " + "  ".join(f"j{i}:{fmt_deg(cur[i])}" for i in ids))
    near_zero = all(cur[i] is not None and abs(cur[i]) < 0.05 for i in ids)
    print(f"  是否在零位附近：{'✅ 是' if near_zero else '⚠️ 不是（不影响标定，但建议先归零）'}")
    print()
    print("  ⚠️ 全程不要让电机使能；推关节时注意线缆和气管")
    print()

    measured = {}
    try:
        for j in targets:
            print("─" * 74)
            print(f"  joint{j}")
            for which in ("负方向极限", "正方向极限"):
                try:
                    input(f"    把 joint{j} 推到【{which}】并保持住，然后按回车…")
                except EOFError:
                    print("    （非交互终端，退出）")
                    rd.close()
                    return 2
                v = rd.read(0.25).get(j)
                if v is None:
                    print("    ⚠️ 读不到，跳过")
                    continue
                measured.setdefault(j, {})[which] = v
                print(f"    → 记录 {v:+.4f} rad = {math.degrees(v):+.2f}°")
    except KeyboardInterrupt:
        print("\n  中断")
    finally:
        rd.close()

    if not measured:
        print("\n❌ 没有采到任何数据")
        return 2

    # ── 汇总 ──
    print()
    print("=" * 74)
    print("  标定结果")
    print("=" * 74)

    import yaml
    old_cfg = yaml.safe_load(open(yaml_path, encoding="utf-8")) if os.path.isfile(yaml_path) else {}
    print(f"  {'关节':6s}{'旧下限':>10s}{'新下限':>10s}{'旧上限':>10s}{'新上限':>10s}   变化")
    new_cfg = {}
    for j in sorted(measured):
        lo = measured[j].get("负方向极限")
        hi = measured[j].get("正方向极限")
        old = old_cfg.get(f"joint{j}", {}).get("limit", {})
        olo, ohi = old.get("lower"), old.get("upper")
        if lo is not None and hi is not None and lo > hi:
            lo, hi = hi, lo
            print(f"  ⚠️ joint{j} 两次读数反了（负方向比正方向大），已自动交换 ——"
                  f" 检查 direction 是否是 +1")
        d = []
        if lo is not None and olo is not None:
            d.append(f"下限 {math.degrees(lo - olo):+.1f}°")
        if hi is not None and ohi is not None:
            d.append(f"上限 {math.degrees(hi - ohi):+.1f}°")
        print(f"  joint{j} {('%10.4f' % olo) if olo is not None else '         —'}"
              f" {('%10.4f' % lo) if lo is not None else '         —'}"
              f" {('%10.4f' % ohi) if ohi is not None else '         —'}"
              f" {('%10.4f' % hi) if hi is not None else '         —'}   {'  '.join(d)}")
        new_cfg[f"joint{j}"] = {"lower": lo, "upper": hi}

    print()
    print("  单位 rad；'—' 表示没标定（保持原值）")

    if args.write:
        if os.path.isfile(yaml_path):
            bak = yaml_path + f".bak.{_dt.datetime.now():%Y%m%d_%H%M%S}"
            shutil.copy2(yaml_path, bak)
            print(f"  已备份 → {bak}")
        for j, v in new_cfg.items():
            key = f"joint{j}"
            entry = old_cfg.setdefault(key, {}).setdefault("limit", {})
            if v.get("lower") is not None:
                entry["lower"] = round(float(v["lower"]), 6)
            if v.get("upper") is not None:
                entry["upper"] = round(float(v["upper"]), 6)
            # 保留 effort / velocity
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(old_cfg, f, allow_unicode=True, sort_keys=False)
        print(f"  ✅ 已写入 {yaml_path}")
        print("     接着重新编译： colcon build --symlink-install --packages-select hw_arm_description")
    else:
        print(f"  要写进 yaml： python3 {os.path.relpath(__file__)} --write")
    return 0


if __name__ == "__main__":
    sys.exit(main())
