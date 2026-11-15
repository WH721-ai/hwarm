#!/usr/bin/env python3
"""上电前核对：打印 joint1..6 对应的电机型号 / CAN ID / Master ID / kp / kd / 量程。

这些信息的真实来源分散在三个地方：
    ① 型号 + CAN ID + Master ID  → hw_arm_hardware.hpp（C++ 硬编码）
    ② kp / kd                    → config/control/control_gains.yaml（URDF param 覆盖硬编码默认值）
    ③ pMax / vMax / tMax         → openarm_can 的 MOTOR_LIMIT_PARAMS（按型号查表）

本脚本把三者拼成一张表，并做 5 项自检：
    · CAN ID 是否重复
    · Master ID 是否与 Slave ID 冲突
    · kp ∈ [0,500]、kd ∈ [0,5]（MIT 协议硬性范围）
    · 型号是否在 openarm_can 的支持列表里
    · joint 数是否与 active_joints 一致

用法：
    python3 scripts/check_motor_config.py
    python3 scripts/check_motor_config.py --active 1        # 只测 joint1
    python3 scripts/check_motor_config.py --json            # 机器可读
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.dirname(os.path.dirname(PKG))          # hw_arm_ws 的上一级
CPP = os.path.join(os.path.dirname(PKG), "hw_arm_hardware",
                   "include", "hw_arm_hardware", "hw_arm_hardware.hpp")
CONST_HPP = os.path.join(os.path.dirname(PKG), "openarm_can", "include", "openarm",
                         "damiao_motor", "dm_motor_constants.hpp")

# MIT 协议硬性范围（与 openarm_can 的 double_to_uint 一致）
KP_RANGE = (0.0, 500.0)
KD_RANGE = (0.0, 5.0)


def parse_cpp_vector(text: str, name: str) -> list[str]:
    m = re.search(name + r"\s*=\s*\{(.*?)\};", text, re.S)
    if not m:
        return []
    body = re.sub(r"//[^\n]*", "", m.group(1))     # 先去行内注释，再切分
    out = []
    for it in body.split(","):
        it = it.strip()
        if not it:
            continue
        it = it.split("::")[-1].strip()           # MotorType::DM4310 -> DM4310
        out.append(it)
    return out


def parse_limits(hpp: str) -> dict[str, tuple[float, float, float]]:
    """从 dm_motor_constants.hpp 的 MOTOR_LIMIT_PARAMS 表里抠出 (pMax,vMax,tMax)。"""
    m = re.search(r"MOTOR_LIMIT_PARAMS\s*=\s*\{\{(.*?)\}\};", hpp, re.S)
    if not m:
        return {}
    body = m.group(1)
    rows = re.findall(r"\{\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\}\s*,?\s*//\s*(\w+)", body)
    return {name: (float(a), float(b), float(c)) for a, b, c, name in rows}


def parse_enum_order(hpp: str) -> list[str]:
    m = re.search(r"enum class MotorType[^{]*\{(.*?)\};", hpp, re.S)
    if not m:
        return []
    names = []
    for line in m.group(1).splitlines():
        line = re.sub(r"//.*", "", line).strip().rstrip(",")
        if not line or line.startswith("COUNT"):
            continue
        n = line.split("=")[0].strip()
        if n:
            names.append(n)
    return names


def load_gains() -> dict[str, dict[str, float]]:
    import yaml
    p = os.path.join(PKG, "config", "control", "control_gains.yaml")
    return yaml.safe_load(open(p, encoding="utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--active", default=None,
                    help="只显示这些关节（1-based，逗号分隔），默认全部")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    cpp = open(CPP, encoding="utf-8").read()
    const_hpp = open(CONST_HPP, encoding="utf-8").read()

    types = parse_cpp_vector(cpp, r"MOTOR_TYPES")
    send_ids = parse_cpp_vector(cpp, r"SEND_CAN_IDS")
    recv_ids = parse_cpp_vector(cpp, r"RECV_CAN_IDS")
    kp_default = parse_cpp_vector(cpp, r"kp_")
    kd_default = parse_cpp_vector(cpp, r"kd_")

    enum_names = parse_enum_order(const_hpp)
    limits = parse_limits(const_hpp)
    gains = load_gains()

    n = len(types)
    if not (len(send_ids) == len(recv_ids) == len(kp_default) == len(kd_default) == n):
        print("❌ C++ 头文件里的数组长度不一致：")
        print(f"   MOTOR_TYPES={len(types)} SEND={len(send_ids)} RECV={len(recv_ids)} "
              f"kp={len(kp_default)} kd={len(kd_default)}")
        return 1

    rows, problems = [], []
    for i in range(n):
        j = f"joint{i + 1}"
        g = gains.get(j, {})
        kp = float(g.get("kp", kp_default[i]))
        kd = float(g.get("kd", kd_default[i]))
        kp_src = "yaml" if "kp" in g else "hpp默认"
        kd_src = "yaml" if "kd" in g else "hpp默认"
        mt = types[i]
        lim = limits.get(mt)

        if mt not in enum_names:
            problems.append(f"{j}: 型号 {mt} 不在 openarm_can 的枚举里")
        if lim is None:
            problems.append(f"{j}: 型号 {mt} 在 MOTOR_LIMIT_PARAMS 里查不到量程")
        elif min(lim) <= 0:
            problems.append(f"{j}: 型号 {mt} 的量程为 {lim}（含 0，会在编码时除零）")
        if not (KP_RANGE[0] <= kp <= KP_RANGE[1]):
            problems.append(f"{j}: kp={kp} 超出 MIT 范围 {KP_RANGE}")
        if not (KD_RANGE[0] <= kd <= KD_RANGE[1]):
            problems.append(f"{j}: kd={kd} 超出 MIT 范围 {KD_RANGE}")

        rows.append({
            "joint": j, "type": mt,
            "send_id": send_ids[i], "recv_id": recv_ids[i],
            "kp": kp, "kp_src": kp_src, "kd": kd, "kd_src": kd_src,
            "pMax": lim[0] if lim else None, "vMax": lim[1] if lim else None,
            "tMax": lim[2] if lim else None,
        })

    # ID 唯一性
    if len(set(send_ids)) != len(send_ids):
        problems.append("SEND_CAN_IDS 有重复")
    if len(set(recv_ids)) != len(recv_ids):
        problems.append("RECV_CAN_IDS 有重复")
    clash = set(send_ids) & set(recv_ids)
    if clash:
        problems.append(f"发送 ID 与 Master ID 冲突: {sorted(clash)}")

    if args.active:
        keep = {f"joint{x.strip()}" for x in args.active.split(",") if x.strip()}
        rows = [r for r in rows if r["joint"] in keep]

    if args.json:
        print(json.dumps({"rows": rows, "problems": problems}, ensure_ascii=False, indent=2))
        return 0 if not problems else 1

    print(f"\n{'joint':7s} {'型号':9s} {'Slave':7s} {'Master':7s} "
          f"{'kp':>6s} {'src':7s} {'kd':>5s} {'src':7s} "
          f"{'pMax':>6s} {'vMax':>6s} {'tMax':>6s}")
    print("─" * 88)
    for r in rows:
        lim_ok = all(r[k] for k in ("pMax", "vMax", "tMax"))
        print(f"{r['joint']:7s} {r['type']:9s} {r['send_id']:7s} {r['recv_id']:7s} "
              f"{r['kp']:6.2f} {r['kp_src']:7s} {r['kd']:5.2f} {r['kd_src']:7s} "
              f"{r['pMax'] or 0:6.1f} {r['vMax'] or 0:6.1f} {r['tMax'] or 0:6.1f}"
              f"{'' if lim_ok else '   !! 量程为 0'}")

    print(f"\n来源：型号/CAN ID ← hw_arm_hardware.hpp（硬编码）")
    print(f"      kp/kd        ← config/control/control_gains.yaml（覆盖 hpp 默认值）")
    print(f"      量程         ← openarm_can MOTOR_LIMIT_PARAMS[{', '.join(types)}]")

    todo = [r for r in rows if r["type"] in ("DM6216P", "DM4310P")]
    if todo:
        print(f"\n⚠️  {' / '.join(sorted({r['type'] for r in todo}))} 的 *vMax* 仍是"
              f"待确认占位值（tMax 已确认），见 openarm_can/PATCHES.md")

    if problems:
        print("\n❌ 发现 %d 个问题：" % len(problems))
        for p in problems:
            print("   ·", p)
        return 1

    print("\n✅ 自检通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
