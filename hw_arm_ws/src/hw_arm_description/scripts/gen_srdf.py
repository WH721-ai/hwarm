#!/usr/bin/env python3
"""采样生成 SRDF 的自碰撞禁用矩阵（等价于 MoveIt Setup Assistant 的做法）。

规则：
  · 相邻（同一关节连接）          -> reason="Adjacent"
  · 在 N 组随机位形中从未碰撞      -> reason="Never"
  · 其余保留（真实会碰，必须检查）

用法::

    python3 scripts/gen_srdf.py <urdf> <out.srdf> [--samples 3000]

⚠️ 2026-11-14 修复
    老版本在代码里写死了 link1..link7：
        links = ["base_link"] + [f"link{i}" for i in range(1, 8)] + ["tool0"]
    换 CAD 后模型只有 link1..link6，但脚本照样生成 link7，
    导致 SRDF 引用不存在的 link → MoveIt 加载 RobotModel 直接失败。
    现在改成**从 URDF 动态读取**，再也不会有硬编码残留。
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin


def parse_args():
    ap = argparse.ArgumentParser(description="生成 SRDF 碰撞矩阵")
    ap.add_argument("urdf", help="输入 URDF（建议先 xacro 展开）")
    ap.add_argument("out", help="输出 SRDF")
    ap.add_argument("--samples", type=int, default=3000,
                    help="随机采样位形数（默认 3000）")
    ap.add_argument("--corner-samples", type=int, default=300,
                    help="关节限位角点采样数（默认 300）")
    ap.add_argument("--seed", type=int, default=0)
    return ap.parse_args()


def main() -> int:
    args = parse_args()

    root = ET.parse(args.urdf).getroot()
    all_links = [l.get("name") for l in root.findall("link")]

    m = pin.buildModelFromUrdf(args.urdf)
    gm = pin.buildGeomFromUrdf(m, args.urdf, pin.GeometryType.COLLISION)
    gm.addAllCollisionPairs()
    gd, d = gm.createData(), m.createData()

    owner = {go.name: m.frames[go.parentFrame].name for go in gm.geometryObjects}

    # ★ 只保留**有碰撞几何**的 link，按运动链顺序排列
    #   （没有几何的 link 比如 world / tool0 不参与碰撞，写进矩阵也没意义）
    geom_owners = {owner[go.name] for go in gm.geometryObjects}
    chain_order = ["base_link"]
    for j in root.findall("joint"):
        c = j.find("child").get("link")
        if c in geom_owners and c not in chain_order:
            chain_order.append(c)
    links = [l for l in chain_order if l in geom_owners]
    links += [l for l in all_links if l in geom_owners and l not in links]

    # 相邻对（直接来自 URDF 的 joint 列表，不猜）
    adjacent = set()
    for j in root.findall("joint"):
        a = j.find("parent").get("link")
        b = j.find("child").get("link")
        adjacent.add((a, b))
        adjacent.add((b, a))

    # 可动关节名（给 group_state 用，按 URDF 顺序）
    movable = [j.get("name") for j in root.findall("joint")
               if j.get("type") in ("revolute", "continuous", "prismatic")]

    # 采样
    rng = np.random.default_rng(args.seed)
    lo, hi = m.lowerPositionLimit, m.upperPositionLimit
    samples = [pin.neutral(m)]
    samples += [rng.uniform(lo, hi) for _ in range(args.samples)]
    for _ in range(args.corner_samples):
        pick = rng.integers(0, 2, m.nv)
        samples.append(np.array([lo, hi])[pick, range(m.nv)])

    # 统计每对碰撞命中次数
    hit = {}
    for q in samples:
        pin.computeCollisions(m, d, gm, gd, q, False)
        for k, cp in enumerate(gm.collisionPairs):
            if gd.collisionResults[k].isCollision():
                a = owner[gm.geometryObjects[cp.first].name]
                b = owner[gm.geometryObjects[cp.second].name]
                key = tuple(sorted((a, b)))
                hit[key] = hit.get(key, 0) + 1

    # ── 生成 SRDF ────────────────────────────────────────────────
    lines = [
        '<?xml version="1.0"?>',
        '<robot name="hw_arm">',
        f'  <!-- 由 scripts/gen_srdf.py 采样 {len(samples)} 组位形自动生成',
        '       相邻 -> Adjacent；全程未碰撞 -> Never；其余保留检查',
        '       重新生成: python3 scripts/gen_srdf.py <urdf> <out.srdf> -->',
        '  <group name="arm">',
        '    <chain base_link="base_link" tip_link="tool0"/>',
        '  </group>',
        '',
        '  <group_state name="home" group="arm">',
    ]
    for jn in movable:
        lines.append(f'    <joint name="{jn}" value="0"/>')
    lines += ['  </group_state>', '']
    lines += [
        '  <!-- 立起来的姿态：joint2 抬到 90° -->',
        '  <group_state name="upright" group="arm">',
    ]
    for jn in movable:
        v = "1.5708" if jn == "joint2" else "0"
        lines.append(f'    <joint name="{jn}" value="{v}"/>')
    lines += ['  </group_state>', '']

    n_adj = n_never = n_keep = 0
    kept = []
    for i, a in enumerate(links):
        for b in links[i + 1:]:
            key = tuple(sorted((a, b)))
            if (a, b) in adjacent:
                lines.append(f'  <disable_collisions link1="{a}" link2="{b}" '
                             f'reason="Adjacent"/>')
                n_adj += 1
            elif hit.get(key, 0) == 0:
                lines.append(f'  <disable_collisions link1="{a}" link2="{b}" '
                             f'reason="Never"/>')
                n_never += 1
            else:
                kept.append((key, hit[key]))
                n_keep += 1

    lines += ['</robot>', '']
    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    total = len(links) * (len(links) - 1) // 2
    print(f"links({len(links)}) = {links}")
    print(f"候选对 {total}：Adjacent {n_adj} / Never {n_never} / **保留检查 {n_keep}**")
    if kept:
        print("真实会碰撞、必须检查的对：")
        for k, v in sorted(kept, key=lambda x: -x[1]):
            print(f"   {k[0]} <-> {k[1]}   采样命中 {v} 次")
    else:
        print("（没有需要保留检查的对 —— 全位形内无自碰撞）")
    print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
