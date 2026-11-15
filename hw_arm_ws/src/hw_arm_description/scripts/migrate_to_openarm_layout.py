#!/usr/bin/env python3
"""把旧版 config/{joints,links}.yaml 迁移到 OpenArm 风格的目录结构。

旧结构（2 个文件，几何/惯性/控制混在一起）：
    config/joints.yaml   config/links.yaml

新结构（照搬 OpenArm 的分层）：
    config/struct/{topology,name_mapping,reference_points}.yaml
    config/joint/{joint_origins,joint_axes,joint_limits}.yaml
    config/link/{visuals,collisions}.yaml
    config/inertials/nominal.yaml
    config/control/{control_gains,friction}.yaml

分类原则（按"每个文件的功能"）：
    topology      ← order / parent / child / type
    joint_origins ← origin（在哪）
    joint_axes    ← axis（绕哪个方向转）
    joint_limits  ← limit（能转多少）
    visuals       ← links.body + joints.housing（看得见的东西）
    collisions    ← links.collision（碰撞体）
    inertials     ← mass / com / inertia（质量属性）
    control       ← 原版没有增益/摩擦数据 → 空文件

用法：
    python3 scripts/migrate_to_openarm_layout.py            # 写文件
    python3 scripts/migrate_to_openarm_layout.py --dry-run  # 只打印
"""
from __future__ import annotations

import argparse
import os
import sys

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGACY_JOINTS = os.path.join(PKG, "config", "joints.yaml")
LEGACY_LINKS = os.path.join(PKG, "config", "links.yaml")

# tool0 不在旧 YAML 里（写死在 xacro 中），迁移时补进来
TOOL0_LINK = "tool0"
TOOL0_JOINT = "tool0_joint"
TOOL0_VISUAL = {
    "origin": {"x": 0.0, "y": 0.0, "z": 0.012, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "geometry": {"type": "cylinder", "radius": 0.012, "length": 0.024},
    "color": {"r": 0.10, "g": 0.80, "b": 0.30, "a": 1.0},
}

HEADER = "# {title}\n#\n# {note}\n# 由 scripts/migrate_to_openarm_layout.py 从旧版 config/{{joints,links}}.yaml 生成\n"


def dump(items: list[str]) -> str:
    """拼成文本并去掉块之间的多余空行。"""
    return "\n".join(x.rstrip("\n") for x in items) + "\n"


def fnum(x: float) -> float:
    """去掉浮点噪声，保持 YAML 可读。"""
    x = float(x)
    if abs(x) < 1e-12:
        return 0.0
    return round(x, 9)


def origin_block(o: dict, indent: str = "  ") -> str:
    return (
        f"{indent}origin:\n"
        f"{indent}  x: {fnum(o['x'])}\n"
        f"{indent}  y: {fnum(o['y'])}\n"
        f"{indent}  z: {fnum(o['z'])}\n"
        f"{indent}  roll: {fnum(o.get('roll', 0.0))}\n"
        f"{indent}  pitch: {fnum(o.get('pitch', 0.0))}\n"
        f"{indent}  yaw: {fnum(o.get('yaw', 0.0))}\n"
    )


def cyl(radius: float, length: float) -> dict:
    """旧数据把 radius/length 直接放在 body/collision/housing 里，这里包一层。"""
    return {"geometry": {"type": "cylinder", "radius": radius, "length": length}}


def geometry_block(g: dict, indent: str = "  ") -> str:
    """圆柱：type=cylinder；换成 CAD 网格时写 type=mesh + filename + scale。"""
    geo = g["geometry"]
    out = f"{indent}geometry:\n{indent}  type: {geo['type']}\n"
    if geo["type"] == "cylinder":
        out += f"{indent}  radius: {fnum(geo['radius'])}\n"
        out += f"{indent}  length: {fnum(geo['length'])}\n"
    elif geo["type"] == "mesh":
        out += f"{indent}  filename: {geo['filename']}\n"
        s = geo.get("scale", {"x": 1.0, "y": 1.0, "z": 1.0})
        out += f"{indent}  scale: {{x: {s['x']}, y: {s['y']}, z: {s['z']}}}\n"
    return out


def color_block(c: dict, indent: str = "  ") -> str:
    return (
        f"{indent}color:\n"
        f"{indent}  r: {fnum(c['r'])}\n"
        f"{indent}  g: {fnum(c['g'])}\n"
        f"{indent}  b: {fnum(c['b'])}\n"
        f"{indent}  a: {fnum(c.get('a', 1.0))}\n"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not (os.path.exists(LEGACY_JOINTS) and os.path.exists(LEGACY_LINKS)):
        print("旧版 config/{joints,links}.yaml 已删除 —— 迁移早已完成。")
        print("现在源文件是 config/{struct,joint,link,inertials,control}/ 下的 YAML，请直接改那些。")
        print("（若需要重跑迁移，先 git checkout 旧文件，或从 /tmp/hw_backup 恢复）")
        return 0

    with open(LEGACY_JOINTS, encoding="utf-8") as f:
        old_joints = yaml.safe_load(f)
    with open(LEGACY_LINKS, encoding="utf-8") as f:
        old_links = yaml.safe_load(f)

    order = old_joints["order"]
    # 链顺序：base_link -> link1..link6 -> tool0
    link_names = ["base_link"] + [old_joints[j]["child"] for j in order] + [TOOL0_LINK]
    joint_names = list(order) + [TOOL0_JOINT]

    files: dict[str, str] = {}

    # ---------------------------------------------------------------- struct
    topo = [HEADER.format(title="拓扑结构（谁连谁）", note="links[] 的顺序就是运动链顺序；chain[] 是关节顺序。")]
    topo.append(f"links: [{', '.join(link_names)}]\n")
    topo.append(f"chain: [{', '.join(joint_names)}]\n")
    topo.append("joints:")
    for j in order:
        c = old_joints[j]
        topo.append(f"  {j}:")
        topo.append(f"    type: {c['type']}")
        topo.append(f"    parent: {c['parent']}")
        topo.append(f"    child: {c['child']}")
    topo.append(f"  {TOOL0_JOINT}:")
    topo.append("    type: fixed")
    topo.append(f"    parent: {old_joints[order[-1]]['child']}")
    topo.append(f"    child: {TOOL0_LINK}")
    files["config/struct/topology.yaml"] = dump(topo)

    nm = [HEADER.format(title="名称映射（规范名 ↔ 原始名）", note="当前没有 CAD 原始名，全部同名；接入 SolidWorks 导出后填 original 列。")]
    nm.append("links:")
    nm.append("  canonical_to_original:")
    for n in link_names:
        nm.append(f"    {n}: {n}")
    nm.append("  original_to_canonical:")
    for n in link_names:
        nm.append(f"    {n}: {n}")
    nm.append("joints:")
    nm.append("  canonical_to_original:")
    for j in joint_names:
        nm.append(f"    {j}: {j}")
    nm.append("  original_to_canonical:")
    for j in joint_names:
        nm.append(f"    {j}: {j}")
    files["config/struct/name_mapping.yaml"] = dump(nm)

    files["config/struct/reference_points.yaml"] = (
        HEADER.format(
            title="装配参考点（挂载 / TCP / 抓取坐标系）",
            note="旧版数据里没有装配参考点：tool0 作为法兰已写在 topology 里。\n"
                 "# 接入 CAD 后在这里补 left_arm_mount_point / grasp_frame 之类的点，\n"
                 "# 格式参考 OpenArm：<name>: {parent: <link>, origin: {x,y,z,roll,pitch,yaw}}",
        )
        + "\n# （当前为空）\n"
    )

    # ---------------------------------------------------------------- joint
    jo = [HEADER.format(title="关节原点（关节中心在哪 + 子坐标系怎么摆）", note="来自旧版 joints.yaml 的 origin。")]
    for j in order:
        jo.append(f"{j}:")
        jo.append(origin_block(old_joints[j]["origin"]))
    files["config/joint/joint_origins.yaml"] = dump(jo)

    ja = [HEADER.format(title="关节转轴（绕哪个方向转）", note="当前约定：每个关节的转轴都等于自身 z 轴。")]
    for j in order:
        a = old_joints[j]["axis"]
        ja.append(f"{j}:")
        ja.append("  axis:")
        ja.append(f"    x: {fnum(a['x'])}")
        ja.append(f"    y: {fnum(a['y'])}")
        ja.append(f"    z: {fnum(a['z'])}")
    files["config/joint/joint_axes.yaml"] = dump(ja)

    jl = [HEADER.format(title="关节限位（能转多少）", note="来自旧版 joints.yaml 的 limit；effort 单位 N·m，velocity 单位 rad/s。")]
    for j in order:
        lim = old_joints[j]["limit"]
        jl.append(f"{j}:")
        jl.append("  limit:")
        jl.append(f"    lower: {fnum(lim['lower'])}")
        jl.append(f"    upper: {fnum(lim['upper'])}")
        jl.append(f"    effort: {fnum(lim['effort'])}")
        jl.append(f"    velocity: {fnum(lim['velocity'])}")
    files["config/joint/joint_limits.yaml"] = dump(jl)

    # ---------------------------------------------------------------- link
    vis = [
        HEADER.format(
            title="视觉几何（看得见的形状）",
            note="每个 link 可以有多个 visual（本体 + 关节壳体）。\n"
                 "# geometry.type: cylinder（占位）| mesh（接入 CAD 后换成 filename + scale）",
        )
    ]
    for n in link_names:
        vis.append(f"{n}:")
        if n == TOOL0_LINK:
            vis.append("  - " + origin_block(TOOL0_VISUAL["origin"], indent="    ").lstrip())
            vis.append(geometry_block(TOOL0_VISUAL, indent="    "))
            vis.append(color_block(TOOL0_VISUAL["color"], indent="    "))
        elif n in old_links:
            body = old_links[n]["body"]
            vis.append("  - " + origin_block(body["origin"], indent="    ").lstrip())
            vis.append(geometry_block(cyl(body["radius"], body["length"]), indent="    "))
            if "color" in body:
                vis.append(color_block(body["color"], indent="    "))
            # 关节壳体：挂在 child link 上（仅视觉）
            for j in order:
                if old_joints[j]["child"] == n and "housing" in old_joints[j]:
                    h = old_joints[j]["housing"]
                    vis.append("  - " + origin_block(h["origin"], indent="    ").lstrip())
                    vis.append(
                        "    geometry:\n"
                        "      type: cylinder\n"
                        f"      radius: {fnum(h['radius'])}\n"
                        f"      length: {fnum(h['length'])}\n"
                    )
                    if "color" in h:
                        vis.append(color_block(h["color"], indent="    "))
    files["config/link/visuals.yaml"] = dump(vis)

    col = [
        HEADER.format(
            title="碰撞几何（碰撞检测用的包络）",
            note="比视觉模型小一圈（两端缩进 + 半径 0.8 倍），且不含关节壳体；\n"
                 "# 零长度 link（tool0）没有体积，不生成碰撞体。",
        )
    ]
    for n in link_names:
        if n in old_links and "collision" in old_links[n]:
            c = old_links[n]["collision"]
            col.append(f"{n}:")
            col.append("  - " + origin_block(c["origin"], indent="    ").lstrip())
            col.append(geometry_block(cyl(c["radius"], c["length"]), indent="    "))
    files["config/link/collisions.yaml"] = dump(col)

    # ---------------------------------------------------------------- inertials
    inert = [HEADER.format(title="惯性参数（质量 / 质心 / 惯量）", note="nominal = 标称值（来自 OpenArm 设计值）；实测标定后另写 measured.yaml。")]
    for n in link_names:
        if n not in old_links:
            continue
        L = old_links[n]
        i = L["inertia"]
        inert.append(f"{n}:")
        inert.append(origin_block(L["com"], indent="  "))
        inert.append(f"  mass: {fnum(L['mass'])}")
        inert.append("  inertia:")
        inert.append(f"    xx: {fnum(i['ixx'])}")
        inert.append(f"    xy: {fnum(i['ixy'])}")
        inert.append(f"    xz: {fnum(i['ixz'])}")
        inert.append(f"    yy: {fnum(i['iyy'])}")
        inert.append(f"    yz: {fnum(i['iyz'])}")
        inert.append(f"    zz: {fnum(i['izz'])}")
    files["config/inertials/nominal.yaml"] = dump(inert)

    # ---------------------------------------------------------------- control
    files["config/control/control_gains.yaml"] = (
        HEADER.format(
            title="MIT 控制增益（kp / kd）",
            note="旧版数据里没有增益（还没上实机）；上机标定后按下面格式填：\n"
                 "# joint1: {kp: 70.0, kd: 2.75}",
        )
        + "\n# （当前为空）\n"
    )
    files["config/control/friction.yaml"] = (
        HEADER.format(title="摩擦模型参数", note="旧版数据里没有摩擦辨识结果。") + "\n# （当前为空）\n"
    )

    for path, content in files.items():
        full = os.path.join(PKG, path)
        rel = os.path.relpath(full, PKG)
        if args.dry_run:
            print(f"=== {rel} ===\n{content}")
            continue
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"写出 {rel:44s} {len(content.splitlines()):4d} 行")
    return 0


if __name__ == "__main__":
    sys.exit(main())
