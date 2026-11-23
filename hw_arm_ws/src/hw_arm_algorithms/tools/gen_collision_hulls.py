#!/usr/bin/env python3
"""从 STL 生成凸包数据头文件（离线，一次）

为什么要在离线生成、而不是在 C++ 里现算：
    本机 coal 编译时**没有开 COAL_HAS_QHULL**（实测 `#ifdef COAL_HAS_QHULL` 未定义），
    `ConvexBase::convexHull()` 不可用；而 `coal::Convex<Triangle>` 的构造函数
    需要"顶点数组 + 三角形索引"，正好可以由本脚本用 scipy 算好后写进头文件。

用法：
    python3 tools/gen_collision_hulls.py \
        --mesh-dir ../hw_arm_description/meshes \
        --out include/hw_arm_algorithms/collision_hulls.hpp

⚠️ 本工程 STL 是**毫米单位**（URDF 里 scale=0.001），脚本会自动 ×0.001。
"""
from __future__ import annotations

import argparse
import os
import struct
import sys

import numpy as np
from scipy.spatial import ConvexHull

LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"]


def read_binary_stl(path: str) -> np.ndarray:
    """读二进制 STL，返回 (N,3) 顶点（单位：毫米）

    ⚠️ 不要用 'solid' 开头判断 ASCII —— 本工程 STL 头是
       "StackForce CAD Importer local STL"，但格式是二进制。
    """
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError(f"{path} 太短")
    n = struct.unpack("<I", data[80:84])[0]
    expect = 84 + 50 * n
    if len(data) != expect:
        raise ValueError(f"{path}: 期望 {expect} 字节，实际 {len(data)}（可能是 ASCII STL）")
    arr = np.frombuffer(data, dtype=np.uint8, count=50 * n, offset=84).reshape(n, 50)
    verts = np.frombuffer(arr[:, 12:48].copy().tobytes(), dtype="<f4").reshape(n, 3, 3)
    return verts.reshape(-1, 3).astype(np.float64)


def emit(links, out_path: str) -> None:
    lines = []
    lines.append("// ⚠️ 自动生成，请勿手改")
    lines.append("// 生成脚本：hw_arm_algorithms/tools/gen_collision_hulls.py")
    lines.append("// 用途：给 coal::Convex<Triangle> 提供顶点 + 三角形索引")
    lines.append("//       本机 coal 未编译 qhull，无法在 C++ 里现算凸包")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <array>")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("namespace hw_arm_algorithms {")
    lines.append("namespace collision_hulls {")
    lines.append("")
    lines.append("struct HullData {")
    lines.append("  const char* link;")
    lines.append("  int num_points;")
    lines.append("  int num_triangles;")
    lines.append("  const double* points;   // 3*num_points，单位 m")
    lines.append("  const int* triangles;   // 3*num_triangles，索引进 points")
    lines.append("};")
    lines.append("")

    # 每个连杆的数组
    for name, verts, tris in links:
        lines.append(f"// ── {name}: {len(verts)} 顶点, {len(tris)} 三角形 ──")
        lines.append(f"static const double k{name}_points[] = {{")
        for v in verts:
            lines.append(f"    {v[0]:.9f}, {v[1]:.9f}, {v[2]:.9f},")
        lines.append("};")
        lines.append(f"static const int k{name}_tris[] = {{")
        for t in tris:
            lines.append(f"    {t[0]}, {t[1]}, {t[2]},")
        lines.append("};")
        lines.append("")

    lines.append("/// 所有连杆的凸包（顺序与 LINKS 一致）")
    lines.append("static const HullData kHulls[] = {")
    for name, verts, tris in links:
        lines.append(f"    {{\"{name}\", {len(verts)}, {len(tris)}, "
                     f"k{name}_points, k{name}_tris}},")
    lines.append("};")
    lines.append("")
    lines.append("static const int kNumHulls = sizeof(kHulls) / sizeof(kHulls[0]);")
    lines.append("")
    lines.append("}  // namespace collision_hulls")
    lines.append("}  // namespace hw_arm_algorithms")
    lines.append("")

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", type=float, default=0.001, help="STL 单位 → m")
    args = ap.parse_args()

    links = []
    total_v = total_t = 0
    for name in LINKS:
        p = os.path.join(args.mesh_dir, name + ".stl")
        if not os.path.isfile(p):
            print(f"❌ 缺 {p}")
            return 1
        v = read_binary_stl(p) * args.scale
        hull = ConvexHull(v)
        pts = v[hull.vertices]                     # (K,3) 凸包顶点
        # scipy 的 simplices 是三角形索引（3D 的凸包面一定是三角形）
        tris = hull.simplices.astype(int)
        # 重新编号：simplices 索引的是原顶点数组，要映射到 pts 的下标
        remap = {int(orig): i for i, orig in enumerate(hull.vertices)}
        tris = np.array([[remap[int(a)], remap[int(b)], remap[int(c)]] for a, b, c in tris])
        links.append((name, pts, tris))
        total_v += len(pts)
        total_t += len(tris)
        print(f"  {name:10s} 三角面 {len(v)//3:6d} → 凸包 {len(pts):4d} 顶点 / {len(tris):4d} 三角形")

    emit(links, args.out)
    print(f"\n  ✅ 写入 {args.out}")
    print(f"     合计 {total_v} 顶点 / {total_t} 三角形")
    return 0


if __name__ == "__main__":
    sys.exit(main())
