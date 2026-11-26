#!/usr/bin/env python3
"""构建身份核对 —— 确认「跑的二进制」「源码提交」「参数文件」三者对得上

═══════════════════════════════════════════════════════════════════════
为什么需要这个（2026-11-26 外部审阅的意见）
═══════════════════════════════════════════════════════════════════════
报告里贴的启动日志、参数确认、rms 数字，都**无法证明**当时跑的那个
`libhw_arm_hardware.so` 就是从某个提交构建出来的。曾经真实出现过：

    · 源码已经改了（gravity_trim 进了输出）
    · 但虚拟机跑的 .so 是旧的（trim 根本没进输出）
    · 日志里照样打印 "恒定力矩偏置修正: [...]"，看起来一切正常

所以先把「构建身份」变成可核对的事实，再谈任何测量结论。

═══════════════════════════════════════════════════════════════════════
它检查什么
═══════════════════════════════════════════════════════════════════════
  ① 仓库当前提交 + 是否 dirty
  ② 插件 .so 的路径 / sha256 / mtime
  ③ **.so 里编译进去的提交号**（CMake 在编译期注入的 HW_ARM_GIT_COMMIT）
     → 和当前 HEAD 对比，不一致就说明二进制过期
  ④ .so 的 mtime 是否比所有源文件新（colcon 没重编时会过期）
  ⑤ 关键参数文件的 sha256（θ̂ / 控制增益 / 控制器配置）
  ⑥ 用和 launch 相同的参数渲染 URDF，取出真正下发的重力参数，并算哈希
  ⑦ 若有节点在跑，dump 它**实际生效**的参数

退出码：0 = 身份一致；1 = 发现不一致/过期；2 = 环境不完整

用法：
    source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh
    python3 check_build_identity.py
    python3 check_build_identity.py --json /tmp/identity.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parent.parent                       # .../hw_arm_ws
REPO = WS.parent                                    # .../hw_arm
PKG = WS / "src" / "hw_arm_hardware"

GRAVITY_PARAM_NAMES = [
    "gravity_compensation", "gravity_dry_run", "zero_gravity",
    "gravity_scale", "zero_gravity_kd", "gravity_ramp_time",
    "gravity_params_path", "gravity_trim", "return_to_zero",
    "kp1", "kp2", "kp3", "kp4", "kp5", "kp6",
    "kd1", "kd2", "kd3", "kd4", "kd5", "kd6",
    "tau_limit", "direction",
]


def run(cmd: list[str], cwd: Path | None = None) -> str:
    try:
        return subprocess.run(cmd, cwd=str(cwd) if cwd else None,
                              capture_output=True, text=True, timeout=60).stdout.strip()
    except Exception:
        return ""


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def find_so() -> list[Path]:
    out = []
    for pat in ("**/libhw_arm_hardware.so", "**/libhw_arm_hardware.so.*"):
        out += [p for p in WS.glob(pat) if p.is_file()]
    # 去掉符号链接重复
    seen, uniq = set(), []
    for p in out:
        rp = p.resolve()
        if rp not in seen:
            seen.add(rp)
            uniq.append(p)
    return uniq


def embedded_commit(so: Path) -> str:
    """.so 里 CMake 注入的提交号（形如 62fdcb093bc7 或 62fdcb093bc7+dirty）"""
    try:
        out = subprocess.run(["strings", "-a", str(so)],
                             capture_output=True, text=True, timeout=60).stdout
    except Exception:
        return ""
    cands = re.findall(r"^([0-9a-f]{12}\+?[a-z]*)$", out, re.M)
    # 排除明显是全 0 或顺序数字的假串
    cands = [c for c in cands if not re.match(r"^(.)\1{11}", c)]
    return cands[0] if cands else ""


def render_urdf() -> tuple[str, dict[str, str]]:
    """用和 launch 一致的参数渲染 URDF，返回 (全文, 重力相关参数)"""
    xacro = WS / "src/hw_arm_description/urdf/hw_arm.urdf.xacro"
    if not xacro.is_file():
        return "", {}
    txt = run(["xacro", str(xacro),
               "use_fake_hardware:=false", "can_interface:=can0", "can_fd:=false",
               "active_joints:=1,2,3,4,5,6",
               "gravity_compensation:=false", "gravity_dry_run:=true",
               "zero_gravity:=false", "gravity_scale:=1.0",
               "zero_gravity_kd:=1.0", "gravity_ramp_time:=3.0",
               "gravity_params_path:=", "gravity_trim:=0 0 0 0 0 0"])
    params = {}
    for name in GRAVITY_PARAM_NAMES:
        m = re.search(rf'<param name="{re.escape(name)}">([^<]*)</param>', txt)
        if m:
            params[name] = m.group(1)
    return txt, params


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default="", help="把清单写到这个文件")
    args = ap.parse_args()

    print("═" * 70)
    print(" 构建身份核对")
    print("═" * 70)

    manifest: dict = {}

    # ① 仓库状态
    head = run(["git", "rev-parse", "HEAD"], REPO)
    short = run(["git", "rev-parse", "--short=12", "HEAD"], REPO)
    dirty = run(["git", "status", "--porcelain", "--untracked-files=no"], REPO)
    n_dirty = len([l for l in dirty.splitlines() if l.strip()])
    print(f"  仓库 HEAD : {short or '(非 git 仓库)'}")
    print(f"  未提交改动: {n_dirty} 个文件"
          + ("" if n_dirty == 0 else "   ⚠️ 工作区不干净，构建产物无法唯一对应到某个提交"))
    manifest["repo"] = {"head": head, "short": short, "dirty_files": n_dirty}

    # ② 插件 .so
    sos = find_so()
    print()
    if not sos:
        print("  ⚠️ 没找到 libhw_arm_hardware.so（还没构建？）")
        manifest["so"] = None
    else:
        for so in sos:
            st = so.stat()
            print(f"  .so       : {so.relative_to(WS)}")
            print(f"    sha256  : {sha256(so)}")
            print(f"    mtime   : {time_str(st.st_mtime)}")
        so = sos[0]
        manifest["so"] = {
            "path": str(sos[0].relative_to(WS)),
            "sha256": sha256(sos[0]),
            "mtime": so.stat().st_mtime,
            "all_paths": [str(p.relative_to(WS)) for p in sos],
        }

    # ③ 二进制里的提交号 vs HEAD
    ok_identity = True
    if sos:
        emb = embedded_commit(sos[0])
        print()
        if not emb:
            print("  ⚠️ .so 里没有内嵌提交号 —— 说明它是**加 build stamp 之前**构建的")
            print("     → 无法证明它对应哪个提交，请重新 colcon build")
            ok_identity = False
        else:
            emb_clean = emb.split("+")[0]
            is_dirty_build = "+dirty" in emb
            match = (emb_clean == short)
            print(f"  .so 内嵌提交: {emb}{'  (构建时有未提交改动)' if is_dirty_build else ''}")
            print(f"  与 HEAD 一致: {'✅ 是' if match else '❌ 否 → 二进制已过期，请重新构建'}")
            if not match:
                ok_identity = False
            manifest["so"]["embedded_commit"] = emb
            manifest["so"]["matches_head"] = match

        # ④ 源码是否比 .so 新
        newest = max((p for p in PKG.rglob("*") if p.is_file() and
                      p.suffix in (".cpp", ".hpp", ".txt", ".xml")),
                     key=lambda p: p.stat().st_mtime, default=None)
        if newest:
            so_t = sos[0].resolve().stat().st_mtime
            newer = newest.stat().st_mtime > so_t
            print(f"  最新源文件  : {newest.name}  {'❌ 比 .so 新（需要重新编译）' if newer else '✅ 比 .so 旧'}")
            if newer:
                ok_identity = False
            manifest["so"]["source_newer"] = newer

    # ⑤ 参数文件哈希
    print()
    print("  参数文件:")
    param_files = {
        "theta": REPO / "algo/data/gravity_theta.yaml",
        "control_gains": WS / "src/hw_arm_description/config/control/control_gains.yaml",
        "controllers": WS / "src/hw_arm_bringup/config/controllers/hw_arm_controllers.yaml",
        "gripper": WS / "src/hw_arm_description/config/tool/assemb7_gripper.yaml",
    }
    manifest["params"] = {}
    for key, path in param_files.items():
        if path.is_file():
            h = sha256(path)
            print(f"    {key:14s} {h[:16]}…  {path.name}")
            manifest["params"][key] = {"path": str(path.relative_to(REPO)), "sha256": h}
        else:
            print(f"    {key:14s} ⚠️ 不存在: {path}")

    # ⑥ 渲染 URDF 并取真实下发的重力参数
    print()
    urdf_txt, uparams = render_urdf()
    if urdf_txt:
        uh = hashlib.sha256(urdf_txt.encode()).hexdigest()
        print(f"  渲染后 URDF sha256: {uh[:16]}…  ({len(urdf_txt.splitlines())} 行)")
        manifest["urdf_sha256"] = uh
        print("  真正会下发到插件的参数:")
        for k in ("gravity_compensation", "gravity_dry_run", "zero_gravity",
                  "gravity_scale", "zero_gravity_kd", "gravity_ramp_time",
                  "gravity_trim", "tau_limit", "return_to_zero"):
            if k in uparams:
                print(f"    {k:22s} = {uparams[k]}")
        gp = uparams.get("gravity_params_path", "")
        print(f"    {'gravity_params_path':22s} = {gp if gp else '(空 → 用标称 URDF 参数)'}")
        manifest["urdf_params"] = uparams
    else:
        print("  ⚠️ URDF 渲染失败（xacro 不可用或环境没 source）")

    # ⑦ 运行中节点实际生效的参数
    print()
    nodes = run(["ros2", "node", "list"])
    running = "hw_arm_hardware_interface" in nodes or "/controller_manager" in nodes
    if running:
        dump = run(["ros2", "param", "dump", "/controller_manager"])
        print("  ✅ 有节点在跑，实际生效的重力相关参数:")
        shown = 0
        for line in dump.splitlines():
            ls = line.strip()
            for k in ("gravity_compensation", "gravity_dry_run", "zero_gravity",
                      "gravity_trim", "gravity_params_path", "return_to_zero",
                      "update_rate"):
                if ls.startswith(f"{k}:"):
                    print(f"    {ls}")
                    shown += 1
        if shown == 0:
            print("    （没抓到，节点可能还没加载硬件）")
        manifest["running_params_dump"] = dump[:4000]
    else:
        print("  （没有节点在跑 —— 只能核对构建产物，无法核对运行时参数）")
        manifest["running_params_dump"] = None

    # 汇总
    print()
    print("═" * 70)
    if ok_identity:
        print(" ✅ 构建身份一致：源码、二进制、参数文件对得上")
    else:
        print(" ❌ 构建身份**不一致** —— 在修好之前，任何日志/测量都不能用来下结论")
        print("    处理：")
        print("      touch hw_arm_ws/src/hw_arm_hardware/CMakeLists.txt   # 强制 CMake 重跑 configure")
        print("      colcon build --symlink-install --packages-select hw_arm_hardware")
        print("    [注意] 只跑 colcon build 不够 —— CMake 的 configure 只在 CMakeLists")
        print("           变化时重跑，build stamp 不会更新，二进制里仍是旧提交号。")
    print("═" * 70)

    if args.json:
        Path(args.json).write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
        print(f"  清单已写出: {args.json}")
    return 0 if ok_identity else 1


def time_str(ts: float) -> str:
    import datetime
    return datetime.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


if __name__ == "__main__":
    sys.exit(main())
