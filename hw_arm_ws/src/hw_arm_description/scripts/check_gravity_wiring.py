#!/usr/bin/env python3
"""重力前馈接线自检 —— 防止"参数读了、日志打印了、但根本没进输出"

═══════════════════════════════════════════════════════════════════════
为什么需要这个脚本（2026-11-26 的教训）
═══════════════════════════════════════════════════════════════════════
加 `gravity_trim` 时，我用 Python 的 `str.replace` 改 C++ 源码：

    s.replace('std::clamp(gscale * g[...]', 'std::clamp(gscale * g[...] + ramp * trim[...]')

锚点没匹配上 → **静默什么都没改** → 脚本还打印了 "✅ cpp 应用 trim"。

结果：
    · 参数解析了          ✅
    · 日志打印了          ✅  "恒定力矩偏置修正: [0 0.30 -0.55 0 0 0]"
    · 输出里根本没有它    ❌

于是"传了 -0.55 仍然上漂"被误读成"trim 不够大"，白白多试了两轮。

同类问题还有：`ramp_t0_` 声明了、日志说"斜坡 5.0 s"、但**从来没被赋值**，
导致 `(steady_now() - 0) / T` 恒 >> 1，clamp 后恒为 1 —— 斜坡根本不存在。

═══════════════════════════════════════════════════════════════════════
这个脚本做什么
═══════════════════════════════════════════════════════════════════════
静态检查（不启动 ROS、不碰电机）：
  W01  gravity_trim_ 必须出现在 write() 的力矩表达式里
  W02  ramp_t0_ 必须有赋值（不只是声明和读取）
  W03  速度守护必须用实际模式（zg_now），不能只用启动参数
  W04  零重力服务的进入分支必须有前置条件检查
  W05  kp 必须按 kp_scale 过渡，不能瞬间归零

用法：
    python3 check_gravity_wiring.py            # 检查，失败退出码 1
    python3 check_gravity_wiring.py --verbose
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parent.parent          # .../hw_arm_ws
SRC = WS / "src" / "hw_arm_hardware" / "src" / "hw_arm_hardware.cpp"
HPP = WS / "src" / "hw_arm_hardware" / "include" / "hw_arm_hardware" / "hw_arm_hardware.hpp"

RESULT: list[tuple[str, bool, str]] = []


def check(tag: str, ok: bool, detail: str) -> None:
    RESULT.append((tag, ok, detail))


def find_function_body(text: str, signature: str) -> str | None:
    """粗粒度地截出某个函数体（按大括号配平）"""
    i = text.find(signature)
    if i < 0:
        return None
    j = text.find("{", i)
    if j < 0:
        return None
    depth = 0
    for k in range(j, len(text)):
        if text[k] == "{":
            depth += 1
        elif text[k] == "}":
            depth -= 1
            if depth == 0:
                return text[j : k + 1]
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if not SRC.is_file():
        print(f"❌ 找不到 {SRC}")
        return 2
    src = SRC.read_text(encoding="utf-8")
    hpp = HPP.read_text(encoding="utf-8") if HPP.is_file() else ""

    # ── W01：gravity_trim_ 必须进 write() 的力矩表达式
    body = find_function_body(src, "return_type HwArmHW::write(")
    if body is None:
        check("W01", False, "找不到 write() 函数体")
    else:
        # 找给 gravity_tau_ 赋值所在的**整个 for 块**（trim 常被放进局部变量 ff，
        # 只匹配赋值语句右侧会漏判）
        m = re.search(r"for\s*\([^)]*\)\s*\{[^{}]*gravity_tau_\[[^\]]*\]\s*=[^{}]*\}",
                      body, re.S)
        if m is None:
            check("W01", False, "找不到给 gravity_tau_ 赋值的循环块")
        else:
            blk = m.group(0)
            uses = "gravity_trim_" in blk
            flat = " ".join(blk.split())
            check("W01", uses,
                  "给 gravity_tau_ 赋值的循环块"
                  + ("里引用了 gravity_trim_ ✅" if uses
                     else "**里没有 gravity_trim_ —— trim 不会生效**")
                  + f"\n           片段: {flat[:150]}...")

    # ── W02：ramp_t0_ 必须有赋值
    writes = re.findall(r"ramp_t0_\s*=(?!=)\s*([^;]+);", src)
    check("W02", len(writes) > 0,
          f"ramp_t0_ 被赋值 {len(writes)} 次"
          + (f"（{', '.join(w.strip()[:28] for w in writes)}）" if writes else "")
          + ("" if writes else " —— **只有声明和读取，斜坡恒为满额**"))

    # ── W03：速度守护用实际模式
    gm = re.search(r"^\s*if\s*\((.*zero_gravity_vel_limit_.*)\)\s*\{", src, re.M)
    guard = gm
    if guard is None:
        check("W03", False, "找不到含 zero_gravity_vel_limit_ 的 if 条件")
    else:
        cond = guard.group(1)
        ok = "zg_now" in cond or "zero_gravity_srv_mode_" in cond
        check("W03", ok,
              f"速度守护条件 = `{cond.strip()}`"
              + ("" if ok else " —— **只判启动参数，服务进入的模式绕过守护**"))

    # ── W04：服务进入分支必须有条件检查
    svc = find_function_body(src, 'create_service<std_srvs::srv::SetBool>(')
    if svc is None:
        check("W04", True, "没有零重力服务（可选功能），跳过")
    else:
        need = ["gravity_comp_", "gravity_dry_run_", "gparams_.valid", "motors_ready_"]
        missing = [n for n in need if n not in svc]
        check("W04", not missing,
              "服务回调的进入检查："
              + ("齐全 ✅" if not missing else f"**缺少 {missing}**"))

    # ── W05：kp 按 kp_scale 过渡
    kp_line = re.search(r"const double kp_use\s*=\s*([^;]+);", src)
    if kp_line is None:
        check("W05", False, "找不到 kp_use 的定义")
    else:
        expr = kp_line.group(1)
        ok = "kp_scale" in expr
        check("W05", ok,
              f"kp_use = `{expr.strip()}`"
              + ("" if ok else " —— **kp 会瞬间归零，没有两段式过渡**"))

    # ── 报告
    print("重力前馈接线自检")
    print(f"  源文件: {SRC.relative_to(WS.parent)}")
    print()
    width = 0
    for tag, ok, _ in RESULT:
        print(f"  {'✅' if ok else '❌'} {tag}")
        print(f"       {RESULT[width][2]}")
        width += 1
    nbad = sum(1 for _, ok, _ in RESULT if not ok)
    print()
    if nbad == 0:
        print(f"  ✅ 全部通过（{len(RESULT)} 项）")
        return 0
    print(f"  ❌ {nbad} / {len(RESULT)} 项未通过")
    return 1


if __name__ == "__main__":
    sys.exit(main())
