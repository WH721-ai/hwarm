#!/usr/bin/env python3
"""hw_arm 的 MuJoCo 仿真环境（**只做环境 + 本体，不含任何控制算法**）。

════════════════════════════════════════════════════════════════════
职责边界（重要）
════════════════════════════════════════════════════════════════════
    本包 / 本脚本只负责：
        ① 搭建 MuJoCo 物理环境（几何 / 惯性 / 接触 / 执行器 / 地面）
        ② 让仿真尽可能对齐真机（同样的 ctrlrange、同样的关节限位）
        ③ 把控制量原样喂给 MuJoCo，并把仿真内部量打印出来

    **不包含任何控制算法**：
        · 没有 PD 律
        · 没有重力补偿
        · 没有 IK / 轨迹生成
        · 不读取 control_gains.yaml

    控制算法一律在 hw_arm_algorithms 里，由**外部控制器**产生力矩后，
    通过 torque 模式（或将来接一个订阅 cmd 话题的桥）喂进来。

════════════════════════════════════════════════════════════════════
前置：先生成 MJCF
════════════════════════════════════════════════════════════════════
    python3 scripts/urdf_to_mjcf.py        # → mjcf/hw_arm.xml + scene.xml

════════════════════════════════════════════════════════════════════
关于"底座固定"
════════════════════════════════════════════════════════════════════
URDF 里 base_link 通过 world_joint（fixed）挂在 `world` 上，MuJoCo 合并进 world body：
    · base_link 刚性焊死在地面坐标系（nbody=7：world + link1..link6）
    · 重力只会让 link1~link6 动，底座不会跟着掉

════════════════════════════════════════════════════════════════════
两种模式（--mode）
════════════════════════════════════════════════════════════════════
  gravity  默认。ctrl 恒 0 —— 机械臂在重力下自由下坠，撞到关节限位停住。
           底座不动。纯物理，没有任何控制。

  torque   ctrl 交给 viewer 右侧 Control 面板的滑条，手动给每个关节 N·m
           （滑条范围 = 额定扭矩）。这是"外部给力矩、仿真只执行"的最小形式：
           你自己当控制器。

════════════════════════════════════════════════════════════════════
手动拖关节（两种模式都能用）
════════════════════════════════════════════════════════════════════
  ① Ctrl + 左键双击某个连杆   → 选中
  ② 按住 Ctrl + 左键拖动       → 施加力/力矩
  ③ 松开                        → 力撤掉

  mujoco 的 passive viewer 只更新 viewer.perturb，不会替你施加力，
  所以脚本里显式调用了 mjv_applyPerturbPose / mjv_applyPerturbForce。

════════════════════════════════════════════════════════════════════
其他开关
════════════════════════════════════════════════════════════════════
  --scene PATH       指定 scene.xml（默认 mjcf/scene.xml）
  --duration N       跑 N 秒（仿真时间）后退出
  --headless         不开 viewer，只跑物理（CI / 远程验证）
  --print-period N   终端打印周期（秒，仿真时间），默认 0.1

════════════════════════════════════════════════════════════════════
终端输出
════════════════════════════════════════════════════════════════════
         |       j1       j2       j3       j4       j5       j6
  ── t= 0.201s
   q(deg)|    -0.00   -10.27    -0.02    -0.02    -5.79    -0.00
     ctrl|    0.000    0.000    0.000    0.000    0.000    0.000   外部给的
 qfrc_act|    0.000    0.000    0.000    0.000    0.000    0.000   实际输出
     mj_g|    0.000    2.553    2.536    0.014    0.188    0.000   MuJoCo 内部量
 qfrc_con|   -0.001    2.115    2.538    0.014    0.100    0.000   限位反力

  · ctrl       外部下发的力矩（gravity 模式恒 0；torque 模式来自滑条）
  · qfrc_act   电机实际输出（= ctrl 经 forcerange 限幅）
  · mj_g       MuJoCo 自己的 qfrc_bias（**仅观察用**，不参与任何控制）
  · qfrc_con   限位/接触反力（撞限位时迅速涨起来）

  时间戳 t 是仿真时间，不是墙钟。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def pkg_share(name: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(name)
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.abspath(os.path.join(here, "..", "..", name))


# 表头（只打一次）
JOINT_HEADER = f"{'':>9s}|" + "".join(f"{f'j{i+1}':>9s}" for i in range(6))


def print_state(t, data):
    """打印 6 个电机的力矩（N·m）+ 关节角。全部是仿真内部量，不做任何计算。"""

    def row(label, v, fmt="{:>9.3f}"):
        return f"{label:>9s}|" + "".join(fmt.format(x) for x in v)

    print(f"  ── t={t:6.3f}s")
    print(row("q(deg)", np.degrees(data.qpos[:6]), "{:>9.2f}"))
    print(row("ctrl", data.ctrl[:6]) + "   外部给的")
    print(row("qfrc_act", data.qfrc_actuator[:6]) + "   实际输出")
    print(row("mj_g", data.qfrc_bias[:6]) + "   MuJoCo 内部量")
    print(row("qfrc_con", data.qfrc_constraint[:6]) + "   限位反力")


def main() -> int:
    ap = argparse.ArgumentParser(description="hw_arm MuJoCo 仿真环境（不含控制算法）")
    ap.add_argument("--scene", default=None, help="scene.xml 路径")
    ap.add_argument("--mode", default="gravity", choices=["gravity", "torque"],
                    help="gravity=ctrl 恒 0（自由下坠）；torque=手动滑条给力矩")
    ap.add_argument("--duration", type=float, default=0.0,
                    help=">0 时跑这么多**仿真秒**后自动退出")
    ap.add_argument("--headless", action="store_true",
                    help="不开 viewer，只跑物理")
    ap.add_argument("--print-period", type=float, default=0.1,
                    help="终端打印周期（秒，仿真时间），默认 0.1")
    args = ap.parse_args()

    import mujoco
    import mujoco.viewer

    share = pkg_share("hw_arm_mujoco")
    scene = args.scene or os.path.join(share, "mjcf", "scene.xml")
    if not os.path.isfile(scene):
        scene = os.path.abspath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "mjcf", "scene.xml"))
    if not os.path.isfile(scene):
        print(f"❌ 找不到 {scene}\n   先运行: python3 scripts/urdf_to_mjcf.py")
        return 2

    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)

    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
    mujoco.mj_forward(model, data)

    base_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                 "base_link_collision")
    if base_gid < 0:
        base_gid = 0
    base_pos0 = data.geom_xpos[base_gid].copy()

    print("=" * 70)
    print(f"  场景   : {scene}")
    print(f"  模型   : nbody={model.nbody}  njnt={model.njnt}  nu={model.nu}")
    print(f"  总质量 : {np.sum(model.body_mass):.6f} kg   "
          f"timestep={model.opt.timestep*1000:.1f} ms")
    print(f"  模式   : {args.mode}")
    print("  说明   : 本脚本不含任何控制算法；重力/惯性/接触全部由 MuJoCo 计算")
    print(f"  滑条范围 : ±{np.round(model.actuator_ctrlrange[:, 1], 2)} N·m"
          "  （额定扭矩，改峰值用 urdf_to_mjcf.py --motor-tmax）")
    print("  鼠标拖关节：Ctrl + 左键双击选中连杆，再 Ctrl + 左键拖动")
    print("=" * 70)
    print(JOINT_HEADER)

    next_print = 0.0

    # ── 无头 ──────────────────────────────────────────────────
    if args.headless:
        while True:
            if args.mode == "gravity":
                data.ctrl[:] = 0.0          # 纯物理：不给任何力矩
            # torque 模式：不碰 ctrl（无头时它保持 0）
            mujoco.mj_step(model, data)
            now = data.time
            if now >= next_print:
                print_state(now, data)
                next_print = now + args.print_period
            if args.duration > 0 and now >= args.duration:
                break
        print("\n[headless] 最终 q =", np.round(data.qpos[:6], 5))
        print("[headless] 底座几何体位移 =",
              f"{np.linalg.norm(data.geom_xpos[base_gid] - base_pos0):.3e} m"
              "（应恒为 0 → base_link 焊死在地面）")
        return 0

    # ── 带 viewer ────────────────────────────────────────────
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.15]
        viewer.cam.distance = 0.85
        viewer.cam.azimuth = 130
        viewer.cam.elevation = -18

        while viewer.is_running():
            step_start = time.time()

            if args.mode == "gravity":
                data.ctrl[:] = 0.0          # 纯物理：不给任何力矩
            # torque 模式：ctrl 完全由 viewer 滑条决定，脚本不写

            # 鼠标拖拽 → 真实的力/力矩（passive viewer 不会自动施加）
            if viewer.perturb.active:
                mujoco.mjv_applyPerturbPose(model, data, viewer.perturb, 0)
                mujoco.mjv_applyPerturbForce(model, data, viewer.perturb)

            mujoco.mj_step(model, data)

            elapsed = time.time() - step_start
            time.sleep(max(0.0, model.opt.timestep - elapsed))
            viewer.sync()

            now = data.time
            if now >= next_print:
                print_state(now, data)
                next_print = now + args.print_period

            if args.duration > 0 and now >= args.duration:
                break

    print("\n退出。最终 q =", np.round(data.qpos[:6], 5))
    return 0


if __name__ == "__main__":
    sys.exit(main())
