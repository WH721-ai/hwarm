#!/usr/bin/env python3
"""路线 B：hw_arm 的 URDF → MJCF（带电机参数的 MuJoCo 模型）。

════════════════════════════════════════════════════════════════════
工作流（顺序不要反）
════════════════════════════════════════════════════════════════════
    ① ros2 launch hw_arm_mujoco display.launch.py     # 肉眼确认 URDF
    ② python3 scripts/urdf_to_mjcf.py                 # 转 MJCF
    ③ python3 scripts/launch_mujoco.py                # 在物理世界里看

第 ① 步不可省：MuJoCo 不显示关节轴/限位，转换后才发现轴向错、
原点偏，排查成本极高（MJCF 里全被烘焙成 quat/pos 数字）。

════════════════════════════════════════════════════════════════════
这个脚本做了什么
════════════════════════════════════════════════════════════════════
1. xacro 展开 hw_arm.urdf.xacro
2. 把 mesh 从 package:// 改写成相对路径，并把 STL 复制到 mjcf/assets/
   （MuJoCo 不认 package://，会报 Error opening file 'package:/...'）
3. mujoco.MjSpec.from_file(urdf)  —— MuJoCo 自带的 URDF 导入器
4. 给 6 个关节补 MuJoCo 才有的物理参数：
      armature     转子惯量折算（很重要，影响响应速度）
      damping      粘性阻尼
      frictionloss 库仑摩擦
   这些参数 URDF 里没有，不补的话仿真会"过轻"、抖动、响应过快
5. 加 6 个 <motor>（纯力矩执行器），forcerange 取 joint_limits.yaml 的 effort
6. 加 keyframe "home"（全零位）
7. 编译 + 自检（质量 / 执行器数 / q=0 重力矩）
8. 导出 mjcf/hw_arm.xml（机器人）+ mjcf/scene.xml（地面 + 光照 + 相机）

════════════════════════════════════════════════════════════════════
用法
════════════════════════════════════════════════════════════════════
    python3 scripts/urdf_to_mjcf.py              # 转换 + 自检
    python3 scripts/urdf_to_mjcf.py --check      # 只检查 URDF，不转换
    python3 scripts/urdf_to_mjcf.py --motor-tmax # forcerange 用电机峰值(200/45/27/11/7/7)
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

# ── 我们的电机选型（与硬件一致，不随 URDF 变）───────────────────────
# trated     电机**额定**力矩 N·m  ← 默认的 ctrlrange / forcerange（滑条范围）
# tmax       电机**峰值**力矩 N·m  ← 加 --motor-tmax 时用
# armature   转子惯量折算到关节侧 kg·m²
#            DM4310/DM4340 抄 OpenArm 的实测值；DM10010L/DM6216P 暂无数据，
#            先用 0.01 占位，等做过 sysid 再换。
#
# 为什么用额定而不是峰值：MuJoCo viewer 右侧滑条的范围取自 ctrllimited/ctrlrange。
# 不设置的话 viewer 默认给 ±1，只能给 1 N·m，没法做有意义的力矩实验。
# 用额定扭矩既符合电机长期工作能力，也让滑条一拖就是真实量级。
MOTORS = {
    "joint1": dict(model="DM10010L", trated=20.0, tmax=200.0, armature=0.0100),
    "joint2": dict(model="DM6216P",  trated=10.0, tmax=45.0,  armature=0.0100),
    "joint3": dict(model="DM4340",   trated=9.0,  tmax=27.0,  armature=0.1600),
    "joint4": dict(model="DM4310P",  trated=3.5,  tmax=11.0,  armature=0.0100),
    "joint5": dict(model="DM4310",   trated=3.0,  tmax=7.0,   armature=0.0100),
    "joint6": dict(model="DM4310",   trated=3.0,  tmax=7.0,   armature=0.0100),
}

# 关节阻尼 / 库仑摩擦的初值。
# ⚠️ damping 只用于**数值稳定**，不要拿它当控制阻尼 —— 控制阻尼是 kp/kd。
#    OpenArm 的 motor class 里用 1.0，是因为它整体是位置控制；
#    我们做"自由下坠"演示时 1.0 会让机械臂慢慢"渗"下去（30 秒才到限位），
#    看起来像卡住了。这里取 0.05，下坠 2 秒内到位，更接近真实断使能。
JOINT_DAMPING = 0.05       # N·m·s/rad
JOINT_FRICTIONLOSS = 0.1   # N·m


def log(msg=""):
    print(msg, flush=True)


# ────────────────────────────────────────────────────────────── 路径
def pkg_share(name: str) -> str:
    """优先用 ament index；没有 ROS 环境时回退到源码树相对路径。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(name)
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        src = os.path.abspath(os.path.join(here, "..", ".."))
        cand = os.path.join(src, name)
        if os.path.isdir(cand) and os.path.isdir(os.path.join(cand, "urdf")):
            return cand
        raise SystemExit(
            f"找不到包 {name}：先 `source install/setup.bash`，"
            f"或在源码树里运行本脚本")


def expand_xacro(xacro_path: str) -> str:
    """展开 xacro，返回 URDF 文本。"""
    log(f"[1/8] 展开 xacro: {xacro_path}")
    out = subprocess.run(
        ["xacro", xacro_path, "use_fake_hardware:=true"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise SystemExit("xacro 失败:\n" + out.stderr[-2000:])
    return out.stdout


# ────────────────────────────────────────────────────── 检查 URDF
def check_urdf(desc_share: str, urdf_text: str) -> None:
    """转换前的文本自检（display.launch.py 是肉眼版，这里是数值版）。"""
    root = ET.fromstring(urdf_text)
    joints = [j for j in root.findall("joint") if j.get("type") != "fixed"]
    links = root.findall("link")

    log(f"[2/8] URDF 自检")
    log(f"      连杆 {len(links)} 个 / 可动关节 {len(joints)} 个")

    problems = []
    for j in joints:
        name = j.get("name")
        ax = j.find("axis")
        lim = j.find("limit")
        if ax is None:
            problems.append(f"{name}: 缺 <axis>")
        if lim is None:
            problems.append(f"{name}: 缺 <limit>")
        else:
            lo, hi = float(lim.get("lower")), float(lim.get("upper"))
            if not (hi > lo):
                problems.append(f"{name}: 限位 upper({hi}) <= lower({lo})")
        a = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 0]
        norm = sum(v * v for v in a) ** 0.5
        if abs(norm - 1.0) > 1e-6:
            problems.append(f"{name}: axis 不是单位向量 (norm={norm:.6f})")

    total_mass = 0.0
    for link in links:
        ine = link.find("inertial")
        if ine is None:
            continue
        m = float(ine.find("mass").get("value"))
        total_mass += m
        I = ine.find("inertia")
        ixx, iyy, izz = (float(I.get(k)) for k in ("ixx", "iyy", "izz"))
        if min(ixx, iyy, izz) <= 0:
            problems.append(f"{link.get('name')}: 惯量出现非正值")
        # 三角不等式：物理可实现性
        if not (ixx + iyy >= izz - 1e-12 and
                iyy + izz >= ixx - 1e-12 and
                izz + ixx >= iyy - 1e-12):
            problems.append(f"{link.get('name')}: 惯量不满足三角不等式")

    log(f"      总质量 {total_mass:.6f} kg（含固定底座）")

    # mesh 是否存在
    mesh_dir = os.path.join(desc_share, "meshes")
    missing = []
    for link in links:
        for tag in ("visual", "collision"):
            for el in link.findall(tag):
                g = el.find("geometry")
                mesh = g.find("mesh") if g is not None else None
                if mesh is None:
                    continue
                fn = mesh.get("filename")
                base = fn.rsplit("/", 1)[-1]
                if not os.path.isfile(os.path.join(mesh_dir, base)):
                    missing.append(f"{link.get('name')}: {fn}")
    log(f"      mesh 引用 {sum(1 for l in links for t in ('visual','collision') for e in l.findall(t) if e.find('geometry') is not None and e.find('geometry').find('mesh') is not None)} 处"
        f"，缺失 {len(missing)} 个")
    problems += [f"缺 mesh: {m}" for m in missing]

    if problems:
        log("\n      ❌ 自检发现问题：")
        for p in problems:
            log(f"         · {p}")
        raise SystemExit("请先修 URDF（或跑 display.launch.py 看具体现象）")
    log("      ✅ 自检通过\n")


# ─────────────────────────────────────────────── mesh 路径重写
def prepare_urdf_with_assets(urdf_text: str, desc_share: str, out_dir: str) -> str:
    """把 package:// 的 mesh 改写成 assets/xxx.stl，并把 STL 复制过来。"""
    log("[3/8] 处理 mesh：package:// → assets/")
    assets = os.path.join(out_dir, "assets")
    os.makedirs(assets, exist_ok=True)
    mesh_dir = os.path.join(desc_share, "meshes")

    root = ET.fromstring(urdf_text)
    n = 0
    for el in root.iter("mesh"):
        fn = el.get("filename") or ""
        base = fn.rsplit("/", 1)[-1]
        src = os.path.join(mesh_dir, base)
        if os.path.isfile(src):
            dst = os.path.join(assets, base)
            if not os.path.exists(dst) or os.path.getsize(dst) != os.path.getsize(src):
                shutil.copyfile(src, dst)
            el.set("filename", f"assets/{base}")
            n += 1
    log(f"      复制 {n} 处引用 → {assets}")

    tmp_urdf = os.path.join(out_dir, "_hw_arm_from_urdf.urdf")
    ET.ElementTree(root).write(tmp_urdf, encoding="utf-8", xml_declaration=True)
    log(f"      中间 URDF: {tmp_urdf}")
    return tmp_urdf


# ───────────────────────────────────────────── MJCF 生成
def build_mjcf(tmp_urdf: str, out_dir: str, desc_share: str,
               use_motor_tmax: bool) -> tuple[str, dict]:
    import mujoco
    import numpy as np
    import yaml

    log("[4/8] MuJoCo URDF 导入 (MjSpec)")
    spec = mujoco.MjSpec.from_file(tmp_urdf)

    # 关节限位来自我们的配置（effort 保留给 ros2_control，不用作 MuJoCo 的力矩上限）
    lim_path = os.path.join(desc_share, "config", "joint", "joint_limits.yaml")
    limits = yaml.safe_load(open(lim_path, encoding="utf-8"))

    log("[5/8] 补关节物理参数 + 加电机")
    for j in spec.joints:
        name = j.name
        if name not in MOTORS:
            continue
        mot = MOTORS[name]
        j.armature = float(mot["armature"])
        # damping 在 MjSpec 里是 3 分量（自由度方向），hinge 只用第 1 个
        j.damping = np.array([JOINT_DAMPING, JOINT_DAMPING, JOINT_DAMPING])
        j.frictionloss = JOINT_FRICTIONLOSS

        # ctrlrange 决定 viewer 右侧滑条的范围（不设 → 默认 ±1）
        lim = float(mot["tmax"]) if use_motor_tmax else float(mot["trated"])

        a = spec.add_actuator()
        a.name = f"{name}_motor"
        a.target = name
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a.biastype = mujoco.mjtBias.mjBIAS_NONE
        a.ctrllimited = True
        a.ctrlrange = np.array([-lim, lim])
        a.forcelimited = True
        a.forcerange = np.array([-lim, lim])
        log(f"      {name:8s} {mot['model']:9s} armature={mot['armature']:.4f} "
            f"滑条/力矩范围=±{lim:g} N·m"
            f"（额定 {mot['trated']:g} / 峰值 {mot['tmax']:g}）")

    # keyframe home（全零位）
    # 注意：MjSpec 没有 qpos0 属性，nq 要编译后才知道；
    # 我们是 6 个 hinge 关节，nq = 6。
    log("[6/8] 加 keyframe 'home'")
    n_hinge = sum(1 for j in spec.joints if j.name in MOTORS)
    key = spec.add_key()
    key.name = "home"
    key.qpos = np.zeros(n_hinge)

    # ★ 关掉"机械臂自己和自己"的碰撞
    #   CAD 导出的 visual mesh 本来就是**嵌套**的（link1 塞在 base_link 里，
    #   重叠 3 mm）。直接拿它当碰撞体，会在 q=0 就产生 4 个接触点、
    #   上百万牛的约束力，把 joint1 彻底锁死（给 20 N·m 也不动）。
    #   做法：零件之间不碰（conaffinity=0），但和地面要碰
    #         —— 地面在 scene.xml 里设成 contype=2 conaffinity=1，
    #            于是 (arm.contype=1 & floor.conaffinity=1) = 1 → 会碰。
    n_geom = 0
    for g in spec.geoms:
        g.contype = 1
        g.conaffinity = 0
        n_geom += 1
    log(f"      关闭零件间自碰撞：{n_geom} 个 geom 设成 contype=1/conaffinity=0"
        f"（与地面仍会碰撞）")

    log("[7/8] 编译 + 自检")
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    mass = float(np.sum(model.body_mass))
    log(f"      nbody={model.nbody} njnt={model.njnt} nu={model.nu} nq={model.nq}")
    log(f"      总质量 = {mass:.6f} kg")
    log(f"      q=0 重力矩 = {np.round(data.qfrc_bias, 4)}")
    if model.nu != 6:
        raise SystemExit(f"❌ 执行器数 {model.nu} != 6")

    xml = spec.to_xml()
    out_xml = os.path.join(out_dir, "hw_arm.xml")
    with open(out_xml, "w", encoding="utf-8") as f:
        f.write(xml)
    log(f"      → {out_xml}  ({len(xml)} 字节)")

    return out_xml, dict(mass=mass, gravity=np.round(data.qfrc_bias, 4).tolist())


SCENE_TEMPLATE = """<mujoco model="hw_arm_scene">
  <!-- 由 scripts/urdf_to_mjcf.py 生成；机器人本体在 hw_arm.xml -->
  <include file="hw_arm.xml"/>

  <compiler angle="radian"/>
  <option timestep="0.001" integrator="implicitfast"/>

  <statistic center="0 0 0.15" extent="0.6" meansize="0.03"/>

  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="140" elevation="-20"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.20 0.30 0.40" rgb2="0.10 0.20 0.30"
             markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <!-- 地面：z=0 平面。机械臂底座已被抬高 30mm，正好站在上面 -->
    <!-- contype=2 / conaffinity=1：与机械臂（contype=1, conaffinity=0）会碰撞，
         但机械臂各零件之间不碰撞（见 hw_arm.xml 里 geoms 的 conaffinity=0） -->
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"
          contype="2" conaffinity="1"/>
    <camera name="front" pos="0.75 -0.75 0.45" xyaxes="0.707 0.707 0 -0.25 0.25 0.94"/>
  </worldbody>
</mujoco>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description="hw_arm URDF → MJCF")
    ap.add_argument("--check", action="store_true", help="只检查 URDF，不生成 MJCF")
    ap.add_argument("--motor-tmax", action="store_true",
                    help="forcerange 用电机峰值力矩（默认用 joint_limits 的 effort）")
    ap.add_argument("--out", default=None, help="输出目录（默认包内 mjcf/）")
    args = ap.parse_args()

    desc_share = pkg_share("hw_arm_description")
    xacro_path = os.path.join(desc_share, "urdf", "hw_arm.urdf.xacro")

    urdf_text = expand_xacro(xacro_path)
    check_urdf(desc_share, urdf_text)
    if args.check:
        log("✅ --check 模式：URDF 正常，未生成 MJCF")
        return 0

    if args.out:
        out_dir = os.path.abspath(args.out)
    else:
        # 默认写到源码树里的 mjcf/，方便 git 管理
        here = os.path.dirname(os.path.abspath(__file__))
        out_dir = os.path.abspath(os.path.join(here, "..", "mjcf"))
    os.makedirs(out_dir, exist_ok=True)

    tmp_urdf = prepare_urdf_with_assets(urdf_text, desc_share, out_dir)
    out_xml, info = build_mjcf(tmp_urdf, out_dir, desc_share, args.motor_tmax)

    scene = os.path.join(out_dir, "scene.xml")
    with open(scene, "w", encoding="utf-8") as f:
        f.write(SCENE_TEMPLATE)

    log("[8/8] 完成")
    log(f"      机器人 : {out_xml}")
    log(f"      场景   : {scene}")
    log(f"      assets : {os.path.join(out_dir, 'assets')}")
    log("")
    log("下一步：python3 scripts/launch_mujoco.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
