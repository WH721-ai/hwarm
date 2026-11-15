#!/usr/bin/env python3
"""
⚠️ 旧版生成器（v1）——输出已被 v2 的 config/ 分层结构取代，**请勿再运行**。

   旧输出：config/joints.yaml + config/links.yaml          （已删除）
   新结构：config/{struct,joint,link,inertials,control}/   （由 migrate_to_openarm_layout.py 生成）

保留此文件仅作参考：它记录了"把原始轴向旋到 z 轴"的全部矩阵运算
（R_align_z / 相似变换 FᵀTF / BASE_UP 整臂旋转 / TWIST_J3 / 刚性合并）。
如果运行它会重新写出旧版两个 YAML，属于副作用，一般不需要。
"""
import numpy as np

np.set_printoptions(precision=4, suppress=True)


def rpy_from_R(R):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll)  →  (roll, pitch, yaw)"""
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if abs(np.cos(pitch)) < 1e-9:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def R_align_z(a):
    """最小旋转，使 e_z 对准 a（a 归一化）"""
    a = np.asarray(a, float)
    a = a / np.linalg.norm(a)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, a)
    c = float(np.dot(z, a))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def R_zyx(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ---------------------------------------------------------------- 原链数据
# (joint, parent, child, type, t[3], a[3] or None)
# 注意：原 OpenArm joint3（大臂平转）取消后，link2 与 link3 之间是刚性连接；
# 这里直接把那 0.06625 m 的偏置并进 joint3 的 origin，不保留额外的 fixed link。
# 于是链条是"1 个基座 + 6 个连杆"，拖 jointN 就转 linkN，一一对应。
CHAIN = [
    ("joint1", "base_link", "link1", "revolute", (0.0, -0.0625, 0.0), (0.0, -1.0, 0.0)),   # 轴后续会被 BASE_UP 转正
    ("joint2", "link1",     "link2", "revolute", (0.0, -0.06, 0.0),   (-1.0, 0.0, 0.0)),
    ("joint3", "link2",     "link3", "revolute", (0.0, 0.0, -0.22),   (0.0, -1.0, 0.0)),   # = 0.06625 + 0.15375
    ("joint4", "link3",     "link4", "revolute", (0.0, 0.0, -0.0955), (0.0, 0.0, -1.0)),
    ("joint5", "link4",     "link5", "revolute", (0.0, 0.0, -0.1205), (0.0, 1.0, 0.0)),
    ("joint6", "link5",     "link6", "revolute", (0.0, 0.0, 0.0),     (1.0, 0.0, 0.0)),
]

# OpenArm 原始连杆数据：mass, com, inertia(6), 本体半径
OPENARM_LINKS = {
    "base_link": (1.143228, (0.002531, -0.025819, 0.000027),
                  (0.001136078, 0.000000381, 0.000000011, 0.001599352, 0.000000028, 0.000750542), 0.070),
    "link1": (1.141668, (0.000900, -0.044466, 0.000029),
              (0.001094321, 0.000009416, 0.000000045, 0.000759200, 0.000000067, 0.000670659), 0.045),
    "link2": (0.277509, (-0.024718, 0.0, -0.031264),
              (0.000153589, 0.0, 0.000005211, 0.000113062, 0.0, 0.000082300), 0.040),
    "link3": (1.073863, (-0.003009, -0.000515, -0.109368),
              (0.001289396, -0.000000230, -0.000067616, 0.001140190, -0.000010626, 0.000559765), 0.040),
    "link4": (0.634853, (-0.003103, -0.001365, -0.057460),
              (0.000469004, 0.000005636, -0.000001779, 0.000433926, 0.000003418, 0.000245211), 0.038),
    "link5": (0.615659, (-0.000737, -0.000412, -0.047094),
              (0.000279653, 0.000003317, 0.000000768, 0.000334600, -0.000000150, 0.000176662), 0.036),
    "link6": (0.475203, (0.000023, 0.000686, 0.000054),
              (0.000179276, 0.000000066, -0.000000034, 0.000136248, 0.000000033, 0.000135341), 0.034),
    "link7": (0.050000, (0.0, 0.0, 0.0),
              (0.000010, 0.0, 0.0, 0.000010, 0.0, 0.000010), 0.030),
}


def inertia_mat(I6):
    ixx, ixy, ixz, iyy, iyz, izz = I6
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def merge_inertia(A, B, t):
    """把 B（在 B 系，B 系相对 A 系平移 t）合并进 A，返回 (mass, com_A, I_A(3x3), radius)。"""
    mA, cA, IA6, rA = A
    mB, cB, IB6, rB = B
    cA = np.asarray(cA, float)
    cB_A = np.asarray(cB, float) + np.asarray(t, float)
    m = mA + mB
    c = (mA * cA + mB * cB_A) / m

    def shift(I6, mm, cc):
        I = inertia_mat(I6)
        d = (cc - c).reshape(3, 1)
        return I + mm * (float((d.T @ d)[0, 0]) * np.eye(3) - d @ d.T)

    I = shift(IA6, mA, cA) + shift(IB6, mB, cB_A)
    I6 = (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])
    return (m, tuple(c), I6, max(rA, rB))


# 最终连杆表：link2 = 原 link2 + 原 link3（刚性合并），其余名字整体前移一位
LINKS = {
    "base_link": OPENARM_LINKS["base_link"],
    "link1": OPENARM_LINKS["link1"],
    "link2": merge_inertia(OPENARM_LINKS["link2"], OPENARM_LINKS["link3"], (0.0, 0.0, -0.06625)),
    "link3": OPENARM_LINKS["link4"],
    "link4": OPENARM_LINKS["link5"],
    "link5": OPENARM_LINKS["link6"],
    "link6": OPENARM_LINKS["link7"],
}


# ------------------------------------------------- 立起来：让 joint1 的轴与 base 的 z 轴重合
# 做法：把整条臂绕 joint1 的轴线刚性旋转 G（joint1 的安装点不动），
#       于是 joint1 的转轴变成 base 的 +z，其余相对几何完全不变。
BASE_UP = True
if BASE_UP:
    a1 = np.asarray(CHAIN[0][5], float)
    G = R_align_z(a1).T          # F_1^T ：把 a1 转到 +z

    def _G(v):
        return (G @ np.asarray(v, float)).tolist()

    new_chain = []
    for k, (name, parent, child, typ, t, a) in enumerate(CHAIN):
        t2 = t if k == 0 else _G(t)      # joint1 的安装点保持不动
        a2 = None if a is None else _G(a)
        new_chain.append((name, parent, child, typ, t2, a2))
    CHAIN = new_chain

# --------------------------------------- elbow 轴转正：把 link3 及下游绕 link3 的 y 轴转 90°
# 目标：joint3 的转轴从竖直变成水平，与 joint2 的轴平行（工业臂 J2∥J3 的布局）。
# 做法：绕 link3 的局部 y 轴（穿过 joint3 原点）把整条前臂刚性旋转；
#       joint3 的安装点不动、joint4..joint6 的相对偏置与轴向整体跟着转。
TWIST_J3_DEG = 90.0
if TWIST_J3_DEG:
    a3 = np.asarray([c for c in CHAIN if c[0] == "joint3"][0][5], float)
    F3 = R_align_z(a3)                       # link3 的朝向（此时 z 轴 = joint3 轴）
    th = np.radians(TWIST_J3_DEG)
    Ry = np.array([[np.cos(th), 0.0, np.sin(th)],
                   [0.0, 1.0, 0.0],
                   [-np.sin(th), 0.0, np.cos(th)]])
    Rw = F3 @ Ry @ F3.T                      # 世界系下的等效旋转

    def _R(v):
        return (Rw @ np.asarray(v, float)).tolist()

    twisted = []
    for name, parent, child, typ, t, a in CHAIN:
        if name in ("joint4", "joint5", "joint6"):
            t2, a2 = _R(t), _R(a)
        elif name == "joint3":
            t2, a2 = t, _R(a)                # 安装点不动，只转轴向
        else:
            t2, a2 = t, a
        twisted.append((name, parent, child, typ, t2, a2))
    CHAIN = twisted

# ---------------------------------------------------------------- 计算 F_i
F = {"base_link": np.eye(3)}
frames = {}   # joint name -> (xyz_new, rpy_new, axis_new)
for name, parent, child, typ, t, a in CHAIN:
    Fp = F[parent]
    Fc = Fp if typ == "fixed" else R_align_z(a)
    F[child] = Fc
    xyz = Fp.T @ np.array(t)
    rpy = rpy_from_R(Fp.T @ Fc)
    frames[name] = (xyz, rpy, (0.0, 0.0, 1.0) if typ == "revolute" else None)

# ---------------------------------------------------------------- 生成 joints.yaml
def fnum(x):
    x = float(x)
    return 0.0 if abs(x) < 1e-12 else round(x, 6)

out_j = [
    "# 六轴机械臂关节定义（**每个关节绕自身 z 轴**）",
    "#",
    "# 坐标系约定：",
    "#   · 每个 joint 的 axis 固定为 (0,0,1)",
    "#   · 真实转动方向已烘进 origin 的 rpy —— 这样 URDF 看起来是\"标准机器人\"",
    "#   · 每个 child link 的坐标系随之旋转；连杆几何/质心/惯量已用同一旋转重新表达",
    "#   · 零位世界位姿与原 OpenArm 版本完全一致（可用 scripts/gen_frames.py 复现）",
    "#",
    "# 关节壳体（housing）挂在 child link 上，沿本关节 z 轴，与连杆本体分开建模。",
    "",
    "order: [" + ", ".join(c[0] for c in CHAIN) + "]",
    "",
]

LIMITS = {
    "joint1": (-1.3963, 3.4907, 40.0, 16.755),
    "joint2": (-0.17453, 3.3161, 40.0, 16.755),
    "joint3": (0.0, 2.4435, 27.0, 5.4454),
    "joint4": (-1.5708, 1.5708, 7.0, 20.944),
    "joint5": (-0.7854, 0.7854, 7.0, 20.944),
    "joint6": (-1.5708, 1.5708, 7.0, 20.944),
}
COLL_SCALE = 0.80   # 碰撞体半径 = 本体半径 × 该系数（碰撞包络应小于视觉模型）
HCOL = (0.20, 0.45, 0.75)
BODY_COL = {
    "base_link": (0.35, 0.38, 0.45),
    "link1": (0.55, 0.57, 0.60),
    "link2": (0.62, 0.64, 0.66),
    "link3": (0.70, 0.72, 0.74),
    "link4": (0.78, 0.79, 0.80),
    "link5": (0.84, 0.85, 0.86),
    "link6": (0.90, 0.55, 0.25),
    "link7": (0.95, 0.75, 0.20),
}
HEAD_R = {"joint1": 0.052, "joint2": 0.050, "joint3_mount": 0.045, "joint3": 0.046,
          "joint4": 0.042, "joint5": 0.040, "joint6": 0.038}
HEAD_L = {"joint1": 0.075, "joint2": 0.070, "joint3_mount": 0.030, "joint3": 0.075,
          "joint4": 0.070, "joint5": 0.070, "joint6": 0.060}

for name, parent, child, typ, t, a in CHAIN:
    xyz, rpy, axis = frames[name]
    out_j.append(f"{name}:")
    out_j.append(f"  parent: {parent}")
    out_j.append(f"  child: {child}")
    out_j.append(f"  type: {typ}")
    out_j.append(f"  origin: {{x: {fnum(xyz[0])}, y: {fnum(xyz[1])}, z: {fnum(xyz[2])}, "
                 f"roll: {fnum(rpy[0])}, pitch: {fnum(rpy[1])}, yaw: {fnum(rpy[2])}}}")
    if typ == "revolute":
        out_j.append(f"  axis: {{x: 0.0, y: 0.0, z: 1.0}}")
        lo, hi, eff, vel = LIMITS[name]
        out_j.append(f"  limit: {{lower: {lo}, upper: {hi}, effort: {eff}, velocity: {vel}}}")
    out_j.append("  housing:   # 仅视觉，不参与碰撞")

    out_j.append(f"    radius: {HEAD_R[name]}")
    out_j.append(f"    length: {HEAD_L[name]}")
    out_j.append("    origin: {x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}")
    out_j.append(f"    color: {{r: {HCOL[0]}, g: {HCOL[1]}, b: {HCOL[2]}, a: 1.0}}")
    out_j.append("")
open("config/joints.yaml", "w").write("\n".join(out_j))

# ---------------------------------------------------------------- 生成 links.yaml
out_l = [
    "# 六轴机械臂连杆定义（**连杆本体，不含关节壳体**）",
    "#",
    "# · 每个 link 的坐标系 = 对应关节坐标系（z 轴即该关节转轴）",
    "# · body：从本关节到下一个关节的连杆本体，用圆柱占位，方向沿两关节连线",
    "# · 质量/质心/惯量已按新坐标系重新表达（与原 OpenArm 数值等价）",
    "",
]

next_t = {c[1]: (c[4], c[0]) for c in CHAIN if c[1] != "link7"}
# 找出每个 link 之后的下一个关节平移（在旧 link 系下）
seq = [("base_link", None)] + [(c[2], c) for c in CHAIN]

for link, joint in seq:
    mass, com, I6, rad = LINKS[link]
    Fc = F[link]
    com_new = Fc.T @ np.array(com)
    I_new = Fc.T @ inertia_mat(I6) @ Fc

    if joint is not None:
        # 本 link 由 joint 连接进来；下一个关节 = CHAIN 中 parent == link 的那条
        nxt = next((c for c in CHAIN if c[1] == link), None)
        if nxt is None:
            d = np.array([0.0, 0.0, -0.02])       # 末端法兰：沿 -z 一小段
        else:
            d = Fc.T @ np.array(nxt[4])
        length = float(np.linalg.norm(d))
        if length < 1e-6:
            d = np.array([0.0, 0.0, -0.02])
            length = 0.02
        direction = d / length
        origin = d / 2.0
        Rr = R_align_z(direction) if np.linalg.norm(np.cross([0, 0, 1], direction)) > 1e-12 else np.eye(3)
        if np.dot(direction, [0, 0, 1]) < -0.9999:
            Rr = np.diag([1.0, -1.0, -1.0])
        rpy = rpy_from_R(Rr)
    else:
        origin = np.array([0.0, 0.0, 0.0])
        length = 0.07
        rpy = (0.0, 0.0, 0.0)

    out_l.append(f"{link}:")
    out_l.append(f"  mass: {mass}")
    out_l.append(f"  com: {{x: {fnum(com_new[0])}, y: {fnum(com_new[1])}, z: {fnum(com_new[2])}}}")
    out_l.append(f"  inertia: {{ixx: {fnum(I_new[0,0])}, ixy: {fnum(I_new[0,1])}, ixz: {fnum(I_new[0,2])}, "
                 f"iyy: {fnum(I_new[1,1])}, iyz: {fnum(I_new[1,2])}, izz: {fnum(I_new[2,2])}}}")
    out_l.append("  body:")
    out_l.append(f"    radius: {rad}")
    out_l.append(f"    length: {round(length, 6)}")
    out_l.append(f"    origin: {{x: {fnum(origin[0])}, y: {fnum(origin[1])}, z: {fnum(origin[2])}, "
                 f"roll: {fnum(rpy[0])}, pitch: {fnum(rpy[1])}, yaw: {fnum(rpy[2])}}}")
    c = BODY_COL[link]
    out_l.append(f"    color: {{r: {c[0]}, g: {c[1]}, b: {c[2]}, a: 1.0}}")

    # 碰撞体：与本体同轴，但两端各缩进一个间隙，避免与相邻 link 的端面重叠
    gap = min(0.015, 0.25 * length)
    coll_len = max(length - 2.0 * gap, 0.012)
    coll_origin = origin * (coll_len / length) if length > 1e-9 else origin
    if length < 0.005:
        # 零长度 link（球形腕：joint5 / joint6 共点）没有体积，不生成碰撞体
        out_l.append("  # 零长度 link（球形腕共点关节），无碰撞体")
    else:
        out_l.append("  collision:")
        out_l.append(f"    radius: {round(rad * COLL_SCALE, 6)}")
        out_l.append(f"    length: {round(coll_len, 6)}")
        out_l.append(f"    origin: {{x: {fnum(coll_origin[0])}, y: {fnum(coll_origin[1])}, z: {fnum(coll_origin[2])}, "
                     f"roll: {fnum(rpy[0])}, pitch: {fnum(rpy[1])}, yaw: {fnum(rpy[2])}}}")
    out_l.append("")
open("config/links.yaml", "w").write("\n".join(out_l))

print("=== 新坐标系（每个关节 z 轴 = 原轴向）===")
for name, parent, child, typ, t, a in CHAIN:
    xyz, rpy, axis = frames[name]
    print(f"{name:14s} xyz=({xyz[0]:7.4f},{xyz[1]:7.4f},{xyz[2]:7.4f})  "
          f"rpy=({rpy[0]:6.3f},{rpy[1]:6.3f},{rpy[2]:6.3f})  "
          f"axis={'z' if axis else '-'}")
print("\n已写出 config/joints.yaml 与 config/links.yaml")
