# 迁移到 OpenArm 风格：准备工作（未执行，等方案确认）

> 状态：**只做了规划，没有改动任何现有文件**
> 目标：`hw_arm_description` 的目录结构、数据分层、命名方式全部对齐 OpenArm v2.0

---

## 1. 现状 → 目标

| | 现在 | 目标（OpenArm 风格） |
|---|---|---|
| 数据层 | `config/{joints,links}.yaml`（2 个，几何+惯性混在一起） | `config/arm/{struct,joint,link,inertials,control}/` 五层分离 |
| 模板层 | `urdf/utils/{hw_arm_utils,hw_arm_arm}.xacro`（142 行） | `urdf/utils/*.xacro` 分层（原语 / 组件 / 总装） |
| 生成器 | `scripts/gen_frames.py`（321 行，做矩阵运算） | **保留**，职责改为「CAD → YAML 导入 + 校验」 |
| 入口 | `hw_arm.urdf.xacro`（18 行，无参数） | 带 `robot_preset` 等参数 |
| 多形态 | 无 | `config/robot_presets/*.yaml` |
| 网格 | 圆柱占位 | mesh（DAE/STL），换 CAD 后启用 |

---

## 2. 目标目录（提案）

```text
hw_arm_description/
├── assets/
│   ├── robot/hw_arm_v1.0/
│   │   ├── config/
│   │   │   ├── meta.md
│   │   │   ├── arm/
│   │   │   │   ├── struct/{topology,name_mapping,reference_points}.yaml
│   │   │   │   ├── joint/{joint_origins,joint_axes,joint_limits,joint_mimics}.yaml
│   │   │   │   ├── link/{visuals,collisions}.yaml
│   │   │   │   ├── inertials/{nominal,measured}.yaml
│   │   │   │   └── control/{control_gains,friction}.yaml
│   │   │   ├── body/{struct,link,inertials}/...
│   │   │   ├── ee/{...}                     # 夹爪（如果有）
│   │   │   └── robot_presets/{default,arm_with_gripper}.yaml
│   │   ├── meshes/arm/{visual,collision}/
│   │   └── urdf/
│   │       ├── hw_arm.urdf.xacro            # 入口
│   │       ├── utils/{hw_arm_utils,hw_arm_robot,hw_arm_arm,hw_arm_ee}.xacro
│   │       └── ros2_control/hw_arm.ros2_control.xacro
│   └── sensor/                              # 相机（如果有）
├── launch/, rviz/, srdf/, scripts/, docs/
```

---

## 3. 三个必须你拍板的决策 ⭐

### D1. 坐标系约定：保留「轴 = z」还是回归 OpenArm 的「rpy = 0 + 任意轴」

**这条最关键，决定了 CAD 怎么导出。**

| | 现在（轴=z） | OpenArm（rpy=0） |
|---|---|---|
| joint rpy | 大多非零 `(0,±1.571,0)` 等 | **全 0** |
| joint axis | 恒 `(0,0,1)` | 任意，如 `(0,-1,0)` |
| mesh 与 link 系 | **不一致**，导出时要按 link 系重定向 | **一致**，CAD 直接导出即可 |
| 镜像（reflect） | 需要 `M·R·M`（OpenArm 代码不支持） | 开箱即用 |
| URDF 观感 | 标准 6 轴臂 | 看起来"歪" |
| 手写动力学 | 直接取 z 轴 | 要自己算旋转 |

- 选 **OpenArm 约定** → CAD mesh 零改造，reflection/preset 全部可用；**代价**：`gen_frames.py` 的矩阵运算（R_align_z / 相似变换 / BASE_UP / TWIST）全部作废
- 选 **保留轴=z** → 现有的"标准外观"保留；**代价**：CAD 里要为每个 link 建对应坐标系再导出，且 OpenArm 的 `reflect` 逻辑要改

> **我的建议**：既然 CAD 还没画完，**现在定约定成本最低** —— 建议采用 OpenArm 约定（rpy=0），让每个零件的 CAD 原点就是关节中心、轴向就是坐标轴。这样"数据直接来自 CAD"，不需要任何中间变换。

### D2. 组件粒度

- 只有 `arm`？（最简）
- 还是 `arm` + `body`（基座/躯干）+ `ee`（夹爪）？

有 CAD 之后，基座和夹爪通常是独立装配体，拆成组件后复用性更高。

### D3. 是否需要 preset / 多形态

- 单臂单形态 → **不需要** `robot.xacro` 的派发层（省 600 行）
- 要左右双臂 / 换夹爪 / 加相机 → 需要 preset + 派发

---

## 4. 你给方案时最好带上这些信息

| 项 | 为什么需要 |
|---|---|
| CAD 单位（mm？） | URDF 用 m，需要 `scale: 0.001` 或导出时换算 |
| 每个零件的原点约定 | 是否在关节中心？轴向是否对齐坐标轴？直接决定 D1 |
| 装配零位 = 代码零位？ | 决定 `joint_origins` 要不要加偏置 |
| 质量/质心/惯量来源 | SolidWorks 材质属性？还是手算/称重 |
| 夹爪型号与接口 | 决定 ee 组件的 config 结构 |
| 是否要双臂/相机 | 决定要不要 preset 和 sensor 层 |
| 网格导出格式 | visual: DAE（带色）collision: STL（减面 + 凸分解） |

---

## 5. 分阶段迁移（每步可回退）

| 阶段 | 内容 | 风险 |
|---|---|---|
| **P0（现在）** | 只写规划，不动代码 | 无 |
| **P1** | 建 `assets/robot/hw_arm_v1.0/config/` 骨架 + 空的 utils.xacro | 无（新目录，不影响现有） |
| **P2** | 写「CAD/旧数据 → 新 YAML」的导入脚本，跑通数据 | 低（只读旧数据） |
| **P3** | 新入口 `hw_arm_v2.urdf.xacro` 与旧入口**并存**，diff 对比 URDF | 低（可对照） |
| **P4** | 切换 launch/rviz 到新入口，删旧文件 | 中（需 RViz 目视确认） |
| **P5** | 接 mesh、重跑 `gen_srdf.py`、补 ros2_control 段 | 中 |

---

## 6. 风险清单

1. **`reflect` 依赖 rpy=0** —— 若选保留轴=z，OpenArm 的镜像代码会算错，必须改写
2. **负 scale 镜像** —— RViz 支持，但部分物理引擎/网格工具对负 scaling 处理不一致；建议镜像时**重新导出网格**而不是靠 scale
3. **硬编码灰色材质** —— OpenArm 的 `emit_link` 把所有 link 强制成灰色，要上色得改宏 + 加 `color` 字段
4. **CAD 单位** —— mm → m 漏了就是 1000 倍的灾难
5. **SRDF 必须重跑** —— 换 mesh/改拓扑后碰撞对全变
6. **我们做过的三个改造要重新表达**：
   - 立起来 → 变成一个 `world → base` 的 mount joint rpy（OpenArm 的 `world_to_pedestal` 就是这么做的）
   - 肘部转正 → 改 `joint_axes` 的方向（不再需要相似变换）
   - 合并 link3 → 拓扑里不写那个 joint 即可

---

## 7. 我可以立刻准备好的（等你确认 D1~D3）

- [ ] 目录骨架（空 YAML + 说明）
- [ ] `hw_arm_utils.xacro`（从 OpenArm 移植，修掉 rpy 镜像问题）
- [ ] 「旧 YAML → 新 YAML」转换脚本（保证迁移可验证：diff 两份 URDF）
- [ ] 一致性校验脚本（拓扑 ↔ 几何 ↔ 限位）
- [ ] URDF diff 工具（对比新旧零位位姿，误差 < 1e-9）
