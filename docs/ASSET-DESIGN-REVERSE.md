# 逆向 OpenArm 的 `assets/` 目录：三个文件夹 + 开发顺序

> 对象：`/mnt/hgfs/VMshare/OpenArm/openarm_description-main/assets`
> 只读分析，未修改任何文件。所有结论都标注了证据，推断部分明确标为"推断"。

---

## 1. 三个文件夹各干什么

| 文件夹 | 职责 | 内容 | 体量 |
|---|---|---|---|
| **`robot/`** | **整机**：把躯干 + 双臂 + 末端 + 控制接口装配成一个完整机器人；按版本分目录 | `openarm_v1.0/`、`openarm_v2.0/`，各自带 `urdf/`、`config/`、`mesh(es)/`，外加 `urdf/example/*.urdf` 预生成快照 | 104 文件 / 135 MB |
| **`end_effector/`** | **可换末端**：每个末端是**独立、可单独加载**的模块（自带 urdf/config/meshes） | `parallel_link/`（平行夹爪）、`pinch_gripper/`（两指夹爪） | 42 文件 / 14 MB |
| **`sensor/`** | **可换传感器挂载**：不自己造传感器模型，只做"挂到机器人上"的 wrapper | `zed/urdf_wrapper/openarm_zed_wrapper.xacro`（include 外部 `zed_description`） | 1 文件 / 2.5 KB |

**设计原则（一句话）**：**按"可替换的物理模块"分文件夹，不按"文件类型"分**。
所以不是 `urdf/`、`meshes/`、`config/` 三大坨，而是每个可换件自带全套三件套 —— 换夹爪 = 换一个文件夹，不用动整机。

---

## 2. 判定依据（怎么"看"出顺序）

| 证据 | 说明 |
|---|---|
| **include 依赖图** | `robot/v2.0/urdf/utils/openarm_v20_robot.xacro` → include `arm` / `ee` / `sensor` / `utils`；`sensor/zed` 只被 v2 引用，v1 完全没有 → **传感器最晚加** |
| **`end_effector/pinch_gripper/config/old/`** | 里面躺着 4 个平铺 yaml（`inertials / joint_limits / kinematics / kinematics_link`），**与 `parallel_link/config/` 完全同名同结构** → 这是重构前的旧配置格式，迁移后留档 → **平铺配置在前，结构化配置在后** |
| **两个 standalone 的写法差异** | `parallel_link_standalone`（114 行）**0 个 include**，完全自包含；`pinch_gripper_standalone`（70 行）第一行就 include **v2 的 utils + ee 宏** → 老末端是"手写自包含"，新末端是"复用 v2 框架" → **parallel_link 在前，pinch_gripper 在后** |
| **组合器复杂度** | v1 的 `openarm_robot.xacro` 只有 **1 个宏**（198 行，单体装配）；v2 的 `openarm_v20_robot.xacro` 有 **9 个宏**（624 行：加载组件描述符、校验谱系、校验条目、校验 preset、解析运行时、发射组件树、发射挂载组件…） → **v1 是手写装配，v2 是数据驱动 + 校验** |
| **配置密度** | v1：`arm/` 6 个平铺 yaml + `body/` 3 个；v2：`arm/{struct,joint,link,inertials,control}` + `body/...` 约 20 个，还多了 `meta.md` 说明 → **v2 是 v1 的细化重构** |
| **presets 出现** | v2 有 `config/robot_presets/`：`default_bimanual`、`left_arm`、`right_arm`、`left/right_arm_with_pinch_gripper`（5 个）→ 说明**组合爆炸逼出了 preset 机制**，也说明 pinch_gripper 与 v2 同时代 |
| **命名不一致** | v1 用 `mesh/`（单数），v2 用 `meshes/` → 中途统一命名 |
| **`example/*.urdf`** | v1/v2 各有一份预生成的扁平 URDF（39 KB）→ 他们意识到"下游不想装 xacro"，所以**导出了消费快照** |

---

## 3. 逆向出来的时间线（6 个阶段）

```
① 单体原型
   一个 xacro 手写整机，能进 RViz 就行
   ↳ 痕迹：example/v1.urdf 这种"扁平快照"的存在

② v1.0：拆模块 + 宏/实例分离
   body / arm / ee / ros2_control 四个目录
   每个模块一对文件：*_macro.xacro（定义参数化宏）+ *.xacro（实例化）
   配置：平铺 yaml（inertials / joint_limits / kinematics / kinematics_link）
   装配：一个宏 openarm_robot() 从头串到尾
   ↳ 痕迹：v1 的 1 个装配宏、平铺 config、mesh/ 单数

③ 抽离可换末端（end_effector 独立）
   把 parallel_link 从 robot/v1.0/ee/ 提到 assets/end_effector/
   写成 standalone（0 include）+ 自己的 config + 自己的 mesh
   目的：末端能单独加载、单独调试、单独上 RViz
   ↳ 痕迹：parallel_link_standalone 完全自包含；parallel_link 的 config 就是后来被归档的"平铺格式"

④ v2.0：数据驱动重构
   urdf/utils/ 四个组合器：arm / ee / sensor / robot（+ utilities 基础库）
   config 结构化：struct(topology/name_mapping/reference_points) / joint / link / inertials / control
   robot_presets/*.yaml：用 YAML 声明"这台机器由哪些组件组成"，取代 if/else
   9 个校验宏：谱系校验、条目校验、preset 校验 → 把错误提前到 xacro 展开期
   mesh/ → meshes/
   ↳ 痕迹：624 行的 v2 组合器、20 个 config 文件、meta.md

⑤ 新末端 pinch_gripper
   因为要支持两指夹爪（带 mimic 关节、可镜像、可输出 grasp frame），
   把 ee 宏泛化成"拓扑 + 关节轴 + 限位 + mimic + 反射"驱动
   旧配置格式归档到 pinch_gripper/config/old/
   ↳ 痕迹：openarm_v20_ee.xacro 里那句注释
     "Current EE implementation targets the two-finger gripper topology."
     以及 presets 里 *with_pinch_gripper 组合

⑥ 传感器（最后）
   assets/sensor/zed/urdf_wrapper/openarm_zed_wrapper.xacro
   自己不建模，include 上游 zed_description 的宏，只负责"挂载点 + 外参"
   被 v2 的 openarm_v20_sensor.xacro 调用；v1 里没有
   ↳ 痕迹：v1 无 sensor 引用；sensor 只有 1 个文件
```

**一句话**：**先能跑（单体）→ 再拆开（模块）→ 再抽可换件（EE）→ 再数据驱动（v2 + presets）→ 最后接外设（sensor）。**
不是一次性设计出来的，是"每加一个新需求，就把上一层抽象重构一次"。

---

## 4. 提炼成可复用的 8 步（套到我们 6 轴臂上）

| 步 | 做什么 | 产出 | 验收 |
|---|---|---|---|
| 1 | **画组件树**：base / 6×link / ee / sensor 谁挂谁 | 一张树状图 | 能说清"换夹爪要改哪几个文件" |
| 2 | **写单体 URDF**（先别 xacro） | `robot_proto.urdf` | RViz 能显示、TF 连成一棵树 |
| 3 | **宏/实例分离**：每个 link 一个 macro，参数是尺寸与质量 | `*_macro.xacro` + `*.xacro` | 改一个参数，整机跟着变 |
| 4 | **抽可换件**：末端/传感器独立目录，自带 urdf+config+mesh | `end_effector/xxx/` | 能 `ros2 launch` 单独显示夹爪 |
| 5 | **数字全部进 YAML**（xacro 只做装配，不放魔法数） | `config/{struct,joint,link,inertials}/` | 改质量/限位不用碰 urdf |
| 6 | **加校验宏**：拓扑连通、关节数、mimic 目标存在、preset 组件齐全 | `validate_*` | 拼错一个名字，展开期就报错 |
| 7 | **加 preset**：常用组合写成 YAML | `robot_presets/*.yaml` | 换配置只改一个参数 |
| 8 | **导出扁平 URDF 快照** + launch + rviz 配置 | `example/*.urdf`、`launch/`、`rviz/` | 下游不用装 xacro 也能用 |

**关键取舍（学自他们踩过的坑）**
- 不要一开始就做 v2 那种数据驱动 —— 会陷进"为抽象而抽象"；先用 v1 那种手写装配跑通。
- 到**第三次出现同类重复**时再抽象（他们是在加 pinch_gripper 时才把 ee 泛化的）。
- 旧配置**留档 `old/`** 而不是删掉 —— 迁移期能对照。
- 传感器**不要自己建模**，做 wrapper 去 include 上游包 —— 上游更新自动跟着走。

---

## 5. 对我们项目的直接可用结论

1. 我们的 6 轴臂 description 包可以直接照搬这个骨架：
   ```
   assets/robot/<我们>/urdf/{arm,body,ee}/ + config/{struct,joint,link}/ + example/
   ```
2. `presets` 机制正好解决我们"过渡 URDF vs 实机 URDF"的问题 —— 两份 preset，一份 YAML，不用改代码。
3. 他们的 `config/struct/topology.yaml` 就是"机器人拓扑声明"，**我们的 `armgen` 完全可以吃这个文件**，比解析 URDF 更干净。
4. `example/*.urdf` 的教训：**给下游一份扁平 URDF**，否则所有工具都得装 xacro。

---

## 6. 附录：`openarm_v2.0/` 逐文件清单

> 55 个文件 / 24 MB。结构：**`config/` 是数据，`urdf/` 是解释器，`meshes/` 是几何，`example/` 是产物。**

### 6.1 `urdf/`（9 个文件，装配程序）

| 文件 | 行数 | 作用 |
|---|---|---|
| `openarm_v20.urdf.xacro` | 53 | **入口**。只做三件事：include `utils/openarm_v20_robot.xacro`、include 双臂 ros2_control、读 preset 的 YAML 并调用 `openarm_v20_robot`。可调参数：`robot_preset`、`collapse_internal_empty_links`、`emit_grasp_frame`、`use_fake_hardware` |
| `utils/openarm_v20_utils.xacro` | 223 | **原子发射器**：`emit_origin` / `emit_composed_origin` / `emit_limit` / `emit_link` / `emit_reference_link` / `emit_joint`。把 YAML 里的 dict 变成 URDF 的 XML 片段，全项目只在这里拼字符串 |
| `utils/openarm_v20_arm.xacro` | 136 | **7 轴臂**：`openarm_v20_emit_arm_chain`（把拓扑展开成 joints/links）+ `openarm_v20_arm`（加前缀、镜像、限位覆盖） |
| `utils/openarm_v20_ee.xacro` | 229 | **末端**：`openarm_v20_ee`，完全由拓扑+关节轴+限位+mimic+反射驱动（注释明说"当前实现面向两指夹爪拓扑"） |
| `utils/openarm_v20_sensor.xacro` | 31 | **传感器**：一行 include `assets/sensor/zed/urdf_wrapper/openarm_zed_wrapper.xacro`，加 `openarm_v20_sensor` 宏负责挂载点 |
| `utils/openarm_v20_robot.xacro` | 624 | **整机装配器 + 校验器**，10 个宏：`require` / `body` / `load_component_descriptor` / `validate_component_lineage` / `validate_component_entries` / `validate_robot_preset` / `resolve_component_runtime` / `emit_component_from_runtime` / `emit_component_tree` / `emit_attached_components` / `openarm_v20_robot` |
| `ros2_control/openarm.ros2_control.xacro` | 107 | **单臂控制接口**：`openarm_arm_ros2_control`（参数：`arm_type`、`control_gains`、`can_interface`、`can_fd`、`use_fake_hardware`、`hand`、`ee_type`）。真机插件 `openarm_hardware/OpenArmHW`，仿真插件 `mock_components/GenericSystem` |
| `ros2_control/openarm.bimanual.ros2_control.xacro` | 157 | **双臂控制接口**：`openarm_bimanual_ros2_control`，内部映射左右前缀 |
| `example/v2.urdf` | — | **产物快照**：`xacro` 预展开的扁平 URDF（22 link / 39 joint），给不想装 xacro 的下游用 |

### 6.2 `config/`（24 个文件，纯数据）

| 路径 | 内容 |
|---|---|
| `meta.md` | 各 preset 的截图索引（**注意：`imgs/` 目录未随仓库发布，图都挂了**） |
| `robot_presets/*.yaml`（5） | `default_bimanual`、`left_arm`、`right_arm`、`left_arm_with_pinch_gripper`、`right_arm_with_pinch_gripper`。每个文件声明 `robot.components` 列表 + 每个组件的 type/enabled/product/config/connection/reflect/limit_override |
| `arm/struct/topology.yaml` | **拓扑真值源**：`links: [base_link … link7]` + 每个 joint 的 `type/parent/child` |
| `arm/struct/name_mapping.yaml` | **名字双向映射**：`canonical_to_original` / `original_to_canonical`（例：`link7 ↔ empty_link_for_J7`，因为第 7 轴只到法兰） |
| `arm/struct/reference_points.yaml` | **挂载契约**：`ee_mount_point`（父 link7，x=0.0205），带 `meta.kind: assembly_mount` |
| `arm/joint/joint_axes.yaml` | 每关节旋转轴（如 joint1 = (0,−1,0)） |
| `arm/joint/joint_limits.yaml` | 每关节 lower/upper/effort/velocity（joint1: 40 N·m, 16.76 rad/s） |
| `arm/joint/joint_origins.yaml` | 每关节相对父 link 的 xyz/rpy |
| `arm/link/visuals.yaml` | 每 link 的 visual mesh 路径 + scale |
| `arm/link/collisions.yaml` | 每 link 的 collision mesh 路径 + scale |
| `arm/inertials/nominal.yaml` | 每 link 的质心 + 质量 + 6 个惯量分量（**"nominal" 是版本名：以后可以加 `measured/` 做模型辨识**） |
| `arm/control/control_gains.yaml` | 每关节 kp/kd（joint1–4: kp=70/60；joint5–7: kp=10） |
| `arm/control/friction.yaml` | **0 字节占位**（还没标定摩擦） |
| `arm/meta.md` | 该模块的图 |
| `body/struct/topology.yaml` | 躯干只有 1 个 link：`body_link0` |
| `body/struct/reference_points.yaml` | 左右臂挂点：`y=±0.031, z=0.698` |
| `body/{inertials,link}/{nominal,visuals,collisions}.yaml` | 躯干的物理与外观 |
| `body/meta.md` | 0 字节占位 |

### 6.3 `meshes/`（22 个文件，24 MB 的大头）

| 路径 | 内容 |
|---|---|
| `arm/visual/*.dae`（7） | 渲染用网格，0.4–1 MB/个（DAE 带材质） |
| `arm/collision/*.stl`（7） | 碰撞用网格，20–70 KB/个（**STL 是凸包友好的粗模**） |
| `arm/collision/splited/link5/*.stl`（3） | **link5 的碰撞体被切成 3 段** —— 单块凸包会把凹形"包死"，切开才能保住凹腔 |
| `body/visual/body_link0.dae` | **10.7 MB**（最大的文件，RViz 首次加载会明显卡一下） |
| `body/visual/body_link0.stl` | 7.9 MB（备用/轻量版） |
| `body/collision/body_link0_symp.stl` | 293 KB，`symp` = simplified，躯干碰撞用的简化体 |

### 6.4 这一层最值得抄的两个设计

1. **`struct/` + `joint/` + `link/` 三分**：拓扑（谁连谁）、关节（轴/限位/原点）、连杆（外观/碰撞/惯量）各自独立文件。改惯量不动拓扑，改拓扑不用碰网格。
2. **`reference_points.yaml` 是"挂载契约"**：手臂挂点、末端挂点都写成带 `meta` 的参考点，整机装配器只认这个契约——**换臂/换末端不需要改整机**。这正是他们三个顶层文件夹（robot / end_effector / sensor）能解耦的技术基础。
