# hw_arm 碰撞检测设计报告

> 版本：v1 · 2026-11-22
> 适用：`hw_arm_algorithms`（C++ / Pinocchio / Coal）
> 参考教程：`M04_碰撞检测`、`M15_minimanip`、`M12_ros2control`

---

## 一、需求与边界

### 1.1 要解决什么

| 项 | 结论 | 依据 |
|---|---|---|
| 工作空间有外部障碍吗 | **没有**（用户确认） | — |
| 需要查什么 | **自碰撞** + **桌面** | 实测：21% 随机构型 TCP 会钻到桌面下 |
| 用在哪一层 | **规划期一次性检查**，不是伺服环 | 见 1.3 |
| 判定性质 | **选分支 / 拒绝不可行目标**，不是"防止危险" | 实测：限位内 60~76% 的 IK 分支是碰撞的 |

### 1.2 自碰撞到底会不会发生（实测，本机 Pinocchio + Coal 3.0.3）

```
排除相邻连杆后，真正要查 15 对
3000 个随机构型（均匀采样于关节限位内）→ 1.6% 有真实自碰撞
```

**碰撞位形的特征：**

| joint2 | joint3 | TCP 高度 | 碰撞对 |
|---|---|---|---|
| 174° | 1° | **z = −10.9 cm** | base_link ↔ link4 |
| 120° | −0° | z = −1.1 cm | base_link ↔ link5 |
| 169° | 3° | z = −7.6 cm | base_link ↔ link4 |

```
规律：joint2 ≈ 105~180°（大臂转下去）+ joint3 ≈ 0°（小臂完全折回）
      90% 是 base_link ↔ link4/5/6，不是典型肘部自碰
      joint2 限位 −2°~180° ⇒ "折下去"完全在限位内
```

**更关键的统计**（每个可达目标枚举 8 个去重 IK 解）：

| 目标位置 | 解数 | 碰撞解 | 全部解都碰撞的目标 |
|---|---|---|---|
| r=12 cm, z=5 cm | 320 | 243（**76%**） | **15%** |
| r=18 cm, z=10 cm | 320 | 206（64%） | 2.5% |
| r=24 cm, z=15 cm | 320 | 213（67%） | 0% |

**结论：**
- 「在关节限位内」是**必要不充分**条件；
- 但 85%+ 的目标**存在**无碰撞解 ⇒ 碰撞检测的作用是**选分支**；
- 那 15% 全部解都碰撞的目标应当被明确拒绝，而不是硬做。

### 1.3 为什么不是实时

| 层 | 周期/预算 | 10 µs (L1) | 2293 µs (mesh) |
|---|---|---|---|
| 伺服环（驱动器内） | 1 ms | 1% | 229% ❌ |
| ros2_control 主循环 | 4 ms | 0.25% | 58% ❌ |
| **goto_xyz 一次规划** | **~20 ms** | **1%** | 230% ❌ |
| 抓取候选 100 个 | ~100 ms | 1% | 230% ❌ |

**碰撞检测属于规划功能，不属于控制功能**（对照 `M12:48`「命令模式决定生成什么，别在位置模式里白算动力学」）。

在位置控制（JTC）下，一旦轨迹下发，机械臂就照着走，**不会偏离** ⇒ 发送前查一次即可。
执行期监控（T2）只有在上力控/阻抗/拖动示教时才需要（见 C-35）。

---

## 二、方案选型

### 2.1 库：Coal（不是 FCL）

| 项 | FCL | **Coal** |
|---|---|---|
| 与 Pinocchio 集成 | 需手写桥接 | ✅ `pin.computeCollisions` 原生 |
| 几何模型来源 | 自己建 | ✅ 直接 `buildGeomFromUrdf` |
| 版本 | 已停止演进 | 活跃（hpp-fcl 后继） |
| 本机可用 | — | ✅ `coal 3.0.3` |

**选 Coal。** 且 `PINOCCHIO_WITH_COLLISION=True`（已确认）。

### 2.2 几何表示：凸包（精查）+ Box（粗筛），**不用胶囊**

#### 实测：胶囊方案的过度包络

（STL 按 URDF `scale=0.001` 换算；单胶囊 = 沿最长 AABB 轴、半径取垂直方向最大距离）

| 连杆 | 凸包体积 | 单胶囊 | 双胶囊 | **单/凸包** | 双/凸包 |
|---|---|---|---|---|---|
| base_link | 564 cm³ | 2708 | 1695 | 4.8× | 3.0× |
| link1 | 480 | 1864 | 2518 | 3.9× | 5.3× |
| **link2** | 2064 | **28811** | 3733 | **14.0×** | 1.8× |
| **link3** | 1657 | **13332** | 6653 | **8.1×** | 4.0× |
| link4 | 309 | 1303 | 1758 | 4.2× | 5.7× |
| link5 | 382 | 1905 | 1939 | 5.0× | 5.1× |
| link6 | 25 | 112 | 68 | 4.5× | 2.8× |
| **合计** | 5480 | — | — | **9.1×** | 3.4× |

**❌ 单胶囊不可用**：半径 130 mm 的胶囊套在 280 mm 臂展的臂上，
几乎任何位形都会互相「碰撞」⇒ 检测永远返回 true。

**⚠️ 双胶囊 3.4 倍仍偏高**，且每连杆要从 1 个碰撞体变成 2 个（碰撞对翻倍）。

#### ✅ 凸包：精确且够快

| 连杆 | 三角面 | **凸包顶点** |
|---|---|---|
| base_link | 1252 | 85 |
| link1 | 6460 | 311 |
| link2 | 29037 | 303 |
| link3 | 12165 | 302 |
| link4 | 3972 | 290 |
| link5 | 4036 | 342 |
| link6 | 696 | 74 |

**速度对照**（`M04:7` 的桌面实测 + 本机实测）：

| 表示 | 单对 | 15 对 | 来源 |
|---|---|---|---|
| **Box**（L1） | **10 µs** | 150 µs | 本机实测 |
| **凸包**（L2） | ~1~10 µs | **15~150 µs** | 按 `M04:7` 推算 |
| mesh（现状） | **2293 µs** | 34 ms | 本机实测 |

> **几何选型的真判据不是精度，而是「距离查询有没有闭式解」**（`M04:6`）。
> 但我们**不需要距离**、只需要布尔，所以凸包的 GJK（3~10 次 support，`M04:29`）
> 完全在预算内。用胶囊是为了省掉迭代，代价是 9 倍的过度包络 —— 不划算。

#### 两级策略

```
L1  Box 近似（10 µs/次）
      → 对所有候选、所有路径采样点跑一遍
      → 虚报不漏报（Box 是外接盒），安全
L2  凸包精查（1~10 µs/次）
      → 只对通过的少数候选 / 最终选中的解
      → 消除 L1 的虚报
```

### 2.3 桌面：**用不等式，不用碰撞体**

**实测：世界系 TCP 的 z 分布（2000 个随机构型）**

```
最小 -315 mm | 5% -181 mm | 中位 +326 mm | 最大 +766 mm
        ↓
**21% 的位形 TCP 会钻到桌面以下**
```

两种实现对比：

| | 碰撞体（Box 1m×1m） | **不等式** |
|---|---|---|
| 每周期检查 | 7 连杆 × 10 µs = **70 µs** | 3~5 个点 × FK ≈ **2 µs** |
| 误报 | Box 的角会误报 | 无 |
| 适用 | 桌面有夹具/孔/台阶 | **平面桌面** |

```cpp
// 关键点：TCP、link6 下端、夹爪两个指尖（夹爪进 URDF 后）
for (auto& p : key_points(q))
    if (p.z() < z_table + margin) return false;
```

**⚠️ 只有当桌面不是平面时，才退回碰撞体方案。**

### 2.4 碰撞对：**15 对全查，暂不裁剪**

实测 3000 个位形里实际发生碰撞的只有 7 对：

```
base_link ↔ link5   41 次      link1 ↔ link5   4 次
base_link ↔ link4   24 次      link2 ↔ link5   2 次
base_link ↔ link6   19 次      link2 ↔ link6   2 次
                               link1 ↔ link6   1 次
```

**但不建议裁到 7 对：**
- 凸包查 15 对只要 15~150 µs，裁掉 8 对省不到 75 µs
- `M04:20` 明确警告：**ACM 自动生成的必须独立验证（默认只采 1e4 配置）**
- 漏检是安全问题，省时间是性能问题 —— 不对称

**等以后真要进 1 kHz 环（且在 MCU 上）再裁。**

### 2.5 距离梯度：**暂不需要**

`M04:11` 给了公式，但只有以下场景需要：

| 用途 | 需要梯度 |
|---|---|
| **过滤候选（布尔 + 距离）** | ❌ ← **当前需求** |
| QP 优化里的碰撞约束 / 时间参数化 | ✅ |
| MoveIt CHOMP / STOMP 轨迹优化 | ✅ |
| 反应式避让 / 阻抗里叠加避碰力 | ✅ |
| 碰撞距离监控（只看 d 变化） | ❌ |

**真要做时的三个坑（`M04:12/13/19`）：**

1. **必须用 `LOCAL_WORLD_ALIGNED` 雅可比** —— Coal 的 `nearest_points` 是世界系，用 `LOCAL` 会坐标系错位；
2. **梯度只在 d>0 成立** —— d≤0 硬置零会让优化器停在碰撞内部，须切 EPA 的 contact normal 或用带裕度的 hinge 代价；
3. **碰撞距离分段光滑** —— 最近特征切换处梯度不连续，不能整段塞进 AD tape。

代价函数（`M04:14`）：`c(d) = (d − d_act)² / d_act²`，`d_safe = 5~20 mm`、`d_act = 50~100 mm`。
本臂长 280 mm，建议 `d_safe = 5 mm`、`d_act = 50 mm`。

---

## 三、代码设计

### 3.1 文件结构

```
hw_arm_ws/src/hw_arm_algorithms/
├── include/hw_arm_algorithms/
│   ├── collision_model.hpp
│   ├── collision_robot.hpp
│   └── collision_world.hpp
└── src/
    ├── collision_model.cpp
    ├── collision_robot.cpp
    └── collision_world.cpp
```

> **为什么不按用户最初的两文件（`CollisionWorld` + `CollisionRobot`）拆？**
> 两者都要用同一份几何模型。凸包构建是**几十毫秒级**的一次性开销，
> 必须建一次、缓存、共享。所以抽一个 `collision_model` 出来。

### 3.2 各文件职责

| 文件 | 职责 | 依赖 |
|---|---|---|
| `collision_model` | 从 URDF 建几何（凸包 / Box 两套）、建碰撞对表（去相邻）、缓存 | Pinocchio + Coal |
| `collision_robot` | 自碰撞 L1/L2、最小距离、末端执行器碰撞 | `collision_model` |
| `collision_world` | 桌面不等式约束 | `RobotModel` + FK |

### 3.3 数据流

```
启动时（一次）
    URDF ──→ CollisionModel::build()
                 ├─ mesh → 凸包（coal::Convex）
                 ├─ mesh → Box  （AABB，L1 用）
                 └─ 碰撞对表 [21 → 去相邻 15]

运行时（每次规划/过滤）
    q ──→ CollisionModel::updatePlacements(q)
             ├─ CollisionRobot::checkSelfCollision(L1)    10 µs
             ├─ CollisionRobot::checkSelfCollisionExact(L2) 1~10 µs
             └─ CollisionWorld::checkTable()               2 µs
```

### 3.4 接口设计

```cpp
// ───────────────────────── collision_model.hpp ─────────────────────────
#pragma once
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <pinocchio/multibody/geometry.hpp>
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

/// 几何近似级别
enum class GeomLevel {
  kBox,      ///< AABB 外接盒：10 µs，虚报不漏报（L1 粗筛）
  kHull,     ///< 凸包：1~10 µs，精确（L2 精查）
};

class CollisionModel {
 public:
  /// 从 URDF 建一次。内部同时构建 Box 与 Hull 两套几何，切换零成本。
  /// @param urdf_path  与 RobotModel 用的是同一个文件
  bool build(const RobotModel& model, const std::string& urdf_path);

  /// 切换近似级别（只改指针，不重建几何）
  void setLevel(GeomLevel level);

  /// 更新所有几何体的世界位姿（每个 q 必须调用一次）
  void updatePlacements(const Eigen::VectorXd& q);

  /// 当前级别的几何模型与数据（供 computeCollisions 使用）
  pinocchio::GeometryModel& geom();
  pinocchio::GeometryData& geomData();

  /// 碰撞对数量与名字（调试/日志用）
  int numPairs() const;
  std::string pairName(int idx) const;

  bool valid() const { return valid_; }

 private:
  pinocchio::GeometryModel geom_box_, geom_hull_;
  std::unique_ptr<pinocchio::GeometryData> data_box_, data_hull_;
  GeomLevel level_ = GeomLevel::kBox;
  bool valid_ = false;
};

}  // namespace hw_arm_algorithms
```

```cpp
// ───────────────────────── collision_robot.hpp ─────────────────────────
#pragma once
#include <string>
#include <Eigen/Core>
#include "hw_arm_algorithms/collision_model.hpp"

namespace hw_arm_algorithms {

struct CollisionHit {
  bool colliding = false;
  int pair_index = -1;
  std::string pair_name;

  explicit operator bool() const { return colliding; }
};

struct SelfCollisionOptions {
  /// 安全余量（m）：距离小于它算"碰"。
  /// ⚠️ 不要写进 DistanceRequest::security_margin —— M04:16：
  ///    那只是 CollisionRequest 的字段，distance() 永远返回真实距离。
  double margin = 0.0;
  /// 是否检测末端执行器（夹爪进 URDF 后打开）
  bool check_end_effector = false;
};

/// L1：Box 粗筛（约 10 µs）
CollisionHit checkSelfCollisionCoarse(CollisionModel& cm,
                                      const Eigen::VectorXd& q,
                                      const SelfCollisionOptions& opt = {});

/// L2：凸包精查（约 15~150 µs）
CollisionHit checkSelfCollisionExact(CollisionModel& cm,
                                     const Eigen::VectorXd& q,
                                     const SelfCollisionOptions& opt = {});

/// 最小自碰撞距离（m）。有穿透时返回负值（= 穿透深度，M04:15）
double minSelfDistance(CollisionModel& cm, const Eigen::VectorXd& q);

}  // namespace hw_arm_algorithms
```

```cpp
// ───────────────────────── collision_world.hpp ─────────────────────────
#pragma once
#include <functional>
#include <Eigen/Core>
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

struct TableConstraintOptions {
  double z_table = 0.0;    ///< 桌面在 world 系的 z（本机 base_link 装在 z=+0.03）
  double margin = 0.01;    ///< 安全余量（m）
};

/// 桌面平面约束：检查所有关键点是否高于 z_table + margin
/// @param key_point_fk 给定 q，返回若干关键点（TCP / link6 下端 / 指尖）的世界坐标
///                     用 std::function 注入，避免本模块依赖具体末端几何
bool checkTable(const RobotModel& model, RobotWorkspace& ws,
                const Eigen::VectorXd& q,
                const std::function<void(const Eigen::VectorXd&,
                                         Eigen::MatrixXd&)>& key_point_fk,
                const TableConstraintOptions& opt = {});

}  // namespace hw_arm_algorithms
```

### 3.5 关键实现要点

#### ① 凸包构建（C++）

```cpp
// 对每个几何体：
//   1) 若已是 BVHModel（mesh），取其 vertices
//   2) 用 coal::Convex 的构造（顶点 + 索引）或用 ConvexHull 算法
//   3) ⚠️ Python 绑定不支持从 list 构造 Convex，必须在 C++ 里做
coal::Convex<coal::Triangle> hull =
    coal::Convex<coal::Triangle>(points, triangles);
```

#### ② 相邻连杆排除

```cpp
// 从 URDF 的 parent/child 关系推导：base_link-1-2-3-4-5-6-tool0
// 只排除直接父子，不排除隔一个的
static const std::set<std::pair<int,int>> kAdjacent = {
  {0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7}
};
```

#### ③ `security_margin` 的坑（`M04:16`）

```cpp
// ❌ 错：写进 DistanceRequest，编译过、不报错、静默无效
req.security_margin = 0.005;

// ✅ 对：在外层比较
double d = minSelfDistance(cm, q);
if (d < opt.margin) → 碰撞
```

#### ④ 穿透深度（`M04:15`）

```cpp
// min_distance < 0 是**穿透深度**（取绝对值），不是库出错
if (dist < 0) penetration = -dist;
```

#### ⑤ 不要对 d≤0 做 sqrt 或当除数（`M04:33`）

```cpp
// d≤0 时 nearest_points 无效，梯度计算会 NaN
```

---

## 四、教程参考代码对照

### 4.1 M04 条目 → 本设计决策

| M04 行 | 原文要点 | 本设计对应 |
|---|---|---|
| **6** | 几何选型的真判据是「距离查询有没有闭式解」 | 我们用布尔查询 ⇒ 凸包可接受（2.2） |
| **7** | 桌面实测：50 顶点凸包 0.8–3.2 µs | 本机 74~342 顶点 ⇒ 1~10 µs（2.2） |
| **9** | 胶囊距离 = 线段最近距离 − r_A − r_B，无迭代 | 因过度包络 9.1× 被否（2.2） |
| **11** | 距离梯度 = `n̂ᵀ(J_pB − J_pA)` | 暂不需要（2.5） |
| **12** | 梯度必须配 LWA 雅可比 | 记入 2.5 的坑 #1 |
| **13** | 梯度只在 d>0 成立 | 记入 2.5 的坑 #2 |
| **14** | `c(d)=(d−d_act)²/d_act²`，d_safe=5–20mm | 建议 5mm/50mm（2.5） |
| **15** | `min_distance<0` 是穿透深度 | 3.5 ④ |
| **16** | `security_margin` 静默无效 | 3.5 ③ |
| **19** | 距离分段光滑、梯度不连续 | 记入 2.5 的坑 #3 |
| **20** | 自碰撞对 7 轴 45 → 15–25 对；ACM 必须独立验证 | 15 对全查，不裁（2.4） |
| **42** | 固件不引入 GJK/EPA/FCL | 我们在 **PC 侧**，不受此限 |
| **45** | 自碰撞对用 `static const uint8_t COLL_PAIRS[][2]` | 我们用 vector + 名字（PC 侧无需定长） |

### 4.2 教程给的 Pinocchio + Coal 骨架（直接可用）

```cpp
// M04 附录 / 用户整理版 —— 我们的实现以此为基础
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

// 1. 加载几何模型（从 URDF 的 collision 标签）
pinocchio::GeometryModel geom_model;
pinocchio::urdf::buildGeom(model, "urdf/hw_arm.urdf",
    pinocchio::COLLISION, geom_model);

// 2. 添加所有碰撞对（排除相邻 link）
geom_model.addAllCollisionPairs();
pinocchio::srdf::removeCollisionPairs(model, geom_model, "srdf/hw_arm.srdf");

// 3. 创建几何数据
pinocchio::GeometryData geom_data(geom_model);

// 4. 碰撞检查
pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);
bool is_collision = pinocchio::computeCollisions(
    model, data, geom_model, geom_data, q, false);

// 5. 距离计算
pinocchio::computeDistances(model, data, geom_model, geom_data, q);
for (size_t i = 0; i < geom_model.collisionPairs.size(); ++i) {
  double dist = geom_data.distanceResults[i].min_distance;
}
```

**⚠️ 本机实测发现的两个必须改动：**

1. `buildGeomFromUrdf` **不会**自动生成碰撞对 —— `collisionPairs` 是 **0**。
   必须显式调用 `addAllCollisionPairs()`，否则 `computeCollisions` 永远返回 false（假阴性！）。
2. `pinocchio.srdf.removeCollisionPairs` 的 Python 版要求 `.srdf` **扩展名**
   （传 `.urdf` 会报 "does not have the right extension"）。
   C++ 版无此限制，但要注意传对路径。

### 4.3 Coal 的低层 API（备查）

```cpp
// 距离查询（GJK 路径，d>0）
coal::DistanceRequest req;
req.enable_nearest_points = true;
coal::DistanceResult res;
coal::distance(geom_a.get(), tf_a, geom_b.get(), tf_b, req, res);
// res.min_distance > 0 → 距离 + 最近点
// res.min_distance < 0 → 穿透深度（EPA 路径，M04:15）

// 碰撞查询（只要布尔，更快）
coal::CollisionRequest creq;
coal::CollisionResult cres;
coal::collide(geom_a.get(), tf_a, geom_b.get(), tf_b, creq, cres);
bool hit = cres.isCollision();
```

---

## 五、接入点

### 5.1 接进 `ik_selection` 的硬过滤链

```cpp
// ik_selection.cpp，加在"路径奇异"之后、"速度/加速度"之前
// （顺序原则：便宜的在前，贵的在后）

// ⑦ 桌面约束（2 µs/点）
for (int k = 0; k < filter.path_samples; ++k) {
  const auto q = pinocchio::interpolate(model.pin(), q_now, ev.q, s);
  if (!checkTable(model, ws, q, key_points_fn, table_opt)) {
    reject("路径会扎桌面");  continue;
  }
}

// ⑧ 自碰撞 L1 粗筛（10 µs/点）
for (int k = 0; k < filter.path_samples; ++k) {
  const auto q = pinocchio::interpolate(model.pin(), q_now, ev.q, s);
  if (checkSelfCollisionCoarse(cm, q, sc_opt)) {
    reject("路径自碰撞（粗筛）");  continue;
  }
}

// ⑨ 自碰撞 L2 精查（只查终点，1~10 µs）
if (checkSelfCollisionExact(cm, ev.q, sc_opt)) {
  reject("终点精确碰撞");  continue;
}
```

### 5.2 成本预算

| 步骤 | 单价 | 次数 | 小计 |
|---|---|---|---|
| 桌面约束 | 2 µs | 20 | 40 µs |
| L1 粗筛 | 10 µs | 20 | 200 µs |
| L2 精查 | 5 µs | 1 | 5 µs |
| **合计** | | | **≈ 245 µs** |

**对比：`selectBestIK` 内 IK 约 159 µs/次。碰撞检测会让人均开销约翻倍，仍在毫秒级内。**

---

## 六、测试计划

### 6.1 单元测试

| # | 用例 | 期望 |
|---|---|---|
| 1 | `CollisionModel::build` 后 `numPairs()` | **15**（不是 0、不是 21） |
| 2 | home 位形 `checkSelfCollisionExact` | **无碰撞** |
| 3 | `q = [0, 3.0, 0, 0, 0.6, 0]`（joint2=172°, joint3=0°） | **碰撞**（base↔link5） |
| 4 | L1 与 L2 结果一致性 | L1 碰撞 ⊇ L2 碰撞（虚报不漏报） |
| 5 | `minSelfDistance(home)` | > 0，量级 1~10 cm |
| 6 | 桌面上方 10 cm 的位形 | `checkTable` 通过 |
| 7 | TCP 在 z = −5 cm 的位形 | `checkTable` 拒绝 |

### 6.2 与 mesh 版对照验证（关键）

```cpp
// 对 5000 个随机构型，比较 Box/Hull/mesh 三种表示
// 期望：
//   · 碰撞集合关系：mesh ⊂ hull ⊂ box（逐级放宽）
//   · mesh 与 hull 的差异 < 1%（凸包是精确外轮廓，差异来自 mesh 的凹陷）
//   · mesh 与 box 的差异 2~5%（box 虚报）
```

### 6.3 回归测试（来自既有分析）

```cpp
// 之前测到的 21% TCP 钻桌面 —— 用桌面约束必须能拦住
TEST(CollisionWorld, BlocksUnderTablePoses) {
  // 采样 2000 个随机构型，其中约 21% z<0
  // checkTable 必须全部拒绝
}
```

---

## 七、实施步骤与工作量

| 步骤 | 内容 | 工作量 | 风险 |
|---|---|---|---|
| 1 | `collision_model`：双几何构建 + 碰撞对表 | 半天 | 🟡 凸包构建 API 要试 |
| 2 | `collision_world`：桌面不等式 | 1 小时 | 🟢 |
| 3 | `collision_robot`：L1/L2/距离 | 半天 | 🟢 |
| 4 | 单测（6.1 的 7 条） | 2 小时 | 🟢 |
| 5 | 接入 `ik_selection` 过滤链 | 1 小时 | 🟡 注意顺序与预算 |
| 6 | 与 mesh 版对照验证 | 2 小时 | 🟢 |
| **合计** | | **约 2 天** | |

**优先级建议：**
1. 🔴 **先做第 2 步（桌面）** —— 它是唯一真正的安全项（21% 位形会钻桌面）
2. 🟡 再做 1/3/4（自碰撞，选解质量）
3. 🟢 最后接过滤链（第 5 步）

---

## 八、遗留与后续

| 项 | 触发条件 |
|---|---|
| 距离梯度 `∂d/∂q` | 上 QP 优化 / CHOMP/STOMP / 反应式避让时 |
| 碰撞对裁剪（15→7） | 需要进 1 kHz 环时（且会在 MCU 上重写） |
| 夹爪 / 被抓物体碰撞 | 夹爪进 URDF 后；物体要 attach 到 arm |
| 执行期监控（T2） | 上阻抗控制 / 拖动示教（C-35）后 |
| CCD（连续碰撞检测） | 出现"子弹穿纸"漏检时（`M04:18`） |
| 环境碰撞体 | 桌面出现夹具/孔/台阶时 |

---

## 附录 A：本机实测原始数据

```
环境：ROS 2 Jazzy · Pinocchio 4.1.0 · coal 3.0.3 · 8 核 / 7.7 GB（VMware）

碰撞体：7 个（base_link + link1..6），全部 BVHModelOBBRSS
碰撞对：addAllCollisionPairs → 21；去相邻 → 15
自碰撞：1.6%（3000 个限位内随机构型）
每次检测（mesh）：2293 µs
每次检测（Box）：10 µs
凸包顶点数：85 / 311 / 303 / 302 / 290 / 342 / 74
单胶囊过度包络：9.1×（link2 达 14.0×）
双胶囊过度包络：3.4×
TCP 低于桌面（z<0）的位形占比：21%
每个可达目标的碰撞 IK 分支占比：60~76%
全部解都碰撞的目标占比：0~15%（近基座、低 z 时最高）
IK 单次耗时（NumPy DLS）：159 µs
```

## 附录 B：STL 读取注意

```
本工程 STL 是**毫米单位**，URDF 里 `scale: 0.001`
        ↓
直接读 STL 的顶点值是 mm，不是 m
        ↓
任何离线分析（拟合胶囊/算凸包）都要乘 0.001
```

## 附录 C：文件格式

```
STL 头 80 字节 + uint32 三角面数 + 50 字节/面
⚠️ 本工程 STL 文件头不是 "solid" 开头（是 "StackForce CAD Importer local STL"），
   但**是二进制格式** —— 不要用 "solid" 判断 ASCII/二进制
```

---

## 九、实现结果（2026-11-22 已完成）

### 9.1 交付物

| 文件 | 内容 |
|---|---|
| `tools/gen_collision_hulls.py` | 从 STL 生成凸包（scipy），输出 `collision_hulls.hpp` |
| `include/.../collision_hulls.hpp` | 自动生成：7 个连杆共 1707 顶点 / 3386 三角形 |
| `include/.../collision_model.hpp` + `src/collision_model.cpp` | 建几何（mesh→凸包）、碰撞对（去相邻 15 对） |
| `include/.../collision_robot.hpp` + `src/collision_robot.cpp` | `checkSelfCollision` / `minSelfDistance` |
| `include/.../collision_world.hpp` + `src/collision_world.cpp` | 桌面不等式 `checkTable` |
| `test/test_collision.cpp` | 11 个用例（7 单元 + 4 集成），全过 |
| `ik_selection` 扩展 | 新增 `collision_check` 回调 + 路径/终点两级过滤 |

### 9.2 ⚠️ 实现中发现的与设计假设不符之处

#### ① coal 编译时**没有** `COAL_HAS_QHULL`

```cpp
#ifdef COAL_HAS_QHULL   // ← 实测未定义
```

⇒ `ConvexBase::convexHull()` 不可用，凸包必须**离线生成**。
⇒ 增加 `tools/gen_collision_hulls.py`（scipy ConvexHull），产出一个头文件。
⇒ `coal::Convex<Triangle>` 的三角形是**索引三元组**（`vids[3]`），不是三个 3D 点
（用 `Triangle(a,b,c)` 传 Vec3 会编译不过）。

#### ② AABB Box **完全不可用**（实测 100% 误报）

```
box : 100.0% 碰撞率   ← 15 对几乎全碰
mesh: 1.6%
```

原因：本臂的连杆不是长方体，AABB 比实际形状大得多
（link2 的 AABB = 352×236×157 mm，凸包体积只有 2064 cm³）。

⚠️ 另一个坑：**`coal::Box(x,y,z)` 的参数是"全长"，不是半长**
（内部存 `halfSide = side/2`）。写错会让 box 只有一半大 → **漏检**。

⇒ **取消原设计的"L1 Box 粗筛 + L2 凸包精查"两级策略**，
改为**单级凸包**（实测 9.1 µs，已经很便宜）。

#### ③ home 位形的间隙只有 **3.5 mm**

```
mesh 精确值：link3 ↔ link5 = 3.73 mm
凸包版      ：            3.52 mm
```

⇒ **`margin` 不能设大**：> 3.5 mm 会让 home 自己报碰撞。
⇒ 默认 `margin = 0`；若要安全裕度，取 **2 mm**。

### 9.3 实测性能（本机）

| 函数 | 耗时 | 说明 |
|---|---|---|
| `checkSelfCollision` | **9.1 µs/次** | 15 对，early-exit |
| `minSelfDistance` | **35.9 µs/次** | 查全部对 + 距离 |
| `checkTable` | ~2 µs/次 | 1 个关键点 |
| 对比：mesh 版 | 2293 µs/次 | 慢 250 倍 |

**20 点路径 + 终点 = 21 次 × 9.1 µs ≈ 0.19 ms** —— 与设计预估一致。

### 9.4 自碰撞率对照

| 表示 | 碰撞率（1500 个随机构型） |
|---|---|
| mesh（精确） | 1.6% |
| 凸包 | 2.2% |

凸包略高 ✓ 符合"凸包 ≥ mesh"的预期，且**不漏检**。

### 9.5 接入方式（已实现）

`ik_selection` 用 `std::function` 注入，避免决策层依赖几何实现：

```cpp
SolutionFilterOptions fo;
fo.collision_check = [&](const Eigen::VectorXd& q) {
  if (checkSelfCollision(*cm, q).colliding) return true;      // 撞了
  if (!checkTable(*model, *ws, q, key, topt)) return true;    // 穿桌面
  return false;
};
fo.require_collision_check = true;   // ← 忘了传回调会明确报错，不静默跳过
```

过滤链中的位置：**路径奇异 → 碰撞（路径逐点 + 终点）→ 速度/加速度**。

### 9.6 测试覆盖（11 个用例）

单元（7）：
1. `PairCountAfterAdjacentRemoval` —— 必须是 15（不是 0、不是 21）
2. `HomeIsCollisionFree`
3. `FoldedBackCollides` —— joint2=172°/joint3=0° 必须检出 base_link 碰撞
4. `ClearanceAtHomeIsSmallButPositive` —— home 间隙 3.5 mm 量级
5. `MarginIsRespected`
6. `TablePassesAboveTable`
7. `TableRejectsWhenTooLow`

集成（4）：
8. 不给回调 = 不检查（行为兼容）
9. `require_collision_check` 但没给回调 → 明确报错
10. 永远返回"碰撞"的回调 → 候选被拒
11. 真实 `CollisionModel` 做回调 → home 附近小位移通过

