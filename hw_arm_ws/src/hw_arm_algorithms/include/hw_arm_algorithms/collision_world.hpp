// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 环境约束（当前只有桌面）
// ─────────────────────────────────────────────────────────────
// 方案：**平面不等式 + 凸包顶点精确支持**
//
//   对每个几何体的凸包顶点 v（几何体自身坐标），世界位姿 (R, t)，
//   平面 n·x ≥ h 的最低点：
//
//       min_v [ nᵀ(R v + t) ]
//
//   线性函数在凸包上的极值**必然落在顶点**，所以遍历顶点得到的是
//   **精确结果**，不是稀疏采样近似 —— 不需要构建桌面 Box。
//
// 为什么不用碰撞体查桌面：
//   · 代价 70 µs（7 连杆 × 10 µs）vs 这里 ~5 µs
//   · Box 的角会误报
//   · 实测 21% 的随机构型 TCP 会钻到桌面下 —— 这是唯一的"真安全项"
//
// ⚠️ 历史教训（审查报告 C02）：
//   旧版只查 tool0 一个点，存在 TCP 在桌面上方 +12.98 mm、
//   但 link4/5/6 钻到桌面下 −20.5 mm 的位形，**照样通过**。
//   现在按全部几何体算，不再有这个问题。
#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "hw_arm_algorithms/collision_model.hpp"
#include "hw_arm_algorithms/collision_types.hpp"

namespace hw_arm_algorithms {

struct TableConstraintOptions {
  /// 桌面的外法向（单位向量，指向"允许机器人存在"的一侧）。
  /// 默认 (0,0,1)：机器人在桌面上方。
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  /// 平面方程 n·x = offset。
  /// 默认 0：本机 base_link 装在 world z = +0.03，桌面在 z = 0。
  double offset = 0.0;

  /// 全局安全余量（m）
  double margin = 0.01;

  /// 按 link 覆盖余量。
  ///
  /// ⚠️ 底座是**螺栓固定在桌面上**的，它离桌面只有约 2.4 mm，
  ///    套用 10 mm 全局余量会让所有位形都判碰撞。
  ///    正确做法是给底座单独一个"允许接触"的余量（0），
  ///    而不是"忽略底座的全部检查"（那会漏掉底座真的穿桌面）。
  std::map<std::string, double> link_margin;

  /// 完全不参与桌面检查的 link（默认空）。
  ///
  /// 只有在确认某个 link 永远不会下探到桌面附近时才用。
  /// 用它来"绕开"底座问题是不对的 —— 请用 link_margin。
  std::vector<std::string> ignored_links;

  /// 便捷构造：底座允许接触（margin=0），其余用全局余量
  static TableConstraintOptions withMountedBase(double margin = 0.01,
                                                const std::string& base_link = "base_link") {
    TableConstraintOptions o;
    o.margin = margin;
    o.link_margin[base_link] = 0.0;
    return o;
  }
};

/// 检查某位形下**全部几何体**是否都在桌面上方
///
/// @return kFree / kCollision（detail 里是最低的 link 名）/ kInvalid
///         kInvalid：模型无效、q 维度不符/含 NaN、法向非单位/含 NaN、
///                   offset/margin 非有限、margin 为负、几何体没有顶点
CheckResult checkTable(CollisionModel& cm, const Eigen::VectorXd& q,
                       const TableConstraintOptions& opt = {});

/// 诊断用：返回最危险的（最小的）有符号间隙（m），即 min_link_z − (offset+margin)
/// 负数表示越界。模型/输入无效时返回 NaN。
double minTableClearance(CollisionModel& cm, const Eigen::VectorXd& q,
                         const TableConstraintOptions& opt = {});

}  // namespace hw_arm_algorithms
