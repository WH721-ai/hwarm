// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 自碰撞检测（含末端执行器）
// ─────────────────────────────────────────────────────────────
// 用途：**选解**，不是"实时监控"。
//   实测：限位内 60~76% 的 IK 分支是自碰撞的，但 85%+ 的目标存在无碰撞解
//   ⇒ 碰撞检测的价值在于"挑对分支 + 拒绝不可行目标"
//
// 成本（本机实测，凸包版 15 对）：约 9~15 µs/次
//
// ★ 与旧版的区别（按审查报告 C03/C04 修）
//   · 返回 CheckResult（Free / Collision / Invalid），不再用 bool
//     —— 模型没建好、q 维度错、NaN 一律 kInvalid，**调用方不会误放行**
//   · 先确定"本次要查哪些对"，再逐对查询
//     —— 不再"全局 early-exit 后遍历未更新的结果"（会漏检/读到旧值）
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hw_arm_algorithms/collision_model.hpp"
#include "hw_arm_algorithms/collision_types.hpp"

namespace hw_arm_algorithms {

struct SelfCollisionOptions {
  /// 安全余量（m）：最小距离小于它也算"碰"。
  ///
  /// ⚠️ 不要写进 coal 的 DistanceRequest::security_margin —— 那只是
  ///    CollisionRequest 的字段，distance() 永远返回真实距离（M04:16）。
  ///    本实现一律在外层比较。
  ///
  /// ⚠️ 本臂 home 位形的 link3↔link5 间隙只有 3.5 mm，
  ///    margin 取大于它会让 home 自己报碰撞。建议 ≤ 2 mm。
  double margin = 0.0;

  /// 是否把末端执行器相关碰撞对也纳入（夹爪进 URDF 后打开）。
  /// 判定用 CollisionModel 里预计算的**精确 link 名匹配**，不做字符串子串猜测。
  bool check_end_effector = false;
};

/// 自碰撞检测：返回第一个碰撞的连杆对（early-exit，比全查快）
///
/// @return  kFree / kCollision / kInvalid
///          kInvalid 的情况：模型未 build、q 维度不符、q 含 NaN/Inf、
///          margin 非有限或为负
CheckResult checkSelfCollision(CollisionModel& cm, const Eigen::VectorXd& q,
                               const SelfCollisionOptions& opt = {});

/// 最小自碰撞距离（m）。有穿透时返回负值 = 穿透深度（M04:15）
/// 模型无效或 q 非法时返回 NaN —— **调用方必须检查 isfinite**
///
/// 不做 early-exit —— 会查完所有对，所以比 checkSelfCollision 慢，
/// 只用于"距离监控/诊断"，不要放进候选过滤的热路径。
double minSelfDistance(CollisionModel& cm, const Eigen::VectorXd& q);

}  // namespace hw_arm_algorithms
