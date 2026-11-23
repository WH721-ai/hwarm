// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 自碰撞检测（含末端执行器）
// ─────────────────────────────────────────────────────────────
// 用途：**选解**，不是"实时监控"。
//   实测：限位内 60~76% 的 IK 分支是自碰撞的，但 85%+ 的目标存在无碰撞解
//   ⇒ 碰撞检测的价值在于"挑对分支 + 拒绝不可行目标"
//
// 成本（本机实测，凸包版 15 对）：约 15~150 µs/次
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
  /// 安全余量（m）：最小距离小于它也算"碰"。
  ///
  /// ⚠️ 不要写进 coal 的 DistanceRequest::security_margin —— 那只是
  ///    CollisionRequest 的字段，distance() 永远返回真实距离（M04:16）。
  ///    本实现一律在外层比较。
  double margin = 0.0;

  /// 是否把末端执行器相关碰撞对也纳入（夹爪进 URDF 后打开）。
  /// 目前靠 link 名前缀过滤，见 .cpp。
  bool check_end_effector = false;
};

/// 自碰撞检测：返回第一个碰撞的连杆对（early-exit，比全查快）
///
/// @param cm  已 build() 的碰撞模型
/// @param q   关节角（内部会 updatePlacements）
CollisionHit checkSelfCollision(CollisionModel& cm, const Eigen::VectorXd& q,
                                const SelfCollisionOptions& opt = {});

/// 最小自碰撞距离（m）。有穿透时返回负值 = 穿透深度（M04:15）
///
/// 不做 early-exit —— 会查完所有对，所以比 checkSelfCollision 慢，
/// 只用于"距离监控/诊断"，不要放进候选过滤的热路径。
double minSelfDistance(CollisionModel& cm, const Eigen::VectorXd& q);

}  // namespace hw_arm_algorithms
