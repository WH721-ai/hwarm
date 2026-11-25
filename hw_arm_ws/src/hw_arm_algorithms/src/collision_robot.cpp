// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_robot.hpp"

#include <cmath>
#include <limits>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

namespace hw_arm_algorithms {
namespace {

/// 输入合法性：q 维度 + 有限性 + margin 有限性
CheckResult validateInputs(const CollisionModel& cm, const Eigen::VectorXd& q,
                           const SelfCollisionOptions& opt) {
  if (!cm.valid()) {
    return CheckResult::invalid("碰撞模型未初始化（CollisionModel::build 失败）");
  }
  if (q.size() != cm.nq()) {
    return CheckResult::invalid("q 维度不符：期望 " + std::to_string(cm.nq()) +
                                "，实际 " + std::to_string(q.size()));
  }
  if (!q.allFinite()) return CheckResult::invalid("q 含 NaN/Inf");
  if (!std::isfinite(opt.margin)) return CheckResult::invalid("margin 非有限值");
  if (opt.margin < 0.0) return CheckResult::invalid("margin 为负");
  return CheckResult::free();
}

}  // namespace

CheckResult checkSelfCollision(CollisionModel& cm, const Eigen::VectorXd& q,
                               const SelfCollisionOptions& opt) {
  const CheckResult bad = validateInputs(cm, q, opt);
  if (bad.is_invalid()) return bad;

  cm.updatePlacements(q);
  auto& geom = cm.geom();
  auto& gdata = cm.geomData();

  // ── ★ C04：先确定本次要查哪些对，再逐对查询 ──
  //
  // 旧写法先 computeCollisions(stopAtFirstCollision=true)，然后遍历**全部**
  // 结果并按名字跳过 EE 对。问题：
  //   对 0 是 EE（要跳过）且真的碰撞时，查询在 0 就停了，
  //   对 1..N 的结果根本没被本次更新（可能是初始 false，也可能是上次的旧值）
  //   ⇒ 真实的手臂碰撞被漏检。
  // 现在按"有效对"逐对调用 computeCollision(pair)，只读本轮写过的结果。
  for (size_t k = 0; k < geom.collisionPairs.size(); ++k) {
    if (!opt.check_end_effector && cm.pairIsEndEffector(static_cast<int>(k))) continue;

    if (pinocchio::computeCollision(geom, gdata, k)) {
      if (opt.margin <= 0.0) {
        return CheckResult::collision(static_cast<int>(k),
                                      cm.pairName(static_cast<int>(k)), 0.0);
      }
      // margin > 0：贴住但没穿透的，用真实距离再判一次
      pinocchio::computeDistance(geom, gdata, k);
      const double d = gdata.distanceResults[k].min_distance;
      if (d < opt.margin) {
        return CheckResult::collision(static_cast<int>(k),
                                      cm.pairName(static_cast<int>(k)), d);
      }
    }
  }

  // margin > 0：没穿透但可能离得太近
  if (opt.margin > 0.0) {
    for (size_t k = 0; k < geom.collisionPairs.size(); ++k) {
      if (!opt.check_end_effector && cm.pairIsEndEffector(static_cast<int>(k))) continue;
      pinocchio::computeDistance(geom, gdata, k);
      const double d = gdata.distanceResults[k].min_distance;
      if (!std::isfinite(d)) {
        return CheckResult::invalid("距离查询返回非有限值（pair " +
                                    cm.pairName(static_cast<int>(k)) + "）");
      }
      if (d < opt.margin) {
        return CheckResult::collision(static_cast<int>(k),
                                      cm.pairName(static_cast<int>(k)) + " (within margin)",
                                      d);
      }
    }
  }

  return CheckResult::free();
}

double minSelfDistance(CollisionModel& cm, const Eigen::VectorXd& q) {
  SelfCollisionOptions opt;
  if (!validateInputs(cm, q, opt).is_free()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  cm.updatePlacements(q);
  auto& geom = cm.geom();
  auto& gdata = cm.geomData();

  double best = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < geom.collisionPairs.size(); ++k) {
    if (cm.pairIsEndEffector(static_cast<int>(k))) continue;
    pinocchio::computeDistance(geom, gdata, k);
    const double d = gdata.distanceResults[k].min_distance;
    if (!std::isfinite(d)) return std::numeric_limits<double>::quiet_NaN();
    best = std::min(best, d);
  }
  return std::isfinite(best) ? best : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace hw_arm_algorithms
