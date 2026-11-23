// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_robot.hpp"

#include <limits>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

namespace hw_arm_algorithms {
namespace {

bool isEndEffectorPair(const std::string& name) {
  // 夹爪进 URDF 后，link 名一般含 gripper / finger / tool / ee
  static const char* kKeys[] = {"gripper", "finger", "tool", "ee_", "hand"};
  for (const char* k : kKeys) {
    if (name.find(k) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

CollisionHit checkSelfCollision(CollisionModel& cm, const Eigen::VectorXd& q,
                                const SelfCollisionOptions& opt) {
  CollisionHit hit;
  if (!cm.valid()) return hit;

  cm.updatePlacements(q);
  auto& geom = cm.geom();
  auto& gdata = cm.geomData();

  // 先把所有对的碰撞结果算出来（Pinocchio 内部做 early-exit 优化）
  pinocchio::computeCollisions(cm.geom(), gdata,
                               /*stopAtFirstCollision=*/true);

  for (size_t k = 0; k < geom.collisionPairs.size(); ++k) {
    if (gdata.collisionResults[k].isCollision()) {
      const std::string name = cm.pairName(static_cast<int>(k));
      if (!opt.check_end_effector && isEndEffectorPair(name)) continue;
      hit.colliding = true;
      hit.pair_index = static_cast<int>(k);
      hit.pair_name = name;
      // margin > 0 时用距离再确认一次（凸包刚好接触的情况）
      if (opt.margin > 0.0) {
        pinocchio::computeDistance(geom, gdata, k);
        if (gdata.distanceResults[k].min_distance > opt.margin) {
          continue;  // 只是贴着，不算碰
        }
      }
      return hit;
    }
  }

  // 没有穿透，但有 margin 时还要看"离得够不够远"
  if (opt.margin > 0.0) {
    for (size_t k = 0; k < geom.collisionPairs.size(); ++k) {
      const std::string name = cm.pairName(static_cast<int>(k));
      if (!opt.check_end_effector && isEndEffectorPair(name)) continue;
      pinocchio::computeDistance(geom, gdata, k);
      if (gdata.distanceResults[k].min_distance < opt.margin) {
        hit.colliding = true;
        hit.pair_index = static_cast<int>(k);
        hit.pair_name = name + " (within margin)";
        return hit;
      }
    }
  }
  return hit;
}

double minSelfDistance(CollisionModel& cm, const Eigen::VectorXd& q) {
  if (!cm.valid()) return std::numeric_limits<double>::quiet_NaN();

  cm.updatePlacements(q);
  auto& geom = cm.geom();
  auto& gdata = cm.geomData();
  pinocchio::computeDistances(cm.geom(), gdata);

  double best = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < geom.collisionPairs.size(); ++k) {
    best = std::min(best, gdata.distanceResults[k].min_distance);
  }
  return std::isfinite(best) ? best : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace hw_arm_algorithms
