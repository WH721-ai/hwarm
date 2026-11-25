// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hw_arm_algorithms {
namespace {

CheckResult validate(const CollisionModel& cm, const Eigen::VectorXd& q,
                     const TableConstraintOptions& opt) {
  if (!cm.valid()) return CheckResult::invalid("碰撞模型未初始化");
  if (q.size() != cm.nq()) {
    return CheckResult::invalid("q 维度不符：期望 " + std::to_string(cm.nq()) +
                                "，实际 " + std::to_string(q.size()));
  }
  if (!q.allFinite()) return CheckResult::invalid("q 含 NaN/Inf");
  if (!opt.normal.allFinite()) return CheckResult::invalid("桌面法向含 NaN/Inf");
  if (std::abs(opt.normal.norm() - 1.0) > 1e-6) {
    return CheckResult::invalid("桌面法向不是单位向量（norm=" +
                                std::to_string(opt.normal.norm()) + "）");
  }
  if (!std::isfinite(opt.offset)) return CheckResult::invalid("offset 非有限值");
  if (!std::isfinite(opt.margin)) return CheckResult::invalid("margin 非有限值");
  if (opt.margin < 0.0) return CheckResult::invalid("margin 为负");
  for (const auto& kv : opt.link_margin) {
    if (!std::isfinite(kv.second) || kv.second < 0.0) {
      return CheckResult::invalid("link_margin[" + kv.first + "] 非法");
    }
  }
  if (cm.numGeoms() == 0) return CheckResult::invalid("模型里没有几何体");
  return CheckResult::free();
}

/// 该 link 用的余量（优先 link_margin，其次全局 margin）
double marginFor(const std::string& link, const TableConstraintOptions& opt) {
  const auto it = opt.link_margin.find(link);
  return (it != opt.link_margin.end()) ? it->second : opt.margin;
}

bool isIgnored(const std::string& link, const TableConstraintOptions& opt) {
  return std::find(opt.ignored_links.begin(), opt.ignored_links.end(), link) !=
         opt.ignored_links.end();
}

}  // namespace

CheckResult checkTable(CollisionModel& cm, const Eigen::VectorXd& q,
                       const TableConstraintOptions& opt) {
  const CheckResult bad = validate(cm, q, opt);
  if (bad.is_invalid()) return bad;

  cm.updatePlacements(q);

  // 逐几何体算"沿 -normal 方向的最低点"
  std::string   worst_link;
  double        worst_val = std::numeric_limits<double>::infinity();
  double        worst_limit = 0.0;

  for (int i = 0; i < cm.numGeoms(); ++i) {
    const std::string& link = cm.geomLinkName(i);
    if (isIgnored(link, opt)) continue;

    const auto* verts = cm.geomVertices(i);
    if (verts == nullptr || verts->empty()) {
      return CheckResult::invalid("几何体 " + link + " 没有顶点数据（不是凸包？）");
    }

    const pinocchio::SE3& M = cm.geomWorldPlacement(i);
    const Eigen::Matrix3d R = M.rotation();
    const Eigen::Vector3d t = M.translation();

    // nᵀ(R v + t) = (nᵀR)·v + nᵀt
    const Eigen::RowVector3d a = opt.normal.transpose() * R;
    const double b = opt.normal.dot(t);

    double lo = std::numeric_limits<double>::infinity();
    for (const auto& v : *verts) {
      lo = std::min(lo, a.dot(v));
    }
    lo += b;

    const double limit = opt.offset + marginFor(link, opt);
    if (lo < worst_val) {
      worst_val = lo;
      worst_limit = limit;
      worst_link = link;
    }
    if (lo < limit) {
      return CheckResult::collision(
          -1, "穿桌面：" + link + " 最低点 " +
                  std::to_string(lo) + " m < 限 " + std::to_string(limit) + " m",
          lo - limit);
    }
  }

  if (worst_link.empty()) {
    return CheckResult::invalid("所有几何体都被忽略，桌面检查没有实际检查任何东西");
  }

  // 通过：把最贴近桌面的 link 记在 detail 里，便于日志
  CheckResult ok = CheckResult::free();
  ok.detail = "最贴近桌面：" + worst_link + " 间隙 " +
              std::to_string(worst_val - worst_limit) + " m";
  return ok;
}

double minTableClearance(CollisionModel& cm, const Eigen::VectorXd& q,
                         const TableConstraintOptions& opt) {
  if (!validate(cm, q, opt).is_free()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  cm.updatePlacements(q);

  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < cm.numGeoms(); ++i) {
    const std::string& link = cm.geomLinkName(i);
    if (isIgnored(link, opt)) continue;
    const auto* verts = cm.geomVertices(i);
    if (verts == nullptr || verts->empty()) return std::numeric_limits<double>::quiet_NaN();

    const pinocchio::SE3& M = cm.geomWorldPlacement(i);
    const Eigen::RowVector3d a = opt.normal.transpose() * M.rotation();
    const double b = opt.normal.dot(M.translation());

    double lo = std::numeric_limits<double>::infinity();
    for (const auto& v : *verts) lo = std::min(lo, a.dot(v));
    lo += b;

    best = std::min(best, lo - (opt.offset + marginFor(link, opt)));
  }
  return std::isfinite(best) ? best : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace hw_arm_algorithms
