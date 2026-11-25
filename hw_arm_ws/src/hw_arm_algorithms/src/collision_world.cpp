// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_world.hpp"

#include <cmath>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace hw_arm_algorithms {

KeyPointFn makeTool0KeyPoints(const std::string& frame_name) {
  return [frame_name](const RobotModel& model, RobotWorkspace& ws,
                      const Eigen::VectorXd& q, Eigen::MatrixXd& out) {
    pinocchio::FrameIndex f = 0;
    bool found = false;
    for (size_t i = 0; i < model.pin().frames.size(); ++i) {
      if (model.pin().frames[i].name == frame_name) {
        f = static_cast<pinocchio::FrameIndex>(i);
        found = true;
        break;
      }
    }
    if (!found) {
      // ★ C03：算不出来就返回 0 列，由 checkTable 判为 Invalid，
      //    **不要**用 (0,0,0) 冒充一个真实点（那在桌面低时会被放行）
      out.resize(3, 0);
      return;
    }
    auto& data = ws.pin();
    pinocchio::forwardKinematics(model.pin(), data, q);
    pinocchio::updateFramePlacements(model.pin(), data);
    out.resize(3, 1);
    out.col(0) = data.oMf[f].translation();
  };
}

CheckResult checkTable(const RobotModel& model, RobotWorkspace& ws,
                       const Eigen::VectorXd& q, const KeyPointFn& key_points,
                       const TableConstraintOptions& opt) {
  // ── 输入合法性（★ C03：任何一项不成立都返回 kInvalid，不是"通过"）──
  if (!model.valid()) return CheckResult::invalid("RobotModel 无效");
  if (q.size() != model.nq()) {
    return CheckResult::invalid("q 维度不符：期望 " + std::to_string(model.nq()) +
                                "，实际 " + std::to_string(q.size()));
  }
  if (!q.allFinite()) return CheckResult::invalid("q 含 NaN/Inf");
  if (!std::isfinite(opt.z_table)) return CheckResult::invalid("z_table 非有限值");
  if (!std::isfinite(opt.margin)) return CheckResult::invalid("margin 非有限值");
  if (opt.margin < 0.0) return CheckResult::invalid("margin 为负");
  if (!key_points) return CheckResult::invalid("关键点回调为空");

  Eigen::MatrixXd pts;
  key_points(model, ws, q, pts);

  if (pts.rows() != 3) {
    return CheckResult::invalid("关键点矩阵行数应为 3，实际 " +
                                std::to_string(pts.rows()));
  }
  if (pts.cols() == 0) {
    return CheckResult::invalid("关键点为空（末端帧找不到？）");
  }
  if (!pts.allFinite()) return CheckResult::invalid("关键点含 NaN/Inf");

  const double z_min = opt.z_table + opt.margin;
  for (int i = 0; i < pts.cols(); ++i) {
    if (pts(2, i) < z_min) {
      return CheckResult::collision(-1, "桌面以下", pts(2, i) - z_min);
    }
  }
  return CheckResult::free();
}

}  // namespace hw_arm_algorithms
