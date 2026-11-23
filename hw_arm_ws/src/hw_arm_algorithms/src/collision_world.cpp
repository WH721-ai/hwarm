// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_world.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace hw_arm_algorithms {

KeyPointFn makeTool0KeyPoints(const std::string& frame_name) {
  return [frame_name](const RobotModel& model, RobotWorkspace& ws,
                      const Eigen::VectorXd& q, Eigen::MatrixXd& out) {
    out.resize(3, 1);
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
      out.setZero();
      return;
    }
    auto& data = ws.pin();
    pinocchio::forwardKinematics(model.pin(), data, q);
    pinocchio::updateFramePlacements(model.pin(), data);
    out.col(0) = data.oMf[f].translation();
  };
}

bool checkTable(const RobotModel& model, RobotWorkspace& ws,
                const Eigen::VectorXd& q, const KeyPointFn& key_points,
                const TableConstraintOptions& opt) {
  if (!model.valid() || q.size() != model.nq()) return false;
  if (!key_points) return true;

  Eigen::MatrixXd pts;
  key_points(model, ws, q, pts);
  if (pts.cols() == 0) return true;

  const double z_min = opt.z_table + opt.margin;
  for (int i = 0; i < pts.cols(); ++i) {
    if (!pts.col(i).allFinite()) return false;   // NaN 一律当"不通过"
    if (pts(2, i) < z_min) return false;
  }
  return true;
}

}  // namespace hw_arm_algorithms
