// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/gravity.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/rnea.hpp>


namespace hw_arm_algorithms {

// ────────────────────────────────────────────────────────── 重力项
void computeGravity(const RobotModel& model, RobotWorkspace& ws,
                    const Eigen::VectorXd& q, Eigen::VectorXd& g_out) {
  if (!model.valid()) {
    throw std::runtime_error("computeGravity: 模型未加载");
  }
  if (q.size() != model.nq()) {
    throw std::invalid_argument("computeGravity: q 维度不符");
  }
  if (g_out.size() != model.nv()) {
    g_out.resize(model.nv());
  }

  // g(q) = RNEA(q, 0, 0)
  //   Pinocchio 的 computeGeneralizedGravity 内部就是 rnea(q,0,0)，
  //   这里显式用零速度零加速度调用 rnea，语义更直白。
  const Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model.nv());
  pinocchio::rnea(model.pin(), ws.pin(), q, zero_v, zero_v);

  g_out = ws.pin().tau;
  ws.g() = g_out;
}

void computeGravityMotor(const RobotModel& model, RobotWorkspace& ws,
                         const Eigen::VectorXd& q,
                         const std::vector<double>& direction,
                         Eigen::VectorXd& tau_ff_out) {
  if (direction.size() != static_cast<size_t>(model.nv())) {
    throw std::invalid_argument("computeGravityMotor: direction 长度不符");
  }
  if (tau_ff_out.size() != model.nv()) {
    tau_ff_out.resize(model.nv());
  }

  computeGravity(model, ws, q, tau_ff_out);

  // URDF 关节坐标系 → 电机坐标系
  for (int i = 0; i < model.nv(); ++i) {
    tau_ff_out[i] *= direction[static_cast<size_t>(i)];
  }
}

double verifyGravityConsistency(const RobotModel& model, RobotWorkspace& ws,
                                const Eigen::VectorXd& q) {
  const Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model.nv());

  pinocchio::rnea(model.pin(), ws.pin(), q, zero_v, zero_v);
  const Eigen::VectorXd g_rnea = ws.pin().tau;

  pinocchio::computeGeneralizedGravity(model.pin(), ws.pin(), q);
  const Eigen::VectorXd g_cgg = ws.pin().g;

  return (g_rnea - g_cgg).cwiseAbs().maxCoeff();
}

}  // namespace hw_arm_algorithms
