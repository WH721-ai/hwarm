// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/gravity.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/regressor.hpp>


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

int gravityParamCount(const RobotModel& model) {
  return 10 * (static_cast<int>(model.pin().njoints) - 1);
}

void computeGravityRegressor(const RobotModel& model, RobotWorkspace& ws,
                             const Eigen::VectorXd& q, Eigen::MatrixXd& Y_out) {
  const int nv = static_cast<int>(model.nv());
  const int np = gravityParamCount(model);
  Y_out.resize(nv, np);
  // 零速零加速度：此时 "关节力矩回归矩阵" 就等于重力回归矩阵
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd a0 = Eigen::VectorXd::Zero(nv);
  Y_out = pinocchio::computeJointTorqueRegressor(model.pin(), ws.pin(), q, v0, a0);
}

void computeGravityFromParams(const RobotModel& model, RobotWorkspace& ws,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& theta,
                              Eigen::VectorXd& g_out) {
  Eigen::MatrixXd Y;
  computeGravityRegressor(model, ws, q, Y);
  if (theta.size() != Y.cols()) {
    g_out = Eigen::VectorXd::Zero(model.nv());
    return;
  }
  g_out.noalias() = Y * theta;
}

void computeGravityFromParamsAndOffset(const RobotModel& model, RobotWorkspace& ws,
                                       const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& theta,
                                       const Eigen::VectorXd& offset,
                                       Eigen::VectorXd& g_out) {
  computeGravityFromParams(model, ws, q, theta, g_out);
  if (offset.size() == g_out.size()) g_out += offset;
}

Eigen::VectorXd gravityParamPrior(const RobotModel& model) {
  const int np = gravityParamCount(model);
  Eigen::VectorXd theta = Eigen::VectorXd::Zero(np);
  const auto& pin_model = model.pin();
  const int nb = np / 10;
  for (int i = 0; i < nb; ++i) {
    const auto params = pin_model.inertias[static_cast<size_t>(i + 1)].toDynamicParameters();
    for (int k = 0; k < 10; ++k) theta[i * 10 + k] = params[k];
  }
  return theta;
}

}  // namespace hw_arm_algorithms
