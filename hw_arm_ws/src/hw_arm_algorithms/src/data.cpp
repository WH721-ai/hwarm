// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/data.hpp"

namespace hw_arm_algorithms {

RobotWorkspace::RobotWorkspace(const RobotModel& model)
    : data_(model.pin()),
      q_(Eigen::VectorXd::Zero(model.nq())),
      dq_(Eigen::VectorXd::Zero(model.nv())),
      ddq_(Eigen::VectorXd::Zero(model.nv())),
      tau_(Eigen::VectorXd::Zero(model.nv())),
      g_(Eigen::VectorXd::Zero(model.nv())) {}

void RobotWorkspace::clear() {
  q_.setZero();
  dq_.setZero();
  ddq_.setZero();
  tau_.setZero();
  g_.setZero();
}

}  // namespace hw_arm_algorithms
