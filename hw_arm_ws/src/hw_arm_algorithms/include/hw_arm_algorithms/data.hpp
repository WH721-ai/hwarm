// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// RobotWorkspace —— Pinocchio Data 的持有者（可写，每线程一份）
//
// 设计要点（实时安全）：
//   · 所有向量在**构造时**分配，之后永不 resize
//   · 250 Hz 控制循环里只复用，不分配 → 不会触发堆分配
//   · 一个 RobotModel 可以配 N 个 RobotWorkspace（多线程无锁）
#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>

#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

class RobotWorkspace {
 public:
  /// 尺寸完全由 model 决定，必须在构造时给定
  explicit RobotWorkspace(const RobotModel& model);

  // ── Pinocchio 的草稿纸 ──
  pinocchio::Data& pin() { return data_; }
  const pinocchio::Data& pin() const { return data_; }

  // ── 预分配的工作缓冲（避免每周期构造 Eigen 向量）──
  Eigen::VectorXd& q() { return q_; }
  Eigen::VectorXd& dq() { return dq_; }
  Eigen::VectorXd& ddq() { return ddq_; }
  Eigen::VectorXd& tau() { return tau_; }
  Eigen::VectorXd& g() { return g_; }

  const Eigen::VectorXd& q() const { return q_; }
  const Eigen::VectorXd& dq() const { return dq_; }
  const Eigen::VectorXd& ddq() const { return ddq_; }
  const Eigen::VectorXd& tau() const { return tau_; }
  const Eigen::VectorXd& g() const { return g_; }

  int nq() const { return static_cast<int>(q_.size()); }
  int nv() const { return static_cast<int>(dq_.size()); }

  /// 全部清零（调试用，不在热路径调用）
  void clear();

 private:
  pinocchio::Data data_;
  Eigen::VectorXd q_;    // nq
  Eigen::VectorXd dq_;   // nv
  Eigen::VectorXd ddq_;  // nv
  Eigen::VectorXd tau_;  // nv
  Eigen::VectorXd g_;    // nv
};

}  // namespace hw_arm_algorithms
