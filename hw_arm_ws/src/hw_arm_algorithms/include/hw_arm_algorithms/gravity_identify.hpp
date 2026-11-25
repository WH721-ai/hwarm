// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// C-05 静态重力辨识
// ─────────────────────────────────────────────────────────────
// 原理：让机械臂在若干姿态 q_k 下**静止**，测各关节力矩 τ_k。
//       零速零加速度下 τ = g(q) + 摩擦，而 g(q) = Y_g(q)·θ 对 θ 线性
//       ⇒ 最小二乘解 θ。
//
// ⚠️ 摩擦是主要干扰源：
//       稳态静摩擦的方向取决于"从哪个方向接近这个位形"
//       ⇒ **每个位形从两个方向各采一次**，取平均消掉摩擦，差值还能顺便估摩擦
//
// ⚠️ 可辨识性（实测本臂）：
//       60 个参数里只有 24 个（每 body 的 m/m·cx/m·cy/m·cz）对重力有贡献，
//       这 24 列的**有效秩 = 10**
//       ⇒ 不能要求 θ 收敛到真值，只能要求 g(q) 复现准确
//       ⇒ 本实现用 SVD 取可辨识子空间拟合，并用**留出集**验证 g 的精度
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

/// 一个静态采样点
struct GravityIdSample {
  Eigen::VectorXd q;        ///< 关节角（URDF 坐标，nq）
  Eigen::VectorXd tau;      ///< 实测关节力矩（URDF 坐标，nv）
  std::string tag;          ///< 备注（"from_below" / "from_above" ...）
  double weight = 1.0;      ///< 权重
};

struct GravityIdOptions {
  /// 奇异值相对阈值：S_i / S_0 < rank_tol 视为数值零
  /// 本臂 rank 10 与 12 之间的相对奇异值是 2.7e-8，所以 1e-6 是安全的
  double rank_tol = 1e-6;

  /// 留出验证：每隔 holdout_stride 个样本留一个不参与拟合
  /// 0 = 不验证
  int holdout_stride = 0;
};

struct GravityIdResult {
  bool ok = false;
  Eigen::VectorXd theta;          ///< 辨识出的参数（10*(njoints-1)）
  int rank = 0;                   ///< 回归矩阵的有效秩
  int n_samples = 0;
  int n_used = 0;

  double rms = 0.0;               ///< 拟合残差 RMS（N·m）
  double max_abs = 0.0;           ///< 最大绝对残差（N·m）
  Eigen::VectorXd per_joint_rms;  ///< 每关节 RMS（nv）

  bool holdout_ok = false;
  double holdout_rms = 0.0;       ///< 留出集上的 RMS —— 这才是"泛化精度"
  double holdout_max = 0.0;

  std::string message;
};

/// 最小二乘辨识（在可辨识子空间内）
GravityIdResult identifyGravity(const RobotModel& model, RobotWorkspace& ws,
                                const std::vector<GravityIdSample>& samples,
                                const GravityIdOptions& opt = {});

/// 用辨识参数在给定位形上评估重力矩（供验证/对比）
Eigen::VectorXd evalGravityTheta(const RobotModel& model, RobotWorkspace& ws,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& theta);

}  // namespace hw_arm_algorithms
