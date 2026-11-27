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

  /// 是否同时辨识每轴恒定力矩偏置
  ///
  /// 为什么需要（2026-11-25 实测）：
  ///   joint1（竖直偏航轴）按模型重力矩应恒为 0，但实测 -0.83 N·m
  ///   —— 来自线束拉扯 / 扭矩零点 / 底座不绝对水平。
  ///   joint3 实测也比模型高 2.15 N·m。
  ///   这些偏置与重力参数正交（一个随位形变、一个不变），可以一起解。
  bool estimate_offsets = true;

  /// 是否同时辨识**每轴库仑摩擦**（靠 A/B 双向接近的符号差）
  ///
  /// 原理：机械臂停在"静摩擦刚好平衡"的位置，停下的点取决于接近方向。
  ///   A 组：直接走到目标（接近方向 d）
  ///   B 组：先偏移 δ 再回到目标（接近方向 −d）
  /// 所以两组测到的力矩差 = 2×库仑摩擦：
  ///   τ_A = g(q_A) + o + f_c,   τ_B = g(q_B) + o − f_c
  /// 在回归矩阵里给每轴加一列 sign_k·1（A 取 +1、B 取 −1），
  /// 与重力参数正交，可一起解。
  ///
  /// 为什么必须做（2026-11-26 实测）：
  ///   只解重力+偏置时 rms=0.489、holdout=0.600，超过 0.5 门槛；
  ///   joint3 的 A/B 力矩差折算库仑摩擦 **0.83 N·m**，远大于模型误差。
  bool estimate_friction = true;

  /// 正则化：把解往 URDF 的标称参数拉（Tikhonov / ridge）
  ///
  /// 为什么必须做（2026-11-26 实测）：
  ///   40 个样本、20 个唯一位形，设计矩阵的有效秩只有 17，
  ///   奇异值从 137 一路掉到 **0.4**（σ16/σ1 = 3e-3）。
  ///   纯最小二乘在这些"几乎没被激励"的方向上会放大噪声：
  ///     rms 0.475（训练集还行）  但 holdout **0.635**（泛化差）
  ///   加正则后，辨识不出来的方向自动退回标称值，
  ///   保证结果不会比"直接用 URDF"更差。
  ///
  /// lambda 的量纲要和 A^T A 匹配：A 的元素是力矩/参数，
  /// 取 lambda = (ridge_ratio · σ_max)^2 比较直观，默认 ridge_ratio = 1e-2。
  /// 默认**关**：纯最小二乘，语义是"把数据里的信息全部用上"，
  /// 也便于单元测试验证"无噪声时能精确还原偏置"。
  /// 真机数据上用 CLI 的 --ridge 打开做对比。
  bool regularize = false;
  double ridge_ratio = 1e-2;

  /// Huber-IRLS 稳健回归（2026-11-28 加）
  ///
  /// 为什么需要：
  ///   开源的 arm-param-identification（txk1228）报告 —— 同一批数据
  ///     · 普通 OLS          inlier RMSE ≈ 0.24 N·m
  ///     · Huber / 稳健 WLS  inlier RMSE ≈ 0.05 N·m   （好 5 倍）
  ///   差别全在离群点上。实测本臂同一自由位形三次测 τ3 得到
  ///   5.969 / 6.284 / 5.682（散布 0.6 N·m = 10%），这类点会把 OLS 拖偏。
  ///
  /// 做法：
  ///   ① 先做一次普通 LS 得到初值
  ///   ② 用 MAD 估稳健尺度 σ̂ = 1.4826 · median(|r − median(r)|)
  ///   ③ 权重 w_i = 1 (|r_i| ≤ k·σ̂) 或 k·σ̂/|r_i|（Huber，k 默认 1.345）
  ///   ④ 加权重解，迭代 huber_iters 次
  ///   ⑤ 同时报告 inlier_rms（只用权重=1 的点算）和离群点数
  bool robust = false;
  int huber_iters = 4;
  double huber_k = 1.345;
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

  Eigen::VectorXd offsets;        ///< nv 个恒定力矩偏置（estimate_offsets=false 时全 0）
  Eigen::VectorXd friction;       ///< nv 个库仑摩擦幅值（estimate_friction=false 时全 0）
  double ridge_lambda = 0.0;      ///< 实际用的 λ（排查用）

  bool holdout_ok = false;
  double holdout_rms = 0.0;       ///< 留出集上的 RMS —— 这才是"泛化精度"
  double holdout_max = 0.0;

  /// 稳健回归诊断
  double inlier_rms = 0.0;    ///< 只用"未被降权"的点算的 RMS
  int n_outliers = 0;         ///< 被降权的点数（|r| > k·σ̂）

  std::string message;
};

/// 最小二乘辨识（在可辨识子空间内）
GravityIdResult identifyGravity(const RobotModel& model, RobotWorkspace& ws,
                                const std::vector<GravityIdSample>& samples,
                                const GravityIdOptions& opt = {});

/// 辨识结果（从 YAML 读回来，供运行时重力补偿用）
struct GravityParams {
  bool valid = false;
  Eigen::VectorXd theta;      ///< 重力参数（np = 10×(njoints−1)）
  Eigen::VectorXd offsets;    ///< 每轴恒定偏置（空 = 无）
  Eigen::VectorXd friction;   ///< 每轴库仑摩擦（空 = 无）
  double rms = 0.0;
  double holdout_rms = 0.0;
  int rank = 0;
  std::string source;         ///< 来源文件路径
};

/// 从 gravity_id_cli 产出的 YAML 读回参数
///
/// 失败时返回 false 并填 `error`；**不要把无效参数当成"没有辨识"继续用** ——
/// 那样会静默退回标称模型，而使用者以为自己加载了辨识结果。
bool loadGravityParams(const std::string& path, GravityParams& out,
                       std::string* error = nullptr);

/// 用辨识参数在给定位形上评估重力矩（供验证/对比）
Eigen::VectorXd evalGravityTheta(const RobotModel& model, RobotWorkspace& ws,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& theta);

}  // namespace hw_arm_algorithms
