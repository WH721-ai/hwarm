// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/gravity_identify.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

#include "hw_arm_algorithms/gravity.hpp"

namespace hw_arm_algorithms {
namespace {

/// 把样本堆成 (nv*N) × np 的回归矩阵 + (nv*N) 的力矩向量
void stackSamples(const RobotModel& model, RobotWorkspace& ws,
                  const std::vector<GravityIdSample>& samples,
                  const std::vector<int>& idx,
                  Eigen::MatrixXd& A, Eigen::VectorXd& b,
                  std::vector<double>* weights = nullptr) {
  const int nv = static_cast<int>(model.nv());
  const int np = gravityParamCount(model);
  const int n = static_cast<int>(idx.size());

  A.setZero(n * nv, np);
  b.setZero(n * nv);
  if (weights) weights->assign(n * nv, 1.0);

  Eigen::MatrixXd Y;
  for (int k = 0; k < n; ++k) {
    const auto& s = samples[static_cast<size_t>(idx[static_cast<size_t>(k)])];
    computeGravityRegressor(model, ws, s.q, Y);
    const double w = std::sqrt(std::max(0.0, s.weight));
    A.middleRows(k * nv, nv) = w * Y;
    b.segment(k * nv, nv) = w * s.tau;
    if (weights) std::fill_n(weights->begin() + k * nv, nv, s.weight);
  }
}

}  // namespace

Eigen::VectorXd evalGravityTheta(const RobotModel& model, RobotWorkspace& ws,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& theta) {
  Eigen::VectorXd g;
  computeGravityFromParams(model, ws, q, theta, g);
  return g;
}

GravityIdResult identifyGravity(const RobotModel& model, RobotWorkspace& ws,
                                const std::vector<GravityIdSample>& samples,
                                const GravityIdOptions& opt) {
  GravityIdResult out;

  if (!model.valid()) {
    out.message = "模型无效";
    return out;
  }
  const int nv = static_cast<int>(model.nv());
  const int nq = static_cast<int>(model.nq());
  const int np = gravityParamCount(model);

  // ── 输入校验（C03 的教训：非法输入不能当"成功"）──
  std::vector<int> valid;
  valid.reserve(samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto& s = samples[i];
    if (s.q.size() != nq || s.tau.size() != nv) continue;
    if (!s.q.allFinite() || !s.tau.allFinite()) continue;
    if (!(s.weight > 0.0)) continue;
    valid.push_back(static_cast<int>(i));
  }
  out.n_samples = static_cast<int>(samples.size());
  out.n_used = static_cast<int>(valid.size());
  if (valid.size() < static_cast<size_t>(np / 4)) {   // 样本太少，拟合无意义
    out.message = "有效样本太少：" + std::to_string(valid.size()) + " 个";
    return out;
  }

  // ── 留出集 ──
  std::vector<int> fit_idx, hold_idx;
  if (opt.holdout_stride > 0) {
    for (size_t k = 0; k < valid.size(); ++k) {
      if (static_cast<int>(k) % opt.holdout_stride == 0) hold_idx.push_back(valid[k]);
      else fit_idx.push_back(valid[k]);
    }
  } else {
    fit_idx = valid;
  }
  if (fit_idx.size() < static_cast<size_t>(nv)) {
    out.message = "参与拟合的样本太少";
    return out;
  }

  // ── 堆叠 + SVD 求可辨识子空间 ──
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  stackSamples(model, ws, samples, fit_idx, A, b);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  const auto& S = svd.singularValues();
  if (S.size() == 0 || !(S[0] > 0.0)) {
    out.message = "回归矩阵全零（样本位形无变化？）";
    return out;
  }
  int rank = 0;
  for (Eigen::Index i = 0; i < S.size(); ++i) {
    if (S[i] / S[0] > opt.rank_tol) ++rank;
  }
  out.rank = rank;
  if (rank == 0) {
    out.message = "可辨识秩为 0";
    return out;
  }

  // B = V 的前 rank 列（np × rank），即可辨识子空间的正交基
  const Eigen::MatrixXd B = svd.matrixV().leftCols(rank);

  // ── 在子空间内最小二乘：min ‖A·B·x − b‖ ──
  const Eigen::MatrixXd AB = A * B;                 // (nv*N) × rank
  const Eigen::VectorXd x =
      AB.colPivHouseholderQr().solve(b);            // rank 很小，QR 足够
  out.theta = B * x;                                // np

  // ── 拟合残差 ──
  const Eigen::VectorXd r = A * out.theta - b;
  const int n = static_cast<int>(fit_idx.size());
  out.per_joint_rms = Eigen::VectorXd::Zero(nv);
  double ss = 0.0, mx = 0.0;
  for (int k = 0; k < n; ++k) {
    const Eigen::VectorXd rk = r.segment(k * nv, nv);
    ss += rk.squaredNorm();
    for (int j = 0; j < nv; ++j) {
      out.per_joint_rms[j] += rk[j] * rk[j];
      mx = std::max(mx, std::abs(rk[j]));
    }
  }
  out.rms = std::sqrt(ss / static_cast<double>(n * nv));
  out.max_abs = mx;
  for (int j = 0; j < nv; ++j) {
    out.per_joint_rms[j] = std::sqrt(out.per_joint_rms[j] / static_cast<double>(n));
  }

  // ── 留出集验证（真正的泛化精度）──
  if (!hold_idx.empty()) {
    Eigen::MatrixXd Ah;
    Eigen::VectorXd bh;
    stackSamples(model, ws, samples, hold_idx, Ah, bh);
    const Eigen::VectorXd rh = Ah * out.theta - bh;
    out.holdout_rms = rh.norm() / std::sqrt(static_cast<double>(rh.size()));
    out.holdout_max = rh.cwiseAbs().maxCoeff();
    out.holdout_ok = true;
  }

  out.ok = true;
  out.message = "rank=" + std::to_string(rank) + "，样本 " +
                std::to_string(fit_idx.size()) + "/" + std::to_string(valid.size());
  return out;
}

}  // namespace hw_arm_algorithms
