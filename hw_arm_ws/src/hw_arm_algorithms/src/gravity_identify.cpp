// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/gravity_identify.hpp"

#include <algorithm>
#include <cmath>

#include <yaml-cpp/yaml.h>
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

/// 在回归矩阵末尾追加"每轴库仑摩擦"的列（N*nv × nv）
///
/// 第 k 个样本的第 j 行、第 j 个摩擦列填 signs[k]：
///   A 组（直接接近）→ +1，B 组（反向接近）→ −1。
/// 与重力/偏置列线性无关，可以一起解。
Eigen::MatrixXd augmentWithFriction(const Eigen::MatrixXd& A, int nv,
                                    const std::vector<double>& signs) {
  const int n_row = static_cast<int>(A.rows());
  const int np = static_cast<int>(A.cols());
  const int n_smp = static_cast<int>(signs.size());
  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(n_row, np + nv);
  A2.leftCols(np) = A;
  for (int k = 0; k < n_smp; ++k) {
    for (int j = 0; j < nv; ++j) {
      A2(k * nv + j, np + j) = signs[static_cast<size_t>(k)];
    }
  }
  return A2;
}

/// 在回归矩阵末尾追加"每轴恒定偏置"的单位块（N*nv × nv）
Eigen::MatrixXd augmentWithOffsets(const Eigen::MatrixXd& A, int nv) {
  const int n_row = static_cast<int>(A.rows());
  const int np = static_cast<int>(A.cols());
  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(n_row, np + nv);
  A2.leftCols(np) = A;
  for (int k = 0; k < n_row / nv; ++k) {
    A2.block(k * nv, np, nv, nv).setIdentity();
  }
  return A2;
}

}  // namespace

bool loadGravityParams(const std::string& path, GravityParams& out,
                       std::string* error) {
  out = GravityParams{};
  out.source = path;
  try {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root["theta"]) {
      if (error) *error = "缺少 theta 字段";
      return false;
    }
    const auto& tn = root["theta"];
    out.theta.resize(static_cast<Eigen::Index>(tn.size()));
    for (size_t i = 0; i < tn.size(); ++i) out.theta[static_cast<Eigen::Index>(i)] = tn[i].as<double>();

    auto read_vec = [](const YAML::Node& n, Eigen::VectorXd& v) {
      if (!n || !n.IsSequence()) return false;
      v.resize(static_cast<Eigen::Index>(n.size()));
      for (size_t i = 0; i < n.size(); ++i) v[static_cast<Eigen::Index>(i)] = n[i].as<double>();
      return true;
    };
    read_vec(root["offsets"], out.offsets);
    read_vec(root["friction"], out.friction);

    if (root["meta"]) {
      const auto& m = root["meta"];
      if (m["rms"]) out.rms = m["rms"].as<double>();
      if (m["holdout_rms"]) out.holdout_rms = m["holdout_rms"].as<double>();
      if (m["rank"]) out.rank = m["rank"].as<int>();
    }
    out.valid = true;
    return true;
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
}

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

  // 把"每轴恒定偏置"并进回归矩阵：重力项随位形变、偏置项不变，两者正交。
  // A 变成 N*nv x (np + nv)，末尾追加 6 个单位块。
  const int np_add = opt.estimate_offsets ? nv : 0;
  const int np_total = np + np_add + (opt.estimate_friction ? nv : 0);
  if (np_add > 0) A = augmentWithOffsets(A, nv);

  // ★ 库仑摩擦列：A 组 +1、B 组 −1（靠双向接近抵消）
  const bool do_fric = opt.estimate_friction;
  const int np_fric = do_fric ? nv : 0;
  if (do_fric) {
    std::vector<double> signs;
    signs.reserve(fit_idx.size());
    for (int k : fit_idx) {
      const std::string& tg = samples[static_cast<size_t>(k)].tag;
      if (!tg.empty() && (tg[0] == 'B' || tg[0] == 'b')) {
        signs.push_back(-1.0);
      } else if (!tg.empty() && (tg[0] == 'A' || tg[0] == 'a')) {
        signs.push_back(+1.0);
      } else {
        signs.push_back(0.0);   // 没标 tag 就没有摩擦信息
      }
    }
    A = augmentWithFriction(A, nv, signs);
  }

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

  // ── 在子空间内求解 ──
  //
  // 无正则：min ‖A·B·x − b‖                     （纯最小二乘）
  // 有正则：min ‖A·B·x − b‖² + λ‖B·x − x_prior‖² （往标称参数拉）
  //
  // 正则化之所以能改善**留出**误差：设计矩阵的奇异值从 137 掉到 0.4，
  // 那些"几乎没被激励"的方向上纯 LS 会把噪声放大成很大的参数偏差，
  // 训练集看起来还行、换一批位形就崩。加 λ 后这些方向自动退回标称值。
  const Eigen::MatrixXd AB = A * B;                 // (nv*N) × rank
  Eigen::VectorXd x;
  if (opt.regularize) {
    // 先验变换到子空间里：x_prior_sub = B^T x_prior
    const Eigen::VectorXd theta_prior = gravityParamPrior(model);
    // ⚠️ x_prior_sub 的长度必须是 **rank**，不是 np_total。
    //    B 是 (np_total × rank)，Bᵀ·θ_prior 的结果自然是 rank 长。
    //    之前写成 Zero(np_total)，于是 rhs 变成 72 维而 M 是 22×22，
    //    NDEBUG 下 Eigen 不做断言 → 读越界 → 解直接变成垃圾
    //    （表现为 offsets / friction 全 0、rms 从 0.47 涨到 2.18）。
    Eigen::VectorXd x_prior_sub = Eigen::VectorXd::Zero(rank);
    const int npb = std::min<int>(np, static_cast<int>(theta_prior.size()));
    x_prior_sub = B.topRows(npb).transpose() * theta_prior.head(npb);
    // 偏置/摩擦的先验取 0（已经在 x_prior_sub 里是 0）
    const double lam =
        std::pow(opt.ridge_ratio * svd.singularValues()(0), 2.0);
    out.ridge_lambda = lam;
    Eigen::MatrixXd M = AB.transpose() * AB + lam * Eigen::MatrixXd::Identity(rank, rank);
    Eigen::VectorXd rhs = AB.transpose() * b + lam * x_prior_sub;
    x = M.ldlt().solve(rhs);
  } else {
    x = AB.colPivHouseholderQr().solve(b);           // rank 很小，QR 足够
  }
  const Eigen::VectorXd x_full = B * x;
  out.theta = x_full.head(np);
  if (np_add > 0) {
    out.offsets = x_full.segment(np, nv);
  } else {
    out.offsets = Eigen::VectorXd::Zero(nv);
  }
  if (np_fric > 0) {
    out.friction = x_full.segment(np + np_add, nv);
  } else {
    out.friction = Eigen::VectorXd::Zero(nv);
  }

  // ── 拟合残差 ──
  Eigen::VectorXd x_hat(A.cols());
  x_hat.setZero();
  x_hat.head(np) = out.theta;
  if (np_add > 0) x_hat.segment(np, nv) = out.offsets;
  if (np_fric > 0) x_hat.segment(np + np_add, nv) = out.friction;
  const Eigen::VectorXd r = A * x_hat - b;
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
    // ★ 留出集必须和拟合集同样追加偏置列 ——
    //    否则 Ah.cols() 还是 60，xh.tail(nv) 会写进最后 6 个重力参数里，
    //    算出来的 holdout 残差完全是错的（实测会报 0.96 N·m 这种假数字）。
    if (np_add > 0) Ah = augmentWithOffsets(Ah, nv);
    if (np_fric > 0) {
      std::vector<double> hs;
      hs.reserve(hold_idx.size());
      for (int k : hold_idx) {
        const std::string& tg = samples[static_cast<size_t>(k)].tag;
        hs.push_back((!tg.empty() && (tg[0] == 'B' || tg[0] == 'b')) ? -1.0
                     : (!tg.empty() && (tg[0] == 'A' || tg[0] == 'a')) ? 1.0 : 0.0);
      }
      Ah = augmentWithFriction(Ah, nv, hs);
    }
    Eigen::VectorXd xh(Ah.cols());
    xh.setZero();
    xh.head(np) = out.theta;
    if (np_add > 0) xh.segment(np, nv) = out.offsets;
    if (np_fric > 0) {
      std::vector<double> hs;
      hs.reserve(hold_idx.size());
      for (int k : hold_idx) {
        const std::string& tg = samples[static_cast<size_t>(k)].tag;
        hs.push_back((!tg.empty() && (tg[0] == 'B' || tg[0] == 'b')) ? -1.0
                     : (!tg.empty() && (tg[0] == 'A' || tg[0] == 'a')) ? 1.0 : 0.0);
      }
      xh.segment(np + np_add, nv) = out.friction;
      (void)hs;   // 留出集矩阵本身也要加摩擦列，见下
    }
    const Eigen::VectorXd rh = Ah * xh - bh;
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
