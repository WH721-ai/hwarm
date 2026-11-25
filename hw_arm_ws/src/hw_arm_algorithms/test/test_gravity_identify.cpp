// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// C-05 辨识算法的单元测试：用**仿真重力矩**验证能恢复 g(q)
//
// 注意：不能要求参数 θ̂ 等于标称 θ（rank 10 < 24 个自由参数），
//       只能要求 g(q) 复现准确 —— 这才是补偿需要的。
#include <cmath>
#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/gravity_identify.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

using namespace hw_arm_algorithms;

class GravityIdTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<RobotModel>();
    ASSERT_TRUE(model_->load(HW_ARM_DEFAULT_URDF));
    ws_ = std::make_unique<RobotWorkspace>(*model_);
  }

  /// 造样本：τ = g_sim(q)，可选加噪声/摩擦
  std::vector<GravityIdSample> makeSamples(int n, double noise,
                                           double friction) {
    std::mt19937 rng(42);
    std::normal_distribution<double> nd(0.0, noise);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    auto lo = model_->pin().lowerPositionLimit;
    auto hi = model_->pin().upperPositionLimit;
    std::vector<GravityIdSample> out;
    Eigen::VectorXd g;
    for (int k = 0; k < n; ++k) {
      GravityIdSample s;
      s.q.resize(model_->nq());
      for (int j = 0; j < model_->nq(); ++j) s.q[j] = lo[j] + (hi[j] - lo[j]) * u(rng);
      computeGravity(*model_, *ws_, s.q, g);
      s.tau = g;
      for (int j = 0; j < g.size(); ++j) s.tau[j] += nd(rng) + friction * (u(rng) > 0.5 ? 1.0 : -1.0);
      out.push_back(std::move(s));
    }
    return out;
  }

  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<RobotWorkspace> ws_;
};

// 回归矩阵：g(q) = Y·θ_nominal 必须精确成立
TEST_F(GravityIdTest, RegressorMatchesGravity) {
  Eigen::MatrixXd Y;
  Eigen::VectorXd g;
  const Eigen::VectorXd q = (Eigen::VectorXd(6) << 0., 0.8, 0.4, 0., 0.6, 0.).finished();
  computeGravity(*model_, *ws_, q, g);
  computeGravityRegressor(*model_, *ws_, q, Y);

  ASSERT_EQ(Y.rows(), 6);
  ASSERT_EQ(Y.cols(), gravityParamCount(*model_));
  EXPECT_EQ(Y.cols(), 60);

  // 标称 θ
  Eigen::VectorXd theta(60);
  for (int b = 0; b < 6; ++b) {
    const auto dp = model_->pin().inertias[b + 1].toDynamicParameters();
    theta.segment(b * 10, 10) = dp.head(10);
  }
  EXPECT_LT((Y * theta - g).cwiseAbs().maxCoeff(), 1e-10);
}

// 无噪声：拟合后应精确复现（残差 ~ 机器精度）
TEST_F(GravityIdTest, NoiselessFitIsExact) {
  auto samples = makeSamples(40, 0.0, 0.0);
  const auto r = identifyGravity(*model_, *ws_, samples);
  ASSERT_TRUE(r.ok) << r.message;
  EXPECT_EQ(r.rank, 10) << "本臂重力回归矩阵的有效秩实测为 10";
  EXPECT_LT(r.rms, 1e-8) << "无噪声时应精确（实测 ~2e-10，受条件数限制）";
  EXPECT_LT(r.max_abs, 1e-9);
}

// 有噪声：残差应接近噪声水平，且留出集也接近（不过拟合）
TEST_F(GravityIdTest, NoisyFitGeneralizes) {
  auto samples = makeSamples(40, 0.02, 0.0);   // 20 mN·m 噪声
  GravityIdOptions opt;
  opt.holdout_stride = 4;
  const auto r = identifyGravity(*model_, *ws_, samples, opt);
  ASSERT_TRUE(r.ok) << r.message;
  EXPECT_LT(r.rms, 0.03);
  ASSERT_TRUE(r.holdout_ok);
  EXPECT_LT(r.holdout_rms, 0.04) << "留出集残差应和拟合残差同量级";
}

// ★ 摩擦是主要干扰：加 0.5 N·m 的随机方向库仑摩擦后，
//   单向采集会把摩擦当成重力的一部分 → 残差变大
//   （真机上靠"双向采集取平均"消除，这里只验证它会变差，说明必须双向）
TEST_F(GravityIdTest, FrictionDegradesSingleDirectionFit) {
  auto clean = makeSamples(40, 0.0, 0.0);
  auto dirty = makeSamples(40, 0.0, 0.5);
  const auto rc = identifyGravity(*model_, *ws_, clean);
  const auto rd = identifyGravity(*model_, *ws_, dirty);
  ASSERT_TRUE(rc.ok); ASSERT_TRUE(rd.ok);
  EXPECT_LT(rc.rms, 1e-8);
  EXPECT_GT(rd.rms, 0.1) << "0.5 N·m 库仑摩擦应当明显抬高残差";
}

// 辨识参数在**训练集外**的位形上仍能准确复现 g
TEST_F(GravityIdTest, ThetaReproducesGravityAtUnseenPoses) {
  auto samples = makeSamples(40, 0.0, 0.0);
  const auto r = identifyGravity(*model_, *ws_, samples);
  ASSERT_TRUE(r.ok);

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto lo = model_->pin().lowerPositionLimit;
  auto hi = model_->pin().upperPositionLimit;
  double worst = 0.0;
  for (int k = 0; k < 200; ++k) {
    Eigen::VectorXd q(6);
    for (int j = 0; j < 6; ++j) q[j] = lo[j] + (hi[j] - lo[j]) * u(rng);
    Eigen::VectorXd g_true;
    computeGravity(*model_, *ws_, q, g_true);
    const Eigen::VectorXd g_hat = evalGravityTheta(*model_, *ws_, q, r.theta);
    worst = std::max(worst, (g_true - g_hat).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(worst, 1e-9) << "200 个未参与拟合的位形，最大误差 " << worst;
}

// 非法样本必须被剔除，而不是让拟合"成功"
TEST_F(GravityIdTest, InvalidSamplesAreDropped) {
  auto samples = makeSamples(20, 0.0, 0.0);
  GravityIdSample bad;
  bad.q = Eigen::VectorXd::Constant(6, std::nan(""));
  bad.tau = Eigen::VectorXd::Zero(6);
  samples.push_back(bad);
  GravityIdSample wrong;
  wrong.q = Eigen::VectorXd::Zero(5);
  wrong.tau = Eigen::VectorXd::Zero(6);
  samples.push_back(wrong);

  const auto r = identifyGravity(*model_, *ws_, samples);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.n_samples, 22);
  EXPECT_EQ(r.n_used, 20);
}

// 空/过少样本 → 明确失败，不能返回 ok
TEST_F(GravityIdTest, TooFewSamplesFails) {
  std::vector<GravityIdSample> none;
  EXPECT_FALSE(identifyGravity(*model_, *ws_, none).ok);

  auto two = makeSamples(2, 0.0, 0.0);
  const auto r = identifyGravity(*model_, *ws_, two);
  EXPECT_FALSE(r.ok) << r.message;
}

}  // namespace
