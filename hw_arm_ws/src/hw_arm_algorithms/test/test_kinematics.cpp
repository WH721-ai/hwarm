// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
#include "hw_arm_algorithms/model.hpp"

using hw_arm_algorithms::IKSolution;
using hw_arm_algorithms::IKOptions;
using hw_arm_algorithms::IKResult;
using hw_arm_algorithms::JacobianFrame;
using hw_arm_algorithms::Manipulability;
using hw_arm_algorithms::RobotModel;
using hw_arm_algorithms::RobotWorkspace;

namespace {

const char* kUrdf = HW_ARM_DEFAULT_URDF;

class KinematicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<RobotModel>();
    ASSERT_TRUE(model_->load(kUrdf));
    ws_ = std::make_unique<RobotWorkspace>(*model_);
    ee_ = model_->frameId("tool0");
  }

  Eigen::VectorXd randomQ(unsigned seed) const {
    std::srand(seed);
    Eigen::VectorXd q(model_->nq());
    const auto& lo = model_->pin().lowerPositionLimit;
    const auto& hi = model_->pin().upperPositionLimit;
    for (int i = 0; i < q.size(); ++i) {
      const double u = static_cast<double>(std::rand()) / RAND_MAX;
      q[i] = lo[i] + (hi[i] - lo[i]) * u;
    }
    return q;
  }

  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<RobotWorkspace> ws_;
  pinocchio::FrameIndex ee_{0};
};

}  // namespace

// ─────────────────────────────────────────────────────────────
// FK
// ─────────────────────────────────────────────────────────────
TEST_F(KinematicsTest, FkMatchesPinocchio) {
  const Eigen::VectorXd q = randomQ(1);
  pinocchio::SE3 pose;
  hw_arm_algorithms::computeFK(*model_, *ws_, q, ee_, pose);

  auto& data = ws_->pin();
  pinocchio::forwardKinematics(model_->pin(), data, q);
  pinocchio::updateFramePlacement(model_->pin(), data, ee_);

  EXPECT_LT((pose.translation() - data.oMf[ee_].translation()).norm(), 1e-12);
  EXPECT_LT((pose.rotation() - data.oMf[ee_].rotation()).norm(), 1e-12);
}

TEST_F(KinematicsTest, FkAtZeroPose) {
  // 零位（CAD 姿态 + base_link 翻 180° + 整机抬高 30mm）：tool0 = (0.0577, 0.0341, 0.2442)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq());
  pinocchio::SE3 pose;
  hw_arm_algorithms::computeFK(*model_, *ws_, q, ee_, pose);
  EXPECT_NEAR(pose.translation().x(), 0.05774, 1e-3);
  EXPECT_NEAR(pose.translation().y(), 0.03411, 1e-3);
  EXPECT_NEAR(pose.translation().z(), 0.24421, 1e-3);
}

// ─────────────────────────────────────────────────────────────
// 雅可比
// ─────────────────────────────────────────────────────────────
TEST_F(KinematicsTest, JacobianMatchesFiniteDifference) {
  const Eigen::VectorXd q = randomQ(2);
  const int nv = model_->nv();
  const int nq = model_->nq();

  Eigen::MatrixXd J(6, nv);
  hw_arm_algorithms::computeJacobian(*model_, *ws_, q, ee_,
                                     JacobianFrame::LOCAL_WORLD_ALIGNED, J);

  // 数值微分：末端位置/姿态对关节角的变化率
  const double h = 1e-7;
  Eigen::MatrixXd Jn(6, nv);
  pinocchio::SE3 base;
  hw_arm_algorithms::computeFK(*model_, *ws_, q, ee_, base);

  for (int i = 0; i < nq; ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp[i] += h;
    qm[i] -= h;
    pinocchio::SE3 Tp, Tm;
    hw_arm_algorithms::computeFK(*model_, *ws_, qp, ee_, Tp);
    hw_arm_algorithms::computeFK(*model_, *ws_, qm, ee_, Tm);

    const Eigen::Vector3d dp = (Tp.translation() - Tm.translation()) / (2 * h);
    // 姿态：用 log3 求相对旋转向量
    const Eigen::Matrix3d dR = (Tp.rotation() - Tm.rotation()) / (2 * h);
    // ω̂ = dR · Rᵀ （反对称部分解出角速度）
    const Eigen::Matrix3d W = dR * base.rotation().transpose();
    Eigen::Vector3d w(W(2, 1) - W(1, 2), W(0, 2) - W(2, 0), W(1, 0) - W(0, 1));
    w *= 0.5;

    Jn.block<3, 1>(0, i) = dp;
    Jn.block<3, 1>(3, i) = w;
  }

  EXPECT_LT((J - Jn).cwiseAbs().maxCoeff(), 1e-4)
      << "解析雅可比与数值微分不一致";
}

TEST_F(KinematicsTest, JacobianFrameTypesDiffer) {
  const Eigen::VectorXd q = randomQ(3);
  const int nv = model_->nv();
  Eigen::MatrixXd Jl(6, nv), Jw(6, nv);
  hw_arm_algorithms::computeJacobian(*model_, *ws_, q, ee_,
                                     JacobianFrame::LOCAL, Jl);
  hw_arm_algorithms::computeJacobian(*model_, *ws_, q, ee_,
                                     JacobianFrame::LOCAL_WORLD_ALIGNED, Jw);
  // 两种参考系的旋转块必然不同（除非 R = I）
  EXPECT_GT((Jl - Jw).cwiseAbs().maxCoeff(), 1e-6);
}

// ─────────────────────────────────────────────────────────────
// IK
// ─────────────────────────────────────────────────────────────
TEST_F(KinematicsTest, IkRoundTrip) {
  const Eigen::VectorXd q_true = randomQ(4);

  pinocchio::SE3 target;
  hw_arm_algorithms::computeFK(*model_, *ws_, q_true, ee_, target);

  // 从一个明显不同的种子出发
  Eigen::VectorXd seed = q_true;
  for (int i = 0; i < seed.size(); ++i) {
    seed[i] += 0.15;
  }

  IKOptions opt;
  const IKSolution sol = hw_arm_algorithms::solveIK(
      *model_, *ws_, ee_, target, seed, opt);

  ASSERT_TRUE(sol.converged);
  EXPECT_LT(sol.position_error, 1e-4);
  EXPECT_LT(sol.orientation_error, 1e-4);

  // 解的 FK 必须复现目标位姿
  pinocchio::SE3 check;
  hw_arm_algorithms::computeFK(*model_, *ws_, sol.q, ee_, check);
  EXPECT_LT((check.translation() - target.translation()).norm(), 1e-4);
}

TEST_F(KinematicsTest, IkRespectsJointLimits) {
  const Eigen::VectorXd q_true = randomQ(5);
  pinocchio::SE3 target;
  hw_arm_algorithms::computeFK(*model_, *ws_, q_true, ee_, target);

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(model_->nq());
  IKOptions opt;
  opt.num_seeds = 24;
  const IKResult res = hw_arm_algorithms::solveIKMultiSeed(
      *model_, *ws_, ee_, target, opt);

  ASSERT_TRUE(res.ok()) << "多起点应当至少找到一组解";
  const auto& lo = model_->pin().lowerPositionLimit;
  const auto& hi = model_->pin().upperPositionLimit;
  for (const auto& s : res.solutions) {
    for (int i = 0; i < s.q.size(); ++i) {
      EXPECT_GE(s.q[i], lo[i] - 1e-9) << "解超出下限 joint" << i + 1;
      EXPECT_LE(s.q[i], hi[i] + 1e-9) << "解超出上限 joint" << i + 1;
    }
  }
}

TEST_F(KinematicsTest, IkUnreachableReturnsNoSolution) {
  // 目标远超臂展（臂展 ~0.44 m，给 2 m）
  const pinocchio::SE3 target(Eigen::Matrix3d::Identity(),
                              Eigen::Vector3d(2.0, 2.0, 1.0));
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(model_->nq());
  IKOptions opt;
  const IKSolution sol = hw_arm_algorithms::solveIK(
      *model_, *ws_, ee_, target, seed, opt);
  EXPECT_FALSE(sol.converged);
}

TEST_F(KinematicsTest, IkMultiSeedDeduplicates) {
  const Eigen::VectorXd q_true = randomQ(6);
  pinocchio::SE3 target;
  hw_arm_algorithms::computeFK(*model_, *ws_, q_true, ee_, target);

  IKOptions opt;
  opt.num_seeds = 24;
  const IKResult res = hw_arm_algorithms::solveIKMultiSeed(
      *model_, *ws_, ee_, target, opt);
  ASSERT_TRUE(res.ok());

  // 任意两组解之间的距离必须大于去重阈值
  for (int i = 0; i < res.n_solutions(); ++i) {
    for (int j = i + 1; j < res.n_solutions(); ++j) {
      EXPECT_GT(hw_arm_algorithms::jointDistance(res.solutions[i].q,
                                                 res.solutions[j].q),
                opt.dedup_tolerance);
    }
  }
}

// ─────────────────────────────────────────────────────────────
// 奇异度
// ─────────────────────────────────────────────────────────────
TEST_F(KinematicsTest, ManipulabilityIsConsistent) {
  // 新结构（CAD 装配姿态）零位不是奇异位形：σ_min ≈ 2.2e-3，cond ≈ 814
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq());
  const Manipulability m =
      hw_arm_algorithms::computeManipulability(*model_, *ws_, q, ee_);
  EXPECT_GT(m.sigma_min, 0.0);
  EXPECT_GE(m.sigma_max, m.sigma_min);
  EXPECT_NEAR(m.condition, m.sigma_max / m.sigma_min, 1e-6);
  EXPECT_GT(m.yoshikawa, 0.0);
  // 该姿态接近奇异（cond 大），但没过 1e-3 阈值
  EXPECT_LT(m.sigma_min, 1e-2);
}

TEST_F(KinematicsTest, BentPoseIsNotSingular) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq());
  q[1] = 0.6;   // joint2 抬起
  q[2] = 0.5;   // joint3 弯曲
  q[4] = 0.4;   // joint5 偏离腕奇异
  const Manipulability m =
      hw_arm_algorithms::computeManipulability(*model_, *ws_, q, ee_);
  EXPECT_GT(m.sigma_min, 1e-2);
  EXPECT_FALSE(m.is_singular);
  EXPECT_LT(m.condition, 1e3);
}

// ─────────────────────────────────────────────────────────────
// 工具函数
// ─────────────────────────────────────────────────────────────
TEST_F(KinematicsTest, JointDistanceHandlesWrapAround) {
  Eigen::VectorXd a(2), b(2);
  a << 3.0, 0.0;
  b << -3.0, 0.0;              // 与 3.0 相差 2π-6 ≈ 0.283
  const double d = hw_arm_algorithms::jointDistance(a, b);
  EXPECT_NEAR(d, std::abs(std::remainder(6.0, 2 * M_PI)), 1e-9);
}

TEST_F(KinematicsTest, SplitPoseErrorComponents) {
  // 纯平移 0.1 m
  const pinocchio::SE3 cur = pinocchio::SE3::Identity();
  const pinocchio::SE3 tgt(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.1, 0, 0));
  double pe = 0, oe = 0;
  hw_arm_algorithms::splitPoseError(cur, tgt, pe, oe);
  EXPECT_NEAR(pe, 0.1, 1e-9);
  EXPECT_NEAR(oe, 0.0, 1e-9);

  // 纯旋转 90°
  const pinocchio::SE3 rot(pinocchio::exp3(Eigen::Vector3d(0, 0, M_PI / 2)),
                           Eigen::Vector3d::Zero());
  hw_arm_algorithms::splitPoseError(cur, rot, pe, oe);
  EXPECT_NEAR(pe, 0.0, 1e-9);
  EXPECT_NEAR(oe, M_PI / 2, 1e-9);
}
