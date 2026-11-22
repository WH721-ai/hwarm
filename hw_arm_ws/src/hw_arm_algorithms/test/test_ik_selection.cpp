// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// ik_selection 单元测试
//   ① 小工具：限位余量 / σ 比值 / 路径 σ 比值
//   ② 硬过滤：单轴跳变超限被拒
//   ③ 硬过滤：限位余量不足被拒
//   ④ 硬过滤：路径穿奇异被拒（★ 终点本身是好的）
//   ⑤ 打分：两个都合格时选行程小的
//   ⑥ 全部被拒 → ok=false 且有汇总说明
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <pinocchio/algorithm/frames.hpp>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/ik_selection.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

using hw_arm_algorithms::CandidateEvaluation;
using hw_arm_algorithms::IKResult;
using hw_arm_algorithms::IKSolution;
using hw_arm_algorithms::RobotModel;
using hw_arm_algorithms::RobotWorkspace;
using hw_arm_algorithms::SelectionResult;
using hw_arm_algorithms::SelectionWeights;
using hw_arm_algorithms::SolutionFilterOptions;

class SelectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<RobotModel>();
    ASSERT_TRUE(model_->load(HW_ARM_DEFAULT_URDF)) << "URDF 加载失败";
    ws_ = std::make_unique<RobotWorkspace>(*model_);
    for (size_t i = 0; i < model_->pin().frames.size(); ++i) {
      if (model_->pin().frames[i].name == "tool0") {
        frame_ = static_cast<pinocchio::FrameIndex>(i);
        break;
      }
    }
  }

  // 造一个"收敛"的解（q 的取值决定它能不能通过过滤）
  static IKSolution makeSol(const Eigen::VectorXd& q) {
    IKSolution s;
    s.q = q;
    s.converged = true;
    s.limited = false;
    s.position_error = 1e-7;
    s.orientation_error = 1e-7;
    return s;
  }

  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<RobotWorkspace> ws_;
  pinocchio::FrameIndex frame_ = 0;

  // 条件好的位形（σ_min/σ_max ≈ 0.06）
  const Eigen::VectorXd q_good_ =
      (Eigen::VectorXd(6) << 0.0, 0.8, 0.4, 0.0, 0.6, 0.0).finished();
};

// ① 小工具
TEST_F(SelectionTest, HelperFunctions) {
  // 限位余量：q_good=(0,0.8,0.4,0,0.6,0) 的最小余量在 joint3 下限侧
  //   0.4 − (−0.0244) = 0.4244 rad
  const double margin = hw_arm_algorithms::minLimitMargin(*model_, q_good_);
  EXPECT_NEAR(margin, 0.4244, 1e-3);

  // 条件好的位形：σ 比值应当明显大于奇异阈值
  const double r = hw_arm_algorithms::sigmaRatioAt(*model_, *ws_, frame_,
                                                   q_good_, 0.1);
  EXPECT_GT(r, 2e-3) << "好位形的 σ 比值不该这么小";

  // 同一构型到自身的路径：λ 恒定
  const double pr = hw_arm_algorithms::pathSigmaRatio(*model_, *ws_, frame_,
                                                     q_good_, q_good_, 0.1, 5);
  EXPECT_NEAR(pr, r, 1e-9);
}

// ② 单轴跳变超限被拒
TEST_F(SelectionTest, RejectsLargeJointJump) {
  Eigen::VectorXd q_far = q_good_;
  q_far[1] += 1.2;  // joint2 跳 1.2 rad > 0.5 上限

  IKResult res;
  res.solutions = {makeSol(q_far)};

  SolutionFilterOptions fo;
  fo.max_joint_jump = 0.5;
  const auto sel = hw_arm_algorithms::selectSolution(*model_, *ws_, frame_, res,
                                                    q_good_, fo);
  ASSERT_EQ(sel.candidates.size(), 1u);
  EXPECT_FALSE(sel.candidates[0].accepted);
  EXPECT_NE(sel.candidates[0].reject_reason.find("跳变"), std::string::npos)
      << sel.candidates[0].reject_reason;
  EXPECT_FALSE(sel.ok);
}

// ③ 限位余量不足被拒
TEST_F(SelectionTest, RejectsSmallLimitMargin) {
  // joint5 限位 ±1.23，取 1.20 → 余量 0.03 rad < 0.087
  Eigen::VectorXd q_edge = q_good_;
  q_edge[4] = 1.20;

  IKResult res;
  res.solutions = {makeSol(q_edge)};

  SolutionFilterOptions fo;
  fo.max_joint_jump = 3.0;      // 放宽，让它是被"余量"拒的
  fo.min_limit_margin = 0.087;
  const auto sel = hw_arm_algorithms::selectSolution(*model_, *ws_, frame_, res,
                                                    q_good_, fo);
  ASSERT_EQ(sel.candidates.size(), 1u);
  EXPECT_FALSE(sel.candidates[0].accepted);
  EXPECT_NE(sel.candidates[0].reject_reason.find("限位余量"), std::string::npos)
      << sel.candidates[0].reject_reason;
}

// ④ 路径穿奇异被拒（终点本身是好的）
TEST_F(SelectionTest, RejectsPathThroughSingularity) {
  // q=(0, 0.3, 0.6, 0, ±1.2, 0)：两端 σ≈7.7e-3 都算好，
  // 但直线插值**穿过 q5=0**，路径上实测最小 σ ≈ 3e-4 → 必须被拒。
  // （q2=0.8/q3=0.4 那组位形不够折叠，路径 min σ 还有 2.8e-3，触发不了）
  Eigen::VectorXd q_a(6), q_b(6);
  q_a << 0.0, 0.3, 0.6, 0.0, 1.2, 0.0;
  q_b << 0.0, 0.3, 0.6, 0.0, -1.2, 0.0;

  // 先确认两端各自是好的
  const double r_b = hw_arm_algorithms::sigmaRatioAt(*model_, *ws_, frame_,
                                                     q_b, 0.1);
  ASSERT_GT(r_b, 2e-3) << "终点本身应当远离奇异";

  const double path = hw_arm_algorithms::pathSigmaRatio(*model_, *ws_, frame_,
                                                       q_a, q_b, 0.1, 21);
  EXPECT_LT(path, r_b) << "穿越 q5=0 的路径应当比端点更差";

  IKResult res;
  res.solutions = {makeSol(q_b)};

  SolutionFilterOptions fo;
  fo.max_joint_jump = 3.0;          // 放宽跳变
  fo.max_joint_norm = 5.0;          // joint5 要转 2.4 rad，放宽行程
  fo.min_limit_margin = 0.0;        // 放宽余量
  fo.path_samples = 21;
  fo.min_sigma_ratio = 2e-3;
  ASSERT_LT(path, 2e-3) << "这组位形的路径应当穿过奇异";

  const auto sel = hw_arm_algorithms::selectSolution(*model_, *ws_, frame_, res,
                                                    q_a, fo);
  ASSERT_EQ(sel.candidates.size(), 1u);
  EXPECT_FALSE(sel.candidates[0].accepted);
  EXPECT_NE(sel.candidates[0].reject_reason.find("路径"), std::string::npos)
      << sel.candidates[0].reject_reason;
}

// ⑤ 两个都合格时选行程小的
TEST_F(SelectionTest, PrefersSmallerTravel) {
  Eigen::VectorXd q_near = q_good_;
  q_near[1] += 0.05;
  Eigen::VectorXd q_far = q_good_;
  q_far[1] += 0.30;

  IKResult res;
  res.solutions = {makeSol(q_far), makeSol(q_near)};  // 故意把远的放前面

  SolutionFilterOptions fo;
  fo.max_joint_jump = 0.5;
  const auto sel = hw_arm_algorithms::selectSolution(*model_, *ws_, frame_, res,
                                                    q_good_, fo);
  ASSERT_TRUE(sel.ok) << sel.message;
  EXPECT_EQ(sel.index, 1) << "应当选行程更小的 #1，实际选了 #" << sel.index;
  EXPECT_NEAR(sel.candidates[1].score, 0.5, 0.6);  // 只是确保打分量级正常
  EXPECT_LT(sel.candidates[1].score, sel.candidates[0].score);
}

// ⑥ 全部被拒
TEST_F(SelectionTest, AllRejectedGivesMessage) {
  Eigen::VectorXd q_bad = q_good_;
  q_bad[1] += 2.0;  // 跳变巨大

  IKResult res;
  res.solutions = {makeSol(q_bad), makeSol(q_bad)};

  SolutionFilterOptions fo;
  fo.max_joint_jump = 0.5;
  const auto sel = hw_arm_algorithms::selectSolution(*model_, *ws_, frame_, res,
                                                    q_good_, fo);
  EXPECT_FALSE(sel.ok);
  EXPECT_FALSE(sel.message.empty());
  EXPECT_EQ(sel.n_accepted, 0);
}

// ⑦ 端到端：selectBestIK 必须能找到"离当前构型最近"的那个解
//    这是方案 A（seed_hint）的核心价值：多起点枚举不能漏掉近解。
TEST_F(SelectionTest, BestIKFindsNearestSolution) {
  // 目标 = 当前位形 FK 后沿 +x 平移 20 mm
  pinocchio::forwardKinematics(model_->pin(), ws_->pin(), q_good_);
  pinocchio::updateFramePlacement(model_->pin(), ws_->pin(), frame_);
  pinocchio::SE3 target = ws_->pin().oMf[frame_];
  target.translation() += Eigen::Vector3d(0.02, 0.0, 0.0);

  // 先看单种子（seed = q_good）能解出什么 —— 这是"最近的解"的基准
  hw_arm_algorithms::IKOptions io;
  io.position_tol = 1e-6;
  io.orientation_tol = 1e-6;
  const IKSolution single =
      hw_arm_algorithms::solveIK(*model_, *ws_, frame_, target, q_good_, io);
  ASSERT_TRUE(single.converged) << "单种子应当能解出来";
  const double ref_jump = (single.q - q_good_).cwiseAbs().maxCoeff();
  EXPECT_LT(ref_jump, 0.2) << "单种子解应当离当前构型很近";

  // 再走完整链路
  hw_arm_algorithms::IKSelectionOptions opt;
  opt.ik = io;
  opt.keep_diagnostics = true;
  const auto res = hw_arm_algorithms::selectBestIK(*model_, *ws_, frame_,
                                                  target, q_good_, opt);

  // 打印候选，方便排查
  for (const auto& c : res.candidates) {
    std::printf("    [#%d] accepted=%d limited=%d jump=%.3f σ=%.4f :: %s\n",
                c.index, static_cast<int>(c.accepted), static_cast<int>(c.limited),
                c.max_joint_jump, c.sigma_ratio, c.reject_reason.c_str());
  }

  EXPECT_TRUE(res.ok) << res.message;
  if (res.ok) {
    const double jump = (res.q - q_good_).cwiseAbs().maxCoeff();
    EXPECT_LT(jump, 0.2) << "选出来的解应当也是近解（不应跳到别的分支）";
  }
}

}  // namespace
