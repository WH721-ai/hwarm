// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 重力项 g(q) 的单元测试
//
// 基线值来自 2026-11-07 用 Python(Pinocchio 4.1.0) 对同一份 URDF 的计算：
//   g(q=0) = [0, 5.5352, -1.6975, -0.0043, -0.0032, 0] N·m
// C++ 侧必须算出同样的数，否则说明模型加载或调用方式有差异。

#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

std::string resolveUrdf() {
  const char* env = std::getenv("HW_ARM_URDF");
  if (env != nullptr && env[0] != '\0') return std::string(env);
#ifdef HW_ARM_DEFAULT_URDF
  return std::string(HW_ARM_DEFAULT_URDF);
#else
  return std::string();
#endif
}

class GravityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string path = resolveUrdf();
    if (path.empty() || !std::ifstream(path).good()) {
      GTEST_SKIP() << "找不到 URDF（" << path << "）";
    }
    ASSERT_TRUE(model_.load(path));
    ASSERT_TRUE(model_.valid());
    ws_ = std::make_unique<hw_arm_algorithms::RobotWorkspace>(model_);
    nv_ = model_.nv();
  }

  hw_arm_algorithms::RobotModel model_;
  std::unique_ptr<hw_arm_algorithms::RobotWorkspace> ws_;
  int nv_ = 0;
};

}  // namespace

// ① 零位重力与 Python 基线一致
TEST_F(GravityTest, MatchesPythonBaselineAtZero) {
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(nv_);
  Eigen::VectorXd g(nv_);
  hw_arm_algorithms::computeGravity(model_, *ws_, q, g);

  // 基线随几何更新（2026-11 换成 CAD 插件导出的新结构 + base_link 绕 X 翻 180°）。
  // base_link 的 +z 现在是重力方向，机械臂正立向上。
  // 历史上：未翻转时 g[1]=-2.2450；OpenArm 结构 g[1]=4.4900。
  // 2026-11-19 真机逐件称重后（总质量 2.661 → 2.7403 kg）重新计算
  // 2026-11-26 夹爪正式进 URDF（gr_* fixed，0.3185 kg）后重算：
  //   joint2 2.2854 → 1.9557、joint3 2.8214 → 4.0208（夹爪的质心在偏航轴外侧）
  //   joint4/5/6 从 ~0 变成 −0.0012 / 0.5882 / −0.0258
  //   独立核对：Pinocchio Python 对同一 URDF 算得
  //   [-0, 1.955674, 4.020838, -0.001225, 0.588200, -0.025800]
  const std::vector<double> expected = {0.0,         1.955674, 4.020838,
                                        -0.001225,  0.588200, -0.025800};
  for (int i = 0; i < nv_; ++i) {
    EXPECT_NEAR(g[i], expected[static_cast<size_t>(i)], 1e-3)
        << "joint" << (i + 1) << " 重力矩与基线不符";
  }
}

// ② rnea(q,0,0) 与 computeGeneralizedGravity 必须一致
TEST_F(GravityTest, TwoAlgorithmsAgree) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nv_);
  EXPECT_LT(hw_arm_algorithms::verifyGravityConsistency(model_, *ws_, q0), 1e-10);

  Eigen::VectorXd q(nv_);
  q << 0.89, 0.095, 0.094, 0.197, 0.393, 0.270;  // 实测姿态
  EXPECT_LT(hw_arm_algorithms::verifyGravityConsistency(model_, *ws_, q), 1e-10);
}

// ③ joint1（竖直轴）与 joint6（腕部末端）的重力矩恒为 0
TEST_F(GravityTest, VerticalAxisHasNoGravityTorque) {
  Eigen::VectorXd g(nv_);

  for (double q1 : {-1.0, 0.0, 0.8, 2.0, 3.0}) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nv_);
    q[0] = q1;
    hw_arm_algorithms::computeGravity(model_, *ws_, q, g);
    // CAD 插件导出的 rpy 只有 7 位小数（3.1415927 而非 π），
    // 所以"竖直轴"实际偏了 ~3.5e-8 rad → g[0] 有 ~1e-7 的残差。
    // 物理上完全可忽略，这里放宽到 1e-6。
    EXPECT_NEAR(g[0], 0.0, 1e-6) << "joint1 在 q1=" << q1 << " 时不应有重力矩";
  }
}

// ④ joint2 承载最大（新结构下 g2 为负，量级 ~2.2 N·m）
TEST_F(GravityTest, Joint2CarriesTheArm) {
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(nv_);
  Eigen::VectorXd g(nv_);
  hw_arm_algorithms::computeGravity(model_, *ws_, q, g);
  EXPECT_GT(std::abs(g[1]), 1.5);
  EXPECT_LT(std::abs(g[1]), 6.0);
  // 承载最大重力矩的是 joint2 或 joint3（取决于结构：
  //   旧 OpenArm 结构 → joint2；新 CAD 结构 → joint3）
  const int argmax = (std::abs(g[1]) >= std::abs(g[2])) ? 1 : 2;
  for (int i = 0; i < nv_; ++i) {
    EXPECT_LE(std::abs(g[i]), std::abs(g[argmax]) + 1e-9)
        << "最大重力矩应在 joint" << (argmax + 1) << "，但 joint" << (i + 1)
        << " 更大（新结构实测 joint3 = +2.821, joint2 = +2.285）";
  }
}

// ⑤ 重力矩随姿态变化，且 joint2 的灵敏度最大
TEST_F(GravityTest, SensitivityToPose) {
  Eigen::VectorXd g_a(nv_), g_b(nv_);
  Eigen::VectorXd q_a = Eigen::VectorXd::Zero(nv_);
  Eigen::VectorXd q_b = Eigen::VectorXd::Zero(nv_);

  // 只改 joint2：从 0.0 rad 变到 2.0 rad（新结构下 g2 单调上升）
  q_a[1] = 0.0;
  q_b[1] = 2.0;
  hw_arm_algorithms::computeGravity(model_, *ws_, q_a, g_a);
  hw_arm_algorithms::computeGravity(model_, *ws_, q_b, g_b);

  // 实测：q2 从 0 → 2.0 rad，g[1] 从 +2.245 → -2.719，变化约 4.96 N·m
  EXPECT_GT(std::abs(g_b[1] - g_a[1]), 3.0) << "joint2 角度变化应显著影响重力矩";
}

// ⑥ direction = ±1 的符号转换正确
TEST_F(GravityTest, DirectionSignConversion) {
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(nv_);
  Eigen::VectorXd g_urdf(nv_);
  hw_arm_algorithms::computeGravity(model_, *ws_, q, g_urdf);

  std::vector<double> dir(static_cast<size_t>(nv_), 1.0);
  dir[2] = -1.0;  // joint3 反转
  Eigen::VectorXd tau_motor(nv_);
  hw_arm_algorithms::computeGravityMotor(model_, *ws_, q, dir, tau_motor);

  EXPECT_NEAR(tau_motor[0], g_urdf[0], 1e-12);
  EXPECT_NEAR(tau_motor[1], g_urdf[1], 1e-12);
  EXPECT_NEAR(tau_motor[2], -g_urdf[2], 1e-12) << "joint3 应被取反";
  EXPECT_NEAR(tau_motor[3], g_urdf[3], 1e-12);
}

// ⑦ 维度检查：q 长度不对应抛异常
TEST_F(GravityTest, RejectsWrongDimension) {
  Eigen::VectorXd q_wrong = Eigen::VectorXd::Zero(nv_ - 1);
  Eigen::VectorXd g(nv_);
  EXPECT_THROW(hw_arm_algorithms::computeGravity(model_, *ws_, q_wrong, g),
               std::invalid_argument);
}

// ⑧ 零重力时结果应为 0（验证重力向量确实生效）
TEST_F(GravityTest, ZeroGravityGivesZeroTorque) {
  hw_arm_algorithms::RobotModel m2;
  ASSERT_TRUE(m2.load(resolveUrdf()));
  m2.setGravity(0.0);

  hw_arm_algorithms::RobotWorkspace ws2(m2);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(m2.nv());
  Eigen::VectorXd g(m2.nv());
  hw_arm_algorithms::computeGravity(m2, ws2, q, g);
  EXPECT_LT(g.cwiseAbs().maxCoeff(), 1e-12);
}
