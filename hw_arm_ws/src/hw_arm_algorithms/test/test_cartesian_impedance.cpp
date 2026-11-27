// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// cartesian_impedance 的单元测试
// ─────────────────────────────────────────────────────────────
// 2026-11-28：这个控制器的 .cpp **从来没被编进库**（CMakeLists 漏了），
// 所以它既没编译过也没测过。本次同时补上编译 + 测试。
//
// 实现的是：tau = g(q) + J^T (K e - D v)，外面套
//   - 参考位姿一阶滤波（带速度上限）
//   - 力/力矩上限、关节力矩上限、力矩变化率上限
//   - 激活混合（从上一个实际下发力矩平滑切过来；activation_time 必须 > 0）
//   - 跟踪误差 / 持续饱和 两类锁存故障
//
// 用 J = I、gravity = 常数的"假模型"，期望值可以手算。

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "hw_arm_algorithms/cartesian_impedance.hpp"

namespace ci = hw_arm_algorithms::impedance;

namespace {

constexpr double kDt = 0.008;  // 125 Hz

ci::Sample makeSample(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
                      const ci::Vec6& dq, const ci::Vec6& gravity) {
  ci::Sample s;
  s.pose.p = p;
  s.pose.q = q.normalized();
  s.J = ci::Mat6::Identity();
  s.dq = dq;
  s.gravity = gravity;
  s.valid = true;
  return s;
}

// activation_time 必须 > 0（configure 拒绝 0）。给 1 ms 而 dt = 8 ms，
// 第一拍 u 就饱和到 1，等效"立即生效"。
ci::Config makeConfig(const ci::Vec6& k, const ci::Vec6& inertia) {
  ci::Config c;
  c.stiffness = k;
  c.effective_inertia = inertia;
  c.damping_ratio = ci::Vec6::Ones();
  c.wrench_limit = ci::Vec6::Constant(1e6);
  c.torque_limit = ci::Vec6::Constant(1e6);
  c.torque_slew = ci::Vec6::Constant(1e6);
  c.joint_speed_limit = ci::Vec6::Constant(100.0);
  c.filter_time = 1e-3;
  c.reference_linear_speed = 10.0;
  c.reference_angular_speed = 10.0;
  c.activation_time = 1e-3;
  c.min_dt = 1e-5;
  c.max_dt = 0.05;
  c.max_position_error = 10.0;
  c.max_rotation_error = 3.0;
  c.max_limited_time = 10.0;
  return c;
}

// activate 之后必须先跑一拍才进 Active；setTarget 只在 Active 下允许
void activateToActive(ci::CartesianImpedance& imp, const ci::Sample& s,
                      const ci::Vec6& prev) {
  ASSERT_TRUE(imp.activate(s, prev)) << "activate 失败";
  auto c = imp.update(s, prev, kDt);
  ASSERT_TRUE(c.has_value()) << "首拍 update 失败";
  ASSERT_EQ(imp.state(), ci::State::Active);
}

}  // namespace

// (1) configure 参数校验
TEST(CartesianImpedance, ConfigureRejectsInvalid) {
  ci::CartesianImpedance fresh;
  EXPECT_FALSE(fresh.activate(
      makeSample(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                 ci::Vec6::Zero(), ci::Vec6::Zero()),
      ci::Vec6::Zero()))
      << "未 configure 就 activate 必须失败";

  {
    // ⚠️ 实测：stiffness = 0 是**被接受**的。
    //    此时 damping = 2*zeta*sqrt(0)*sqrt(m) = 0，控制器只输出 g(q)，
    //    即"无刚度、无阻尼、纯重力前馈"= 自由落体模式。
    //    这和 kp_floor=0 的零重力模式等价，作为退化情形可接受，
    //    但**没有任何保护**。见文件末尾的 TODO 备注。
    ci::CartesianImpedance x;
    EXPECT_TRUE(x.configure(makeConfig(ci::Vec6::Zero(), ci::Vec6::Ones())));
    EXPECT_EQ(x.state(), ci::State::Inactive);
  }
  {
    ci::CartesianImpedance x;
    EXPECT_FALSE(x.configure(makeConfig(ci::Vec6::Constant(-1.0), ci::Vec6::Ones())));
  }
  {
    ci::CartesianImpedance x;
    ci::Config c = makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones());
    c.damping_ratio = ci::Vec6::Zero();
    EXPECT_FALSE(x.configure(c));
  }
  {
    ci::CartesianImpedance x;
    ci::Config c = makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones());
    c.activation_time = 0.0;
    EXPECT_FALSE(x.configure(c)) << "activation_time=0 必须被拒";
  }
  {
    ci::CartesianImpedance x;
    ci::Config c = makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones());
    c.min_dt = 0.05;
    c.max_dt = 0.001;
    EXPECT_FALSE(x.configure(c));
  }
  {
    ci::CartesianImpedance x;
    EXPECT_TRUE(x.configure(makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones())));
    EXPECT_EQ(x.state(), ci::State::Inactive);
  }
}

// (2) 在目标位形静止 -> 力矩正好等于重力
TEST(CartesianImpedance, SteadyStateEqualsGravity) {
  const ci::Vec6 K = ci::Vec6::Constant(200.0);
  const ci::Vec6 g = (ci::Vec6() << 0.1, 2.0, 6.0, 0.05, 0.2, 0.08).finished();
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(makeConfig(K, ci::Vec6::Ones())));
  const auto s = makeSample(p0, q0, ci::Vec6::Zero(), g);
  activateToActive(imp, s, g);
  auto cmd = imp.update(s, g, kDt);
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->activation_weight, 1.0, 1e-9);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(cmd->torque[i], g[i], 1e-9) << "joint" << i + 1;
  }
  EXPECT_FALSE(cmd->wrench_limited);
  EXPECT_FALSE(cmd->torque_limited);
}

// (3) 弹簧项：稳态 tau_x ~ K * dx
TEST(CartesianImpedance, SpringTermMatchesStiffness) {
  const double kx = 200.0;
  const ci::Vec6 K = ci::Vec6::Constant(kx);
  const Eigen::Vector3d p_ref(0.3, 0.1, 0.4);
  const Eigen::Vector3d p_meas = p_ref + Eigen::Vector3d(0.01, 0.0, 0.0);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(makeConfig(K, ci::Vec6::Ones())));
  const auto s = makeSample(p_meas, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());
  ASSERT_TRUE(imp.setTarget({p_ref, q0}));

  ci::Vec6 cur = ci::Vec6::Zero();
  for (int i = 0; i < 60; ++i) {
    auto c = imp.update(s, cur, kDt);
    ASSERT_TRUE(c.has_value());
    cur = c->torque;
  }
  // 实测点比参考点偏 +1 cm，恢复力矩指向参考点 -> 沿 -x，即 -2 N
  EXPECT_NEAR(cur[0], -kx * 0.01, 0.05) << "稳态 tau_x 应约等于 -K*dx";
  EXPECT_NEAR(cur[1], 0.0, 1e-6);
  EXPECT_NEAR(cur[2], 0.0, 1e-6);
}

// (4) 阻尼项：沿 +x 运动 -> 沿 -x 的阻尼力
TEST(CartesianImpedance, DampingOpposesMotion) {
  const ci::Vec6 K = ci::Vec6::Constant(100.0);
  const ci::Vec6 inertia = ci::Vec6::Constant(0.5);
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  ci::Vec6 dq = ci::Vec6::Zero();
  dq[0] = 0.1;

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(makeConfig(K, inertia)));
  const auto s = makeSample(p0, q0, dq, ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());

  auto cmd = imp.update(s, ci::Vec6::Zero(), kDt);
  ASSERT_TRUE(cmd.has_value());
  const double d = 2.0 * std::sqrt(100.0 * 0.5);  // 14.142
  EXPECT_NEAR(cmd->torque[0], -d * 0.1, 0.05);
}

// (5) 关节力矩上限
TEST(CartesianImpedance, TorqueLimitClamps) {
  ci::Config c = makeConfig(ci::Vec6::Constant(5000.0), ci::Vec6::Ones());
  c.torque_limit = ci::Vec6::Constant(1.0);
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Vector3d p_meas = p0 + Eigen::Vector3d(0.2, 0.0, 0.0);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(c));
  const auto s = makeSample(p_meas, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());
  ASSERT_TRUE(imp.setTarget({p0, q0}));

  auto cmd = imp.update(s, ci::Vec6::Zero(), kDt);
  ASSERT_TRUE(cmd.has_value());
  for (int i = 0; i < 6; ++i) {
    EXPECT_LE(std::abs(cmd->torque[i]), 1.0 + 1e-12) << "joint" << i + 1;
  }
  EXPECT_TRUE(cmd->torque_limited);
}

// (6) 力矩变化率上限
TEST(CartesianImpedance, TorqueSlewLimit) {
  const double slew = 100.0;
  ci::Config c = makeConfig(ci::Vec6::Constant(5000.0), ci::Vec6::Ones());
  c.torque_slew = ci::Vec6::Constant(slew);
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Vector3d p_meas = p0 + Eigen::Vector3d(0.1, 0.0, 0.0);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(c));
  const auto s = makeSample(p_meas, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());
  ASSERT_TRUE(imp.setTarget({p0, q0}));

  const ci::Vec6 prev = ci::Vec6::Zero();
  auto cmd = imp.update(s, prev, kDt);
  ASSERT_TRUE(cmd.has_value());
  for (int i = 0; i < 6; ++i) {
    EXPECT_LE(std::abs(cmd->torque[i] - prev[i]), slew * kDt + 1e-9) << "joint" << i + 1;
  }
  EXPECT_TRUE(cmd->rate_limited);
}

// (7) 跟踪误差故障
TEST(CartesianImpedance, TrackingErrorFault) {
  ci::Config c = makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones());
  c.max_position_error = 0.02;
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(c));
  const auto s = makeSample(p0, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());

  const Eigen::Vector3d p_bad = p0 + Eigen::Vector3d(0.05, 0.0, 0.0);
  auto cmd = imp.update(makeSample(p_bad, q0, ci::Vec6::Zero(), ci::Vec6::Zero()),
                        ci::Vec6::Zero(), kDt);
  EXPECT_FALSE(cmd.has_value());
  EXPECT_EQ(imp.state(), ci::State::Fault);
  EXPECT_EQ(imp.error(), ci::Error::TrackingError);
}

// (8) 持续饱和故障
TEST(CartesianImpedance, SaturationTimeoutFault) {
  ci::Config c = makeConfig(ci::Vec6::Constant(5000.0), ci::Vec6::Ones());
  c.torque_limit = ci::Vec6::Constant(1.0);
  c.max_limited_time = 0.05;
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Vector3d p_meas = p0 + Eigen::Vector3d(0.2, 0.0, 0.0);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(c));
  const auto s = makeSample(p_meas, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());
  ASSERT_TRUE(imp.setTarget({p0, q0}));

  ci::Vec6 prev = ci::Vec6::Zero();
  bool faulted = false;
  for (int i = 0; i < 20; ++i) {
    auto cmd = imp.update(s, prev, kDt);
    if (!cmd.has_value()) { faulted = true; break; }
    prev = cmd->torque;
  }
  EXPECT_TRUE(faulted);
  EXPECT_EQ(imp.state(), ci::State::Fault);
  EXPECT_EQ(imp.error(), ci::Error::SaturationTimeout);
}

// (9) 非法输入
TEST(CartesianImpedance, RejectsBadInputs) {
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();

  {
    ci::CartesianImpedance imp;
    ASSERT_TRUE(imp.configure(makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones())));
    const auto s = makeSample(p0, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
    activateToActive(imp, s, ci::Vec6::Zero());
    EXPECT_FALSE(imp.update(s, ci::Vec6::Zero(), 0.0).has_value());
    EXPECT_EQ(imp.error(), ci::Error::InvalidDt);
  }
  {
    ci::CartesianImpedance imp;
    ASSERT_TRUE(imp.configure(makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones())));
    const auto s = makeSample(p0, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
    activateToActive(imp, s, ci::Vec6::Zero());
    auto bad = s;
    bad.valid = false;
    EXPECT_FALSE(imp.update(bad, ci::Vec6::Zero(), kDt).has_value());
    EXPECT_EQ(imp.error(), ci::Error::InvalidSample);
  }
  {
    ci::CartesianImpedance imp;
    ASSERT_TRUE(imp.configure(makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones())));
    const auto s = makeSample(p0, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
    activateToActive(imp, s, ci::Vec6::Zero());
    auto bad = s;
    bad.gravity[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(imp.update(bad, ci::Vec6::Zero(), kDt).has_value());
    // validSample() 先检查 gravity.allFinite()，所以这里报 InvalidSample
    EXPECT_EQ(imp.error(), ci::Error::InvalidSample);
  }
}

// (10) 姿态误差 -> 姿态力矩
TEST(CartesianImpedance, OrientationErrorProducesTorque) {
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q_meas(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones())));
  const auto s = makeSample(p0, q_meas, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());
  ASSERT_TRUE(imp.setTarget({p0, q_ref}));

  ci::Vec6 cur = ci::Vec6::Zero();
  for (int i = 0; i < 80; ++i) {
    auto c = imp.update(s, cur, kDt);
    ASSERT_TRUE(c.has_value());
    cur = c->torque;
  }
  // 实测绕 +z 转了 0.1 rad，恢复力矩沿 -z
  EXPECT_NEAR(cur[5], -100.0 * 0.1, 0.2) << "绕 z 的姿态恢复力矩";
  EXPECT_NEAR(cur[3], 0.0, 1e-5);
  EXPECT_NEAR(cur[4], 0.0, 1e-5);
}

// (11) 激活混合
TEST(CartesianImpedance, ActivationBlendsFromPreviousTorque) {
  ci::Config c = makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones());
  c.activation_time = 0.1;
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  const ci::Vec6 prev = ci::Vec6::Constant(5.0);
  const ci::Vec6 g = ci::Vec6::Constant(1.0);

  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(c));
  const auto s = makeSample(p0, q0, ci::Vec6::Zero(), g);
  ASSERT_TRUE(imp.activate(s, prev));
  auto first = imp.update(s, prev, kDt);
  ASSERT_TRUE(first.has_value());
  EXPECT_LT(first->activation_weight, 0.5) << "第一拍权重应很小";
  EXPECT_GT(first->torque[0], 4.0) << "应接近 previous 而不是 g";
  EXPECT_LT(first->torque[0], 5.0);

  ci::Vec6 cur = first->torque;
  for (int i = 0; i < 40; ++i) {
    auto c2 = imp.update(s, cur, kDt);
    ASSERT_TRUE(c2.has_value());
    cur = c2->torque;
  }
  EXPECT_NEAR(cur[0], g[0], 1e-6) << "混合结束后等于 desired = g";
  EXPECT_EQ(imp.state(), ci::State::Active);
}

// (12) deactivate 之后不再输出
TEST(CartesianImpedance, DeactivateStopsUpdates) {
  const Eigen::Vector3d p0(0.3, 0.1, 0.4);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  ci::CartesianImpedance imp;
  ASSERT_TRUE(imp.configure(makeConfig(ci::Vec6::Constant(100.0), ci::Vec6::Ones())));
  const auto s = makeSample(p0, q0, ci::Vec6::Zero(), ci::Vec6::Zero());
  activateToActive(imp, s, ci::Vec6::Zero());
  imp.deactivate();
  EXPECT_FALSE(imp.update(s, ci::Vec6::Zero(), kDt).has_value());
}
