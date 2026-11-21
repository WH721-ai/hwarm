// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 笛卡尔直线轨迹（RMRC）单元测试
//   ① 五次多项式边界条件
//   ② 三轴独立插值走精确直线（数学性质）
//   ③ 规划结果：终点收敛、路点连续、关节限位内
//   ④ 直线性：路点 FK 与起终点连线偏差
//   ⑤ 姿态：定轴旋转，终点姿态匹配
//   ⑥ 非法输入被拒
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "hw_arm_algorithms/cartesian_trajectory.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

class CartTrajTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<hw_arm_algorithms::RobotModel>();
    ASSERT_TRUE(model_->load(HW_ARM_DEFAULT_URDF)) << "URDF 加载失败";
    ws_ = std::make_unique<hw_arm_algorithms::RobotWorkspace>(*model_);
    for (size_t i = 0; i < model_->pin().frames.size(); ++i) {
      if (model_->pin().frames[i].name == "tool0") {
        frame_ = static_cast<pinocchio::FrameIndex>(i);
        break;
      }
    }
  }

  pinocchio::SE3 fk(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model_->pin(), ws_->pin(), q);
    pinocchio::updateFramePlacement(model_->pin(), ws_->pin(), frame_);
    return ws_->pin().oMf[frame_];
  }

  std::unique_ptr<hw_arm_algorithms::RobotModel> model_;
  std::unique_ptr<hw_arm_algorithms::RobotWorkspace> ws_;
  pinocchio::FrameIndex frame_ = 0;
};

// ① 五次多项式：首末位置/速度/加速度都要满足边界条件
TEST_F(CartTrajTest, QuinticBoundaryConditions) {
  hw_arm_algorithms::Quintic q;
  q.setup(1.0, 3.0, 2.0);
  EXPECT_TRUE(q.valid);
  EXPECT_NEAR(q.pos(0.0), 1.0, 1e-12);
  EXPECT_NEAR(q.vel(0.0), 0.0, 1e-12);
  EXPECT_NEAR(q.acc(0.0), 0.0, 1e-12);
  EXPECT_NEAR(q.pos(2.0), 3.0, 1e-12);
  EXPECT_NEAR(q.vel(2.0), 0.0, 1e-12);
  EXPECT_NEAR(q.acc(2.0), 0.0, 1e-12);
  // 中点对称
  EXPECT_NEAR(q.pos(1.0), 2.0, 1e-12);
}

// ② 三轴独立五次多项式（相同边界）→ 参数化比例 s(t) 三轴相同 → 精确直线
TEST_F(CartTrajTest, QuinticTripleIsExactLine) {
  const Eigen::Vector3d p0(0.10, 0.05, 0.20);
  const Eigen::Vector3d pf(0.15, 0.02, 0.25);
  const double T = 2.0;
  hw_arm_algorithms::Quintic qx, qy, qz;
  qx.setup(p0.x(), pf.x(), T);
  qy.setup(p0.y(), pf.y(), T);
  qz.setup(p0.z(), pf.z(), T);

  for (int k = 0; k <= 20; ++k) {
    const double t = T * k / 20.0;
    const Eigen::Vector3d p(qx.pos(t), qy.pos(t), qz.pos(t));
    const double s = (p.x() - p0.x()) / (pf.x() - p0.x());
    const Eigen::Vector3d line = p0 + s * (pf - p0);
    EXPECT_LT((p - line).norm(), 1e-12) << "t=" << t << " 偏离直线";
  }
}

// ③ 规划成功 + 终点收敛 + 路点连续
TEST_F(CartTrajTest, PlanReachesTargetAndIsContinuous) {
  // ⚠️ 位形要选得"好"：本臂 j2/j3/j5 三轴平行，普通位形 σ_min/σ_max 只有
  //    约 0.004，靠近腕部奇异时会掉到 0.002 以下。这里用 0.06 的位形。
  Eigen::VectorXd q0(model_->nq());
  q0 << 0.0, 0.8, 0.4, 0.0, 0.6, 0.0;

  pinocchio::SE3 target = fk(q0);
  target.translation() += Eigen::Vector3d(0.010, 0.005, 0.0);  // 移动 ~11 mm

  hw_arm_algorithms::CartesianPlanOptions opt;
  opt.duration = 2.0;
  opt.sample_dt = 0.02;
  opt.control_dt = 0.004;

  const auto res = hw_arm_algorithms::planCartesianLine(*model_, *ws_, frame_,
                                                        q0, target, opt);
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_GT(res.waypoints.size(), 10u);
  EXPECT_LT(res.final_position_error, 1e-4);
  EXPECT_LT(res.final_orientation_error, 1e-4);
  EXPECT_FALSE(res.limited);

  // 起点必须就是 q0
  EXPECT_LT((res.waypoints.front().q - q0).norm(), 1e-12);

  // 连续性：相邻路点的关节跳变要小
  double max_jump = 0.0;
  for (size_t i = 1; i < res.waypoints.size(); ++i) {
    max_jump = std::max(max_jump,
                        (res.waypoints[i].q - res.waypoints[i - 1].q).norm());
  }
  EXPECT_LT(max_jump, 0.2) << "路点之间有跳变（max=" << max_jump << " rad）";
}

// ④ 直线性：路点 FK 与该直线段的偏差（这是 RMRC + 五次多项式的核心性质）
TEST_F(CartTrajTest, WaypointsLieOnStraightLine) {
  Eigen::VectorXd q0(model_->nq());
  q0 << 0.0, 0.8, 0.4, 0.0, 0.6, 0.0;   // 条件好的位形，见上

  const pinocchio::SE3 start = fk(q0);
  pinocchio::SE3 target = start;                    // 姿态不变，只平移
  target.translation() += Eigen::Vector3d(0.0, 0.010, 0.008);

  hw_arm_algorithms::CartesianPlanOptions opt;
  opt.duration = 2.0;
  opt.sample_dt = 0.02;
  opt.control_dt = 0.002;
  opt.feedback_gain = 10.0;  // 收紧一点，减小跟踪滞后

  const auto res = hw_arm_algorithms::planCartesianLine(*model_, *ws_, frame_,
                                                        q0, target, opt);
  ASSERT_TRUE(res.ok) << res.error;

  const Eigen::Vector3d p0 = start.translation();
  const Eigen::Vector3d pf = target.translation();
  const Eigen::Vector3d dir = pf - p0;

  double max_dev = 0.0;
  for (const auto& w : res.waypoints) {
    const Eigen::Vector3d p = fk(w.q).translation();
    const Eigen::Vector3d rel = p - p0;
    const Eigen::Vector3d perp = rel - rel.dot(dir) / dir.squaredNorm() * dir;
    max_dev = std::max(max_dev, perp.norm());
  }
  EXPECT_LT(max_dev, 1e-4) << "偏离直线 " << max_dev * 1000 << " mm";
}

// ⑤ 姿态不变时终点姿态应与起点一致
TEST_F(CartTrajTest, PureTranslationKeepsOrientation) {
  Eigen::VectorXd q0(model_->nq());
  q0 << 0.0, 0.8, 0.4, 0.0, 0.6, 0.0;

  const pinocchio::SE3 start = fk(q0);
  pinocchio::SE3 target = start;
  target.translation() += Eigen::Vector3d(0.0, 0.0, 0.010);

  hw_arm_algorithms::CartesianPlanOptions opt;
  opt.duration = 2.0;
  const auto res = hw_arm_algorithms::planCartesianLine(*model_, *ws_, frame_,
                                                        q0, target, opt);
  ASSERT_TRUE(res.ok) << res.error;
  const pinocchio::SE3 end = fk(res.waypoints.back().q);
  EXPECT_LT((end.rotation() - start.rotation()).norm(), 1e-3);
}

// ⑥ 非法输入被拒
TEST_F(CartTrajTest, RejectsInvalidInput) {
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model_->nq());
  const pinocchio::SE3 target = fk(q0);

  hw_arm_algorithms::CartesianPlanOptions opt;
  opt.duration = 0.0;  // 非法
  auto r1 = hw_arm_algorithms::planCartesianLine(*model_, *ws_, frame_, q0,
                                                 target, opt);
  EXPECT_FALSE(r1.ok);

  hw_arm_algorithms::CartesianPlanOptions good;
  Eigen::VectorXd bad_q = Eigen::VectorXd::Zero(model_->nq() + 1);
  auto r2 = hw_arm_algorithms::planCartesianLine(*model_, *ws_, frame_, bad_q,
                                                 target, good);
  EXPECT_FALSE(r2.ok);
}

}  // namespace
