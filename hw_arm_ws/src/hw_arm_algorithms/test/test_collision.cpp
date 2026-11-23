// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 碰撞检测单元测试（按 docs/COLLISION_DESIGN.md §6.1 的 7 条）
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "hw_arm_algorithms/collision_model.hpp"
#include "hw_arm_algorithms/collision_robot.hpp"
#include "hw_arm_algorithms/collision_world.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

using namespace hw_arm_algorithms;

class CollisionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<RobotModel>();
    ASSERT_TRUE(model_->load(HW_ARM_DEFAULT_URDF));
    ws_ = std::make_unique<RobotWorkspace>(*model_);
    cm_ = std::make_unique<CollisionModel>();
    ASSERT_TRUE(cm_->build(*model_, HW_ARM_DEFAULT_URDF))
        << "CollisionModel 构建失败（凸包数据缺失？）";
  }

  Eigen::VectorXd home() const {
    Eigen::VectorXd q(6);
    q << 0.0, 0.8, 0.4, 0.0, 0.6, 0.0;
    return q;
  }

  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<RobotWorkspace> ws_;
  std::unique_ptr<CollisionModel> cm_;
};

// ① 碰撞对数量：不能是 0（Pinocchio 不会自动生成），也不该是 21（含相邻）
TEST_F(CollisionTest, PairCountAfterAdjacentRemoval) {
  EXPECT_GT(cm_->numPairs(), 0) << "没有碰撞对 → computeCollisions 会永远返回 false";
  EXPECT_LT(cm_->numPairs(), 21) << "相邻连杆没被排除干净";
  // 本臂 7 个几何体、链式结构 → 去相邻后应为 15
  EXPECT_EQ(cm_->numPairs(), 15);
}

// ② home 位形应当无碰撞
TEST_F(CollisionTest, HomeIsCollisionFree) {
  const auto hit = checkSelfCollision(*cm_, home());
  EXPECT_FALSE(hit.colliding) << "home 不该碰撞: " << hit.pair_name;
}

// ③ 折回去的位形必须能检出（joint2≈172°, joint3≈0°）
TEST_F(CollisionTest, FoldedBackCollides) {
  Eigen::VectorXd q(6);
  q << 0.0, 3.0, 0.0, 0.0, 0.6, 0.0;
  const auto hit = checkSelfCollision(*cm_, q);
  EXPECT_TRUE(hit.colliding) << "手臂折回撞基座，必须检出";
  EXPECT_NE(hit.pair_name.find("base_link"), std::string::npos)
      << "期望是 base_link 相关，实际: " << hit.pair_name;
}

// ④ 距离查询：home 的间隙**只有 3.5 mm**（实测），不是"很宽松"
//
// 实测（mesh 精确值）：home 位形下 link3 ↔ link5 只有 3.73 mm
// 凸包版 3.52 mm（凸包略外扩，符合预期）
// ⇒ **margin 不能设大**：> 3.5 mm 会让 home 自己报碰撞
TEST_F(CollisionTest, ClearanceAtHomeIsSmallButPositive) {
  const double d = minSelfDistance(*cm_, home());
  ASSERT_TRUE(std::isfinite(d));
  EXPECT_GT(d, 0.0) << "home 必须无穿透";
  EXPECT_GT(d, 0.002) << "间隙 " << d * 1000 << " mm 太小，模型可能有问题";
  EXPECT_LT(d, 0.010) << "实测 home 间隙约 3.5 mm，若变成 " << d * 1000
                      << " mm 说明碰撞体被换掉了";
}

// ⑤ margin 语义：给一个很大的 margin，home 也应被判为"碰"
TEST_F(CollisionTest, MarginIsRespected) {
  SelfCollisionOptions opt;
  opt.margin = 10.0;  // 10 m —— 荒谬的大，必然全部"碰"
  const auto hit = checkSelfCollision(*cm_, home(), opt);
  EXPECT_TRUE(hit.colliding);

  SelfCollisionOptions zero;
  zero.margin = 0.0;
  EXPECT_FALSE(checkSelfCollision(*cm_, home(), zero).colliding);
}

// ⑥ 桌面：桌面上方 20 cm 的位形应通过
TEST_F(CollisionTest, TablePassesAboveTable) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions opt;
  opt.z_table = 0.0;
  opt.margin = 0.01;
  const Eigen::VectorXd q = home();       // home 的 TCP 在 z≈0.33
  EXPECT_TRUE(checkTable(*model_, *ws_, q, key, opt));
}

// ⑦ 桌面：把桌面抬高到 TCP 之上，必须拒绝
TEST_F(CollisionTest, TableRejectsWhenTooLow) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions opt;
  opt.z_table = 0.0;
  opt.margin = 0.01;

  const Eigen::VectorXd q = home();          // home 的 TCP z = 0.334
  TableConstraintOptions high = opt;
  high.z_table = 0.34;                       // 0.334 < 0.34 + 0.01 → 必须拒绝
  EXPECT_FALSE(checkTable(*model_, *ws_, q, key, high));
}

}  // namespace

// ══════════════════════════════════════════════════════════════
// 集成测试：ik_selection 的碰撞过滤
// ══════════════════════════════════════════════════════════════
#include "hw_arm_algorithms/ik_selection.hpp"
#include "hw_arm_algorithms/kinematics.hpp"

namespace {

class SelectionCollisionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<RobotModel>();
    ASSERT_TRUE(model_->load(HW_ARM_DEFAULT_URDF));
    ws_ = std::make_unique<RobotWorkspace>(*model_);
    cm_ = std::make_unique<CollisionModel>();
    ASSERT_TRUE(cm_->build(*model_, HW_ARM_DEFAULT_URDF));
    for (size_t i = 0; i < model_->pin().frames.size(); ++i) {
      if (model_->pin().frames[i].name == "tool0") {
        frame_ = static_cast<pinocchio::FrameIndex>(i);
        break;
      }
    }
  }

  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<RobotWorkspace> ws_;
  std::unique_ptr<CollisionModel> cm_;
  pinocchio::FrameIndex frame_ = 0;
};

// 不带碰撞回调：行为和以前完全一致（不静默开启）
TEST_F(SelectionCollisionTest, NoCallbackMeansNoCheck) {
  SolutionFilterOptions fo;
  fo.require_collision_check = false;
  EXPECT_TRUE(static_cast<bool>(fo.collision_check) == false);
}

// 要求碰撞检查却没给回调 → 明确报错，而不是静默跳过
TEST_F(SelectionCollisionTest, RequireWithoutCallbackFails) {
  IKResult res;
  IKSolution s;
  s.q = (Eigen::VectorXd(6) << 0., 0.8, 0.4, 0., 0.6, 0.).finished();
  s.converged = true;
  res.solutions = {s};

  SolutionFilterOptions fo;
  fo.require_collision_check = true;   // 但没给 collision_check
  const auto sel = selectSolution(*model_, *ws_, frame_, res, s.q, fo);
  EXPECT_FALSE(sel.ok);
  EXPECT_NE(sel.message.find("碰撞"), std::string::npos);
}

// 给一个"永远碰撞"的回调 → 候选必须被拒
TEST_F(SelectionCollisionTest, AlwaysCollidingCallbackRejects) {
  IKResult res;
  IKSolution s;
  s.q = (Eigen::VectorXd(6) << 0., 0.8, 0.4, 0., 0.6, 0.).finished();
  s.converged = true;
  res.solutions = {s};

  SolutionFilterOptions fo;
  fo.collision_check = [](const Eigen::VectorXd&) { return true; };
  const auto sel = selectSolution(*model_, *ws_, frame_, res, s.q, fo);
  EXPECT_FALSE(sel.ok);
  ASSERT_EQ(sel.candidates.size(), 1u);
  EXPECT_TRUE(sel.candidates[0].path_collided || sel.candidates[0].collided);
}

// 用真实 CollisionModel 做回调：home 附近的小位移应当通过
TEST_F(SelectionCollisionTest, RealCollisionModelPassesHomeMotion) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions topt;
  topt.z_table = 0.0;
  topt.margin = 0.005;

  SolutionFilterOptions fo;
  fo.collision_check = [&](const Eigen::VectorXd& q) {
    if (checkSelfCollision(*cm_, q).colliding) return true;
    if (!checkTable(*model_, *ws_, q, key, topt)) return true;
    return false;
  };

  Eigen::VectorXd q_now(6);
  q_now << 0.0, 0.8, 0.4, 0.0, 0.6, 0.0;
  IKResult res;
  IKSolution s;
  s.q = q_now;                  // 零位移
  s.converged = true;
  res.solutions = {s};

  const auto sel = selectSolution(*model_, *ws_, frame_, res, q_now, fo);
  EXPECT_TRUE(sel.ok) << sel.message;
}

}  // namespace
