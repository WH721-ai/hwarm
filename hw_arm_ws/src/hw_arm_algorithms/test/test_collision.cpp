// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 碰撞检测单元测试
// 覆盖：设计文档 §6.1 的 7 条 + 审查报告 C03/C04 的回归用例
#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "hw_arm_algorithms/collision_model.hpp"
#include "hw_arm_algorithms/collision_robot.hpp"
#include "hw_arm_algorithms/collision_world.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/ik_selection.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
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
  /// 折回去撞基座（joint2≈172° + joint3≈0°）
  Eigen::VectorXd folded() const {
    Eigen::VectorXd q(6);
    q << 0.0, 3.0, 0.0, 0.0, 0.6, 0.0;
    return q;
  }
  /// 实测：4 对同时碰撞（base↔link4/5/6 与 link1↔link5），用于 C04 回归
  Eigen::VectorXd multiCollision() const {
    Eigen::VectorXd q(6);
    q << 3.0366, 3.0167, -0.0152, -3.08, 0.4336, -3.5435;
    return q;
  }

  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<RobotWorkspace> ws_;
  std::unique_ptr<CollisionModel> cm_;
};

// ══════════════════════════════════════════════════════════════════
// 几何模型
// ══════════════════════════════════════════════════════════════════

// ① 碰撞对数量：不能是 0（Pinocchio 不会自动生成），也不该是 21（含相邻）
TEST_F(CollisionTest, PairCountAfterAdjacentRemoval) {
  EXPECT_GT(cm_->numPairs(), 0) << "没有碰撞对 → 任何'无碰撞'都是假的";
  EXPECT_LT(cm_->numPairs(), 21) << "相邻连杆没被排除干净";
  EXPECT_EQ(cm_->numPairs(), 15);
  EXPECT_EQ(cm_->nq(), 6);
  EXPECT_FALSE(cm_->fingerprint().empty());
}

// ══════════════════════════════════════════════════════════════════
// 自碰撞：基本正确性
// ══════════════════════════════════════════════════════════════════

TEST_F(CollisionTest, HomeIsCollisionFree) {
  const auto r = checkSelfCollision(*cm_, home());
  ASSERT_FALSE(r.is_invalid()) << r.detail;
  EXPECT_TRUE(r.is_free()) << "home 不该碰撞: " << r.detail;
}

TEST_F(CollisionTest, FoldedBackCollides) {
  const auto r = checkSelfCollision(*cm_, folded());
  ASSERT_FALSE(r.is_invalid()) << r.detail;
  EXPECT_TRUE(r.is_collision()) << "手臂折回撞基座，必须检出";
  EXPECT_NE(r.detail.find("base_link"), std::string::npos)
      << "期望是 base_link 相关，实际: " << r.detail;
}

// ④ home 的间隙**只有 3.5 mm**（实测），不是"很宽松"
TEST_F(CollisionTest, ClearanceAtHomeIsSmallButPositive) {
  const double d = minSelfDistance(*cm_, home());
  ASSERT_TRUE(std::isfinite(d));
  EXPECT_GT(d, 0.002) << "间隙 " << d * 1000 << " mm 太小，模型可能有问题";
  EXPECT_LT(d, 0.010) << "实测 home 间隙约 3.5 mm，若变成 " << d * 1000
                      << " mm 说明碰撞体被换掉了";
}

TEST_F(CollisionTest, MarginIsRespected) {
  SelfCollisionOptions big;
  big.margin = 10.0;  // 10 m —— 荒谬的大，必然全部"碰"
  EXPECT_TRUE(checkSelfCollision(*cm_, home(), big).is_collision());

  SelfCollisionOptions zero;
  zero.margin = 0.0;
  EXPECT_TRUE(checkSelfCollision(*cm_, home(), zero).is_free());
}

// ══════════════════════════════════════════════════════════════════
// ★ C03 回归：无效输入必须返回 Invalid，**不能**返回"安全"
// ══════════════════════════════════════════════════════════════════

TEST_F(CollisionTest, UnbuiltModelIsInvalidNotFree) {
  CollisionModel empty;  // 没 build
  const auto r = checkSelfCollision(empty, home());
  EXPECT_TRUE(r.is_invalid()) << "未初始化的模型必须报 Invalid，实际 "
                              << toString(r.status);
  EXPECT_FALSE(static_cast<bool>(r)) << "Invalid 不能被当成通过";
  EXPECT_FALSE(std::isfinite(minSelfDistance(empty, home())));
}

TEST_F(CollisionTest, WrongDimensionIsInvalid) {
  Eigen::VectorXd q(5);
  q.setZero();
  const auto r = checkSelfCollision(*cm_, q);
  EXPECT_TRUE(r.is_invalid());
  EXPECT_NE(r.detail.find("维度"), std::string::npos) << r.detail;
}

TEST_F(CollisionTest, NaNConfigurationIsInvalid) {
  Eigen::VectorXd q = home();
  q[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checkSelfCollision(*cm_, q).is_invalid());

  Eigen::VectorXd q2 = home();
  q2[0] = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(checkSelfCollision(*cm_, q2).is_invalid());
}

TEST_F(CollisionTest, NonFiniteMarginIsInvalid) {
  SelfCollisionOptions opt;
  opt.margin = std::numeric_limits<double>::quiet_NaN();
  // 旧实现里 margin=NaN 会让 `d < margin` 恒假 → 永不拒绝 → 静默放行
  EXPECT_TRUE(checkSelfCollision(*cm_, home(), opt).is_invalid());

  opt.margin = -1.0;
  EXPECT_TRUE(checkSelfCollision(*cm_, home(), opt).is_invalid());
}

// ══════════════════════════════════════════════════════════════════
// ★ C04 回归：先确定有效对再逐对查询
// ══════════════════════════════════════════════════════════════════

// 把 base_link 标成末端执行器 → base 相关的 3 对都成了"EE 对"，
// 而 link1↔link5 那对不是。旧实现会在这里漏检。
TEST_F(CollisionTest, EndEffectorPairDoesNotHideLaterCollision) {
  cm_->setEndEffectorLinks({"base_link"});

  SelfCollisionOptions opt;
  opt.check_end_effector = false;   // 要求忽略 EE 对

  const auto r = checkSelfCollision(*cm_, multiCollision(), opt);
  ASSERT_FALSE(r.is_invalid()) << r.detail;
  EXPECT_TRUE(r.is_collision())
      << "EE 对先命中也必须继续查后面的真实碰撞（旧实现会漏）";
  EXPECT_NE(r.detail.find("link1"), std::string::npos)
      << "应报非 EE 的那对（link1↔link5），实际: " << r.detail;
}

// 结果不能依赖查询历史（旧实现可能读到上一次的 true）
TEST_F(CollisionTest, ResultsDoNotDependOnQueryHistory) {
  const auto a1 = checkSelfCollision(*cm_, folded());
  const auto b = checkSelfCollision(*cm_, home());
  const auto a2 = checkSelfCollision(*cm_, folded());

  EXPECT_TRUE(a1.is_collision());
  EXPECT_TRUE(b.is_free());
  EXPECT_TRUE(a2.is_collision()) << "第二次查同一碰撞位形必须仍然报碰撞";
  EXPECT_EQ(a1.detail, a2.detail);
}

// 打开 check_end_effector 时，EE 对也要查
TEST_F(CollisionTest, EndEffectorPairsCheckedWhenEnabled) {
  cm_->setEndEffectorLinks({"base_link"});
  SelfCollisionOptions opt;
  opt.check_end_effector = true;
  const auto r = checkSelfCollision(*cm_, folded(), opt);
  EXPECT_TRUE(r.is_collision());
  EXPECT_NE(r.detail.find("base_link"), std::string::npos);
}

// EE 标记必须用精确名字匹配（不是子串）
TEST_F(CollisionTest, EndEffectorMatchingIsExact) {
  cm_->setEndEffectorLinks({"link"});   // 不存在的 link 名
  for (int k = 0; k < cm_->numPairs(); ++k) {
    EXPECT_FALSE(cm_->pairIsEndEffector(k)) << "不应把 link1/link2 当成 'link'";
  }
  cm_->setEndEffectorLinks({"link5"});
  bool any = false;
  for (int k = 0; k < cm_->numPairs(); ++k) any = any || cm_->pairIsEndEffector(k);
  EXPECT_TRUE(any);
}

// ══════════════════════════════════════════════════════════════════
// 桌面约束
// ══════════════════════════════════════════════════════════════════

TEST_F(CollisionTest, TablePassesAboveTable) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions opt;
  opt.z_table = 0.0;
  opt.margin = 0.01;
  const auto r = checkTable(*model_, *ws_, home(), key, opt);
  ASSERT_FALSE(r.is_invalid()) << r.detail;
  EXPECT_TRUE(r.is_free());
}

TEST_F(CollisionTest, TableRejectsWhenTooLow) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions high;
  high.z_table = 0.34;   // home 的 TCP z = 0.334 < 0.34 + 0.01
  high.margin = 0.01;
  const auto r = checkTable(*model_, *ws_, home(), key, high);
  EXPECT_TRUE(r.is_collision());
}

// ★ C03：桌面检查的无效输入
TEST_F(CollisionTest, TableNullCallbackIsInvalid) {
  KeyPointFn none;
  const auto r = checkTable(*model_, *ws_, home(), none);
  EXPECT_TRUE(r.is_invalid()) << "空回调不能返回'通过'";
}

TEST_F(CollisionTest, TableMissingFrameIsInvalid) {
  auto key = makeTool0KeyPoints("no_such_frame");
  const auto r = checkTable(*model_, *ws_, home(), key);
  // 旧实现：找不到 frame 用 (0,0,0) 顶替，桌面在 z<0 时会放行
  EXPECT_TRUE(r.is_invalid()) << r.detail;
}

TEST_F(CollisionTest, TableWrongRowCountIsInvalid) {
  KeyPointFn bad = [](const RobotModel&, RobotWorkspace&, const Eigen::VectorXd&,
                      Eigen::MatrixXd& out) { out.resize(2, 1); out.setZero(); };
  EXPECT_TRUE(checkTable(*model_, *ws_, home(), bad).is_invalid());
}

TEST_F(CollisionTest, TableNaNParametersAreInvalid) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions opt;
  opt.margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checkTable(*model_, *ws_, home(), key, opt).is_invalid());

  TableConstraintOptions opt2;
  opt2.z_table = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checkTable(*model_, *ws_, home(), key, opt2).is_invalid());
}

TEST_F(CollisionTest, TableNaNKeyPointIsInvalid) {
  KeyPointFn nan_pt = [](const RobotModel&, RobotWorkspace&, const Eigen::VectorXd&,
                         Eigen::MatrixXd& out) {
    out.resize(3, 1);
    out.setConstant(std::numeric_limits<double>::quiet_NaN());
  };
  EXPECT_TRUE(checkTable(*model_, *ws_, home(), nan_pt).is_invalid());
}

// ══════════════════════════════════════════════════════════════════
// 与 ik_selection 的集成
// ══════════════════════════════════════════════════════════════════

class SelectionCollisionTest : public CollisionTest {
 protected:
  void SetUp() override {
    CollisionTest::SetUp();
    for (size_t i = 0; i < model_->pin().frames.size(); ++i) {
      if (model_->pin().frames[i].name == "tool0") {
        frame_ = static_cast<pinocchio::FrameIndex>(i);
        break;
      }
    }
  }
  IKSolution sol(const Eigen::VectorXd& q) {
    IKSolution s;
    s.q = q;
    s.converged = true;
    return s;
  }
  pinocchio::FrameIndex frame_ = 0;
};

TEST_F(SelectionCollisionTest, NoCallbackMeansNoCheck) {
  SolutionFilterOptions fo;
  fo.require_collision_check = false;
  EXPECT_FALSE(static_cast<bool>(fo.collision_check));
}

// 要求碰撞检查却没给回调 → 明确报错，而不是静默跳过
TEST_F(SelectionCollisionTest, RequireWithoutCallbackFails) {
  IKResult res;
  res.solutions = {sol(home())};
  SolutionFilterOptions fo;
  fo.require_collision_check = true;
  const auto sel = selectSolution(*model_, *ws_, frame_, res, home(), fo);
  EXPECT_FALSE(sel.ok);
  EXPECT_NE(sel.message.find("碰撞"), std::string::npos) << sel.message;
}

// 回调永远返回碰撞 → 候选被拒
TEST_F(SelectionCollisionTest, AlwaysCollidingCallbackRejects) {
  IKResult res;
  res.solutions = {sol(home())};
  SolutionFilterOptions fo;
  fo.collision_check = [](const Eigen::VectorXd&) {
    return CheckResult::collision(0, "test", 0.0);
  };
  const auto sel = selectSolution(*model_, *ws_, frame_, res, home(), fo);
  EXPECT_FALSE(sel.ok);
  ASSERT_EQ(sel.candidates.size(), 1u);
  EXPECT_TRUE(sel.candidates[0].path_collided || sel.candidates[0].collided);
}

// ★ C03：回调返回 Invalid ⇒ **整个选解失败**（不是只丢一个候选）
TEST_F(SelectionCollisionTest, InvalidCallbackFailsWholeSelection) {
  IKResult res;
  res.solutions = {sol(home())};
  SolutionFilterOptions fo;
  fo.collision_check = [](const Eigen::VectorXd&) {
    return CheckResult::invalid("模型没建好");
  };
  const auto sel = selectSolution(*model_, *ws_, frame_, res, home(), fo);
  EXPECT_FALSE(sel.ok);
  EXPECT_NE(sel.message.find("无效"), std::string::npos) << sel.message;
  EXPECT_NE(sel.message.find("模型没建好"), std::string::npos) << sel.message;
}

// 真实 CollisionModel 做回调：home 附近小位移通过
TEST_F(SelectionCollisionTest, RealCollisionModelPassesHomeMotion) {
  auto key = makeTool0KeyPoints("tool0");
  TableConstraintOptions topt;
  topt.z_table = 0.0;
  topt.margin = 0.005;

  SolutionFilterOptions fo;
  fo.collision_check = [&](const Eigen::VectorXd& q) {
    const auto sc = checkSelfCollision(*cm_, q);
    if (!sc.is_free()) return sc;
    return checkTable(*model_, *ws_, q, key, topt);
  };

  IKResult res;
  res.solutions = {sol(home())};   // 零位移
  const auto sel = selectSolution(*model_, *ws_, frame_, res, home(), fo);
  EXPECT_TRUE(sel.ok) << sel.message;
}

}  // namespace
