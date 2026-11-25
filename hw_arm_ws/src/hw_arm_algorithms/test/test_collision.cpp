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
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
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
// 桌面约束（★ C02：按全部几何体算，不是只查 TCP）
// ══════════════════════════════════════════════════════════════════

// ★ C02 的判定性反例（审查报告给的真实位形）
//   TCP z = +0.01298（在桌面 +10 mm 之上 → 旧实现通过）
//   但 link5 最低点 z = −0.02048（钻到桌面下 20.5 mm）
TEST_F(CollisionTest, TableCatchesLinksBelowEvenWhenTcpIsAbove) {
  Eigen::VectorXd q(6);
  q << 0.8416794026, 2.8506534776, 2.3764412684,
       -1.5271100631, 1.0984799951, -5.2424507243;

  // 先把 TCP 高度验证一遍
  auto& data = ws_->pin();
  pinocchio::forwardKinematics(model_->pin(), data, q);
  pinocchio::updateFramePlacements(model_->pin(), data);
  pinocchio::FrameIndex fid = 0;
  for (size_t i = 0; i < model_->pin().frames.size(); ++i) {
    if (model_->pin().frames[i].name == "tool0") fid = static_cast<pinocchio::FrameIndex>(i);
  }
  const double tcp_z = data.oMf[fid].translation().z();
  EXPECT_GT(tcp_z, 0.010) << "这个反例的前提是 TCP 在桌面上方，实际 " << tcp_z;

  auto opt = TableConstraintOptions::withMountedBase(0.01);
  opt.offset = 0.0;
  const auto r = checkTable(*cm_, q, opt);
  ASSERT_FALSE(r.is_invalid()) << r.detail;
  EXPECT_TRUE(r.is_collision()) << "TCP 在上方但连杆穿桌面，必须检出";
  EXPECT_NE(r.detail.find("link"), std::string::npos) << r.detail;
}

// 底座"螺栓固定在桌面"：给它 0 余量就能通过，而不是忽略它
TEST_F(CollisionTest, MountedBaseUsesZeroMarginInsteadOfBeingIgnored) {
  auto opt = TableConstraintOptions::withMountedBase(0.01);
  opt.offset = 0.0;
  EXPECT_EQ(opt.link_margin.at("base_link"), 0.0);
  EXPECT_TRUE(opt.ignored_links.empty()) << "底座不应该被整个忽略";

  // home 位形：全部连杆（含底座）都在桌面上方
  const auto r = checkTable(*cm_, home(), opt);
  ASSERT_FALSE(r.is_invalid()) << r.detail;
  EXPECT_TRUE(r.is_free()) << r.detail;
}

// 旧实现的已知局限：home 位形在"全部几何体"口径下也是安全的
TEST_F(CollisionTest, HomePassesFullGeometryTableCheck) {
  auto opt = TableConstraintOptions::withMountedBase(0.01);
  const auto r = checkTable(*cm_, home(), opt);
  EXPECT_TRUE(r.is_free()) << r.detail;
  const double c = minTableClearance(*cm_, home(), opt);
  ASSERT_TRUE(std::isfinite(c));
  EXPECT_GT(c, 0.0);
}

// 把桌面抬高到不可能的高度 → 必须拒绝
TEST_F(CollisionTest, TableTooHighIsRejected) {
  auto opt = TableConstraintOptions::withMountedBase(0.01);
  opt.offset = 5.0;
  EXPECT_TRUE(checkTable(*cm_, home(), opt).is_collision());
}

// link_margin 覆盖生效：把 link5 的余量设成 10 m → 必碰
TEST_F(CollisionTest, PerLinkMarginOverrideWorks) {
  auto opt = TableConstraintOptions::withMountedBase(0.01);
  opt.link_margin["link5"] = 10.0;
  const auto r = checkTable(*cm_, home(), opt);
  EXPECT_TRUE(r.is_collision());
  EXPECT_NE(r.detail.find("link5"), std::string::npos) << r.detail;
}

// ★ C03：桌面检查的无效输入仍返回 Invalid
TEST_F(CollisionTest, TableInvalidInputs) {
  auto opt = TableConstraintOptions::withMountedBase(0.01);

  // q 维度
  Eigen::VectorXd bad_dim(5); bad_dim.setZero();
  EXPECT_TRUE(checkTable(*cm_, bad_dim, opt).is_invalid());

  // q 含 NaN
  Eigen::VectorXd q = home(); q[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checkTable(*cm_, q, opt).is_invalid());

  // margin 非有限 / 为负
  auto o2 = opt; o2.margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checkTable(*cm_, home(), o2).is_invalid());
  auto o3 = opt; o3.margin = -1.0;
  EXPECT_TRUE(checkTable(*cm_, home(), o3).is_invalid());

  // offset 非有限
  auto o4 = opt; o4.offset = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(checkTable(*cm_, home(), o4).is_invalid());

  // 法向非单位
  auto o5 = opt; o5.normal = Eigen::Vector3d(0, 0, 2);
  EXPECT_TRUE(checkTable(*cm_, home(), o5).is_invalid());

  // link_margin 非法
  auto o6 = opt; o6.link_margin["link3"] = -0.5;
  EXPECT_TRUE(checkTable(*cm_, home(), o6).is_invalid());

  // 全部忽略 ⇒ 什么都没查，必须 Invalid
  auto o7 = opt;
  for (int i = 0; i < cm_->numGeoms(); ++i) o7.ignored_links.push_back(cm_->geomLinkName(i));
  EXPECT_TRUE(checkTable(*cm_, home(), o7).is_invalid());
}

// 非法模型
TEST_F(CollisionTest, TableOnUnbuiltModelIsInvalid) {
  CollisionModel empty;
  auto opt = TableConstraintOptions::withMountedBase(0.01);
  EXPECT_TRUE(checkTable(empty, home(), opt).is_invalid());
  EXPECT_FALSE(std::isfinite(minTableClearance(empty, home(), opt)));
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
  auto topt = TableConstraintOptions::withMountedBase(0.005);

  SolutionFilterOptions fo;
  fo.collision_check = [&](const Eigen::VectorXd& q) {
    const auto sc = checkSelfCollision(*cm_, q);
    if (!sc.is_free()) return sc;
    return checkTable(*cm_, q, topt);
  };

  IKResult res;
  res.solutions = {sol(home())};   // 零位移
  const auto sel = selectSolution(*model_, *ws_, frame_, res, home(), fo);
  EXPECT_TRUE(sel.ok) << sel.message;
}

}  // namespace
