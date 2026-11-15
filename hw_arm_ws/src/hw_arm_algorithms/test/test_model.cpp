// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// RobotModel 加载与校验的单元测试
//
// 这些断言的意义：模型加载错了，后面的 g(q)、辨识、补偿**全部是垃圾**，
// 而且不会报错——只会算出一个"看起来合理"的错值。所以必须在这里拦住。

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

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

class ModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    path_ = resolveUrdf();
    if (path_.empty() || !std::ifstream(path_).good()) {
      GTEST_SKIP() << "找不到 URDF（" << path_
                   << "）。用 HW_ARM_URDF=<path> 或 -DHW_ARM_URDF_XACRO=<xacro> 指定";
    }
    loaded_ = model_.load(path_);
  }

  std::string path_;
  hw_arm_algorithms::RobotModel model_;
  bool loaded_ = false;
};

}  // namespace

// ① 加载成功且校验通过
TEST_F(ModelTest, LoadsAndValidates) {
  ASSERT_TRUE(loaded_) << "模型加载失败";
  EXPECT_TRUE(model_.valid());
  for (const auto& e : model_.validation().errors) {
    ADD_FAILURE() << "校验错误: " << e;
  }
}

// ② 自由度必须是 6
TEST_F(ModelTest, DegreeOfFreedom) {
  EXPECT_EQ(model_.nv(), 6);
  EXPECT_EQ(model_.nq(), 6);
  EXPECT_EQ(model_.njoints(), 7);  // universe + 6
}

// ③ 关节顺序必须是 joint1..joint6
//    顺序错了 → q[i] 对应错关节 → 重力、辨识全错
TEST_F(ModelTest, JointOrder) {
  const auto& names = model_.jointNames();
  ASSERT_EQ(names.size(), 6u);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(names[static_cast<size_t>(i)], "joint" + std::to_string(i + 1))
        << "第 " << i << " 个关节顺序不对";
  }
}

// ④ 总质量应与 CAD 一致（防 URDF 被改坏）
//    新结构：computeTotalMass = 2.661 kg（不含固定的 base_link 1.372 kg）
//    旧结构曾是 4.268755
TEST_F(ModelTest, TotalMass) {
  EXPECT_NEAR(model_.totalMass(), 2.661, 0.01);
}

// ⑤ 每个连杆惯性必须物理合理
TEST_F(ModelTest, InertialsPhysical) {
  for (int i = 1; i < model_.njoints(); ++i) {
    const pinocchio::Inertia iner = model_.pin().inertias[static_cast<size_t>(i)];
    EXPECT_GT(iner.mass(), 0.0) << model_.pin().names[i];
    const pinocchio::Symmetric3 I = iner.inertia();
    for (int k = 0; k < 3; ++k) {
      EXPECT_GT(I(k, k), 0.0)
          << model_.pin().names[i] << " 惯性张量对角元 " << k;
    }
  }
}

// ⑥ 末端帧 tool0 必须存在（IK / FK 依赖它）
TEST_F(ModelTest, EndEffectorFrame) {
  EXPECT_TRUE(model_.hasFrame("tool0"));
  const auto id = model_.frameId("tool0");
  EXPECT_LT(static_cast<int>(id), model_.pin().nframes);
}

// ⑦ 重力方向默认 [0,0,-9.81]
TEST_F(ModelTest, DefaultGravity) {
  EXPECT_NEAR(model_.gravity().z(), -9.81, 1e-6);
  EXPECT_NEAR(model_.gravity().x(), 0.0, 1e-12);
  EXPECT_NEAR(model_.gravity().y(), 0.0, 1e-12);
}

// ⑧ 校验能抓住「关节名顺序被改」的情况
TEST_F(ModelTest, ValidationCatchesWrongJointOrder) {
  hw_arm_algorithms::LoadOptions opt;
  opt.expected_joint_names = {"joint2", "joint1", "joint3",
                              "joint4", "joint5", "joint6"};
  hw_arm_algorithms::RobotModel m;
  EXPECT_FALSE(m.load(path_, opt));
  EXPECT_FALSE(m.validation().errors.empty());
}

// ⑨ 校验能抓住「总质量不符」
TEST_F(ModelTest, ValidationCatchesWrongMass) {
  hw_arm_algorithms::LoadOptions opt;
  opt.expected_total_mass = 10.0;  // 明显错误
  hw_arm_algorithms::RobotModel m;
  EXPECT_FALSE(m.load(path_, opt));
}
