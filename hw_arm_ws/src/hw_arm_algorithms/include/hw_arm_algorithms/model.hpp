// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// RobotModel —— Pinocchio Model 的持有者（只读，建一次，可跨线程共享）
//
// Model / Data 分离（Pinocchio 的核心设计模式）：
//   · Model = 机器人结构：关节类型、父子关系、惯性参数、固定变换
//            加载后不再修改 → const 引用安全传递，多线程无需加锁
//   · Data  = 计算中间结果（见 data.hpp），每线程一份
//
// 本类额外提供**加载校验**：Pinocchio 本身不校验 URDF，写错了只会算出垃圾。
// 校验项见 LoadOptions。
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace hw_arm_algorithms {

/// 期望的模型特征，用于加载后校验
struct LoadOptions {
  int expected_dof = 6;
  std::vector<std::string> expected_joint_names{"joint1", "joint2", "joint3",
                                                "joint4", "joint5", "joint6"};
  /// 总质量期望值（kg）。设为 0 表示不检查。
  ///
  /// 注意 Pinocchio 的 computeTotalMass **不含固定在世界的根连杆**，
  /// 所以 base_link（1.372 kg）不计入。新结构（CAD 导入）：
  ///   base_link 1.372（固定） + link1..6 = 2.661 kg（可动）
  /// 旧结构是 4.268755（当时 base_link 没有质量）。
  double expected_total_mass = 2.661000;
  double mass_tolerance = 0.01;
  /// 末端帧名，留空表示不检查
  std::string ee_frame = "tool0";
  /// true = 校验失败则 load() 返回 false；false = 只记录，仍可使用
  bool strict = true;
};

/// 校验结果
struct ModelValidation {
  bool ok = true;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  void addError(const std::string& msg) {
    ok = false;
    errors.push_back(msg);
  }
  void addWarning(const std::string& msg) { warnings.push_back(msg); }
};

/// Model 封装：加载 + 校验 + 只读访问
class RobotModel {
 public:
  RobotModel() = default;

  /// 从 URDF 文件加载
  bool load(const std::string& urdf_path, const LoadOptions& opt = LoadOptions{});

  /// 从 URDF 字符串加载（xacro 展开后的内容）
  bool loadFromXML(const std::string& urdf_xml, const LoadOptions& opt = LoadOptions{});

  bool valid() const { return valid_; }
  const ModelValidation& validation() const { return validation_; }

  // ── Pinocchio 原始对象（只读）──
  const pinocchio::Model& pin() const { return model_; }

  // ── 维度 ──
  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  int njoints() const { return model_.njoints; }  // 含 universe
  double totalMass() const;

  // ── 关节名与索引 ──
  const std::vector<std::string>& jointNames() const { return joint_names_; }
  /// 返回关节在 Pinocchio 中的索引（1..njoints-1）；找不到返回 -1
  int jointIndex(const std::string& name) const;
  /// 返回关节在 q/v 向量中的起始下标
  int jointQIndex(int joint_index) const;
  int jointVIndex(int joint_index) const;

  // ── 帧（FK / IK 用）──
  bool hasFrame(const std::string& name) const;
  /// 找不到时返回 pinocchio::FrameIndex(-1)
  pinocchio::FrameIndex frameId(const std::string& name) const;

  // ── 限位 ──
  void jointLimits(int joint_index, double& lower, double& upper) const;

  // ── 重力 ──
  void setGravity(double gz);  ///< 默认 [0,0,-9.81]
  Eigen::Vector3d gravity() const { return model_.gravity.linear(); }

 private:
  ModelValidation validate(const LoadOptions& opt) const;

  pinocchio::Model model_;
  std::vector<std::string> joint_names_;  // 去掉 universe 后的顺序
  ModelValidation validation_;
  bool valid_ = false;
};

}  // namespace hw_arm_algorithms
