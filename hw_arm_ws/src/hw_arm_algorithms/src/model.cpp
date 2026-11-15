// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/model.hpp"

#include <cmath>
#include <sstream>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace hw_arm_algorithms {

// ────────────────────────────────────────────────────────── 加载
bool RobotModel::load(const std::string& urdf_path, const LoadOptions& opt) {
  valid_ = false;
  validation_ = ModelValidation{};
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
  } catch (const std::exception& e) {
    validation_.addError(std::string("buildModel 失败: ") + e.what());
    return false;
  }
  joint_names_.clear();
  for (int i = 1; i < model_.njoints; ++i) joint_names_.push_back(model_.names[i]);

  validation_ = validate(opt);
  valid_ = validation_.ok || !opt.strict;
  return valid_;
}

bool RobotModel::loadFromXML(const std::string& urdf_xml, const LoadOptions& opt) {
  valid_ = false;
  validation_ = ModelValidation{};
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
  } catch (const std::exception& e) {
    validation_.addError(std::string("buildModelFromXML 失败: ") + e.what());
    return false;
  }
  joint_names_.clear();
  for (int i = 1; i < model_.njoints; ++i) joint_names_.push_back(model_.names[i]);

  validation_ = validate(opt);
  valid_ = validation_.ok || !opt.strict;
  return valid_;
}

// ────────────────────────────────────────────────────────── 校验
ModelValidation RobotModel::validate(const LoadOptions& opt) const {
  ModelValidation v;

  // ① 自由度
  if (opt.expected_dof > 0 && model_.nv != opt.expected_dof) {
    std::ostringstream os;
    os << "自由度不符: nv=" << model_.nv << "，期望 " << opt.expected_dof;
    v.addError(os.str());
  }
  if (model_.nq != model_.nv) {
    std::ostringstream os;
    os << "nq(" << model_.nq << ") != nv(" << model_.nv
       << ")，说明存在非转动关节（浮动基座？四元数？）";
    v.addError(os.str());
  }
  if (model_.njoints != model_.nv + 1) {
    std::ostringstream os;
    os << "njoints(" << model_.njoints << ") != nv+1(" << model_.nv + 1
       << ")，存在除 universe 外的复合关节";
    v.addWarning(os.str());
  }

  // ② 关节名与顺序（顺序错了后面全错，所以用 error）
  if (!opt.expected_joint_names.empty()) {
    if (joint_names_.size() != opt.expected_joint_names.size()) {
      std::ostringstream os;
      os << "关节数不符: 实际 " << joint_names_.size() << "，期望 "
         << opt.expected_joint_names.size();
      v.addError(os.str());
    } else {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i] != opt.expected_joint_names[i]) {
          std::ostringstream os;
          os << "关节顺序不符: 索引 " << i << " 是 '" << joint_names_[i]
             << "'，期望 '" << opt.expected_joint_names[i] << "'";
          v.addError(os.str());
        }
      }
    }
  }

  // ③ 总质量（防止 URDF 被改坏）
  if (opt.expected_total_mass > 0.0) {
    const double m = totalMass();
    if (std::abs(m - opt.expected_total_mass) > opt.mass_tolerance) {
      std::ostringstream os;
      os << "总质量不符: " << m << " kg，期望 " << opt.expected_total_mass
         << " ± " << opt.mass_tolerance;
      v.addError(os.str());
    }
  }

  // ④ 每个连杆的惯性必须物理合理
  for (int i = 1; i < model_.njoints; ++i) {
    const pinocchio::Inertia iner = model_.inertias[static_cast<size_t>(i)];
    if (iner.mass() <= 0.0) {
      std::ostringstream os;
      os << "连杆 " << model_.names[i] << " 质量非正: " << iner.mass();
      v.addError(os.str());
    }
    // 惯性张量正定（对角线为正）
    const pinocchio::Symmetric3 I = iner.inertia();   // 显式类型，避免重载解析歧义
    for (int k = 0; k < 3; ++k) {
      if (I(k, k) <= 0.0) {
        std::ostringstream os;
        os << "连杆 " << model_.names[i] << " 惯性张量对角元 " << k
           << " 非正: " << I(k, k);
        v.addError(os.str());
      }
    }
  }

  // ⑤ 末端帧
  if (!opt.ee_frame.empty() && !hasFrame(opt.ee_frame)) {
    v.addError("找不到末端帧 '" + opt.ee_frame + "'");
  }

  // ⑥ 重力方向提醒
  if (model_.gravity.linear().isZero(1e-9)) {
    v.addWarning("重力为零向量，g(q) 将恒为 0");
  }

  return v;
}

// ────────────────────────────────────────────────────────── 访问器
double RobotModel::totalMass() const {
  return pinocchio::computeTotalMass(model_);
}

int RobotModel::jointIndex(const std::string& name) const {
  for (int i = 1; i < model_.njoints; ++i) {
    if (model_.names[i] == name) return i;
  }
  return -1;
}

int RobotModel::jointQIndex(int joint_index) const {
  if (joint_index < 0 || joint_index >= model_.njoints) return -1;
  return model_.idx_qs[joint_index];
}

int RobotModel::jointVIndex(int joint_index) const {
  if (joint_index < 0 || joint_index >= model_.njoints) return -1;
  return model_.idx_vs[joint_index];
}

bool RobotModel::hasFrame(const std::string& name) const {
  return model_.existFrame(name);
}

pinocchio::FrameIndex RobotModel::frameId(const std::string& name) const {
  if (!model_.existFrame(name)) return model_.frames.size();
  return model_.getFrameId(name);
}

void RobotModel::jointLimits(int joint_index, double& lower, double& upper) const {
  const int iq = jointQIndex(joint_index);
  if (iq < 0 || iq >= model_.lowerPositionLimit.size()) {
    lower = -1e9;
    upper = 1e9;
    return;
  }
  lower = model_.lowerPositionLimit[iq];
  upper = model_.upperPositionLimit[iq];
}

void RobotModel::setGravity(double gz) {
  model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, gz);
  model_.gravity.angular() = Eigen::Vector3d::Zero();
}

}  // namespace hw_arm_algorithms
