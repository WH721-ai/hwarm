// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 碰撞几何模型（从 URDF 建一次，缓存共享）
// ─────────────────────────────────────────────────────────────
// 为什么单独一个文件：
//   CollisionWorld（桌面）和 CollisionRobot（自碰撞）都要用同一份几何，
//   而"把 mesh 换成凸包"是几十毫秒的一次性开销，不能各自建一遍。
//
// 几何表示：凸包（不是胶囊、不是 Box）
//   实测本臂单胶囊过度包络 9.1 倍（link2 达 14 倍）→ 会让碰撞检测永远返回 true；
//   AABB Box 更糟（15 对全碰，100% 误报）；
//   凸包 74~342 顶点，精确且够快（见 docs/COLLISION_DESIGN.md）。
//
// ⚠️ 本机 coal 编译时**没有开 COAL_HAS_QHULL**，所以凸包数据由
//    tools/gen_collision_hulls.py 离线算好，写进 collision_hulls.hpp。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

class CollisionModel {
 public:
  CollisionModel() = default;
  ~CollisionModel();

  CollisionModel(const CollisionModel&) = delete;
  CollisionModel& operator=(const CollisionModel&) = delete;

  /// 从 URDF 建几何模型：
  ///   ① 读 URDF 的 collision 标签
  ///   ② 把每个 mesh 换成对应的凸包（数据来自 collision_hulls.hpp）
  ///   ③ 生成碰撞对，并排除"相邻连杆"（用关节父子关系自动推导）
  ///   ④ 校验：几何体数量、每个几何都有 hull、碰撞对非空
  bool build(const RobotModel& model, const std::string& urdf_path);

  /// 标记哪些 link 属于末端执行器（**精确名字匹配**，不是子串猜测）。
  /// build 之后调用；会影响哪几对碰撞对被视作"末端相关"。
  void setEndEffectorLinks(const std::vector<std::string>& link_names);

  /// 更新所有几何体的世界位姿（每个 q 必须调一次）
  void updatePlacements(const Eigen::VectorXd& q);

  pinocchio::GeometryModel& geom() { return geom_; }
  pinocchio::GeometryData& geomData() { return *data_; }

  int nq() const { return model_ ? model_->nq : 0; }
  int numPairs() const { return static_cast<int>(pair_names_.size()); }
  std::string pairName(int idx) const;
  const std::vector<std::string>& pairNames() const { return pair_names_; }

  /// 该碰撞对是否涉及末端执行器
  bool pairIsEndEffector(int idx) const;

  /// 模型指纹：用于在日志/结果里标明"这次检查用的是哪套几何"
  std::string fingerprint() const { return fingerprint_; }

  bool valid() const { return valid_; }

 private:
  pinocchio::GeometryModel geom_;
  std::unique_ptr<pinocchio::GeometryData> data_;
  std::unique_ptr<pinocchio::Data> model_data_;
  const pinocchio::Model* model_ = nullptr;

  std::vector<std::string> pair_names_;
  std::vector<int> pair_first_geo_;
  std::vector<int> pair_second_geo_;
  std::vector<std::string> geo_link_names_;   ///< 每个几何体所属 link 名（去掉 _N 后缀）
  std::vector<bool> pair_is_ee_;

  std::string fingerprint_;
  bool valid_ = false;
};

}  // namespace hw_arm_algorithms
