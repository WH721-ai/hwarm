// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_model.hpp"

#include <algorithm>
#include <iostream>
#include <map>
#include <set>

#include <coal/shape/convex.h>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "hw_arm_algorithms/collision_hulls.hpp"

namespace hw_arm_algorithms {
namespace {

/// 把 collision_hulls.hpp 里的数组包成 coal::Convex<Triangle>
///
/// coal 的 Convex 存的是"顶点数组 + 三角形索引"（和 BVHModel 同一套约定）。
/// ⚠️ coal::Triangle 是**索引三元组**（vids[3]），不是三个 3D 点；
///    用错构造函数会编译不过（我们踩过）。
std::shared_ptr<coal::CollisionGeometry> makeConvex(
    const collision_hulls::HullData& h) {
  auto pts = std::make_shared<std::vector<coal::Vec3s>>();
  pts->reserve(static_cast<size_t>(h.num_points));
  for (int i = 0; i < h.num_points; ++i) {
    pts->emplace_back(h.points[3 * i + 0], h.points[3 * i + 1], h.points[3 * i + 2]);
  }

  auto polys = std::make_shared<std::vector<coal::Triangle>>();
  polys->reserve(static_cast<size_t>(h.num_triangles));
  for (int i = 0; i < h.num_triangles; ++i) {
    polys->emplace_back(static_cast<unsigned>(h.triangles[3 * i + 0]),
                        static_cast<unsigned>(h.triangles[3 * i + 1]),
                        static_cast<unsigned>(h.triangles[3 * i + 2]));
  }

  auto* conv = new coal::Convex<coal::Triangle>(
      pts, static_cast<unsigned>(h.num_points), polys,
      static_cast<unsigned>(h.num_triangles));
  return std::shared_ptr<coal::CollisionGeometry>(conv);
}

/// 找到与某个 link 名对应的凸包数据；找不到返回 nullptr
const collision_hulls::HullData* findHull(const std::string& link_name) {
  for (int i = 0; i < collision_hulls::kNumHulls; ++i) {
    if (link_name == collision_hulls::kHulls[i].link) return &collision_hulls::kHulls[i];
  }
  return nullptr;
}

}  // namespace

CollisionModel::~CollisionModel() = default;

bool CollisionModel::build(const RobotModel& model, const std::string& urdf_path) {
  valid_ = false;
  pair_names_.clear();
  data_.reset();

  if (!model.valid()) return false;

  // ── ① 读 URDF 的 collision 标签 ──
  try {
    geom_ = pinocchio::GeometryModel();
    pinocchio::urdf::buildGeom(model.pin(), urdf_path,
                               pinocchio::COLLISION, geom_);
  } catch (const std::exception& e) {
    std::cerr << "[collision_model] buildGeom 失败: " << e.what() << "\n";
    return false;
  }

  // ── ② 每个 mesh → 凸包 ──
  for (auto& obj : geom_.geometryObjects) {
    // obj.name 形如 "base_link_0"，取 "_0" 前的前缀
    std::string base = obj.name;
    const auto pos = base.rfind('_');
    if (pos != std::string::npos) {
      const std::string tail = base.substr(pos + 1);
      if (!tail.empty() && std::all_of(tail.begin(), tail.end(), ::isdigit)) {
        base = base.substr(0, pos);
      }
    }
    const auto* h = findHull(base);
    if (h == nullptr) {
      std::cerr << "[collision_model] 没有 " << base
                << " 的凸包数据（先跑 tools/gen_collision_hulls.py）\n";
      return false;
    }
    obj.geometry = makeConvex(*h);
  }

  // ── ③ 碰撞对：全部 + 排除相邻 ──
  // ⚠️ Pinocchio 不会自动生成碰撞对（实测 collisionPairs 为 0）；
  //    不调 addAllCollisionPairs() 的话 computeCollisions 永远返回 false（假阴性）。
  geom_.addAllCollisionPairs();

  // 相邻判定：两个几何体的 parentJoint 相同、或直接是父子关节 → 相邻
  const auto& parents = model.pin().parents;   // parents[j] = 关节 j 的父关节
  std::vector<size_t> kept;
  for (size_t k = 0; k < geom_.collisionPairs.size(); ++k) {
    const auto& cp = geom_.collisionPairs[k];
    const auto& a = geom_.geometryObjects[cp.first];
    const auto& b = geom_.geometryObjects[cp.second];
    const auto ja = a.parentJoint;
    const auto jb = b.parentJoint;
    if (ja == jb) continue;                       // 同一连杆上的两个几何体
    const bool adjacent = (parents[ja] == jb) || (parents[jb] == ja);
    if (adjacent) continue;
    kept.push_back(k);
  }

  // 用"保留列表"重建碰撞对
  pinocchio::GeometryModel::CollisionPairVector pairs;
  pairs.reserve(kept.size());
  for (size_t k : kept) pairs.push_back(geom_.collisionPairs[k]);
  geom_.collisionPairs = pairs;

  pair_names_.reserve(pairs.size());
  for (const auto& cp : pairs) {
    pair_names_.push_back(geom_.geometryObjects[cp.first].name + " <-> " +
                          geom_.geometryObjects[cp.second].name);
  }

  data_ = std::make_unique<pinocchio::GeometryData>(geom_);
  model_data_ = std::make_unique<pinocchio::Data>(model.pin());
  model_ = &model.pin();
  valid_ = true;
  return true;
}

void CollisionModel::updatePlacements(const Eigen::VectorXd& q) {
  if (!valid_ || !data_ || !model_ || !model_data_) return;
  pinocchio::updateGeometryPlacements(*model_, *model_data_, geom_, *data_, q);
}

std::string CollisionModel::pairName(int idx) const {
  if (idx < 0 || idx >= static_cast<int>(pair_names_.size())) return "?";
  return pair_names_[static_cast<size_t>(idx)];
}

}  // namespace hw_arm_algorithms
