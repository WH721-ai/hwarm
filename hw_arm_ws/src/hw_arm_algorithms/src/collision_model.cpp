// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/collision_model.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <set>
#include <sstream>

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

const collision_hulls::HullData* findHull(const std::string& link_name) {
  for (int i = 0; i < collision_hulls::kNumHulls; ++i) {
    if (link_name == collision_hulls::kHulls[i].link) return &collision_hulls::kHulls[i];
  }
  return nullptr;
}

/// "link6_0" → "link6"；"base_link_0" → "base_link"
std::string stripSuffix(const std::string& name) {
  const auto pos = name.rfind('_');
  if (pos == std::string::npos) return name;
  const std::string tail = name.substr(pos + 1);
  if (tail.empty() || !std::all_of(tail.begin(), tail.end(), ::isdigit)) return name;
  return name.substr(0, pos);
}

}  // namespace

CollisionModel::~CollisionModel() = default;

bool CollisionModel::build(const RobotModel& model, const std::string& urdf_path) {
  valid_ = false;
  pair_names_.clear();
  pair_first_geo_.clear();
  pair_second_geo_.clear();
  geo_link_names_.clear();
  geom_vertices_.clear();
  has_last_q_ = false;
  pair_is_ee_.clear();
  fingerprint_.clear();
  data_.reset();

  if (!model.valid()) {
    std::cerr << "[collision_model] RobotModel 无效\n";
    return false;
  }

  // ── ① 读 URDF 的 collision 标签 ──
  try {
    geom_ = pinocchio::GeometryModel();
    pinocchio::urdf::buildGeom(model.pin(), urdf_path,
                               pinocchio::COLLISION, geom_);
  } catch (const std::exception& e) {
    std::cerr << "[collision_model] buildGeom 失败: " << e.what() << "\n";
    return false;
  }

  // ★ C03：空模型不能当"完整机器人"（URDF 没有 collision 标签时这里会是 0）
  if (geom_.geometryObjects.empty()) {
    std::cerr << "[collision_model] URDF 里没有 collision 几何体\n";
    return false;
  }

  // ── ② 每个 mesh → 凸包 ──
  geo_link_names_.reserve(geom_.geometryObjects.size());
  for (auto& obj : geom_.geometryObjects) {
    const std::string base = stripSuffix(obj.name);
    const auto* h = findHull(base);
    if (h == nullptr) {
      std::cerr << "[collision_model] 没有 " << base
                << " 的凸包数据（先跑 tools/gen_collision_hulls.py）\n";
      return false;
    }
    obj.geometry = makeConvex(*h);
    geo_link_names_.push_back(base);

    // 缓存顶点（几何体自身坐标系）：桌面检查要按顶点算最低点
    std::vector<Eigen::Vector3d> verts;
    verts.reserve(static_cast<size_t>(h->num_points));
    for (int i = 0; i < h->num_points; ++i) {
      verts.emplace_back(h->points[3 * i + 0], h->points[3 * i + 1], h->points[3 * i + 2]);
    }
    geom_vertices_.push_back(std::move(verts));
  }

  // ── ③ 碰撞对：全部 + 排除相邻 ──
  // ⚠️ Pinocchio 不会自动生成碰撞对（实测 collisionPairs 为 0）；
  //    不调 addAllCollisionPairs() 的话 computeCollisions 永远返回 false（假阴性）。
  geom_.addAllCollisionPairs();

  const auto& parents = model.pin().parents;
  pinocchio::GeometryModel::CollisionPairVector pairs;
  for (size_t k = 0; k < geom_.collisionPairs.size(); ++k) {
    const auto& cp = geom_.collisionPairs[k];
    const auto& a = geom_.geometryObjects[cp.first];
    const auto& b = geom_.geometryObjects[cp.second];
    if (a.parentJoint == b.parentJoint) continue;          // 同一连杆上的两个几何体
    if (parents[a.parentJoint] == b.parentJoint) continue;  // 直接父子
    if (parents[b.parentJoint] == a.parentJoint) continue;
    pairs.push_back(cp);
  }
  geom_.collisionPairs = pairs;

  // ★ C03：没有碰撞对 ⇒ 后续任何"无碰撞"都是假的
  if (pairs.empty()) {
    std::cerr << "[collision_model] 排除相邻后没有剩余碰撞对，模型不可用\n";
    return false;
  }

  for (const auto& cp : pairs) {
    pair_first_geo_.push_back(static_cast<int>(cp.first));
    pair_second_geo_.push_back(static_cast<int>(cp.second));
    pair_names_.push_back(geom_.geometryObjects[cp.first].name + " <-> " +
                          geom_.geometryObjects[cp.second].name);
  }
  pair_is_ee_.assign(pair_names_.size(), false);

  data_ = std::make_unique<pinocchio::GeometryData>(geom_);
  model_data_ = std::make_unique<pinocchio::Data>(model.pin());
  model_ = &model.pin();

  std::ostringstream fp;
  fp << "hulls=" << collision_hulls::kNumHulls
     << " geoms=" << geom_.geometryObjects.size()
     << " pairs=" << pair_names_.size()
     << " urdf=" << urdf_path;
  fingerprint_ = fp.str();

  valid_ = true;
  return true;
}

void CollisionModel::setEndEffectorLinks(const std::vector<std::string>& link_names) {
  const std::set<std::string> ee(link_names.begin(), link_names.end());
  for (size_t k = 0; k < pair_names_.size(); ++k) {
    const std::string& la = geo_link_names_[pair_first_geo_[k]];
    const std::string& lb = geo_link_names_[pair_second_geo_[k]];
    pair_is_ee_[k] = ee.count(la) > 0 || ee.count(lb) > 0;
  }
}

void CollisionModel::updatePlacements(const Eigen::VectorXd& q) {
  if (!valid_ || !data_ || !model_ || !model_data_) return;
  // 去重：自碰撞和桌面检查都会调，同一个 q 只做一次 FK
  if (has_last_q_ && last_q_.size() == q.size() && last_q_ == q) return;
  pinocchio::updateGeometryPlacements(*model_, *model_data_, geom_, *data_, q);
  last_q_ = q;
  has_last_q_ = true;
}

const std::vector<Eigen::Vector3d>* CollisionModel::geomVertices(int i) const {
  if (i < 0 || i >= static_cast<int>(geom_vertices_.size())) return nullptr;
  return &geom_vertices_[static_cast<size_t>(i)];
}

std::string CollisionModel::pairName(int idx) const {
  if (idx < 0 || idx >= static_cast<int>(pair_names_.size())) return "?";
  return pair_names_[static_cast<size_t>(idx)];
}

bool CollisionModel::pairIsEndEffector(int idx) const {
  if (idx < 0 || idx >= static_cast<int>(pair_is_ee_.size())) return false;
  return pair_is_ee_[static_cast<size_t>(idx)];
}

}  // namespace hw_arm_algorithms
