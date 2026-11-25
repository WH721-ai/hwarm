// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 环境约束（当前只有桌面）
// ─────────────────────────────────────────────────────────────
// 为什么桌面不用碰撞体：
//   实测 21% 的随机构型 TCP 会钻到桌面以下 —— 这是唯一的"真安全项"。
//   但桌面是**平面**，用不等式 z ≥ z_table + margin 就够了：
//       · 代价 2 µs（几个关键点的 FK） vs 碰撞体 70 µs（7 连杆 × 10 µs）
//       · 不会因为 Box 的角而误报
//   只有桌面出现夹具/孔/台阶时，才需要退回碰撞体方案。
//
// ⚠️ 当前关键点只有 tool0 的原点（见 makeTool0KeyPoints）。
//    审查报告 C02 已证明这**不够**：存在 TCP 在桌面上方 +12.98 mm、
//    但 link4/5/6 钻到桌面下 −20.5 mm 的位形。
//    改用"全部凸包顶点在 ±z 上的 support 最小值"是批次 2 的工作。
#pragma once

#include <functional>

#include <Eigen/Core>

#include "hw_arm_algorithms/collision_types.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

struct TableConstraintOptions {
  /// 桌面在 world 系的 z（本机 base_link 装在 z=+0.03，桌面在 z=0）
  double z_table = 0.0;
  /// 安全余量（m）
  double margin = 0.01;
};

/// 关键点回调：给定 q，填充 (3 × N) 的世界坐标矩阵（每列一个点）
///
/// 用回调注入，避免本模块依赖具体末端几何：
///   · 现在只有 TCP 一个点（C02 的已知局限）
///   · 夹爪进 URDF 后，加成 link6 下端 + 两个指尖
using KeyPointFn =
    std::function<void(const RobotModel&, RobotWorkspace&,
                       const Eigen::VectorXd&, Eigen::MatrixXd&)>;

/// 检查某位形下所有关键点是否都高于桌面
///
/// @return  kFree / kCollision / kInvalid
///          kInvalid 的情况：模型无效、q 维度不符/含 NaN、回调为空、
///          关键点矩阵行数 ≠ 3、关键点为 NaN、z_table/margin 非有限
CheckResult checkTable(const RobotModel& model, RobotWorkspace& ws,
                       const Eigen::VectorXd& q, const KeyPointFn& key_points,
                       const TableConstraintOptions& opt = {});

/// 内置的关键点：末端帧（默认 tool0）的中心
KeyPointFn makeTool0KeyPoints(const std::string& frame_name = "tool0");

}  // namespace hw_arm_algorithms
