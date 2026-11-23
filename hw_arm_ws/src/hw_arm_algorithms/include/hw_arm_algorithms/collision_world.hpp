// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 环境约束（当前只有桌面）
// ─────────────────────────────────────────────────────────────
// 为什么桌面不用碰撞体：
//   实测 21% 的随机构型 TCP 会钻到桌面以下 —— 这是唯一的"真安全项"。
//   但桌面是**平面**，用不等式 z ≥ z_table + margin 就够了：
//       · 代价 2 µs（3~5 个关键点的 FK） vs 碰撞体 70 µs（7 连杆 × 10 µs）
//       · 不会因为 Box 的角而误报
//   只有桌面出现夹具/孔/台阶时，才需要退回碰撞体方案。
#pragma once

#include <functional>

#include <Eigen/Core>

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
///   · 现在只有 TCP 一个点
///   · 夹爪进 URDF 后，加成 link6 下端 + 两个指尖
using KeyPointFn =
    std::function<void(const RobotModel&, RobotWorkspace&,
                       const Eigen::VectorXd&, Eigen::MatrixXd&)>;

/// 检查某位形下所有关键点是否都高于桌面
bool checkTable(const RobotModel& model, RobotWorkspace& ws,
                const Eigen::VectorXd& q, const KeyPointFn& key_points,
                const TableConstraintOptions& opt = {});

/// 内置的关键点：末端帧（默认 tool0）的中心
KeyPointFn makeTool0KeyPoints(const std::string& frame_name = "tool0");

}  // namespace hw_arm_algorithms
