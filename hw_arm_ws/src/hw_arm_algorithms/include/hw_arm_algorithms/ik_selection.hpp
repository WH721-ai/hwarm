// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// IK 多解的**筛选**（选哪个解）
// ─────────────────────────────────────────────────────────────
// kinematics.cpp 负责"怎么解"（solveIKMultiSeed 枚举全部可行解），
// 本模块负责"选哪个解" —— 这是两个独立决策（M03_ik.md:7）。
//
// 两阶段：
//   阶段 1  硬过滤：任一条不满足就丢弃（reject）
//   阶段 2  加权打分：在通过的解里挑 score 最小的
//
// 硬过滤 8 条（前 2 条 kinematics.cpp 已做，这里复核）：
//   ① 收敛           残差 < position_tol / orientation_tol
//   ② 没被限位钳制   limited == false
//   ③ 单轴关节跳变   max_j |q_j − q_now,j| ≤ max_joint_jump
//   ④ 限位余量       min_j min(q_j−lo_j, hi_j−q_j) ≥ min_limit_margin
//   ⑤ 解的奇异度     σ_min/σ_max(q_sol) ≥ min_sigma_ratio
//   ⑥ 路径奇异度     q_now→q_sol 采样 path_samples 点，每点 σ 比值 ≥ 阈值
//                    （★ 只查终点会漏掉"终点好、中途穿奇异"的解）
//   ⑦ 峰值速度       1.5·|Δq_j|/T ≤ vmax_j
//   ⑧ 峰值加速度     6.0·|Δq_j|/T² ≤ amax_j
//
// 打分（越小越好，各项都归一化到 [0,1]）：
//   score = w_travel·(‖Δq‖/ref) + w_manip·(σmax/σ) + w_gravity·(max|g_j|/rated_j)
//         + w_margin·(1/margin_norm)
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/fwd.hpp>

#include "hw_arm_algorithms/collision_types.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

class RobotWorkspace;

// ─────────────────────────────────────────────────────────────
// 过滤参数
// ─────────────────────────────────────────────────────────────
struct SolutionFilterOptions {
  // ① 收敛复核
  double position_tol = 1e-4;      ///< m
  double orientation_tol = 1e-4;   ///< rad

  // ③ 关节跳变（相对 q_now）
  double max_joint_jump = 0.5;     ///< 单轴上限 (rad)，约 28.6°
  double max_joint_norm = 1.0;     ///< ‖Δq‖₂ 上限 (rad)

  // ④ 限位余量
  double min_limit_margin = 0.087; ///< rad（5°）

  // ⑤⑥ 奇异度
  double characteristic_length = 0.1;  ///< m，把角速度行缩放到与线速度同量纲
  double min_sigma_ratio = 2.0e-3;     ///< σ_min/σ_max 下限
  int path_samples = 20;               ///< 路径奇异检查的采样点数（含端点）

  // ⑦⑧ 速度 / 加速度（空 = 跳过该判据）
  Eigen::VectorXd max_joint_velocity;  ///< rad/s，size = nv
  Eigen::VectorXd max_joint_accel;     ///< rad/s²，size = nv
  double duration = 3.0;               ///< s，用于把 Δq 换算成峰值速度/加速度

  /// 评分用的额定力矩（空 = 不做重力项打分；size = nv）
  Eigen::VectorXd rated_torque;

  // ── 碰撞 / 桌面（可选，显式开启）─────────────────────────
  //
  // 用 std::function 注入，避免 ik_selection 依赖 collision_model
  //（分层原则：决策层不该知道几何怎么建的）。
  //
  // 回调返回 **true = 撞了 / 不可行**（与 reject 语义一致）。
  // 为空且 require_collision_check=true 时直接报错，而不是静默跳过
  //（R2 的教训：空向量静默关闭保护）。
  /// 返回 CheckResult：
  ///   kFree      → 放行
  ///   kCollision → 拒绝该候选
  ///   kInvalid   → **整个选解失败**（检查器本身不可信，不能只丢一个候选）
  std::function<CheckResult(const Eigen::VectorXd&)> collision_check;
  bool require_collision_check = false;

  /// 峰值系数：三次插值 1.5/6.0，五次插值 1.875/5.77
  /// JTC 默认 splines（端点 v=0）≈ 三次，取 1.5 / 6.0
  double velocity_peak_factor = 1.5;
  double accel_peak_factor = 6.0;
};

// ─────────────────────────────────────────────────────────────
// 打分权重
// ─────────────────────────────────────────────────────────────
struct SelectionWeights {
  double travel = 1.0;           ///< 关节空间行程越小越好
  double manipulability = 0.5;   ///< 越不奇异越好
  double gravity = 0.3;          ///< 重力负载越小越好（需要 rated_torque）
  double limit_margin = 0.3;     ///< 离限位越远越好
};

// ─────────────────────────────────────────────────────────────
// 单个解的评估结果
// ─────────────────────────────────────────────────────────────
struct CandidateEvaluation {
  int index = -1;                  ///< 在 IKResult::solutions 里的下标
  Eigen::VectorXd q;

  // 各项指标（硬过滤用）
  bool converged = false;
  bool limited = false;
  double position_error = 0.0;
  double orientation_error = 0.0;
  double travel_norm = 0.0;        ///< ‖q − q_now‖₂
  double max_joint_jump = 0.0;     ///< max_j |Δq_j|
  double min_limit_margin = 0.0;   ///< rad
  double sigma_ratio = 0.0;        ///< σ_min/σ_max（终点）
  double path_sigma_ratio = 0.0;   ///< 路径上的最小值
  double peak_velocity_ratio = 0.0;///< max_j (v_peak_j / vmax_j)
  double peak_accel_ratio = 0.0;   ///< max_j (a_peak_j / amax_j)
  double gravity_ratio = 0.0;      ///< max_j |g_j| / rated_j

  bool accepted = false;
  bool collided = false;       ///< 终点碰撞
  bool path_collided = false;  ///< 路径碰撞
  std::string reject_reason;
  double score = 0.0;
};

// ─────────────────────────────────────────────────────────────
// 筛选结果
// ─────────────────────────────────────────────────────────────
struct SelectionResult {
  bool ok = false;
  int index = -1;                 ///< 选中解在 IKResult::solutions 里的下标
  Eigen::VectorXd q;              ///< 选中解的关节角
  bool collided = false;          ///< 选中的解是否碰撞（正常流程下恒为 false）

  std::vector<CandidateEvaluation> candidates;  ///< 全部候选（含被拒的）
  int n_accepted = 0;

  std::string message;            ///< ok=false 时的汇总说明
};

// ─────────────────────────────────────────────────────────────
// ★ 一步到位：枚举 → 过滤 → 打分 → 返回**唯一最优解**
//
// 调用方（CLI / ROS 节点 / Python 胶水）不需要关心中间过程：
// 只给"目标位姿 + 当前关节角 + 参数"，拿回一个 q 和 ok。
// 全部计算都在 kinematics.cpp（solveIKMultiSeed）与本文件里完成。
// ─────────────────────────────────────────────────────────────
struct IKSelectionOptions {
  IKOptions ik;                        ///< 求解参数（种子数/迭代/阻尼/tol）
  SolutionFilterOptions filter;        ///< 硬过滤 + 速度加速度阈值
  SelectionWeights weights;            ///< 打分权重
  bool keep_diagnostics = false;       ///< 是否保留逐候选评估（调试用）
};

struct IKSelectionResult {
  bool ok = false;
  Eigen::VectorXd q;        ///< ★ 唯一最优解（ok=true 时有效）
  double score = 0.0;

  // ── 诊断摘要（都来自被选中的那个解）──
  int source_index = -1;    ///< 它在 IK 候选里的下标
  int n_candidates = 0;     ///< IK 枚举出几个解
  int n_accepted = 0;       ///< 通过硬过滤几个
  double position_error = 0.0;
  double orientation_error = 0.0;
  double travel_norm = 0.0;
  double max_joint_jump = 0.0;
  double min_limit_margin = 0.0;
  double sigma_ratio = 0.0;
  double path_sigma_ratio = 0.0;
  double peak_velocity_ratio = 0.0;
  double peak_accel_ratio = 0.0;
  double gravity_ratio = 0.0;

  std::string message;                      ///< 人读的一句话
  std::vector<CandidateEvaluation> candidates;  ///< 仅 keep_diagnostics=true 时填
};

/// 枚举全部可行解 → 硬过滤 → 加权打分 → 返回最优的那一个
///
/// @param target 目标位姿（世界系）
/// @param q_now  当前关节角：既作为 seed_hint（保证"最近解"在候选里），
///               也用于跳变 / 路径奇异 / 行程判定
IKSelectionResult selectBestIK(const RobotModel& model, RobotWorkspace& ws,
                               pinocchio::FrameIndex frame,
                               const pinocchio::SE3& target,
                               const Eigen::VectorXd& q_now,
                               const IKSelectionOptions& options = {});

// ─────────────────────────────────────────────────────────────
/// 低层入口：在**已有的**多解里挑一个（不给则自己枚举时用上面的）
///
/// @param candidates solveIKMultiSeed 的结果
/// @param q_now      当前关节角（用于跳变、路径奇异、行程）
SelectionResult selectSolution(const RobotModel& model, RobotWorkspace& ws,
                               pinocchio::FrameIndex frame,
                               const IKResult& candidates,
                               const Eigen::VectorXd& q_now,
                               const SolutionFilterOptions& filter = {},
                               const SelectionWeights& weights = {});

// ─────────────────────────────────────────────────────────────
// 小工具（也供 CLI / 测试直接用）
// ─────────────────────────────────────────────────────────────
/// σ_min/σ_max（角速度行按 characteristic_length 缩放）
double sigmaRatioAt(const RobotModel& model, RobotWorkspace& ws,
                    pinocchio::FrameIndex frame, const Eigen::VectorXd& q,
                    double characteristic_length);

/// 关节空间线段 q_a→q_b 上采样的最小 σ 比值
double pathSigmaRatio(const RobotModel& model, RobotWorkspace& ws,
                      pinocchio::FrameIndex frame, const Eigen::VectorXd& q_a,
                      const Eigen::VectorXd& q_b, double characteristic_length,
                      int samples);

/// 到关节限位的最小余量（rad）
double minLimitMargin(const RobotModel& model, const Eigen::VectorXd& q);

}  // namespace hw_arm_algorithms
