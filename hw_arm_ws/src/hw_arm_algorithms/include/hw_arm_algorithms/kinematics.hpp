// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 运动学：正运动学（FK）、雅可比（Jacobian）、逆运动学（IK）、奇异度
//
// 对应待办 C-17。方案取舍（见 algo/docs/90_附录/Appendix_C_待办清单.md）：
//
//   方案            解数   速度       需要标定   我们的可用性
//   ------------    ----  ---------  ---------  -------------------------
//   OPW（解析）       8    ~0.1 us    需要       结构符合，但要标 7+6+6 参数
//   ik_geo（解析）    全部  ~1-10 us   不需要     结构最匹配，但本地未安装
//   KDL（迭代）        1    ~18 us     不需要     角度不归一化，不推荐
//   Pinocchio DLS     1    ~100 us    不需要     ★ 本文件采用（已有依赖）
//
// 本文件实现的是**数值法（阻尼最小二乘，DLS）**，理由：
//   · 零额外依赖（只用 Pinocchio）
//   · 天然支持关节限位、奇异度、多解枚举（多起点）
//   · 对 6 轴球腕，多起点能得到多组解
//
// ⚠️ 数值法**可能漏解**（多起点 16 次实测能找到 1~4 组）。
//    若将来需要"保证不漏"，再上 ik_geo（解析、给全部解）。
#pragma once

#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

// ─────────────────────────────────────────────────────────────
// 雅可比参考系
// ─────────────────────────────────────────────────────────────
enum class JacobianFrame {
  LOCAL,                ///< 原点与朝向都在末端帧
  WORLD,                ///< 世界原点、世界朝向
  LOCAL_WORLD_ALIGNED,  ///< 原点在末端、朝向对齐世界（笛卡尔控制最常用）
  BODY,                 ///< 与 LOCAL 等价的别名
};

// ─────────────────────────────────────────────────────────────
// 正运动学
// ─────────────────────────────────────────────────────────────
/// 计算某帧在 q 下的位姿（世界系）
void computeFK(const RobotModel& model, RobotWorkspace& ws,
               const Eigen::VectorXd& q, pinocchio::FrameIndex frame,
               pinocchio::SE3& pose_out);

// ─────────────────────────────────────────────────────────────
// 雅可比
// ─────────────────────────────────────────────────────────────
/// 计算某帧的 6×nv 雅可比
///
/// @param J_out 输出，必须是 6×nv（内部写入，不 resize）
void computeJacobian(const RobotModel& model, RobotWorkspace& ws,
                     const Eigen::VectorXd& q, pinocchio::FrameIndex frame,
                     JacobianFrame frame_type, Eigen::MatrixXd& J_out);

// ─────────────────────────────────────────────────────────────
// 逆运动学
// ─────────────────────────────────────────────────────────────
/// IK 可调参数
struct IKOptions {
  /// 收敛阈值：平移（m）与旋转（rad）
  double position_tol = 1e-4;
  double orientation_tol = 1e-4;

  int max_iterations = 200;

  /// 阻尼系数 λ（正则化项是 λ²）。
  /// λ 太小 → 奇异处 Δq 爆炸；λ 太大 → 收敛慢。
  /// 实测 6 轴臂 λ=1e-2 比较稳。
  double damping = 1e-2;

  /// 步长（0~1）。1 = 全步；<1 更稳但更慢
  double step_size = 1.0;

  /// 是否把解钳制到关节限位内
  bool enforce_joint_limits = true;

  /// 多起点枚举时最多返回多少组解
  int max_solutions = 8;

  /// 多起点枚举的种子数量
  int num_seeds = 16;

  /// 随机种子（固定值 → 结果可复现）
  unsigned int random_seed = 42;

  /// 判定"两组解相同"的关节空间距离阈值（rad）
  double dedup_tolerance = 1e-3;
};

/// 一组 IK 解
struct IKSolution {
  Eigen::VectorXd q;              ///< 关节角（已钳制到限位内）
  double position_error = 0.0;    ///< 平移残差 (m)
  double orientation_error = 0.0; ///< 旋转残差 (rad)
  int iterations = 0;
  bool converged = false;
  bool limited = false;           ///< 是否被限位钳制过

  double cost() const { return position_error + orientation_error; }
};

/// IK 结果（可能多组解）
struct IKResult {
  std::vector<IKSolution> solutions;

  bool ok() const { return !solutions.empty(); }
  int n_solutions() const { return static_cast<int>(solutions.size()); }

  /// 取残差最小的解
  const IKSolution& best() const;
};

/// 单个种子的 IK（适合轨迹跟踪：种子给上一周期的解）
///
/// @return 解；`converged == false` 表示未收敛，调用方应拒绝使用
IKSolution solveIK(const RobotModel& model, RobotWorkspace& ws,
                   pinocchio::FrameIndex frame, const pinocchio::SE3& target,
                   const Eigen::VectorXd& seed,
                   const IKOptions& options = IKOptions{});

/// 多起点枚举 IK（适合"到达某位姿"：拿全部可行解再按代价挑选）
///
/// 种子在关节限位内均匀采样（固定随机种子 → 可复现）。
/// 只返回：限位内 + 残差 < tol 的解，并自动去重。
IKResult solveIKMultiSeed(const RobotModel& model, RobotWorkspace& ws,
                          pinocchio::FrameIndex frame,
                          const pinocchio::SE3& target,
                          const IKOptions& options = IKOptions{});

/// 从给定的一组种子逐个求解并去重
IKResult solveIKFromSeeds(const RobotModel& model, RobotWorkspace& ws,
                          pinocchio::FrameIndex frame,
                          const pinocchio::SE3& target,
                          const std::vector<Eigen::VectorXd>& seeds,
                          const IKOptions& options = IKOptions{});

// ─────────────────────────────────────────────────────────────
// 奇异度
// ─────────────────────────────────────────────────────────────
/// 由雅可比导出的奇异度指标
struct Manipulability {
  double sigma_min = 0.0;    ///< 最小奇异值（→0 表示接近奇异）
  double sigma_max = 0.0;
  double condition = 0.0;    ///< σ_max / σ_min
  double yoshikawa = 0.0;    ///< sqrt(det(J·Jᵀ))，可操作度
  bool is_singular = false;  ///< sigma_min < tolerance
};

/// 计算末端帧的奇异度
///
/// @param tolerance σ_min 低于该值判定为奇异（默认 1e-3）
Manipulability computeManipulability(const RobotModel& model,
                                     RobotWorkspace& ws,
                                     const Eigen::VectorXd& q,
                                     pinocchio::FrameIndex frame,
                                     double tolerance = 1e-3);

// ─────────────────────────────────────────────────────────────
// 工具
// ─────────────────────────────────────────────────────────────
/// 把位姿误差拆成（平移, 旋转）范数
void splitPoseError(const pinocchio::SE3& current, const pinocchio::SE3& target,
                    double& position_error, double& orientation_error);

/// 两组关节角在"圆环"上的最大差值（处理 ±π 回绕）
double jointDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

}  // namespace hw_arm_algorithms
