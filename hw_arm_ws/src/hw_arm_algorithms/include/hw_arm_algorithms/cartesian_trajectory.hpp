// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 笛卡尔直线轨迹生成（RMRC）
// ─────────────────────────────────────────────────────────────
// 用途：给定起点关节角 q_start 和终点位姿 T_target，在**笛卡尔空间**
//       生成一条**直线**（姿态走最短的定轴旋转），并解算成关节路点序列。
//
// 与控制层的关系：
//   本模块只做"离线生成路点"，不做实时控制。
//   产出的 waypoints（t, q, qd）交给 joint_trajectory_controller 执行，
//   最后一公里仍然是电机的位置环 —— 与阻抗控制是**互补**的两条路。
//
// 算法（和 RMRC_Trajectory 相同的思路）：
//   ① 位置 p(t)：x/y/z 各一条五次多项式（同样边界、同样 T）
//      → 系数都正比于 (qf−q0)，所以 p(t)=p0+h·f(t) 走**精确直线**
//   ② 姿态 R(t)：取 T0⁻¹·Tf 的等效轴角 (axis, θf)，
//      让 θ 走一条五次多项式 → R(t)=R0·exp3(axis·θ(t))
//      角速度 ω = R0·axis·θ̇（世界系，与 LWA 雅可比一致）
//   ③ 每个积分步做一次 RMRC 阻尼最小二乘：
//         q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ (v_des + kp·e)
//      闭环用**当前积分到的 q**算 FK/雅可比和误差 → 不会开环漂移
//
// 与 trajectory.cpp 里 RMRC_Trajectory 的差异：
//   · 用 Pinocchio 的 SE3 / log3 / exp3 / integrate，不依赖自研 Matrixf
//   · 增加了限位钳制、速度限幅、奇异检测（原实现只有 DLS 兜底）
//   · 输出的是**离线路点数组**，不是每拍调用的一次 q_des
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/fwd.hpp>

#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

// 预分配工作缓冲（定义在 data.hpp，这里只需要引用）
class RobotWorkspace;

// ─────────────────────────────────────────────────────────────
// 五次多项式（标量）：q(t)=a0+a1·t+…+a5·t^5
// 由首末的位置/速度/加速度 6 个边界条件定系数
// ─────────────────────────────────────────────────────────────
struct Quintic {
  double a[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double T = 0.0;
  bool valid = false;

  /// @param v0/vf 首末速度  @param a0/af 首末加速度
  void setup(double q0, double qf, double T, double v0 = 0.0, double vf = 0.0,
             double a0 = 0.0, double af = 0.0);

  double pos(double t) const;
  double vel(double t) const;
  double acc(double t) const;
};

// ─────────────────────────────────────────────────────────────
// 规划参数
// ─────────────────────────────────────────────────────────────
struct CartesianPlanOptions {
  double duration = 3.0;      ///< 总时长 (s)，必须 > 0
  double sample_dt = 0.02;    ///< 输出路点间隔 (s)，建议 0.02~0.05
  double control_dt = 0.004;  ///< 内部 RMRC 积分步长 (s)，≤ sample_dt

  // ⚠️ 阻尼 λ 必须远小于 σ_min，否则会把"弱方向"一起压掉。
  //    实测本臂在普通位形下 σ_min≈2e-3，所以 λ 取 1e-3；
  //    取 1e-2 时 λ²=1e-4 ≫ σ_min²=4e-6，弱方向增益被压 25 倍，轨迹收敛不了。
  double damping = 1.0e-3;      ///< DLS 阻尼 λ（奇异附近的正则化）
  double feedback_gain = 5.0;   ///< 闭环增益 kp (1/s)，v_cmd = v_des + kp·e

  double max_joint_speed = 2.0;      ///< |q̇| 上限 (rad/s)
  // 判据用**相对**量 σ_min/σ_max，不用绝对 σ_min：
  //   本臂的 j2/j3/j5 三轴平行，σ_min/σ_max 在普通位形下只有约 0.004，
  //   用绝对阈值 （例如 1e-3）会把正常姿态误判成奇异。
  double min_singular_ratio = 2.0e-3; ///< σ_min/σ_max 低于此判为接近奇异
  double characteristic_length = 0.1; ///< 奇异性分析的特征长度 (m)，见 .cpp

  bool enforce_joint_limits = true;  ///< 钳制到 URDF 限位
  bool stop_on_singularity = true;   ///< 奇异时中止（否则继续但记录）
};

// ─────────────────────────────────────────────────────────────
// 输出
// ─────────────────────────────────────────────────────────────
struct CartesianWaypoint {
  double t = 0.0;            ///< 相对起始时刻 (s)
  Eigen::VectorXd q;         ///< 关节位置 (rad)
  Eigen::VectorXd qd;        ///< 关节速度 (rad/s)
};

struct CartesianPlanResult {
  std::vector<CartesianWaypoint> waypoints;

  bool ok = false;
  std::string error;  ///< ok=false 时的原因

  /// 终点与目标的残差（用 FK(q_last) 和 target 比）
  double final_position_error = 0.0;     ///< (m)
  double final_orientation_error = 0.0;  ///< (rad)

  bool limited = false;   ///< 过程中碰过关节限位
  bool singular = false;  ///< 过程中 σ_min 低于阈值
  double min_sigma_ratio = 0.0; ///< 全程最小 σ_min/σ_max（已按特征长度缩放）
};

// ─────────────────────────────────────────────────────────────
/// 生成从 q_start 到 target 的笛卡尔直线轨迹
///
/// @param frame  末端帧（必须是**真实作用点**，比如 tool0 或夹爪尖）
/// @param target 目标位姿（世界系）
/// @note 使用 LOCAL_WORLD_ALIGNED 雅可比；误差也在世界系表达，二者必须成套
CartesianPlanResult planCartesianLine(
    const RobotModel& model, RobotWorkspace& ws, pinocchio::FrameIndex frame,
    const Eigen::VectorXd& q_start, const pinocchio::SE3& target,
    const CartesianPlanOptions& options);

}  // namespace hw_arm_algorithms
