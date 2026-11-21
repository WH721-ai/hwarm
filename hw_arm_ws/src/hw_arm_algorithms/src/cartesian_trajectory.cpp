// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/cartesian_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "hw_arm_algorithms/data.hpp"

namespace hw_arm_algorithms {
namespace {

// 把 6 维 twist 的角速度分量按特征长度缩放，让奇异值量纲可比
//   线性部分单位 m/rad，角速度部分无量纲 —— 直接混在一起算 SVD，
//   条件数没有物理意义（方案文档 §7 的要求）。
//   缩放后：角速度 × L 的单位是 m/s，与线速度一致。
double scaledSigmaRatio(const Eigen::MatrixXd& J, double length) {
  Eigen::MatrixXd Js = J;
  Js.bottomRows(3) *= length;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Js);
  const auto& s = svd.singularValues();
  if (s.size() == 0) return 0.0;
  const double smax = s[0];
  const double smin = s[s.size() - 1];
  return (smax > 1e-12) ? (smin / smax) : 0.0;
}

// 位姿误差：[平移; 旋转矢量]，都在世界系（与 LWA 雅可比一致）
Eigen::Matrix<double, 6, 1> poseError(const pinocchio::SE3& current,
                                      const pinocchio::SE3& desired) {
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = desired.translation() - current.translation();
  // 世界系旋转误差：log3(R_d · R_cᵀ)
  const Eigen::Matrix3d R_err = desired.rotation() * current.rotation().transpose();
  e.tail<3>() = pinocchio::log3(R_err);
  return e;
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// Quintic
// ─────────────────────────────────────────────────────────────
void Quintic::setup(double q0, double qf, double T_in, double v0, double vf,
                    double a0, double af) {
  T = T_in;
  valid = (T > 0.0) && std::isfinite(q0) && std::isfinite(qf);
  if (!valid) {
    for (double& c : a) c = 0.0;
    a[0] = q0;
    return;
  }
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  a[0] = q0;
  a[1] = v0;
  a[2] = 0.5 * a0;
  a[3] = (20.0 * (qf - q0) - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) /
         (2.0 * T3);
  a[4] = (30.0 * (q0 - qf) + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) /
         (2.0 * T4);
  a[5] = (12.0 * (qf - q0) - 6.0 * (vf + v0) * T - (a0 - af) * T2) / (2.0 * T5);
}

double Quintic::pos(double t) const {
  return ((((a[5] * t + a[4]) * t + a[3]) * t + a[2]) * t + a[1]) * t + a[0];
}

double Quintic::vel(double t) const {
  return (((5.0 * a[5] * t + 4.0 * a[4]) * t + 3.0 * a[3]) * t + 2.0 * a[2]) * t + a[1];
}

double Quintic::acc(double t) const {
  return ((20.0 * a[5] * t + 12.0 * a[4]) * t + 6.0 * a[3]) * t + 2.0 * a[2];
}

// ─────────────────────────────────────────────────────────────
// 主线：笛卡尔直线 + RMRC
// ─────────────────────────────────────────────────────────────
CartesianPlanResult planCartesianLine(const RobotModel& model, RobotWorkspace& ws,
                                      pinocchio::FrameIndex frame,
                                      const Eigen::VectorXd& q_start,
                                      const pinocchio::SE3& target,
                                      const CartesianPlanOptions& opt) {
  CartesianPlanResult result;
  const int nv = model.nv();

  // ── 入参校验 ──────────────────────────────────────────────
  if (!model.valid()) {
    result.error = "模型未加载";
    return result;
  }
  if (q_start.size() != nv) {
    result.error = "q_start 维度不符";
    return result;
  }
  if (!q_start.allFinite()) {
    result.error = "q_start 含 NaN/Inf";
    return result;
  }
  if (frame >= model.pin().frames.size()) {
    result.error = "frame 索引非法";
    return result;
  }
  if (!(opt.duration > 0.0) || !(opt.sample_dt > 0.0) || !(opt.control_dt > 0.0)) {
    result.error = "duration/sample_dt/control_dt 必须为正";
    return result;
  }
  if (opt.control_dt > opt.sample_dt) {
    result.error = "control_dt 不能大于 sample_dt";
    return result;
  }
  if (!target.translation().allFinite() ||
      !target.rotation().allFinite()) {
    result.error = "目标位姿含 NaN/Inf";
    return result;
  }

  auto& data = ws.pin();
  const auto& pin_model = model.pin();

  // ── ① 起始位姿 ────────────────────────────────────────────
  pinocchio::forwardKinematics(pin_model, data, q_start);
  pinocchio::updateFramePlacement(pin_model, data, frame);
  const pinocchio::SE3 T0 = data.oMf[frame];

  // ── ② 位置：三条五次多项式（同样边界 → 精确直线）──────────
  Quintic qx, qy, qz;
  qx.setup(T0.translation().x(), target.translation().x(), opt.duration);
  qy.setup(T0.translation().y(), target.translation().y(), opt.duration);
  qz.setup(T0.translation().z(), target.translation().z(), opt.duration);

  // ── ③ 姿态：定轴旋转 ──────────────────────────────────────
  //   R_rel = R0ᵀ·Rf 在**末端局部系**；用 log3 取等效轴角
  const Eigen::Matrix3d R0 = T0.rotation();
  const Eigen::Matrix3d R_rel = R0.transpose() * target.rotation();
  const Eigen::Vector3d rot_vec = pinocchio::log3(R_rel);
  const double theta_f = rot_vec.norm();
  Eigen::Vector3d axis_local = Eigen::Vector3d::Zero();
  if (theta_f > 1e-6) axis_local = rot_vec / theta_f;

  Quintic q_theta;
  q_theta.setup(0.0, theta_f, opt.duration);

  // ── ④ RMRC 积分 ──────────────────────────────────────────
  Eigen::VectorXd q = q_start;
  Eigen::VectorXd q_prev = q_start;
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(6, nv);   // 6×nv
  Eigen::Matrix<double, 6, 6> JJt;

  const int n_ctrl = static_cast<int>(std::lround(opt.duration / opt.control_dt)) + 1;
  const int stride = std::max(
      1, static_cast<int>(std::lround(opt.sample_dt / opt.control_dt)));

  result.waypoints.reserve(static_cast<size_t>(n_ctrl / stride) + 2);
  result.min_sigma_ratio = std::numeric_limits<double>::infinity();

  // t=0 的路点：就是起点
  result.waypoints.push_back(
      CartesianWaypoint{0.0, q_start, Eigen::VectorXd::Zero(nv)});

  const Eigen::VectorXd& jlo = pin_model.lowerPositionLimit;
  const Eigen::VectorXd& jhi = pin_model.upperPositionLimit;

  for (int k = 1; k <= n_ctrl; ++k) {
    const double t = k * opt.control_dt;
    const double tc = std::min(t, opt.duration);

    // ── 期望位姿与期望 twist ───────────────────────────────
    Eigen::Vector3d p_des(qx.pos(tc), qy.pos(tc), qz.pos(tc));
    Eigen::Vector3d v_des(qx.vel(tc), qy.vel(tc), qz.vel(tc));

    const double th = q_theta.pos(tc);
    const double th_dot = q_theta.vel(tc);
    const Eigen::Matrix3d R_des = R0 * pinocchio::exp3(axis_local * th);
    // 世界系角速度 = R0 · axis_local · θ̇
    const Eigen::Vector3d w_des = R0 * (axis_local * th_dot);

    // ── 当前位姿（用积分到的 q，闭环）────────────────────
    pinocchio::forwardKinematics(pin_model, data, q);
    pinocchio::updateFramePlacement(pin_model, data, frame);
    const pinocchio::SE3 T_act = data.oMf[frame];

    // ── 雅可比（LWA）+ 奇异值检查 ──────────────────────────
    pinocchio::computeJointJacobians(pin_model, data, q);
    pinocchio::updateFramePlacement(pin_model, data, frame);
    pinocchio::getFrameJacobian(pin_model, data, frame,
                                pinocchio::LOCAL_WORLD_ALIGNED, jac);

    const double sigma_ratio = scaledSigmaRatio(jac, opt.characteristic_length);
    result.min_sigma_ratio =
        std::min(result.min_sigma_ratio, sigma_ratio);
    if (sigma_ratio < opt.min_singular_ratio) {
      result.singular = true;
      if (opt.stop_on_singularity) {
        result.error = "接近奇异位形（σ_min/σ_max=" + std::to_string(sigma_ratio) +
                       " < " + std::to_string(opt.min_singular_ratio) + "）";
        result.failure_q = q;
        return result;
      }
    }

    // ── 误差 + 期望关节速度 ───────────────────────────────
    const Eigen::Matrix<double, 6, 1> e = poseError(T_act, pinocchio::SE3(R_des, p_des));
    Eigen::Matrix<double, 6, 1> v_cmd;
    v_cmd.head<3>() = v_des + opt.feedback_gain * e.head<3>();
    v_cmd.tail<3>() = w_des + opt.feedback_gain * e.tail<3>();

    // q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ v_cmd
    JJt = jac * jac.transpose();
    JJt.diagonal().array() += opt.damping * opt.damping;
    Eigen::Matrix<double, 6, 1> y = JJt.ldlt().solve(v_cmd);
    Eigen::VectorXd qd = jac.transpose() * y;

    // 速度限幅
    const double vmax = qd.cwiseAbs().maxCoeff();
    if (vmax > opt.max_joint_speed && vmax > 0.0) {
      qd *= opt.max_joint_speed / vmax;
    }

    // ── 积分 + 限位钳制 ───────────────────────────────────
    q = pinocchio::integrate(pin_model, q, qd * opt.control_dt);
    if (opt.enforce_joint_limits) {
      for (int i = 0; i < nv; ++i) {
        if (q[i] < jlo[i]) {
          q[i] = jlo[i];
          result.limited = true;
        } else if (q[i] > jhi[i]) {
          q[i] = jhi[i];
          result.limited = true;
        }
      }
    }

    // ── 按 sample_dt 采样输出 ─────────────────────────────
    if (k % stride == 0 || k == n_ctrl) {
      Eigen::VectorXd qd_out = (q - q_prev) / (stride * opt.control_dt);
      result.waypoints.push_back(
          CartesianWaypoint{static_cast<double>(k) * opt.control_dt, q, qd_out});
      q_prev = q;
    }
  }

  // ── ⑤ 终点验收 ───────────────────────────────────────────
  pinocchio::forwardKinematics(pin_model, data, q);
  pinocchio::updateFramePlacement(pin_model, data, frame);
  const pinocchio::SE3 T_end = data.oMf[frame];
  result.final_position_error = (target.translation() - T_end.translation()).norm();
  result.final_orientation_error =
      pinocchio::log3(target.rotation() * T_end.rotation().transpose()).norm();

  if (result.limited) {
    result.error = "过程中碰到关节限位";
    result.failure_q = q;
    return result;  // ok 保持 false：限位钳制后轨迹已偏离直线
  }
  if (result.final_position_error > 1e-3 || result.final_orientation_error > 1e-3) {
    result.error = "终点未收敛（位置残差) " +
                   std::to_string(result.final_position_error) + " m";
    result.failure_q = q;
    return result;
  }

  result.ok = true;
  return result;
}

}  // namespace hw_arm_algorithms
