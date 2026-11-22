// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/ik_selection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/SVD>
#include <pinocchio/algorithm/kinematics.hpp>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"

namespace hw_arm_algorithms {

// ─────────────────────────────────────────────────────────────
// 小工具
// ─────────────────────────────────────────────────────────────
double sigmaRatioAt(const RobotModel& model, RobotWorkspace& ws,
                    pinocchio::FrameIndex frame, const Eigen::VectorXd& q,
                    double characteristic_length) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv());
  computeJacobian(model, ws, q, frame, JacobianFrame::LOCAL_WORLD_ALIGNED, J);

  // 线速度行单位 m/rad，角速度行无量纲 —— 按特征长度缩放后才同量纲可比
  Eigen::MatrixXd Js = J;
  Js.bottomRows(3) *= characteristic_length;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Js);
  const auto& s = svd.singularValues();
  if (s.size() == 0 || s[0] <= 1e-12) return 0.0;
  return s[s.size() - 1] / s[0];
}

double pathSigmaRatio(const RobotModel& model, RobotWorkspace& ws,
                      pinocchio::FrameIndex frame, const Eigen::VectorXd& q_a,
                      const Eigen::VectorXd& q_b, double characteristic_length,
                      int samples) {
  const int n = std::max(2, samples);
  double worst = std::numeric_limits<double>::infinity();
  for (int k = 0; k < n; ++k) {
    const double s = static_cast<double>(k) / (n - 1);
    // 用 Pinocchio 的流形插值，等价于关节空间直线段
    const Eigen::VectorXd q = pinocchio::interpolate(model.pin(), q_a, q_b, s);
    const double r = sigmaRatioAt(model, ws, frame, q, characteristic_length);
    worst = std::min(worst, r);
    if (worst <= 0.0) break;
  }
  return std::isfinite(worst) ? worst : 0.0;
}

double minLimitMargin(const RobotModel& model, const Eigen::VectorXd& q) {
  const Eigen::VectorXd& lo = model.pin().lowerPositionLimit;
  const Eigen::VectorXd& hi = model.pin().upperPositionLimit;
  double m = std::numeric_limits<double>::infinity();
  for (int i = 0; i < q.size(); ++i) {
    const double lower = q[i] - lo[i];
    const double upper = hi[i] - q[i];
    m = std::min(m, std::min(lower, upper));
  }
  return std::isfinite(m) ? m : 0.0;
}

namespace {

// 同一个物理姿态可以写成 q + k·2π。
//   · joint1(±π)、joint6(±2π) 这类行程 ≥ 2π 的轴，IK 可能给出"多绕了一圈"的表示
//   · 直接按数值差算跳变，会把 0.08 rad 的小动作误判成 6.2 rad 的大跳变
// 这里对每根轴尝试 q + k·2π（k = −2..2），挑**在限位内且离 q_ref 最近**的那个。
Eigen::VectorXd wrapToward(const pinocchio::Model& model,
                           const Eigen::VectorXd& q,
                           const Eigen::VectorXd& q_ref) {
  const Eigen::VectorXd& lo = model.lowerPositionLimit;
  const Eigen::VectorXd& hi = model.upperPositionLimit;
  Eigen::VectorXd out = q;
  for (int i = 0; i < q.size(); ++i) {
    double best = q[i];
    double best_d = std::abs(q[i] - q_ref[i]);
    for (int k = -2; k <= 2; ++k) {
      if (k == 0) continue;
      const double cand = q[i] + k * 2.0 * M_PI;
      if (cand < lo[i] - 1e-9 || cand > hi[i] + 1e-9) continue;
      const double d = std::abs(cand - q_ref[i]);
      if (d < best_d) {
        best_d = d;
        best = cand;
      }
    }
    out[i] = best;
  }
  return out;
}

// 把 "value ≤ limit" 的判据统一成一句话，方便汇总
std::string ratioText(const char* name, double value, double limit) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%s 超限：%.6g > %.6g", name, value, limit);
  return std::string(buf);
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// 主函数：硬过滤 + 加权打分
// ─────────────────────────────────────────────────────────────
SelectionResult selectSolution(const RobotModel& model, RobotWorkspace& ws,
                               pinocchio::FrameIndex frame,
                               const IKResult& candidates,
                               const Eigen::VectorXd& q_now,
                               const SolutionFilterOptions& filter,
                               const SelectionWeights& weights) {
  SelectionResult out;
  const int nv = model.nv();

  if (!model.valid()) {
    out.message = "模型未加载";
    return out;
  }
  if (q_now.size() != nv) {
    out.message = "q_now 维度不符";
    return out;
  }
  if (candidates.solutions.empty()) {
    out.message = "没有候选解（IK 未收敛或全部超限位）";
    return out;
  }

  const bool has_vmax = filter.max_joint_velocity.size() == nv;
  const bool has_amax = filter.max_joint_accel.size() == nv;
  const bool has_rated = filter.rated_torque.size() == nv;
  const double T = (filter.duration > 0.0) ? filter.duration : 1.0;

  out.candidates.reserve(candidates.solutions.size());

  // ── 阶段 1：硬过滤 ────────────────────────────────────────
  for (size_t i = 0; i < candidates.solutions.size(); ++i) {
    const IKSolution& sol = candidates.solutions[i];
    CandidateEvaluation ev;
    ev.index = static_cast<int>(i);
    // ★ 先绕圈展开到离 q_now 最近的等价表示，避免把"多绕 2π"误判成大跳变
    ev.q = wrapToward(model.pin(), sol.q, q_now);
    ev.converged = sol.converged;
    ev.limited = sol.limited;
    ev.position_error = sol.position_error;
    ev.orientation_error = sol.orientation_error;

    auto reject = [&ev](std::string why) {
      ev.accepted = false;
      ev.reject_reason = std::move(why);
    };

    // ① 收敛
    if (!ev.converged) {
      reject("IK 未收敛");
      out.candidates.push_back(ev);
      continue;
    }
    if (ev.position_error > filter.position_tol ||
        ev.orientation_error > filter.orientation_tol) {
      reject("残差超阈值");
      out.candidates.push_back(ev);
      continue;
    }

    // ② 没被限位钳制
    if (ev.limited) {
      reject("解被关节限位钳制（不是真解）");
      out.candidates.push_back(ev);
      continue;
    }

    // ③ 关节跳变 —— ★ R1 修复
    //
    // ev.q 已经过 wrapToward：在限位内选了"离 q_now 最近"的等价表示，
    // 所以 ev.q − q_now **就是电机实际要转的角度**，绝不能再取余。
    //
    // 反例（修复前）：q_now.j1=+3.0，候选 −3.0，+2π 越界故 wrapToward 仍给 −3.0；
    //   remainder(−3−3, 2π)=+0.283 → 判定"小跳变"通过，
    //   但实际要转 −6.0 rad（343.8°）。这就是"过滤看到的行程 ≠ 实际行程"。
    //
    // wrappedDelta 只适用于"驱动是连续坐标、执行器按最短角走"的情形；
    // 本工程的 DM 电机是有界绝对角（±π / ±2π），不满足。
    const Eigen::VectorXd dq = ev.q - q_now;
    ev.max_joint_jump = dq.cwiseAbs().maxCoeff();
    ev.travel_norm = dq.norm();
    if (ev.max_joint_jump > filter.max_joint_jump) {
      reject(ratioText("单轴跳变", ev.max_joint_jump, filter.max_joint_jump));
      out.candidates.push_back(ev);
      continue;
    }
    if (ev.travel_norm > filter.max_joint_norm) {
      reject(ratioText("关节行程", ev.travel_norm, filter.max_joint_norm));
      out.candidates.push_back(ev);
      continue;
    }

    // ④ 限位余量
    ev.min_limit_margin = minLimitMargin(model, ev.q);
    if (ev.min_limit_margin < filter.min_limit_margin) {
      reject(ratioText("限位余量不足", ev.min_limit_margin,
                       filter.min_limit_margin));
      out.candidates.push_back(ev);
      continue;
    }

    // ⑤ 终点奇异度
    ev.sigma_ratio = sigmaRatioAt(model, ws, frame, ev.q,
                                  filter.characteristic_length);
    if (ev.sigma_ratio < filter.min_sigma_ratio) {
      reject(ratioText("终点接近奇异", ev.sigma_ratio, filter.min_sigma_ratio));
      out.candidates.push_back(ev);
      continue;
    }

    // ⑥ 路径奇异度（只查终点会漏掉"终点好、中途穿奇异"）
    ev.path_sigma_ratio = pathSigmaRatio(model, ws, frame, q_now, ev.q,
                                         filter.characteristic_length,
                                         filter.path_samples);
    if (ev.path_sigma_ratio < filter.min_sigma_ratio) {
      reject(ratioText("路径穿过奇异", ev.path_sigma_ratio,
                       filter.min_sigma_ratio));
      out.candidates.push_back(ev);
      continue;
    }

    // ⑦ 峰值速度（端点 v=0 的插值：v_peak = k_v·Δq/T）
    if (has_vmax) {
      double worst = 0.0;
      for (int j = 0; j < nv; ++j) {
        if (filter.max_joint_velocity[j] <= 0.0) continue;
        const double v = filter.velocity_peak_factor * std::abs(dq[j]) / T;
        worst = std::max(worst, v / filter.max_joint_velocity[j]);
      }
      ev.peak_velocity_ratio = worst;
      if (worst > 1.0) {
        reject(ratioText("峰值速度（相对上限）", worst, 1.0));
        out.candidates.push_back(ev);
        continue;
      }
    }

    // ⑧ 峰值加速度（a_peak = k_a·Δq/T²）
    if (has_amax) {
      double worst = 0.0;
      for (int j = 0; j < nv; ++j) {
        if (filter.max_joint_accel[j] <= 0.0) continue;
        const double a = filter.accel_peak_factor * std::abs(dq[j]) / (T * T);
        worst = std::max(worst, a / filter.max_joint_accel[j]);
      }
      ev.peak_accel_ratio = worst;
      if (worst > 1.0) {
        reject(ratioText("峰值加速度（相对上限）", worst, 1.0));
        out.candidates.push_back(ev);
        continue;
      }
    }

    // 重力负载（只用于打分，不过滤）
    if (has_rated) {
      Eigen::VectorXd g = Eigen::VectorXd::Zero(nv);
      computeGravity(model, ws, ev.q, g);
      double worst = 0.0;
      for (int j = 0; j < nv; ++j) {
        if (filter.rated_torque[j] <= 0.0) continue;
        worst = std::max(worst, std::abs(g[j]) / filter.rated_torque[j]);
      }
      ev.gravity_ratio = worst;
    }

    ev.accepted = true;
    out.candidates.push_back(ev);
  }

  // ── 阶段 2：加权打分 ──────────────────────────────────────
  int best = -1;
  double best_score = std::numeric_limits<double>::infinity();
  int n_acc = 0;

  for (auto& ev : out.candidates) {
    if (!ev.accepted) continue;
    ++n_acc;

    // 各项归一化到 [0,1]（用"越小越好"的形式）
    const double travel_term =
        (filter.max_joint_norm > 0.0) ? (ev.travel_norm / filter.max_joint_norm) : 0.0;

    // 奇异度：σ 比值越小越差 → 用 σ_threshold/σ 表示"离奇异多远"
    const double manip_term =
        (ev.sigma_ratio > 1e-9)
            ? std::min(1.0, filter.min_sigma_ratio / ev.sigma_ratio)
            : 1.0;

    // 限位余量：余量越小越差 → 1 − margin/π（π 当作"很宽裕"的参考）
    const double margin_term = 1.0 - std::min(1.0, ev.min_limit_margin / M_PI);

    ev.score = weights.travel * travel_term +
               weights.manipulability * manip_term +
               weights.gravity * ev.gravity_ratio +
               weights.limit_margin * margin_term;

    if (ev.score < best_score) {
      best_score = ev.score;
      best = ev.index;
    }
  }

  out.n_accepted = n_acc;

  if (best < 0) {
    out.ok = false;
    // 汇总最常见的一条拒绝原因
    std::string common;
    int worst_count = 0;
    for (const auto& ev : out.candidates) {
      int c = 0;
      for (const auto& other : out.candidates)
        if (other.reject_reason == ev.reject_reason) ++c;
      if (c > worst_count) {
        worst_count = c;
        common = ev.reject_reason;
      }
    }
    out.message = "所有候选都被过滤掉（" +
                  std::to_string(out.candidates.size()) + " 个）";
    if (!common.empty()) out.message += "；最常见原因：" + common;
    return out;
  }

  out.ok = true;
  out.index = best;
  for (const auto& ev : out.candidates) {
    if (ev.index == best) {
      out.q = ev.q;
      break;
    }
  }
  out.message = "通过 " + std::to_string(n_acc) + "/" +
                std::to_string(out.candidates.size()) + "，选中 #" +
                std::to_string(best) + "（score=" + std::to_string(best_score) +
                "）";
  return out;
}

// ─────────────────────────────────────────────────────────────
// ★ 一步到位：枚举 → 过滤 → 打分 → 唯一最优解
// ─────────────────────────────────────────────────────────────
IKSelectionResult selectBestIK(const RobotModel& model, RobotWorkspace& ws,
                               pinocchio::FrameIndex frame,
                               const pinocchio::SE3& target,
                               const Eigen::VectorXd& q_now,
                               const IKSelectionOptions& options) {
  IKSelectionResult out;
  const int nv = model.nv();

  if (!model.valid()) {
    out.message = "模型未加载";
    return out;
  }
  if (q_now.size() != nv || !q_now.allFinite()) {
    out.message = "q_now 非法";
    return out;
  }

  // ① 枚举：把 q_now 作为 seed_hint 传进去，
  //    保证"离当前构型最近的那个解"一定在候选里（方案 A）。
  const IKResult all =
      solveIKMultiSeed(model, ws, frame, target, &q_now, options.ik);
  out.n_candidates = all.n_solutions();

  if (out.n_candidates == 0) {
    out.message = "IK 没有找到任何可行解（目标不可达或全部超限位）";
    return out;
  }

  // ② 过滤 + 打分（全部在 selectSolution 里）
  const SelectionResult sel = selectSolution(model, ws, frame, all, q_now,
                                             options.filter, options.weights);
  out.n_accepted = sel.n_accepted;

  if (!sel.ok) {
    out.message = sel.message;
    if (options.keep_diagnostics) out.candidates = sel.candidates;
    return out;
  }

  // ③ 把选中解的指标摊平到结果上
  out.ok = true;
  out.q = sel.q;
  out.source_index = sel.index;
  out.message = sel.message;
  if (options.keep_diagnostics) out.candidates = sel.candidates;

  for (const auto& ev : sel.candidates) {
    if (ev.index != sel.index) continue;
    out.score = ev.score;
    out.position_error = ev.position_error;
    out.orientation_error = ev.orientation_error;
    out.travel_norm = ev.travel_norm;
    out.max_joint_jump = ev.max_joint_jump;
    out.min_limit_margin = ev.min_limit_margin;
    out.sigma_ratio = ev.sigma_ratio;
    out.path_sigma_ratio = ev.path_sigma_ratio;
    out.peak_velocity_ratio = ev.peak_velocity_ratio;
    out.peak_accel_ratio = ev.peak_accel_ratio;
    out.gravity_ratio = ev.gravity_ratio;
    break;
  }
  return out;
}

}  // namespace hw_arm_algorithms
