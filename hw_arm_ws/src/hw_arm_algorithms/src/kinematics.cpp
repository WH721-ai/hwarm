// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_algorithms/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace hw_arm_algorithms {
namespace {

/// 把我们的枚举翻译成 Pinocchio 的参考系
///
/// 注意：Pinocchio 4.x 的 ReferenceFrame 只有 3 个值
///   WORLD=0, LOCAL=1, LOCAL_WORLD_ALIGNED=2
/// **没有 BODY**（旧版/某些文档里提到过），这里 BODY 等价于 LOCAL。
pinocchio::ReferenceFrame toPinocchioFrame(JacobianFrame f) {
  switch (f) {
    case JacobianFrame::WORLD:
      return pinocchio::WORLD;
    case JacobianFrame::BODY:
    case JacobianFrame::LOCAL:
      return pinocchio::LOCAL;
    case JacobianFrame::LOCAL_WORLD_ALIGNED:
    default:
      return pinocchio::LOCAL_WORLD_ALIGNED;
  }
}

/// 用阻尼最小二乘从当前 q 迭代一次，返回新 q（不写回 ws）
void dlsStep(const RobotModel& model, RobotWorkspace& ws,
             pinocchio::FrameIndex frame, const pinocchio::SE3& target,
             const IKOptions& opt, Eigen::VectorXd& q, Eigen::MatrixXd& J,
             bool& limited) {
  const int nv = model.nv();
  auto& data = ws.pin();

  // 1) 正运动学
  pinocchio::forwardKinematics(model.pin(), data, q);
  pinocchio::updateFramePlacement(model.pin(), data, frame);
  const pinocchio::SE3& current = data.oMf[frame];

  // 2) 位姿误差（在末端局部系的 6 维 twist）
  const pinocchio::SE3 iMd = current.actInv(target);
  Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

  // 3) 任务雅可比（LOCAL 系）+ log 映射修正
  pinocchio::computeFrameJacobian(model.pin(), data, q, frame,
                                  pinocchio::LOCAL, J);
  const Eigen::Matrix<double, 6, 6> Jlog = pinocchio::Jlog6(iMd.inverse());
  const Eigen::MatrixXd J_task = -Jlog * J;

  // 4) 阻尼伪逆：dq = -Jᵀ(JJᵀ + λ²I)⁻¹ e
  Eigen::Matrix<double, 6, 6> JJt = J_task * J_task.transpose();
  JJt.diagonal().array() += opt.damping * opt.damping;
  const Eigen::VectorXd dq =
      -J_task.transpose() * JJt.ldlt().solve(err);

  // 5) 流形积分 + 限位钳制
  q = pinocchio::integrate(model.pin(), q, opt.step_size * dq);

  if (opt.enforce_joint_limits) {
    const Eigen::VectorXd& lo = model.pin().lowerPositionLimit;
    const Eigen::VectorXd& hi = model.pin().upperPositionLimit;
    for (int i = 0; i < nv; ++i) {
      if (q[i] < lo[i]) {
        q[i] = lo[i];
        limited = true;
      } else if (q[i] > hi[i]) {
        q[i] = hi[i];
        limited = true;
      }
    }
  }
}

/// 解是否**最终停在**关节限位上（不是"迭代过程中有没有被钳制过"）
///
/// 为什么要有这个区分：dlsStep 一旦钳制就会把 limited 置 true 且不再复位，
/// 于是"中途撞了一下限位、随后又收敛到内部"的解也会被标成 limited。
/// 多起点枚举里这类解一旦先入表，去重时就会把后来那个**干净的解**挡掉，
/// 表现为"最近解明明解出来了却不在候选里"。
bool sitsOnJointLimit(const pinocchio::Model& model,
                      const Eigen::VectorXd& q, double eps = 1e-5) {
  const int n = std::min<int>(q.size(), model.lowerPositionLimit.size());
  for (int i = 0; i < n; ++i) {
    if (q[i] <= model.lowerPositionLimit[i] + eps) return true;
    if (q[i] >= model.upperPositionLimit[i] - eps) return true;
  }
  return false;
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// 正运动学 / 雅可比
// ─────────────────────────────────────────────────────────────
void computeFK(const RobotModel& model, RobotWorkspace& ws,
               const Eigen::VectorXd& q, pinocchio::FrameIndex frame,
               pinocchio::SE3& pose_out) {
  auto& data = ws.pin();
  pinocchio::forwardKinematics(model.pin(), data, q);
  pinocchio::updateFramePlacement(model.pin(), data, frame);
  pose_out = data.oMf[frame];
}

void computeJacobian(const RobotModel& model, RobotWorkspace& ws,
                     const Eigen::VectorXd& q, pinocchio::FrameIndex frame,
                     JacobianFrame frame_type, Eigen::MatrixXd& J_out) {
  auto& data = ws.pin();
  const int nv = model.nv();

  pinocchio::computeJointJacobians(model.pin(), data, q);
  pinocchio::updateFramePlacement(model.pin(), data, frame);

  if (J_out.rows() != 6 || J_out.cols() != nv) {
    J_out.setZero(6, nv);
  }
  J_out = pinocchio::getFrameJacobian(model.pin(), data, frame,
                                      toPinocchioFrame(frame_type));
}

// ─────────────────────────────────────────────────────────────
// IK
// ─────────────────────────────────────────────────────────────
const IKSolution& IKResult::best() const {
  if (solutions.empty()) {
    throw std::out_of_range("IKResult::best(): 没有解");
  }
  return *std::min_element(
      solutions.begin(), solutions.end(),
      [](const IKSolution& a, const IKSolution& b) { return a.cost() < b.cost(); });
}

IKSolution solveIK(const RobotModel& model, RobotWorkspace& ws,
                   pinocchio::FrameIndex frame, const pinocchio::SE3& target,
                   const Eigen::VectorXd& seed, const IKOptions& options) {
  const int nv = model.nv();
  IKSolution sol;
  sol.q = seed;
  sol.converged = false;

  if (sol.q.size() != nv) {
    return sol;
  }

  Eigen::MatrixXd J(6, nv);
  bool limited = false;

  for (int it = 0; it < options.max_iterations; ++it) {
    // 先测当前残差
    pinocchio::SE3 current;
    {
      auto& data = ws.pin();
      pinocchio::forwardKinematics(model.pin(), data, sol.q);
      pinocchio::updateFramePlacement(model.pin(), data, frame);
      current = data.oMf[frame];
    }
    double pe = 0.0, oe = 0.0;
    splitPoseError(current, target, pe, oe);
    sol.position_error = pe;
    sol.orientation_error = oe;
    sol.iterations = it;

    if (pe < options.position_tol && oe < options.orientation_tol) {
      sol.converged = true;
      // ★ 用"最终解是否贴在限位上"判定，而不是"中途有没有钳制过"
      sol.limited = sitsOnJointLimit(model.pin(), sol.q);
      return sol;
    }

    dlsStep(model, ws, frame, target, options, sol.q, J, limited);
  }

  // 迭代用尽后再测一次残差
  pinocchio::SE3 current;
  {
    auto& data = ws.pin();
    pinocchio::forwardKinematics(model.pin(), data, sol.q);
    pinocchio::updateFramePlacement(model.pin(), data, frame);
    current = data.oMf[frame];
  }
  splitPoseError(current, target, sol.position_error, sol.orientation_error);
  sol.iterations = options.max_iterations;
  sol.limited = sitsOnJointLimit(model.pin(), sol.q);
  sol.converged = (sol.position_error < options.position_tol &&
                   sol.orientation_error < options.orientation_tol);
  return sol;
}

IKResult solveIKFromSeeds(const RobotModel& model, RobotWorkspace& ws,
                          pinocchio::FrameIndex frame,
                          const pinocchio::SE3& target,
                          const std::vector<Eigen::VectorXd>& seeds,
                          const IKOptions& options) {
  IKResult result;

  for (const auto& seed : seeds) {
    const IKSolution sol = solveIK(model, ws, frame, target, seed, options);
    if (!sol.converged) {
      continue;
    }

    // 去重：同一点上如果已存的是"贴限位"的解、新来的是干净解，就用新的替换
    bool duplicate = false;
    for (auto& existing : result.solutions) {
      if (jointDistance(existing.q, sol.q) >= options.dedup_tolerance) continue;
      if (existing.limited && !sol.limited) {
        existing = sol;  // 用更干净的表示替换
      }
      duplicate = true;
      break;
    }
    if (duplicate) {
      continue;
    }

    result.solutions.push_back(sol);
    if (static_cast<int>(result.solutions.size()) >= options.max_solutions) {
      break;
    }
  }

  return result;
}

IKResult solveIKMultiSeed(const RobotModel& model, RobotWorkspace& ws,
                          pinocchio::FrameIndex frame,
                          const pinocchio::SE3& target,
                          const Eigen::VectorXd* seed_hint,
                          const IKOptions& options) {
  const int nv = model.nv();
  const Eigen::VectorXd& lo = model.pin().lowerPositionLimit;
  const Eigen::VectorXd& hi = model.pin().upperPositionLimit;

  const bool use_hint =
      (seed_hint != nullptr) && (seed_hint->size() == nv) && seed_hint->allFinite();

  // 固定随机种子 → 结果可复现
  std::mt19937 rng(options.random_seed);
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  std::vector<Eigen::VectorXd> seeds;
  seeds.reserve(options.num_seeds + (use_hint ? 1 : 0));

  // 第 0 个种子：限位中点（球腕一般从这里就能收敛）
  seeds.push_back((lo + hi) / 2.0);

  // ★ 方案 A：先把"当前构型"塞进去，保证"最近解"一定在候选里。
  //   否则纯随机种子可能漏掉它，多解筛选时就会跳到另一个构型分支。
  if (use_hint) {
    seeds.push_back(*seed_hint);
  }

  for (int s = 1; s < options.num_seeds; ++s) {
    Eigen::VectorXd q(nv);
    for (int i = 0; i < nv; ++i) {
      q[i] = lo[i] + (hi[i] - lo[i]) * dist(rng);
    }
    seeds.push_back(std::move(q));
  }

  return solveIKFromSeeds(model, ws, frame, target, seeds, options);
}

// ─────────────────────────────────────────────────────────────
// 奇异度
// ─────────────────────────────────────────────────────────────
Manipulability computeManipulability(const RobotModel& model,
                                     RobotWorkspace& ws,
                                     const Eigen::VectorXd& q,
                                     pinocchio::FrameIndex frame,
                                     double tolerance) {
  Eigen::MatrixXd J(6, model.nv());
  computeJacobian(model, ws, q, frame, JacobianFrame::LOCAL_WORLD_ALIGNED, J);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  const Eigen::VectorXd& s = svd.singularValues();

  Manipulability m;
  m.sigma_max = (s.size() > 0) ? s[0] : 0.0;
  m.sigma_min = (s.size() > 0) ? s[s.size() - 1] : 0.0;
  m.condition = (m.sigma_min > 1e-12) ? (m.sigma_max / m.sigma_min) : 1e300;
  m.yoshikawa = s.prod();
  m.is_singular = (m.sigma_min < tolerance);
  return m;
}

// ─────────────────────────────────────────────────────────────
// 工具
// ─────────────────────────────────────────────────────────────
void splitPoseError(const pinocchio::SE3& current, const pinocchio::SE3& target,
                    double& position_error, double& orientation_error) {
  const pinocchio::SE3 iMd = current.actInv(target);
  const Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();
  // 实测（Pinocchio 4.x）：log6 的向量是 [linear; angular]
  //   log6(纯平移 [1,2,3])   → [1,2,3, 0,0,0]
  //   log6(绕 z 转 π/2)      → [0,0,0, 0,0,1.5708]
  position_error = err.head<3>().norm();    // 平移分量
  orientation_error = err.tail<3>().norm(); // 旋转分量
}

double jointDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if (a.size() != b.size()) {
    return 1e300;
  }
  double dmax = 0.0;
  for (int i = 0; i < a.size(); ++i) {
    // 圆环距离：容忍 ±2π 回绕
    const double d = std::abs(std::remainder(a[i] - b[i], 2.0 * M_PI));
    dmax = std::max(dmax, d);
  }
  return dmax;
}

}  // namespace hw_arm_algorithms
