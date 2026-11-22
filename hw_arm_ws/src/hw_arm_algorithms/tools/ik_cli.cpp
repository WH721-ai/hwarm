// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 逆向运动学命令行：目标位姿 + 种子 → 关节角
//
// 只封装已有的 hw_arm_algorithms/kinematics.cpp：
//   computeFK / solveIK / computeManipulability
// **不引入任何新算法**。给 goto_xyz.py 做"笛卡尔点到点"用。
//
// 用法：
//   ik_cli --urdf <path> --seed "q1,q2,q3,q4,q5,q6" \
//          --target "x y z qx qy qz qw" [--frame tool0] \
//          [--tol 1e-4] [--max-iter 200] [--damping 1e-2]
//
// 输出（stdout，YAML 风格）：
//   ok: true
//   q: [..]
//   q_deg: [..]
//   position_error: 1.2e-07
//   orientation_error: 3.4e-08
//   iterations: 14
//   limited: false
//   sigma_ratio: 0.0581
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

std::vector<double> parseNumbers(const std::string& text) {
  std::vector<double> out;
  std::string cur;
  for (char c : text + ",") {
    if (c == ',' || c == ' ' || c == ';' || c == '\t') {
      if (!cur.empty()) {
        out.push_back(std::atof(cur.c_str()));
        cur.clear();
      }
    } else {
      cur.push_back(c);
    }
  }
  return out;
}

void usage() {
  std::printf(
      "用法:\n"
      "  ik_cli --urdf <file> --seed q1..q6 --target x y z qx qy qz qw\n"
      "         [--frame tool0] [--tol 1e-4] [--max-iter 200] [--damping 1e-2]\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf, seed_s, target_s, frame_name = "tool0";
  double tol = 1e-4;
  int max_iter = 200;
  double damping = 1e-2;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--urdf") urdf = next();
    else if (a == "--seed") seed_s = next();
    else if (a == "--target") target_s = next();
    else if (a == "--frame") frame_name = next();
    else if (a == "--tol") tol = std::atof(next().c_str());
    else if (a == "--max-iter") max_iter = std::atoi(next().c_str());
    else if (a == "--damping") damping = std::atof(next().c_str());
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "未知参数: %s\n", a.c_str()); usage(); return 2; }
  }

  if (urdf.empty() || seed_s.empty() || target_s.empty()) {
    usage();
    return 2;
  }

  const std::vector<double> qv = parseNumbers(seed_s);
  const std::vector<double> tv = parseNumbers(target_s);
  if (qv.size() != 6) {
    std::fprintf(stderr, "--seed 需要 6 个数，实际 %zu\n", qv.size());
    return 2;
  }
  if (tv.size() != 7) {
    std::fprintf(stderr, "--target 需要 7 个数 (x y z qx qy qz qw)，实际 %zu\n",
                 tv.size());
    return 2;
  }

  hw_arm_algorithms::RobotModel model;
  if (!model.load(urdf)) {
    std::fprintf(stderr, "❌ 模型加载失败: %s\n", urdf.c_str());
    return 3;
  }
  hw_arm_algorithms::RobotWorkspace ws(model);

  pinocchio::FrameIndex frame = 0;
  bool found = false;
  for (size_t i = 0; i < model.pin().frames.size(); ++i) {
    if (model.pin().frames[i].name == frame_name) {
      frame = static_cast<pinocchio::FrameIndex>(i);
      found = true;
      break;
    }
  }
  if (!found) {
    std::fprintf(stderr, "❌ 找不到帧 '%s'\n", frame_name.c_str());
    return 3;
  }

  Eigen::VectorXd seed(6);
  for (int i = 0; i < 6; ++i) seed[i] = qv[static_cast<size_t>(i)];

  Eigen::Quaterniond quat(tv[6], tv[3], tv[4], tv[5]);  // w, x, y, z
  if (std::abs(quat.norm() - 1.0) > 1e-3) {
    std::fprintf(stderr, "❌ 四元数未归一化 (norm=%.6f)\n", quat.norm());
    return 2;
  }
  quat.normalize();
  const pinocchio::SE3 target(quat.toRotationMatrix(),
                              Eigen::Vector3d(tv[0], tv[1], tv[2]));

  hw_arm_algorithms::IKOptions opt;
  opt.position_tol = tol;
  opt.orientation_tol = tol;
  opt.max_iterations = max_iter;
  opt.damping = damping;
  opt.enforce_joint_limits = true;

  const auto sol = hw_arm_algorithms::solveIK(model, ws, frame, target, seed, opt);

  // 顺带给出该解的可操作度（会重算雅可比，代价可忽略）
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv());
  hw_arm_algorithms::computeJacobian(model, ws, sol.q, frame,
                                     hw_arm_algorithms::JacobianFrame::LOCAL_WORLD_ALIGNED,
                                     J);
  Eigen::MatrixXd Js = J;
  Js.bottomRows(3) *= 0.1;  // 特征长度 0.1 m，让角速度与线速度量纲可比
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Js);
  const auto& s = svd.singularValues();
  const double sigma_ratio = (s.size() > 0 && s[0] > 1e-12) ? s[s.size() - 1] / s[0] : 0.0;

  std::printf("ok: %s\n", sol.converged ? "true" : "false");
  std::printf("q: [");
  for (int i = 0; i < sol.q.size(); ++i)
    std::printf("%s%.9f", i ? ", " : "", sol.q[i]);
  std::printf("]\n");
  std::printf("q_deg: [");
  for (int i = 0; i < sol.q.size(); ++i)
    std::printf("%s%.2f", i ? ", " : "", sol.q[i] * 180.0 / M_PI);
  std::printf("]\n");
  std::printf("position_error: %.6g\n", sol.position_error);
  std::printf("orientation_error: %.6g\n", sol.orientation_error);
  std::printf("iterations: %d\n", sol.iterations);
  std::printf("limited: %s\n", sol.limited ? "true" : "false");
  std::printf("sigma_ratio: %.6g\n", sigma_ratio);

  return sol.converged ? 0 : 1;
}
