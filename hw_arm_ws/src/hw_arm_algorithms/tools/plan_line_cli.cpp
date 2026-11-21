// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 命令行工具：笛卡尔直线轨迹 → 关节路点（YAML）
//
// 用法（离线生成，和 goto_xyz.py 配合）：
//   plan_line_cli --urdf <path> --start "-0.1,0.0,0.3,0,0,1.2,0.8,0,0.3,0" \
//                 --target "-0.05,0.0,0.32,1,0,0,0" --duration 3.0 --sample 0.02
//
//   --start   10 个数：q(6) + 可选 dq(4)？ ← 固定 6 个数：q1..q6
//   --target  7 个数：x,y,z,qx,qy,qz,qw
//   --frame   末端帧名（默认 tool0）
//
// 输出（stdout）：
//   ok: true
//   position_error: 0.000123
//   orientation_error: 0.000045
//   waypoints:
//     - {t: 0.000, q: [..], qd: [..]}
//     ...
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>

#include "hw_arm_algorithms/cartesian_trajectory.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

std::vector<double> parseNumbers(const std::string& text) {
  std::vector<double> out;
  std::string cur;
  for (char c : text + ",") {
    if (c == ',' || c == ' ' || c == ';') {
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
      "  plan_line_cli --urdf <file> --start q1..q6 --target x y z qx qy qz qw\n"
      "                [--frame tool0] [--duration 3.0] [--sample 0.02]\n"
      "                [--control 0.004] [--damping 0.01] [--gain 5.0]\n"
      "                [--vmax 2.0] [--sigma-ratio 2e-3]\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf;
  std::string start_s;
  std::string target_s;
  std::string frame_name = "tool0";
  double duration = 3.0;
  double sample = 0.02;
  double control = 0.004;
  double damping = 1e-2;
  double gain = 5.0;
  double vmax = 2.0;
  double sigma_ratio = 2e-3;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--urdf") urdf = next();
    else if (a == "--start") start_s = next();
    else if (a == "--target") target_s = next();
    else if (a == "--frame") frame_name = next();
    else if (a == "--duration") duration = std::atof(next().c_str());
    else if (a == "--sample") sample = std::atof(next().c_str());
    else if (a == "--control") control = std::atof(next().c_str());
    else if (a == "--damping") damping = std::atof(next().c_str());
    else if (a == "--gain") gain = std::atof(next().c_str());
    else if (a == "--vmax") vmax = std::atof(next().c_str());
    else if (a == "--sigma-ratio") sigma_ratio = std::atof(next().c_str());
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "未知参数: %s\n", a.c_str()); usage(); return 2; }
  }

  if (urdf.empty() || start_s.empty() || target_s.empty()) {
    usage();
    return 2;
  }

  const std::vector<double> qv = parseNumbers(start_s);
  const std::vector<double> tv = parseNumbers(target_s);
  if (qv.size() != 6) {
    std::fprintf(stderr, "--start 需要 6 个数，实际 %zu\n", qv.size());
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

  // 末端帧
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

  Eigen::VectorXd q0(6);
  for (int i = 0; i < 6; ++i) q0[i] = qv[static_cast<size_t>(i)];

  Eigen::Quaterniond quat(tv[6], tv[3], tv[4], tv[5]);  // w, x, y, z
  if (std::abs(quat.norm() - 1.0) > 1e-3) {
    std::fprintf(stderr, "❌ 四元数没有归一化 (norm=%.6f)\n", quat.norm());
    return 2;
  }
  quat.normalize();
  const pinocchio::SE3 target(quat.toRotationMatrix(),
                              Eigen::Vector3d(tv[0], tv[1], tv[2]));

  hw_arm_algorithms::CartesianPlanOptions opt;
  opt.duration = duration;
  opt.sample_dt = sample;
  opt.control_dt = control;
  opt.damping = damping;
  opt.feedback_gain = gain;
  opt.max_joint_speed = vmax;
  opt.min_singular_ratio = sigma_ratio;

  const auto res = hw_arm_algorithms::planCartesianLine(model, ws, frame, q0,
                                                        target, opt);

  std::printf("ok: %s\n", res.ok ? "true" : "false");
  if (!res.ok) std::printf("error: \"%s\"\n", res.error.c_str());
  std::printf("position_error: %.6g\n", res.final_position_error);
  std::printf("orientation_error: %.6g\n", res.final_orientation_error);
  std::printf("min_sigma_ratio: %.6g\n", res.min_sigma_ratio);
  std::printf("limited: %s\n", res.limited ? "true" : "false");
  std::printf("singular: %s\n", res.singular ? "true" : "false");
  std::printf("n_waypoints: %zu\n", res.waypoints.size());
  std::printf("waypoints:\n");
  for (const auto& w : res.waypoints) {
    std::printf("  - t: %.4f\n    q: [", w.t);
    for (int i = 0; i < w.q.size(); ++i)
      std::printf("%s%.6f", i ? ", " : "", w.q[i]);
    std::printf("]\n    qd: [");
    for (int i = 0; i < w.qd.size(); ++i)
      std::printf("%s%.6f", i ? ", " : "", w.qd[i]);
    std::printf("]\n");
  }
  return res.ok ? 0 : 1;
}
