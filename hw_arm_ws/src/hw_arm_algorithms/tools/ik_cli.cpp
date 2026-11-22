// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 逆向运动学命令行：目标位姿 + 当前关节角 → **唯一最优关节角**
//
// ⚠️ 本文件只做三件事，**不含任何算法**：
//     ① 解析命令行参数
//     ② 调一次 hw_arm_algorithms::selectBestIK(...)
//     ③ 把结果打印成 YAML 风格（给 goto_xyz.py 读）
//
// 枚举多解 / 硬过滤 / 加权打分 / 奇异度 —— 全在 kinematics.cpp
// 和 ik_selection.cpp 里完成。
//
// 用法：
//   ik_cli --urdf <path> --seed "q1..q6" \
//          --target "x y z qx qy qz qw" [--frame tool0] \
//          [--tol 1e-6] [--max-iter 200] [--damping 1e-2] \
//          [--seeds 16] [--solutions 8] \
//          [--max-jump 0.5] [--max-norm 1.0] [--margin 0.087] \
//          [--sigma 2e-3] [--duration 3.0] [--verbose]
//
// 输出（stdout）：
//   ok: true
//   q: [..]
//   q_deg: [..]
//   score: 0.123
//   position_error: 1.2e-07
//   orientation_error: 3.4e-08
//   travel_norm: 0.44
//   max_joint_jump: 0.21
//   min_limit_margin: 0.42
//   sigma_ratio: 0.0581
//   path_sigma_ratio: 0.0412
//   n_candidates: 3
//   n_accepted: 1
//   message: "..."
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/ik_selection.hpp"
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
      "         [--frame tool0] [--tol 1e-6] [--max-iter 200] [--damping 1e-2]\n"
      "         [--seeds 16] [--solutions 8]\n"
      "         [--max-jump 0.5] [--max-norm 1.0] [--margin 0.087]\n"
      "         [--sigma 2e-3] [--duration 3.0] [--verbose]\n");
}

void printArray(const char* key, const Eigen::VectorXd& v, int prec) {
  std::printf("%s: [", key);
  for (int i = 0; i < v.size(); ++i)
    std::printf("%s%.*f", i ? ", " : "", prec, v[i]);
  std::printf("]\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf, seed_s, target_s, frame_name = "tool0";
  double tol = 1e-6;
  int max_iter = 200;
  double damping = 1e-2;
  int num_seeds = 16;
  int max_solutions = 8;
  double max_jump = 0.5;
  double max_norm = 1.0;
  double margin = 0.087;
  double sigma_min = 2e-3;
  double duration = 3.0;
  bool verbose = false;

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
    else if (a == "--seeds") num_seeds = std::atoi(next().c_str());
    else if (a == "--solutions") max_solutions = std::atoi(next().c_str());
    else if (a == "--max-jump") max_jump = std::atof(next().c_str());
    else if (a == "--max-norm") max_norm = std::atof(next().c_str());
    else if (a == "--margin") margin = std::atof(next().c_str());
    else if (a == "--sigma") sigma_min = std::atof(next().c_str());
    else if (a == "--duration") duration = std::atof(next().c_str());
    else if (a == "--verbose") verbose = true;
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

  Eigen::VectorXd q_now(6);
  for (int i = 0; i < 6; ++i) q_now[i] = qv[static_cast<size_t>(i)];

  Eigen::Quaterniond quat(tv[6], tv[3], tv[4], tv[5]);  // (w, x, y, z)
  if (std::abs(quat.norm() - 1.0) > 1e-3) {
    std::fprintf(stderr, "❌ 四元数未归一化 (norm=%.6f)\n", quat.norm());
    return 2;
  }
  quat.normalize();
  const pinocchio::SE3 target(quat.toRotationMatrix(),
                              Eigen::Vector3d(tv[0], tv[1], tv[2]));

  // ── 参数装配（只是把 CLI 参数搬进 options，不做计算）──
  hw_arm_algorithms::IKSelectionOptions opt;
  opt.ik.position_tol = tol;
  opt.ik.orientation_tol = tol;
  opt.ik.max_iterations = max_iter;
  opt.ik.damping = damping;
  opt.ik.num_seeds = num_seeds;
  opt.ik.max_solutions = max_solutions;
  opt.ik.enforce_joint_limits = true;

  opt.filter.max_joint_jump = max_jump;
  opt.filter.max_joint_norm = max_norm;
  opt.filter.min_limit_margin = margin;
  opt.filter.min_sigma_ratio = sigma_min;
  opt.filter.duration = duration;
  opt.keep_diagnostics = verbose;

  // ── ★ 唯一一次计算调用 ──
  const auto res =
      hw_arm_algorithms::selectBestIK(model, ws, frame, target, q_now, opt);

  // ── 打印（只是格式化）──
  std::printf("ok: %s\n", res.ok ? "true" : "false");
  if (res.ok) {
    printArray("q", res.q, 9);
    Eigen::VectorXd deg = res.q * (180.0 / M_PI);
    printArray("q_deg", deg, 2);
    std::printf("score: %.6f\n", res.score);
  }
  std::printf("position_error: %.6g\n", res.position_error);
  std::printf("orientation_error: %.6g\n", res.orientation_error);
  std::printf("travel_norm: %.6g\n", res.travel_norm);
  std::printf("max_joint_jump: %.6g\n", res.max_joint_jump);
  std::printf("min_limit_margin: %.6g\n", res.min_limit_margin);
  std::printf("sigma_ratio: %.6g\n", res.sigma_ratio);
  std::printf("path_sigma_ratio: %.6g\n", res.path_sigma_ratio);
  std::printf("n_candidates: %d\n", res.n_candidates);
  std::printf("n_accepted: %d\n", res.n_accepted);
  std::printf("message: \"%s\"\n", res.message.c_str());

  if (verbose) {
    std::printf("candidates:\n");
    for (const auto& c : res.candidates) {
      std::printf("  - index: %d\n    accepted: %s\n    score: %.6f\n"
                  "    max_joint_jump: %.6f\n    sigma_ratio: %.6f\n"
                  "    path_sigma_ratio: %.6f\n    reason: \"%s\"\n",
                  c.index, c.accepted ? "true" : "false", c.score,
                  c.max_joint_jump, c.sigma_ratio, c.path_sigma_ratio,
                  c.reject_reason.c_str());
    }
  }
  return res.ok ? 0 : 1;
}
