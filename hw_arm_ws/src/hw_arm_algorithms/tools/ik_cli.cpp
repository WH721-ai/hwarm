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
#include <cerrno>
#include <cmath>
#include <cmath>
#include <cstdio>
#include <memory>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/collision_model.hpp"
#include "hw_arm_algorithms/collision_robot.hpp"
#include "hw_arm_algorithms/collision_world.hpp"
#include "hw_arm_algorithms/ik_selection.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

// ★ R11：把 "1x" / "oops" / "inf" 这类输入判为错误，而不是静默变 0 / NaN。
//   std::atof("oops")=0、std::atof("1x")=1 —— 会让"写错一个参数"变成"用错的目标"。
bool parseNumber(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') return false;   // 必须整串消费
  if (errno == ERANGE || !std::isfinite(v)) return false;
  out = v;
  return true;
}

// 返回 false 表示**有 token 解析失败**（不是"空列表"）
bool parseNumbers(const std::string& text, std::vector<double>& out) {
  out.clear();
  std::string cur;
  bool ok = true;
  for (char c : text + ",") {
    if (c == ',' || c == ' ' || c == ';' || c == '\t') {
      if (!cur.empty()) {
        double v = 0.0;
        if (!parseNumber(cur, v)) ok = false;
        out.push_back(v);
        cur.clear();
      }
    } else {
      cur.push_back(c);
    }
  }
  return ok;
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
  std::string vmax_s, amax_s;
  bool verbose = false;

  // ── 碰撞检查（★ 默认开启。关掉必须显式 --no-collision，且会打警告）──
  bool check_collision = true;
  bool check_self = true;
  bool check_table = true;
  double collision_margin = 0.0;  // 自碰撞余量，m（home 间隙仅 3.5 mm）
  double table_z = 0.0;           // 桌面在 world 系的 z
  double table_margin = 0.01;     // 桌面余量，m
  double table_base_margin = 0.0; // 底座（螺栓固定）的余量，m
  bool table_base_margin_set = false;
  std::string ee_links_s;         // 逗号分隔的末端 link 名

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--urdf") urdf = next();
    else if (a == "--seed") seed_s = next();
    else if (a == "--target") target_s = next();
    else if (a == "--frame") frame_name = next();
    else if (a == "--tol") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--tol 不是合法数字\n"); return 2; } tol = v; }
    else if (a == "--max-iter") max_iter = std::atoi(next().c_str());
    else if (a == "--damping") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--damping 不是合法数字\n"); return 2; } damping = v; }
    else if (a == "--seeds") num_seeds = std::atoi(next().c_str());
    else if (a == "--solutions") max_solutions = std::atoi(next().c_str());
    else if (a == "--max-jump") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--max-jump 不是合法数字\n"); return 2; } max_jump = v; }
    else if (a == "--max-norm") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--max-norm 不是合法数字\n"); return 2; } max_norm = v; }
    else if (a == "--margin") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--margin 不是合法数字\n"); return 2; } margin = v; }
    else if (a == "--sigma") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--sigma 不是合法数字\n"); return 2; } sigma_min = v; }
    else if (a == "--duration") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--duration 不是合法数字\n"); return 2; } duration = v; }
    else if (a == "--vmax") vmax_s = next();
    else if (a == "--amax") amax_s = next();
    else if (a == "--verbose") verbose = true;
    else if (a == "--no-collision") check_collision = false;
    else if (a == "--no-self-collision") check_self = false;
    else if (a == "--no-table") check_table = false;
    else if (a == "--collision-margin") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--collision-margin 不是合法数字\n"); return 2; } collision_margin = v; }
    else if (a == "--table-z") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--table-z 不是合法数字\n"); return 2; } table_z = v; }
    else if (a == "--table-margin") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--table-margin 不是合法数字\n"); return 2; } table_margin = v; }
    else if (a == "--table-base-margin") { double v; if (!parseNumber(next(), v)) { std::fprintf(stderr, "--table-base-margin 不是合法数字\n"); return 2; } table_base_margin = v; table_base_margin_set = true; }
    else if (a == "--ee-links") ee_links_s = next();
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "未知参数: %s\n", a.c_str()); usage(); return 2; }
  }

  if (urdf.empty() || seed_s.empty() || target_s.empty()) {
    usage();
    return 2;
  }

  std::vector<double> qv, tv;
  if (!parseNumbers(seed_s, qv)) {
    std::fprintf(stderr, "--seed 含非法数字：%s\n", seed_s.c_str());
    return 2;
  }
  if (!parseNumbers(target_s, tv)) {
    std::fprintf(stderr, "--target 含非法数字：%s\n", target_s.c_str());
    return 2;
  }
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

  // ★ R11：NaN 会让 |norm-1| > 1e-3 为 false（NaN 比较恒假），必须先判有限
  Eigen::Quaterniond quat(tv[6], tv[3], tv[4], tv[5]);  // (w, x, y, z)
  if (!(std::isfinite(quat.w()) && std::isfinite(quat.x()) &&
        std::isfinite(quat.y()) && std::isfinite(quat.z()))) {
    std::fprintf(stderr, "❌ 四元数含 NaN/Inf\n");
    return 2;
  }
  const double qn = quat.norm();
  if (!(qn > 1e-6)) {
    std::fprintf(stderr, "❌ 四元数模长为 0\n");
    return 2;
  }
  if (std::abs(qn - 1.0) > 1e-3) {
    std::fprintf(stderr, "❌ 四元数未归一化 (norm=%.6f)\n", qn);
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

  // ★ R2：速度/加速度判据在 ik_selection 里靠"向量非空"决定是否启用，
  //   不传就等于**静默关闭**两条保护。这里改为**必填**，缺失直接报错。
  std::vector<double> vmax_v, amax_v;
  if (vmax_s.empty() || amax_s.empty()) {
    std::fprintf(stderr,
        "❌ 必须提供 --vmax 与 --amax（6 轴，见 config/control/motor_specs.yaml）\n");
    return 2;
  }
  if (!parseNumbers(vmax_s, vmax_v) || vmax_v.size() != 6) {
    std::fprintf(stderr, "❌ --vmax 需要 6 个合法数字\n");
    return 2;
  }
  if (!parseNumbers(amax_s, amax_v) || amax_v.size() != 6) {
    std::fprintf(stderr, "❌ --amax 需要 6 个合法数字\n");
    return 2;
  }
  opt.filter.max_joint_velocity = Eigen::Map<Eigen::VectorXd>(vmax_v.data(), 6);
  opt.filter.max_joint_accel = Eigen::Map<Eigen::VectorXd>(amax_v.data(), 6);
  for (int i = 0; i < 6; ++i) {
    if (!(opt.filter.max_joint_velocity[i] > 0.0) ||
        !(opt.filter.max_joint_accel[i] > 0.0)) {
      std::fprintf(stderr, "❌ --vmax/--amax 第 %d 项必须为正\n", i + 1);
      return 2;
    }
  }

  // ── ★ 碰撞上下文装配（审查报告 C01）──
  //
  // C01 的教训：库里有接口、测试里有回调、文档里有示例，
  // 但**实际入口从来没装配** → 真机链路上碰撞检测一行都没跑。
  // 这里改成：默认开启，模型建不起来就**退出**（不回退成「无碰撞模式」）。
  //
  // ⚠️ collision_model / topt 必须声明在 if 之外：
  //    回调按引用捕获，若在 else 块里声明，块结束时被析构 → 悬空引用。
  std::unique_ptr<hw_arm_algorithms::CollisionModel> collision_model;
  hw_arm_algorithms::TableConstraintOptions topt;

  if (!check_collision) {
    std::fprintf(stderr,
        "⚠️⚠️ 已用 --no-collision 关闭碰撞检查 —— 本轨迹**未经过碰撞验证**\n");
  } else {
    collision_model = std::make_unique<hw_arm_algorithms::CollisionModel>();
    if (!collision_model->build(model, urdf)) {
      std::fprintf(stderr,
          "❌ 碰撞模型构建失败（凸包数据缺失或 URDF 无 collision 标签）\n"
          "   不会退回到「无碰撞」模式 —— 拒绝继续\n");
      return 3;
    }
    if (!ee_links_s.empty()) {
      std::vector<std::string> ee;
      std::string cur;
      for (char c : ee_links_s) {
        if (c == ',') { if (!cur.empty()) ee.push_back(cur); cur.clear(); }
        else cur.push_back(c);
      }
      if (!cur.empty()) ee.push_back(cur);
      collision_model->setEndEffectorLinks(ee);
    }

    // 桌面：按**全部几何体**算（C02 修完，不再是只查 TCP 一个点）
    // 底座是螺栓固定在桌面上的，单独给 0 余量（不是忽略它）
    topt = hw_arm_algorithms::TableConstraintOptions::withMountedBase(table_margin);
    topt.offset = table_z;
    if (!table_base_margin_set) topt.link_margin["base_link"] = 0.0;

    opt.filter.collision_check = [&](const Eigen::VectorXd& q) {
      if (check_self) {
        hw_arm_algorithms::SelfCollisionOptions sop;
        sop.margin = collision_margin;
        const auto r = hw_arm_algorithms::checkSelfCollision(*collision_model, q, sop);
        if (!r.is_free()) return r;
      }
      if (check_table) {
        const auto r = hw_arm_algorithms::checkTable(*collision_model, q, topt);
        if (!r.is_free()) return r;
      }
      return hw_arm_algorithms::CheckResult::free();
    };
    opt.filter.require_collision_check = true;   // 强制：回调必须存在
  }

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

  // ── 碰撞检查的元信息（发送端不能只看 ok）──
  if (check_collision) {
    std::printf("collision_checked: true\n");
    std::printf("collision_self: %s\n", check_self ? "true" : "false");
    std::printf("collision_table: %s\n", check_table ? "true" : "false");
    std::printf("collision_pairs: %d\n", collision_model->numPairs());
    std::printf("collision_margin: %.6g\n", collision_margin);
    std::printf("table_z: %.6g\n", table_z);
    std::printf("table_margin: %.6g\n", table_margin);
    std::printf("table_base_margin: %.6g\n", table_base_margin);
    const double tclear = hw_arm_algorithms::minTableClearance(*collision_model, q_now, topt);
    if (std::isfinite(tclear)) {
      std::printf("min_table_clearance_from: %.6g\n", tclear);
    } else {
      std::printf("min_table_clearance_from: nan\n");
    }
    std::printf("collision_model: \"%s\"\n", collision_model->fingerprint().c_str());
    const double dmin = hw_arm_algorithms::minSelfDistance(*collision_model, q_now);
    if (std::isfinite(dmin)) {
      std::printf("min_self_distance_from: %.6g\n", dmin);
    } else {
      std::printf("min_self_distance_from: nan\n");
    }
  } else {
    std::printf("collision_checked: false\n");
    std::printf("collision_model: \"disabled\"\n");
  }

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
