// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// C-05 重力辨识命令行：读采样 YAML → 最小二乘 → 输出 θ̂ + 残差
//
// 用法：
//   gravity_id_cli --urdf <file> --samples <yaml> [--out <yaml>]
//                  [--holdout 4] [--rank-tol 1e-6] [--min-n 20]
//
// 输入 YAML（采集脚本产出）：
//   samples:
//     - q: [q1..q6]        # URDF 坐标
//       tau: [t1..t6]      # URDF 坐标（已按 direction 换回）
//       tag: "from_below"
//       weight: 1.0
//
// 输出 YAML：
//   theta: [60 个数]
//   meta: { rank, n_used, rms, max_abs, holdout_rms, urdf }
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/gravity_identify.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

void usage() {
  std::fprintf(stderr,
      "用法: gravity_id_cli --urdf <file> --samples <yaml> [选项]\n"
      "  --urdf <file>      机器人 URDF（必须与采集时一致）\n"
      "  --samples <yaml>   采样文件\n"
      "  --out <yaml>       输出 θ̂（默认 gravity_theta.yaml）\n"
      "  --holdout <n>      每 n 个留一个做验证（0=不验证，默认 4）\n"
      "  --rank-tol <x>     可辨识子空间奇异值阈值（默认 1e-6）\n"
      "  --min-n <n>        最少样本数（默认 20）\n"
      "  --accept-rms <x>   残差 RMS 门槛，超过则退出码 4（默认 0.5 N·m）\n  --robust           Huber-IRLS 稳健回归（抗离群点）\n  --huber-iters <n>  稳健迭代次数（默认 4）\n");
}

bool parseSamples(const std::string& path,
                  std::vector<hw_arm_algorithms::GravityIdSample>& out,
                  std::string& err) {
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node arr = root["samples"];
    if (!arr || !arr.IsSequence()) {
      err = "YAML 里没有 samples 序列";
      return false;
    }
    for (const auto& n : arr) {
      hw_arm_algorithms::GravityIdSample s;
      const auto q = n["q"];
      const auto t = n["tau"];
      if (!q || !t) { err = "某个样本缺 q 或 tau"; return false; }
      s.q.resize(static_cast<Eigen::Index>(q.size()));
      s.tau.resize(static_cast<Eigen::Index>(t.size()));
      for (size_t i = 0; i < q.size(); ++i) s.q[static_cast<Eigen::Index>(i)] = q[i].as<double>();
      for (size_t i = 0; i < t.size(); ++i) s.tau[static_cast<Eigen::Index>(i)] = t[i].as<double>();
      if (n["tag"]) s.tag = n["tag"].as<std::string>();
      if (n["weight"]) s.weight = n["weight"].as<double>();
      out.push_back(std::move(s));
    }
    return true;
  } catch (const std::exception& e) {
    err = e.what();
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf, samples_path, out_path = "gravity_theta.yaml";
  int holdout = 4;
  double rank_tol = 1e-6;
  int min_n = 20;
  double accept_rms = 0.5;
  double ridge = 0.0;        // >0 时打开 Tikhonov 正则（往 URDF 标称参数拉）
  bool ridge_set = false;
  bool robust = false;
  int huber_iters = 4;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--urdf") urdf = next();
    else if (a == "--samples") samples_path = next();
    else if (a == "--out") out_path = next();
    else if (a == "--ridge") { ridge = std::atof(next().c_str()); ridge_set = true; }
    else if (a == "--holdout") holdout = std::atoi(next().c_str());
    else if (a == "--rank-tol") rank_tol = std::atof(next().c_str());
    else if (a == "--min-n") min_n = std::atoi(next().c_str());
    else if (a == "--accept-rms") accept_rms = std::atof(next().c_str());
    else if (a == "--robust") robust = true;
    else if (a == "--huber-iters") huber_iters = std::atoi(next().c_str());
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "未知参数: %s\n", a.c_str()); usage(); return 2; }
  }
  if (urdf.empty() || samples_path.empty()) { usage(); return 2; }

  hw_arm_algorithms::RobotModel model;
  if (!model.load(urdf)) {
    std::fprintf(stderr, "❌ 模型加载失败: %s\n", urdf.c_str());
    return 1;
  }
  hw_arm_algorithms::RobotWorkspace ws(model);

  std::vector<hw_arm_algorithms::GravityIdSample> samples;
  std::string err;
  if (!parseSamples(samples_path, samples, err)) {
    std::fprintf(stderr, "❌ 读采样失败: %s\n", err.c_str());
    return 1;
  }
  std::printf("读入 %zu 个样本\n", samples.size());
  if (static_cast<int>(samples.size()) < min_n) {
    std::fprintf(stderr, "❌ 样本数 %zu < --min-n %d，拒绝拟合\n", samples.size(), min_n);
    return 3;
  }

  hw_arm_algorithms::GravityIdOptions opt;
  opt.rank_tol = rank_tol;
  opt.holdout_stride = holdout;
  if (ridge_set) {
    opt.regularize = true;      // 往 URDF 标称参数拉，改善留出误差
    opt.ridge_ratio = ridge;
  }
  opt.robust = robust;
  opt.huber_iters = huber_iters;

  const auto r = hw_arm_algorithms::identifyGravity(model, ws, samples, opt);

  std::printf("ok: %s\n", r.ok ? "true" : "false");
  std::printf("message: \"%s\"\n", r.message.c_str());
  std::printf("n_samples: %d\n", r.n_samples);
  std::printf("n_used: %d\n", r.n_used);
  std::printf("rank: %d\n", r.rank);
  if (r.ok) {
    std::printf("rms: %.6f\n", r.rms);
    std::printf("max_abs: %.6f\n", r.max_abs);
    std::printf("friction: [");
    for (Eigen::Index j = 0; j < r.friction.size(); ++j)
      std::printf("%s%.4f", j ? ", " : "", r.friction[j]);
    std::printf("]\n");
    std::printf("offsets: [");
    for (Eigen::Index j = 0; j < r.offsets.size(); ++j)
      std::printf("%s%.4f", j ? ", " : "", r.offsets[j]);
    std::printf("]\n");
    std::printf("per_joint_rms: [");
    for (Eigen::Index j = 0; j < r.per_joint_rms.size(); ++j)
      std::printf("%s%.4f", j ? ", " : "", r.per_joint_rms[j]);
    std::printf("]\n");
    if (r.holdout_ok) {
      std::printf("holdout_rms: %.6f\n", r.holdout_rms);
      if (robust) {
        std::printf("inlier_rms: %.6f\n", r.inlier_rms);
        std::printf("n_outliers: %d\n", r.n_outliers);
      }
      std::printf("holdout_max: %.6f\n", r.holdout_max);
    } else {
      std::printf("holdout_rms: nan\n");
    }
  }

  if (!r.ok) return 1;

  // ── 写输出 YAML ──
  try {
    YAML::Node root;
    root["theta"] = std::vector<double>(r.theta.data(), r.theta.data() + r.theta.size());
    YAML::Node meta;
    meta["urdf"] = urdf;
    meta["rank"] = r.rank;
    meta["n_used"] = r.n_used;
    meta["rms"] = r.rms;
    meta["max_abs"] = r.max_abs;
    meta["holdout_rms"] = r.holdout_ok ? r.holdout_rms : -1.0;
    meta["param_layout"] = "per body 10: [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]";
    root["meta"] = meta;
    root["friction"] = std::vector<double>(r.friction.data(),
                                           r.friction.data() + r.friction.size());
    root["offsets"] = std::vector<double>(r.offsets.data(),
                                          r.offsets.data() + r.offsets.size());
    YAML::Emitter em;
    em << root;
    std::FILE* f = std::fopen(out_path.c_str(), "w");
    if (!f) { std::fprintf(stderr, "❌ 写不了 %s\n", out_path.c_str()); return 1; }
    std::fprintf(f, "%s\n", em.c_str());
    std::fclose(f);
    std::printf("written: %s\n", out_path.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "❌ 写 YAML 失败: %s\n", e.what());
    return 1;
  }

  // ── 验收门槛 ──
  const double check = r.holdout_ok ? r.holdout_rms : r.rms;
  std::printf("accept_threshold: %.3f\n", accept_rms);
  if (check > accept_rms) {
    std::fprintf(stderr,
        "⚠️ 残差 %.3f N·m 超过门槛 %.3f —— 参数已写出，但**不建议直接用**\n"
        "   排查顺序：位形覆盖不足 → 摩擦未双向抵消 → 力矩测量零点偏移\n",
        check, accept_rms);
    return 4;
  }
  std::printf("✅ 通过（残差 %.3f N·m ≤ %.3f）\n", check, accept_rms);
  return 0;
}
