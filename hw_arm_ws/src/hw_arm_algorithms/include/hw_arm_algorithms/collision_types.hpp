// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 碰撞检查的统一返回类型
// ─────────────────────────────────────────────────────────────
// 为什么需要三种状态而不是 bool：
//
//   旧实现用 bool 表示"是否碰撞"，于是"模型没建好"、"参数是 NaN"、
//   "关键点算不出来" 这些都只能塞进 false（= 安全）。
//   结果是：**检查失效时，调用方看到的是"无碰撞"，直接放行。**
//
//   现在明确区分：
//     kFree      确认安全，可以放行
//     kCollision 确认碰撞，必须拒绝
//     kInvalid   检查本身不成立（模型/输入/参数有问题）→ **绝不能当放行**
#pragma once

#include <cmath>
#include <limits>
#include <string>

namespace hw_arm_algorithms {

enum class CheckStatus {
  kFree = 0,       ///< 确认无碰撞
  kCollision = 1,  ///< 确认碰撞 / 穿透
  kInvalid = 2,    ///< 检查无法成立（不是"安全"，是需要修配置）
};

inline const char* toString(CheckStatus s) {
  switch (s) {
    case CheckStatus::kFree: return "Free";
    case CheckStatus::kCollision: return "Collision";
    case CheckStatus::kInvalid: return "Invalid";
  }
  return "Unknown";
}

struct CheckResult {
  CheckStatus status = CheckStatus::kInvalid;  ///< 默认就是 Invalid（失败关闭）
  int pair_index = -1;                         ///< 碰撞对下标（kCollision 时有意义）
  std::string detail;                          ///< 碰撞对名 / Invalid 的具体原因
  double min_distance = std::numeric_limits<double>::quiet_NaN();

  bool is_free() const { return status == CheckStatus::kFree; }
  bool is_collision() const { return status == CheckStatus::kCollision; }
  bool is_invalid() const { return status == CheckStatus::kInvalid; }

  /// 只有"确认安全"才返回 true。Invalid 一律 false —— 调用方不会误放行。
  explicit operator bool() const { return is_free(); }

  static CheckResult free() { return CheckResult{CheckStatus::kFree, -1, "ok", 0.0}; }
  static CheckResult invalid(const std::string& why) {
    CheckResult r;
    r.status = CheckStatus::kInvalid;
    r.detail = why;
    return r;
  }
  static CheckResult collision(int pair, const std::string& name, double d) {
    CheckResult r;
    r.status = CheckStatus::kCollision;
    r.pair_index = pair;
    r.detail = name;
    r.min_distance = d;
    return r;
  }
};

}  // namespace hw_arm_algorithms
