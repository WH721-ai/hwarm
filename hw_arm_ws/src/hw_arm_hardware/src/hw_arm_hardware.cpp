// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_hardware/hw_arm_hardware.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hw_arm_hardware {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {

// 单调时钟秒数（用于重力斜坡）
double steady_now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

rclcpp::Logger logger() { return rclcpp::get_logger("HwArmHW"); }
rclcpp::Clock& throttle_clock() {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

bool to_bool(std::string v, bool fallback) {
  // ::tolower 对负值是 UB（非 ASCII 字符高位），先转 unsigned char
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  // ⚠️ xacro 的布尔属性会渲染成 Python 风格的 "True"/"False"，
  //    而这里原来只认 "true"/"false" —— 于是 gravity_compensation=True
  //    被当成非法值、退回 fallback=false，表现为"参数明明传了却不生效"
  //    （2026-11-26 真机踩到：日志显示"重力补偿: 关"）。
  if (v == "true" || v == "1" || v == "yes") return true;
  if (v == "false" || v == "0" || v == "no") return false;
  return fallback;
}

// MIT 协议的硬性量程（openarm_can 的 double_to_uint 会**静默 clamp**，
// 超出范围不报错，实际增益被截断 —— 所以必须在入口拦住）
constexpr double kKpMin = 0.0, kKpMax = 500.0;   // 12 bit
constexpr double kKdMin = 0.0, kKdMax = 5.0;     // 12 bit

// 安全解析：失败时报错并返回 false，不让 std::stod 抛出去炸掉 controller_manager
bool parse_double(const std::string& key, const std::string& raw, double& out) {
  try {
    size_t pos = 0;
    const double v = std::stod(raw, &pos);
    if (pos != raw.size()) throw std::invalid_argument("trailing characters");
    out = v;
    return true;
  } catch (const std::exception&) {
    RCLCPP_ERROR(logger(), "<param name=\"%s\">\"%s\" 不是合法数字", key.c_str(), raw.c_str());
    return false;
  }
}

/// 解析"1 个或 N 个用空格分隔的数"
bool parse_doubles(const std::string& key, const std::string& raw,
                   std::vector<double>& out) {
  out.clear();
  std::istringstream is(raw);
  double v;
  while (is >> v) out.push_back(v);
  if (out.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("HwArmHW"), "%s 解析失败: '%s'", key.c_str(), raw.c_str());
    return false;
  }
  return true;
}

// 硬故障：总线已经不可用，继续发帧没有意义
bool hard_bus_fault(const openarm::canbus::BusStatus& s) {
  return s.bus_off || s.error_passive || s.write_net_down;
}

// 打印总线状态（只在出问题时调用）
void log_bus_status(const openarm::canbus::BusStatus& s, const char* when) {
  RCLCPP_ERROR(logger(),
               "[总线异常 @ %s] bus_off=%lu err_passive=%lu err_warn=%lu ack_err=%lu "
               "tx_ovf=%lu rx_ovf=%lu tx_timeout=%lu net_down=%lu no_buf=%lu other=%lu "
               "err_frames=%lu ok_writes=%lu TEC=%u REC=%u last_errno=%d",
               when, static_cast<unsigned long>(s.bus_off.count),
               static_cast<unsigned long>(s.error_passive.count),
               static_cast<unsigned long>(s.error_warning.count),
               static_cast<unsigned long>(s.ack_error.count),
               static_cast<unsigned long>(s.tx_overflow.count),
               static_cast<unsigned long>(s.rx_overflow.count),
               static_cast<unsigned long>(s.tx_timeout.count),
               static_cast<unsigned long>(s.write_net_down.count),
               static_cast<unsigned long>(s.write_no_buffer.count),
               static_cast<unsigned long>(s.write_other.count),
               static_cast<unsigned long>(s.error_frames),
               static_cast<unsigned long>(s.writes_ok), s.tec, s.rec, s.last_write_errno);
}
}  // namespace

// 按空白切分（" +1 -1 -1 " → {"+1","-1","-1"}）
static std::vector<std::string> split_ws(const std::string& s) {
  std::vector<std::string> out;
  std::istringstream iss(s);
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}

// ────────────────────────────────────────────────────────── 配置解析
bool HwArmHW::parse_config(const hardware_interface::HardwareInfo& info) {
  const auto& p = info.hardware_parameters;
  auto get = [&p](const std::string& k) {
    auto it = p.find(k);
    return it == p.end() ? std::string{} : it->second;
  };

  if (!get("can_interface").empty()) can_interface_ = get("can_interface");
  // 默认 false：实测达妙电机不响应 CANFD 帧（只认经典 CAN）
  if (!get("can_fd").empty()) can_fd_ = to_bool(get("can_fd"), false);
  if (!get("require_all_motors").empty())
    require_all_motors_ = to_bool(get("require_all_motors"), false);
  if (!get("return_to_zero").empty())
    return_to_zero_ = to_bool(get("return_to_zero"), true);

  // refresh_per_cycle：每周期刷新几台电机（轮转覆盖全部）。
  // 0 = 完全不发 refresh（依赖电机自动上报）；默认 2。
  if (!get("refresh_per_cycle").empty()) {
    int v = -1;
    try { v = std::stoi(get("refresh_per_cycle")); } catch (const std::exception&) { v = -1; }
    if (v < 0 || v > 16) {
      RCLCPP_ERROR(logger(), "refresh_per_cycle=%s 非法（应为 0~16）",
                   get("refresh_per_cycle").c_str());
      return false;
    }
    refresh_per_cycle_ = v;
  }
  // 兼容旧参数名
  if (!get("refresh_divider").empty()) {
    int d = 4;
    try { d = std::stoi(get("refresh_divider")); } catch (const std::exception&) { d = 4; }
    if (d >= 1 && !active_idx_.empty()) {
      refresh_per_cycle_ = std::max<int>(1, static_cast<int>(active_idx_.size() + d - 1) / d);
      RCLCPP_WARN(logger(), "refresh_divider 已弃用，换算成 refresh_per_cycle=%d",
                  refresh_per_cycle_);
    }
  }

  // active_joints：实际接了电机的关节（1-based，逗号分隔）。默认全部 6 个。
  // 例：只有 joint1 时传 "1"；有 1~5 时传 "1,2,3,4,5"。
  active_idx_.clear();
  const std::string active_raw = get("active_joints");
  if (active_raw.empty()) {
    for (size_t i = 0; i < ARM_DOF; ++i) active_idx_.push_back(i);
  } else {
    std::stringstream ss(active_raw);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      if (tok.empty()) continue;
      try {
        const int n = std::stoi(tok);
        if (n < 1 || n > static_cast<int>(ARM_DOF)) {
          RCLCPP_ERROR(logger(), "active_joints 里的 %d 超范围 [1, %zu]", n, ARM_DOF);
          return false;
        }
        active_idx_.push_back(static_cast<size_t>(n - 1));
      } catch (const std::exception&) {
        RCLCPP_ERROR(logger(), "active_joints 含非法项 \"%s\"", tok.c_str());
        return false;
      }
    }
    std::sort(active_idx_.begin(), active_idx_.end());
    active_idx_.erase(std::unique(active_idx_.begin(), active_idx_.end()), active_idx_.end());
    if (active_idx_.empty()) {
      RCLCPP_ERROR(logger(), "active_joints 解析为空");
      return false;
    }
  }
  if (require_all_motors_ && active_idx_.size() != ARM_DOF) {
    RCLCPP_ERROR(logger(), "require_all_motors=true 但 active_joints 只给了 %zu 个", active_idx_.size());
    return false;
  }
  {
    std::string list;
    for (size_t i : active_idx_) list += (list.empty() ? "" : ",") + std::to_string(i + 1);
    RCLCPP_INFO(logger(), "  实际挂载的关节: [%s]（%zu/%zu）", list.c_str(), active_idx_.size(), ARM_DOF);
  }

  // 增益：URDF 没给 → 用内置默认值但打 WARN（不静默）；
  //       给了 → 安全解析 + 范围校验（超范围直接失败，避免被驱动静默截断）
  for (size_t i = 0; i < ARM_DOF; ++i) {
    const std::string kp_key = "kp" + std::to_string(i + 1);
    const std::string kd_key = "kd" + std::to_string(i + 1);

    if (p.count(kp_key)) {
      if (!parse_double(kp_key, get(kp_key), kp_[i])) return false;
      if (kp_[i] < kKpMin || kp_[i] > kKpMax) {
        RCLCPP_ERROR(logger(), "%s=%.3f 超出 MIT 协议范围 [%.0f, %.0f]，会被驱动静默截断",
                     kp_key.c_str(), kp_[i], kKpMin, kKpMax);
        return false;
      }
    } else {
      RCLCPP_WARN(logger(), "%s 未在 URDF 中提供，使用内置默认值 %.3f", kp_key.c_str(), kp_[i]);
    }

    if (p.count(kd_key)) {
      if (!parse_double(kd_key, get(kd_key), kd_[i])) return false;
      if (kd_[i] < kKdMin || kd_[i] > kKdMax) {
        RCLCPP_ERROR(logger(), "%s=%.3f 超出 MIT 协议范围 [%.0f, %.0f]，会被驱动静默截断",
                     kd_key.c_str(), kd_[i], kKdMin, kKdMax);
        return false;
      }
    } else {
      RCLCPP_WARN(logger(), "%s 未在 URDF 中提供，使用内置默认值 %.3f", kd_key.c_str(), kd_[i]);
    }

    // ── direction：电机转向（C-02）
    // 支持两种写法：
    //   <param name="direction">+1 -1 -1 +1 +1 +1</param>   ← 一个参数覆盖全部
    //   <param name="direction1">+1</param> ...             ← 逐个给（优先）
    {
      const std::string one = "direction" + std::to_string(i + 1);
      std::string txt;
      if (p.count(one)) {
        txt = get(one);
      } else if (direction_vec_.empty() && p.count("direction")) {
        direction_vec_ = split_ws(get("direction"));   // 懒解析一次
        if (direction_vec_.size() != ARM_DOF) {
          RCLCPP_ERROR(logger(), "direction 需要 %zu 个值，实际 %zu 个：\"%s\"",
                       ARM_DOF, direction_vec_.size(), get("direction").c_str());
          return false;
        }
      }
      if (!txt.empty()) {
        double v = 0.0;
        if (!parse_double(one, txt, v)) return false;
        direction_[i] = v;
      } else if (!direction_vec_.empty()) {
        double v = 0.0;
        if (!parse_double(one, direction_vec_[i], v)) return false;
        direction_[i] = v;
      } else {
        RCLCPP_WARN(logger(),
                    "%s 未在 URDF 中提供，使用内置默认值 %+.0f"
                    "（先用 scripts/mirror_check.py 实测，别猜）",
                    one.c_str(), direction_[i]);
      }
      if (direction_[i] != 1.0 && direction_[i] != -1.0) {
        RCLCPP_ERROR(logger(), "%s=%+.0f 非法，只能是 +1 或 -1",
                     one.c_str(), direction_[i]);
        return false;
      }
    }
  }

  // ── CAN ID 映射（电机装反了但不想改固件 ID 时用）
  //
  //   <param name="joint_can_ids">1 2 3 4 6 5</param>
  //       joint1..joint6 各自用哪个电机发送 ID，接收 ID 自动 = 发送 ID + 0x10
  //   <param name="joint_master_ids">0x11 0x12 0x13 0x14 0x16 0x15</param>
  //       显式指定接收 ID（Master ID 不等于 +0x10 时才需要）
  if (p.count("joint_can_ids")) {
    const auto toks = split_ws(get("joint_can_ids"));
    if (toks.size() != ARM_DOF) {
      RCLCPP_ERROR(logger(), "joint_can_ids 需要 %zu 个值，实际 %zu 个：\"%s\"",
                   ARM_DOF, toks.size(), get("joint_can_ids").c_str());
      return false;
    }
    std::vector<uint32_t> ids;
    for (const auto& t : toks) {
      char* end = nullptr;
      const unsigned long v = std::strtoul(t.c_str(), &end, 0);  // 支持 5 和 0x05
      if (end == t.c_str() || v == 0 || v > 0x7FF) {
        RCLCPP_ERROR(logger(), "joint_can_ids 里有非法 ID：\"%s\"", t.c_str());
        return false;
      }
      ids.push_back(static_cast<uint32_t>(v));
    }
    // 检查重复：同一个 ID 出现两次会互相打架
    for (size_t a = 0; a < ids.size(); ++a)
      for (size_t b = a + 1; b < ids.size(); ++b)
        if (ids[a] == ids[b]) {
          RCLCPP_ERROR(logger(), "joint_can_ids 有重复 ID 0x%02X（关节 %zu 和 %zu）",
                       ids[a], a + 1, b + 1);
          return false;
        }
    send_can_ids_ = ids;
    for (size_t i = 0; i < ARM_DOF; ++i) recv_can_ids_[i] = send_can_ids_[i] + 0x10;
    RCLCPP_INFO(logger(), "CAN ID 映射（来自 joint_can_ids）:");
    for (size_t i = 0; i < ARM_DOF; ++i)
      RCLCPP_INFO(logger(), "  joint%zu → 发送 0x%02X / 接收 0x%02X",
                  i + 1, send_can_ids_[i], recv_can_ids_[i]);
  }
  if (p.count("joint_master_ids")) {
    const auto toks = split_ws(get("joint_master_ids"));
    if (toks.size() != ARM_DOF) {
      RCLCPP_ERROR(logger(), "joint_master_ids 需要 %zu 个值，实际 %zu 个",
                   ARM_DOF, toks.size());
      return false;
    }
    for (size_t i = 0; i < ARM_DOF; ++i) {
      char* end = nullptr;
      const unsigned long v = std::strtoul(toks[i].c_str(), &end, 0);
      if (end == toks[i].c_str() || v == 0 || v > 0x7FF) {
        RCLCPP_ERROR(logger(), "joint_master_ids 里有非法 ID：\"%s\"", toks[i].c_str());
        return false;
      }
      recv_can_ids_[i] = static_cast<uint32_t>(v);
    }
  }

  // ── 重力补偿（C-04）
  if (!get("gravity_compensation").empty())
    gravity_comp_ = to_bool(get("gravity_compensation"), false);
  if (!get("gravity_dry_run").empty())
    gravity_dry_run_ = to_bool(get("gravity_dry_run"), true);
  if (!get("gravity_scale").empty()) {
    double v = 1.0;
    if (!parse_double("gravity_scale", get("gravity_scale"), v)) return false;
    if (v < 0.0 || v > 1.5) {
      RCLCPP_ERROR(logger(), "gravity_scale=%.3f 超出 [0, 1.5]", v);
      return false;
    }
    gravity_scale_ = v;
  }
  if (!get("zero_gravity").empty())
    zero_gravity_ = to_bool(get("zero_gravity"), false);
  if (!get("zero_gravity_kd").empty()) {
    double v = 1.0;
    if (!parse_double("zero_gravity_kd", get("zero_gravity_kd"), v)) return false;
    if (v < 0.0 || v > 5.0) {
      RCLCPP_ERROR(logger(), "zero_gravity_kd=%.3f 超出 MIT 范围 [0, 5]", v);
      return false;
    }
    zero_gravity_kd_ = v;
    zero_gravity_kd_v_.fill(v);
  }
  if (!get("zero_gravity_kd_each").empty()) {
    std::vector<double> v;
    if (!parse_doubles("zero_gravity_kd_each", get("zero_gravity_kd_each"), v)) return false;
    if (v.size() == 1) v.assign(ARM_DOF, v[0]);
    if (v.size() != ARM_DOF) {
      RCLCPP_ERROR(logger(), "zero_gravity_kd_each 需要 1 或 6 个值，给了 %zu", v.size());
      return false;
    }
    for (size_t i = 0; i < ARM_DOF; ++i) {
      if (v[i] < 0.0 || v[i] > 5.0) {
        RCLCPP_ERROR(logger(), "zero_gravity_kd_each[%zu]=%.2f 超出 MIT 范围 [0,5]", i + 1, v[i]);
        return false;
      }
      zero_gravity_kd_v_[i] = v[i];
    }
  }
  if (!get("zero_gravity_kp_floor").empty()) {
    std::vector<double> v;
    if (!parse_doubles("zero_gravity_kp_floor", get("zero_gravity_kp_floor"), v)) return false;
    if (v.size() == 1) v.assign(ARM_DOF, v[0]);
    if (v.size() != ARM_DOF) {
      RCLCPP_ERROR(logger(), "zero_gravity_kp_floor 需要 1 或 6 个值，给了 %zu", v.size());
      return false;
    }
    for (size_t i = 0; i < ARM_DOF; ++i) {
      if (v[i] < 0.0 || v[i] > 50.0) {
        RCLCPP_ERROR(logger(), "zero_gravity_kp_floor[%zu]=%.2f 超出 [0, 50]", i + 1, v[i]);
        return false;
      }
      zero_gravity_kp_floor_[i] = v[i];
    }
  }
  if (!get("zero_gravity_allow_low_pose").empty()) {
    const std::string s = get("zero_gravity_allow_low_pose");
    zero_gravity_allow_low_pose_ = (s == "true" || s == "1" || s == "True");
  }

  if (zero_gravity_) {
    // ══ 2026-11-26 实测后的结论：**开机就进零重力是不安全的** ══
    //
    // joint3 扫掠实测（algo/data/joint3_sweep.yaml）：
    //   q3=  0°  τ实测 0.40  模型 6.09   残差 -5.69  ← 被结构托住
    //   q3= 20°  τ实测 5.38  模型 5.47   残差 -0.43  ← 自由，模型吻合
    //   q3= 55°  τ实测 2.65  模型 3.14   残差 -0.48
    // joint2 更极端：它的 URDF 下限是 -2.10°，而 q2≈0 时只有 1.4° 余量，
    // 整场都坐在机械限位上，电机只出 0.3 N·m 而模型说要 2.4~5.6。
    //
    // 也就是说：零位（return_to_zero 的目的地）恰恰是**补偿误差最大**的位形。
    // 在那儿进零重力，tau_ff 会比真实需要多出 [2.1, 5.7] N·m，
    // 按 kd=1.5 算稳态漂移 80 / 217 °/s —— 机械臂会猛地往上冲。
    //
    // ⇒ 启动时不再允许直接进零重力。必须先位置控制走到自由位形，
    //   再调用服务 ~/set_zero_gravity 进入。
    RCLCPP_ERROR(logger(),
                 "★ 拒绝在启动时进入零重力：零位是机械结构托住的位形，"
                 "那里模型的补偿误差最大（实测 joint2 +2.1 / joint3 +5.7 N·m）。");
    RCLCPP_ERROR(logger(),
                 "  正确流程：① 先以位置控制启动（zero_gravity:=false）"
                 "  ② 走到自由位形（建议 q2 ≥ 30°、q3 ≥ 20°，离开机械限位）");
    RCLCPP_ERROR(logger(),
                 "            ③ ros2 service call ~/set_zero_gravity "
                 "std_srvs/srv/SetBool \"{data: true}\"");
    RCLCPP_ERROR(logger(), "  已自动改为位置控制模式启动。");
    zero_gravity_ = false;          // 强制关掉，走位置控制
    gravity_comp_ = true;           // 但重力补偿照常开着（位置控制下更省力）
    gravity_dry_run_ = false;
  }
  if (!get("urdf_file").empty()) urdf_file_ = get("urdf_file");

  // ── 拖动示教 / 重力补偿的安全参数（C-05 之后新增）
  if (!get("gravity_params_path").empty()) gravity_params_path_ = get("gravity_params_path");

  // 恒定力矩偏置修正（N·m），例如 "0 0.30 -0.55 0 0 0"
  if (!get("gravity_trim").empty()) {
    std::vector<double> v;
    for (const auto& tok : split_ws(get("gravity_trim"))) {
      double d = 0.0;
      if (!parse_double("gravity_trim", tok, d)) return false;
      v.push_back(d);
    }
    if (v.size() != ARM_DOF) {
      RCLCPP_ERROR(logger(), "gravity_trim 需要 %zu 个数，实际 %zu", ARM_DOF, v.size());
      return false;
    }
    gravity_trim_.assign(v.begin(), v.end());
    RCLCPP_WARN(logger(), "恒定力矩偏置修正: [%.3f %.3f %.3f %.3f %.3f %.3f] N·m",
                gravity_trim_[0], gravity_trim_[1], gravity_trim_[2],
                gravity_trim_[3], gravity_trim_[4], gravity_trim_[5]);
  }
  if (!get("gravity_ramp_time").empty()) {
    double v = 3.0;
    if (!parse_double("gravity_ramp_time", get("gravity_ramp_time"), v)) return false;
    if (v < 0.0 || v > 30.0) {
      RCLCPP_ERROR(logger(), "gravity_ramp_time=%.2f 超出 [0, 30] s", v);
      return false;
    }
    gravity_ramp_time_ = v;
  }
  if (!get("tau_limit").empty()) {
    std::vector<double> v;
    for (const auto& tok : split_ws(get("tau_limit"))) {
      double d = 0.0;
      if (!parse_double("tau_limit", tok, d)) return false;
      v.push_back(d);
    }
    if (v.size() != ARM_DOF) {
      RCLCPP_ERROR(logger(), "tau_limit 需要 %zu 个数，实际 %zu", ARM_DOF, v.size());
      return false;
    }
    for (double x : v)
      if (x <= 0.0) { RCLCPP_ERROR(logger(), "tau_limit 必须为正"); return false; }
    tau_limit_.assign(v.begin(), v.end());
  }
  if (!get("zero_gravity_vel_limit").empty()) {
    double v = 2.0;
    if (!parse_double("zero_gravity_vel_limit", get("zero_gravity_vel_limit"), v)) return false;
    if (v <= 0.0 || v > 20.0) {
      RCLCPP_ERROR(logger(), "zero_gravity_vel_limit=%.2f 超出 (0, 20]", v);
      return false;
    }
    zero_gravity_vel_limit_ = v;
  }

  if (zero_gravity_) {
    RCLCPP_INFO(logger(), "模式: **零重力拖动示教**（kp=0, kd=%.2f）", zero_gravity_kd_);
  }
  if (gravity_comp_) {
    // 默认路径：<share>/hw_arm_description/urdf/hw_arm.urdf（构建期生成）
    if (urdf_file_.empty()) {
      try {
        urdf_file_ = ament_index_cpp::get_package_share_directory("hw_arm_description") +
                     "/urdf/hw_arm.urdf";
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "找不到 hw_arm_description 包: %s", e.what());
        return false;
      }
    }
    model_ = std::make_unique<hw_arm_algorithms::RobotModel>();
    if (!model_->load(urdf_file_)) {
      RCLCPP_ERROR(logger(), "加载 URDF 失败: %s", urdf_file_.c_str());
      for (const auto& e : model_->validation().errors)
        RCLCPP_ERROR(logger(), "  %s", e.c_str());
      model_.reset();
      return false;
    }
    for (const auto& w : model_->validation().warnings)
      RCLCPP_WARN(logger(), "URDF 校验: %s", w.c_str());

    ws_ = std::make_unique<hw_arm_algorithms::RobotWorkspace>(*model_);
    gravity_tau_.assign(ARM_DOF, 0.0);

      // ── 加载辨识出的重力参数（C-05）
      //   给了 gravity_params_path 就必须读成功，否则直接返回 false ——
      //   不能静默退回标称 CAD 参数（残差 0.88 N·m 是辨识后 0.18 的 5 倍），
      //   否则使用者以为自己加载了辨识结果，其实用的是 CAD 参数。
      if (!gravity_params_path_.empty()) {
        std::string err;
        if (!hw_arm_algorithms::loadGravityParams(gravity_params_path_, gparams_, &err)) {
          RCLCPP_ERROR(logger(), "读不到重力参数 %s：%s",
                       gravity_params_path_.c_str(), err.c_str());
          return false;
        }
        if (gparams_.theta.size() != static_cast<size_t>(model_->nv()) * 10) {
          RCLCPP_ERROR(logger(), "重力参数维度不符：theta=%zu，期望 %d",
                       gparams_.theta.size(), model_->nv() * 10);
          return false;
        }
        RCLCPP_INFO(logger(),
                    "重力参数: %s\\n  rank=%d  rms=%.4f  holdout=%.4f  偏移 %zu / 摩擦 %zu",
                    gravity_params_path_.c_str(), gparams_.rank, gparams_.rms,
                    gparams_.holdout_rms, gparams_.offsets.size(),
                    gparams_.friction.size());
      } else {
        RCLCPP_WARN(logger(),
                    "未指定 gravity_params_path —— 重力前馈用 URDF 标称惯性参数");
      }
      if (gravity_ramp_time_ > 0.0)
        RCLCPP_INFO(logger(), "重力斜坡: %.1f s", gravity_ramp_time_);

    RCLCPP_INFO(logger(),
                "重力补偿: 开（scale=%.2f, dry_run=%s）URDF=%s 总质量=%.4f kg",
                gravity_scale_, gravity_dry_run_ ? "true" : "false",
                urdf_file_.c_str(), model_->totalMass());
    {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq());
      Eigen::VectorXd g(model_->nv());
      // ⚠️ 这里打的是**标称 URDF**（CAD 惯性）的重力矩，不是 θ̂ 的。
      //    2026-11-26 发现：日志只写 "g(q=0)=...（可用于核对期望值）"，
      //    很容易被误读成"θ̂ 模型在零位的预测"。
      //    q=0 处两者差很多（joint3：标称 4.02 vs θ̂ 6.09），
      //    拿标称值去和实测比会得出完全错误的结论。
      hw_arm_algorithms::computeGravity(*model_, *ws_, q, g);
      RCLCPP_INFO(logger(),
                  "  g(q=0) 标称URDF = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] N·m",
                  g[0], g[1], g[2], g[3], g[4], g[5]);
      if (gparams_.valid && gparams_.theta.size() == model_->nv() * 10) {
        const Eigen::VectorXd off =
            gparams_.offsets.size() == model_->nv() ? gparams_.offsets
                                                    : Eigen::VectorXd::Zero(model_->nv());
        Eigen::VectorXd gid(model_->nv());
        hw_arm_algorithms::computeGravityFromParamsAndOffset(
            *model_, *ws_, q, gparams_.theta, off, gid);
        RCLCPP_INFO(logger(),
                    "  g(q=0) θ̂模型    = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] N·m"
                    "  ← 这才是实际下发的前馈基础",
                    gid[0], gid[1], gid[2], gid[3], gid[4], gid[5]);
      }
    }
  } else {
    RCLCPP_INFO(logger(), "重力补偿: 关（tau_ff 只来自控制器）");
  }

  RCLCPP_INFO(logger(),
              "配置: can=%s can_fd=%s require_all=%s return_to_zero=%s refresh_per_cycle=%d",
              can_interface_.c_str(), can_fd_ ? "true" : "false",
              require_all_motors_ ? "true" : "false", return_to_zero_ ? "true" : "false",
              refresh_per_cycle_);
  return true;
}

// 关节名直接取 URDF 声明（不再用 "openarm_" + prefix + "jointN" 拼接）
void HwArmHW::resolve_joint_names(const hardware_interface::HardwareInfo& info) {
  joint_names_.clear();
  for (const auto& j : info.joints) joint_names_.push_back(j.name);
}

// ────────────────────────────────────────────────────────── on_init
CallbackReturn HwArmHW::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!parse_config(info)) return CallbackReturn::ERROR;

  resolve_joint_names(info);

  if (joint_names_.size() != ARM_DOF) {
    RCLCPP_ERROR(logger(), "URDF 声明了 %zu 个关节，本插件固定 6 轴（HwArmHW::ARM_DOF=%zu）",
                 joint_names_.size(), ARM_DOF);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    RCLCPP_INFO(logger(), "  joint%zu: %s", i + 1, joint_names_[i].c_str());
  }

  RCLCPP_INFO(logger(), "打开 %s（CAN-FD %s）…", can_interface_.c_str(),
              can_fd_ ? "on" : "off");
  try {
    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "打开 CAN 接口失败: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // 只把「实际存在的电机」注册进去：openarm_can 的所有 _all 操作都作用在这批设备上，
  // 因此不会对空 ID 发帧（无 ACK 会推高 TEC 直到 bus-off）
  {
    std::vector<openarm::damiao_motor::MotorType> types;
    std::vector<uint32_t> send_ids, recv_ids;
    for (size_t i : active_idx_) {
      types.push_back(MOTOR_TYPES[i]);
      send_ids.push_back(send_can_ids_[i]);
      recv_ids.push_back(recv_can_ids_[i]);
    }
    openarm_->init_arm_motors(types, send_ids, recv_ids);
  }

  pos_commands_.assign(ARM_DOF, 0.0);
  vel_commands_.assign(ARM_DOF, 0.0);
  tau_commands_.assign(ARM_DOF, 0.0);
  pos_states_.assign(ARM_DOF, 0.0);
  vel_states_.assign(ARM_DOF, 0.0);
  tau_states_.assign(ARM_DOF, 0.0);

  // 构建身份：日志本身成为"跑的是哪个提交"的证据
  RCLCPP_INFO(logger(), "构建身份: %s  源码指纹 %s  @ %sZ",
              HW_ARM_GIT_COMMIT, HW_ARM_SRC_HASH, HW_ARM_BUILD_TIME);
  RCLCPP_INFO(logger(), "HwArmHW 初始化完成");
  return CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────── on_configure
CallbackReturn HwArmHW::on_configure(const rclcpp_lifecycle::State& /*prev*/) {
  if (!openarm_) return CallbackReturn::ERROR;
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  openarm_->recv_all();
  return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────── export interfaces
std::vector<hardware_interface::StateInterface> HwArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(ARM_DOF * 3);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> HwArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(ARM_DOF * 3);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]);
  }
  return out;
}

// ─────────────────────────────────────────────────────── on_activate
CallbackReturn HwArmHW::on_activate(const rclcpp_lifecycle::State& /*prev*/) {
  if (!openarm_) return CallbackReturn::ERROR;

  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

  // ⚠️ 关键：必须先确认能读到反馈，**再**使能。
  // 之前直接 enable_all() 的问题：此刻还没收到任何状态帧，Motor 里的 position 还是 0，
  // 于是"把命令初始化为当前位置"等于把命令初始化成 0 —— 一使能电机就被驱动到 0 位。
  // 实测就是这么把 6 台电机拉回 0 的。
  // ★ R6：逐轴校验，而不是"任一轴有响应就使能全部"。
  //
  // 旧逻辑 `position != 0 || error_code != 0` 有三个问题：
  //   ① 任一轴响应就 got_feedback=true，其余轴可能一次都没回包；
  //   ② 未响应轴的缓存是 0，使能后会拿到**基于假零位**的命令；
  //   ③ 位置合法的 0 会被当成"没反馈"，而故障码非 0 反而被当成"有反馈"。
  //
  // 新逻辑：每个要用的轴都必须 ever_responded() + 未过期 + 无故障码。
  std::vector<bool> axis_ok(active_idx_.size(), false);
  bool all_ready = false;
  for (int attempt = 0; attempt < 20 && !all_ready; ++attempt) {
    openarm_->refresh_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    openarm_->recv_all(5000);

    const auto& ms = openarm_->get_arm().get_motors();
    all_ready = true;
    for (size_t k = 0; k < active_idx_.size(); ++k) {
      bool ok = false;
      if (k < ms.size()) {
        const auto& st = openarm_->get_arm().get_link_stats(static_cast<int>(k));
        const int err = ms[k].get_error_code();
        ok = st.ever_responded() &&
             !st.is_stale(std::chrono::milliseconds(stale_warn_ms_)) &&
             (err < 0x8);
      }
      axis_ok[k] = ok;
      if (!ok) all_ready = false;
    }
  }

  if (!all_ready) {
    // 逐轴点名，指出到底哪几轴没就绪
    for (size_t k = 0; k < axis_ok.size(); ++k) {
      if (axis_ok[k]) continue;
      const size_t j = active_idx_[k];
      RCLCPP_ERROR(logger(), "  轴 %zu (%s) 未就绪：无反馈 / 反馈过期 / 有故障码",
                   j, k < joint_names_.size() ? joint_names_[j].c_str() : "?");
    }
  }
  if (!all_ready) {
    // ⚠️ 以前这里 return ERROR —— 然后 controller_manager 会抛
    //    std::runtime_error("Failed to set the initial state ... to active")，
    //    整个 ros2_control_node **SIGABRT 退出**（exit code -6），
    //    joint_state_broadcaster 也跟着没了 → RViz 一堆连杆叠在原点。
    //
    //    现在改成「软失败」：硬件保持 active 但**不使能电机**，
    //    read/write 照常跑（只读不写），这样：
    //      · RViz / joint_states 正常运行，方便排查
    //      · 电机一动不动（安全）
    //      · 日志里有明确的错误 + 总线计数
    motors_ready_ = false;
    log_bus_status(openarm_->get_bus_status(), "on_activate");
    RCLCPP_ERROR(logger(),
                 "%zu 台电机中有轴未就绪（%d 次尝试 / %.1f 秒）。"
                 "**电机保持失能**，不会动。",
                 active_idx_.size(), 20, 1.0);
    RCLCPP_ERROR(logger(),
                 "  排查：can0 是否 UP（ip -br link show can0）、bitrate 是否 1M、"
                 "电机是否上电、CAN_H/CAN_L 是否接反、终端电阻 120Ω");
    RCLCPP_ERROR(logger(),
                 "  手动验证：python3 src/hw_arm_description/scripts/scan_motors.py");
    return CallbackReturn::SUCCESS;   // ← 不再让整个节点崩掉
  }
  motors_ready_ = true;

  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // 打印每台已注册电机的初始状态
  const auto& motors = openarm_->get_arm().get_motors();
  for (size_t k = 0; k < active_idx_.size() && k < motors.size(); ++k) {
    const size_t j = active_idx_[k];
    // ⚠️ 这里必须和 read() 一样做「电机坐标系 → 关节坐标系」转换。
    //    以前直接写 pos_states_[j] = get_position()（电机坐标），
    //    而 read() 写的是 direction * get_position()（关节坐标）。
    //    结果：joint2/joint3（direction=-1）激活时 pos_commands_ 是镜像值，
    //    write() 再乘一次 direction，目标就变成了 -q_motor。
    //    姿态接近零位时误差很小看不出来；一旦在大角度激活，
    //    误差可达 2·|q|，kp=70 时就是上百 N·m —— "猛冲"的来源之一。
    const double d = direction_[j];
    const double q = d * motors[k].get_position();       // 关节坐标系
    const int err = motors[k].get_error_code();
    pos_states_[j] = q;
    // 关键安全措施：命令必须初始化为「当前实测位置」。
    // 否则 ros2_control 的命令缓冲区是 0，使能后的第一帧 write() 会把 0 发下去，
    // 电机会以 kp * |q| 的力矩猛冲回零（kp=70、q=1.1rad 时就是 79 N·m）。
    pos_commands_[j] = q;
    vel_commands_[j] = 0.0;
    tau_commands_[j] = 0.0;
    RCLCPP_INFO(logger(), "  %s: q=%.4f err=0x%X（命令已初始化为当前位置）",
                joint_names_[j].c_str(), q, err);
  }
  RCLCPP_INFO(logger(), "已使能 %zu 台电机", active_idx_.size());

  if (return_to_zero_) {
    return_to_zero();
  }

  // G02：激活时刻必须记下来，否则 ramp 恒为满额。
  //   放在 return_to_zero() **之后**：回零是个阻塞的 100 步插值，
  //   若在它之前起算，回零结束时斜坡可能已经满额，正好在操作者以为
  //   "还有 5 秒可以扶稳"的时候把重力全部加上。
  ramp_t0_ = steady_now();

  // ── 运行时切换零重力的服务
  //   用法：
  //     ros2 service call /hw_arm_hardware_interface/set_zero_gravity \
  //         std_srvs/srv/SetBool "{data: true}"    # 进入
  //         ... "{data: false}"                    # 退出
  if (!zero_g_srv_ && get_node()) {
    zero_g_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/set_zero_gravity",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          // ══ G03：进入零重力必须先满足条件 ══
          //  否则会出现"kp 被关掉、却没有任何重力前馈"的状态 —— 机械臂直接掉。
          if (req->data) {
            if (!gravity_comp_) { res->success=false; res->message="拒绝：重力补偿未开启"; 
              RCLCPP_ERROR(logger(), "%s", res->message.c_str()); return; }
            if (gravity_dry_run_) { res->success=false; res->message="拒绝：处于 dry-run"; 
              RCLCPP_ERROR(logger(), "%s", res->message.c_str()); return; }
            if (!gparams_.valid) { res->success=false; res->message="拒绝：重力参数未加载"; 
              RCLCPP_ERROR(logger(), "%s", res->message.c_str()); return; }
            if (!motors_ready_) { res->success=false; res->message="拒绝：电机未就绪"; 
              RCLCPP_ERROR(logger(), "%s", res->message.c_str()); return; }

              // ※ 2026-11-28 事故后加：位形检查
              //
              //   q2/q3 接近 0 时机械臂坐在机械限位或结构上，电机实际只输出
              //   0.28~0.40 N·m，而模型预测 2.4 / 6.1 N·m。此时进零重力会
              //   多给 +2.0 / +5.7 N·m，机械臂必然猛抬（11-27 现场发生过两次）。
              //
              //   阈值按"模型可信"的前提设：|q2| < 10° 或 |q3| < 15° 直接拒绝。
              //   确实要研究零位行为的场合，把 zero_gravity_allow_low_pose 设 true。
              if (!zero_gravity_allow_low_pose_) {
                const double q2_deg = std::abs(pos_states_[1]) * 57.29577951308232;
                const double q3_deg = std::abs(pos_states_[2]) * 57.29577951308232;
                if (q2_deg < 10.0 || q3_deg < 15.0) {
                  char buf[256];
                  std::snprintf(buf, sizeof(buf),
                      "拒绝：位形太靠近零位（q2=%.1f deg, q3=%.1f deg；要求 |q2|>=10 且 |q3|>=15）。"
                      "零位附近机械臂被结构托住，模型会大幅高估负载，进零重力会猛抬。"
                      "请先 goto_pose 到自由位形。",
                      q2_deg, q3_deg);
                  res->success = false; res->message = buf;
                  RCLCPP_ERROR(logger(), "%s", buf);
                  return;
                }
              }
          }
          // 切换前把位置命令对齐到当前位形：退出时机械臂停在原地不会弹回去
          for (size_t j = 0; j < ARM_DOF; ++j) pos_commands_[j] = pos_states_[j];
          // 人工显式操作 = 确认已排除故障，清除锁存
          fault_latched_ = false;
          zero_gravity_srv_mode_ = req->data;
          // 复位过渡计时：重新走"前馈先建立、再撤 kp"的两段式
          ramp_t0_ = steady_now();
          RCLCPP_WARN(logger(), "★ %s零重力（kp %s，过渡 %.1f s）",
                      req->data ? "进入" : "退出",
                      req->data ? "→ 0，只靠 tau_ff 支撑" : "恢复，位置环重新接管",
                      gravity_ramp_time_ * (req->data ? 2.0 : 1.0));
          res->success = true;
          res->message = req->data ? "zero-gravity ON" : "zero-gravity OFF";
        });
  } else {
    // 不自动回零时，把当前位姿作为初始命令，避免一上电就往 0 冲
    for (size_t i = 0; i < ARM_DOF; ++i) {
      pos_commands_[i] = pos_states_[i];
      vel_commands_[i] = 0.0;
      tau_commands_[i] = 0.0;
    }
    openarm_->refresh_all();
    openarm_->recv_all();
  }
  return CallbackReturn::SUCCESS;
}

// ───────────────────────────────────────────────────── on_deactivate
CallbackReturn HwArmHW::on_deactivate(const rclcpp_lifecycle::State& /*prev*/) {
  if (openarm_) {
    for (int i = 0; i < 3; ++i) {
      openarm_->disable_all();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      openarm_->recv_all();
    }
  }
  return CallbackReturn::SUCCESS;
}

// ───────────────────────────────────────────────────────────── read
return_type HwArmHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!openarm_) return return_type::ERROR;

  // 状态刷新：每周期刷新 refresh_per_cycle_ 台，轮转覆盖，避免突发
  if (refresh_per_cycle_ > 0 && !active_idx_.empty()) {
    const size_t n = active_idx_.size();
    const size_t cnt = std::min<size_t>(static_cast<size_t>(refresh_per_cycle_), n);
    for (size_t k = 0; k < cnt; ++k) {
      openarm_->refresh_one(static_cast<int>((cycle_count_ + k) % n));
    }
  }
  ++cycle_count_;
  openarm_->recv_all();

  // ① 总线级：bus-off / error-passive / 接口掉线 —— 再发帧也没意义
  if (hard_bus_fault(openarm_->get_bus_status())) {
    RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 2000,
                          "CAN 总线硬故障，请求去激活");
    log_bus_status(openarm_->get_bus_status(), "read");
    state_stale_any_ = true;
    fault_latched_ = true;   // 硬故障同样锁存
    return return_type::ERROR;
  }
  // ② 软故障（ack error / FIFO 溢出）：只告警，不中断控制
  if (!openarm_->is_bus_healthy()) {
    RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000, "CAN 总线有错误计数，详见日志");
    log_bus_status(openarm_->get_bus_status(), "read(warn)");
  }

  // ③ 电机级：把状态按 active_idx_ 映射回关节下标
  //
  // ★ R7：先查**逐轴反馈时效**再采信数据。
  //   只查总线状态不够：CAN 上其他节点还在 ACK 时，掉线的那个轴不会触发
  //   bus-off，它的 Motor 缓存里仍是旧值 —— 旧位置会被当成新位置发布。
  if (axis_stale_latched_.size() != active_idx_.size()) {
    axis_stale_latched_.assign(active_idx_.size(), false);
  }

  const auto& motors = openarm_->get_arm().get_motors();
  bool any_stale = false;
  state_stale_any_ = false;   // 每周期重新判定
  for (size_t k = 0; k < active_idx_.size(); ++k) {
    const size_t j = active_idx_[k];
    const bool has = (k < motors.size());
    const bool stale =
        !has || openarm_->get_arm()
                    .get_link_stats(static_cast<int>(k))
                    .is_stale(std::chrono::milliseconds(stale_warn_ms_));

    if (stale) {
      state_stale_any_ = true;
      fault_latched_ = true;   // 锁存，恢复后不自动重开前馈
      any_stale = true;
      if (!axis_stale_latched_[k]) {
        axis_stale_latched_[k] = true;
        RCLCPP_ERROR(logger(),
                     "%s 反馈过期（>%ld ms 没有新的状态帧）。"
                     "该轴的 pos/vel/tau 状态不再可信。",
                     joint_names_[j].c_str(), (long)stale_warn_ms_);
      }
      continue;  // 不更新该轴状态，保留旧值仅供观察，但整体会返回 ERROR
    }
    axis_stale_latched_[k] = false;

    // 电机坐标系 → 关节坐标系（C-02）
    const double d = direction_[j];
    pos_states_[j] = d * motors[k].get_position();
    vel_states_[j] = d * motors[k].get_velocity();
    tau_states_[j] = d * motors[k].get_torque();

    const int err = motors[k].get_error_code();
    if (err >= 0x8) {  // 0x0 DISABLED / 0x1 ENABLED 之外都是故障
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 1000,
                            "%s 报错 0x%X（0x8+ 为故障），请求去激活",
                            joint_names_[j].c_str(), err);
      return return_type::ERROR;
    }
  }

  if (any_stale) {
    // 交给 ros2_control 的错误链：控制器 INACTIVE → 硬件 ERROR → on_error，
    // on_deactivate 里会把电机失能（不会带着假状态继续跑）。
    return return_type::ERROR;
  }
  return return_type::OK;
}

// ──────────────────────────────────────────────────────────── write
return_type HwArmHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  if (!openarm_) return return_type::ERROR;
  // 电机没就绪（读不到反馈 → 没使能）时，一个 MIT 帧都不发
  if (!motors_ready_) return return_type::OK;

  // 只给实际存在的电机发指令（顺序必须与 init_arm_motors 的注册顺序一致）
  std::vector<openarm::damiao_motor::MITParam> params;
  params.reserve(active_idx_.size());
  // ── 重力补偿（C-04）：tau_ff = scale * g(q_actual)
  //
  // 用**实测关节角**而不是命令角：命令角在轨迹跟踪时超前，
  // 用实测角算出来的重力矩才是此刻真正需要的补偿量。
  const bool compute_gravity = gravity_comp_ && model_ && ws_;
  const bool apply_gravity = compute_gravity && !gravity_dry_run_;
  // ── 过渡状态机（2026-11-26 审阅后重做）
  //
  // 旧做法：从 ramp_t0_ 起算 t，tau_scale = t/T。服务每次切换都重置
  // ramp_t0_，于是：
  //   · 已稳定位置保持（tau=1）→ 请求零重力：前馈 1→0 再爬回 1
  //     ⇒ 切换瞬间把已建立的承重力矩扔掉，机械臂先沉一下
  //   · ramp_time=0 时整个分支被跳过，kp_scale 保持 1
  //     ⇒ 名义"零重力"实际 kp 仍是满的
  //
  // 现在：持久状态 + 速率逼近
  //   · tau_scale_ 一旦建立就保持 1，切模式不会清零
  //   · kp 只在 tau_scale_ 到 1 之后才开始下降（两阶段）
  //   · ramp_time<=0 直接跳到目标
  {
    const double dt = period.seconds() > 0.0 ? period.seconds() : 0.008;
    const bool zg = (zero_gravity_ || zero_gravity_srv_mode_);
    const double tau_target = compute_gravity ? 1.0 : 0.0;
    const double kp_target = (zg && tau_scale_ >= 1.0 - 1e-9) ? 0.0 : 1.0;
    if (gravity_ramp_time_ > 0.0) {
      const double rate = dt / gravity_ramp_time_;
      tau_scale_ += std::clamp(tau_target - tau_scale_, -rate, rate);
      if (tau_scale_ >= 1.0 - 1e-9) {
        kp_scale_ += std::clamp(kp_target - kp_scale_, -rate, rate);
      }
    } else {
      tau_scale_ = tau_target;
      kp_scale_ = zg ? 0.0 : 1.0;
    }
    tau_scale_ = std::clamp(tau_scale_, 0.0, 1.0);
    kp_scale_ = std::clamp(kp_scale_, 0.0, 1.0);
  }
  const double tau_scale = tau_scale_;
  const double kp_scale = kp_scale_;
  // 过渡系数（两段式：前馈先建立，再撤 kp）。在 compute_gravity 块外声明，后面 kp 计算要用
  // 过渡系数由持久状态 tau_scale_/kp_scale_ 推进，见下方
  if (compute_gravity) {
    Eigen::VectorXd q(static_cast<Eigen::Index>(ARM_DOF));
    for (size_t i = 0; i < ARM_DOF; ++i) q[static_cast<Eigen::Index>(i)] = pos_states_[i];
    Eigen::VectorXd g(model_->nv());
      // ── 用辨识参数（有的话）：g = Y(q)·θ̂ + offset
      if (gparams_.valid && gparams_.theta.size() == model_->nv() * 10) {
        const Eigen::VectorXd off =
            gparams_.offsets.size() == model_->nv() ? gparams_.offsets
                                                   : Eigen::VectorXd::Zero(model_->nv());
        hw_arm_algorithms::computeGravityFromParamsAndOffset(
            *model_, *ws_, q, gparams_.theta, off, g);
      } else {
        hw_arm_algorithms::computeGravity(*model_, *ws_, q, g);
      }

      // ── 平滑的库仑摩擦补偿：f_c · tanh(dq/eps)，eps = 0.05 rad/s
      if (gparams_.valid && gparams_.friction.size() == model_->nv()) {
        for (size_t i = 0; i < ARM_DOF; ++i) {
          const double dq = vel_states_[i];
          g[static_cast<Eigen::Index>(i)] +=
              gparams_.friction[static_cast<Eigen::Index>(i)] * std::tanh(dq / 0.05);
        }
      }

      // ── 重力前馈斜坡（2026-11-26 修 G02）
      //
      //  原来的 ramp_t0_ 从来没被赋值（一直是 0），而 steady_now() 返回
      //  time_since_epoch 秒（~1.79e9），所以 (1.79e9-0)/T 永远 >> 1，
      //  clamp 后恒为 1 —— **日志写着"斜坡 5.0 s"，实际一使能就满额**。
      //
      //  两段式过渡（kp 在时先把前馈建立起来，再撤 kp）：
      //    阶段 1  t ∈ [0, T)     tau_scale 0→1，kp 保持满
      //    阶段 2  t ∈ [T, 2T)    tau_scale = 1，kp 满→0（仅零重力模式）
      //  ⚠️ 绝不可以在 kp=0 时让前馈从 0 慢慢升 —— 那是欠补偿，机械臂会先掉下来。
      const bool zg_now_early = (zero_gravity_ || zero_gravity_srv_mode_);
      const double gscale = gravity_scale_ * tau_scale;
      for (size_t i = 0; i < ARM_DOF; ++i) {
        // 关节坐标系内的总前馈：重力 + 恒定偏置修正（trim 也乘过渡系数）
        // 力矩限幅：参数算错时不许把电机打满
        const double ff = gscale * g[static_cast<Eigen::Index>(i)] +
                          tau_scale * gravity_trim_[i];
        gravity_tau_[i] = std::clamp(ff, -tau_limit_[i], tau_limit_[i]);
        gravity_g_diag_[i] = g[static_cast<Eigen::Index>(i)];
      }

    if (gravity_dry_run_) {
      RCLCPP_INFO_THROTTLE(logger(), throttle_clock(), 1000,
                           "[重力dry-run] g(q)=[%.3f %.3f %.3f %.3f %.3f %.3f] "
                           "scale=%.2f → tau_ff=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                           g[0], g[1], g[2], g[3], g[4], g[5], gravity_scale_,
                           gravity_tau_[0], gravity_tau_[1], gravity_tau_[2],
                           gravity_tau_[3], gravity_tau_[4], gravity_tau_[5]);
    }
  }

  for (size_t j : active_idx_) {
    // 关节坐标系 → 电机坐标系（C-02）
    // kp/kd 不乘 direction（增益是正定的）；位置/速度/力矩都要乘。
    const double d = direction_[j];
    // ⚠️ 状态不可信时**必须切断重力前馈**（2026-11-26 事故）
    //    g(q) 用的是冻结的 q，而机械臂可能已经在动 —— 前馈越算越错，
    //    形成正反馈把手臂甩出去（现场"往上飘拉不住"，只能断电）。
    //    宁可按"掉下来"处理，也不要按"甩出去"处理。
    const bool zg_now_diag = (zero_gravity_ || zero_gravity_srv_mode_);
    const bool ff_ok = apply_gravity && !state_stale_any_ && !fault_latched_;
    if (!ff_ok && apply_gravity) {
      RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 1000,
                           "状态不可信，已切断重力前馈（宁掉不甩）");
    }
    const double tau_cmd = tau_commands_[j] + (ff_ok ? gravity_tau_[j] : 0.0);

    // ── 零重力下的逐轴诊断（2026-11-27 加）
    //   现场现象：joint2 往上顶、joint4/5/6 像没补偿往下掉。
    //   要判断到底是"没发出去"还是"发了但不够"，必须看到实际下发的值。
    // 先算本轴的 kp/kd，诊断日志要用（原来诊断排在 kp_use 定义之前，编译不过）
    const double kp_use_pre = kp_scale * kp_[j] + (1.0 - kp_scale) * zero_gravity_kp_floor_[j];
    const double kd_use_pre = zg_now_diag
        ? (zero_gravity_kd_v_[j] * (1.0 - kp_scale) + kd_[j] * kp_scale)
        : kd_[j];
    if (zg_now_diag && j == 0) {
      RCLCPP_INFO_THROTTLE(logger(), throttle_clock(), 1000,
          "[零重力诊断] ff_ok=%d tau_scale=%.2f kp_scale=%.2f kp_now=%.1f kp_floor=%.1f kd=%.2f | "
          "g=[%.3f %.3f %.3f %.3f %.3f %.3f] "
          "trim=[%.2f %.2f %.2f %.2f %.2f %.2f] "
          "tau_ff=[%.3f %.3f %.3f %.3f %.3f %.3f]",
          ff_ok ? 1 : 0, tau_scale_, kp_scale_,
          kp_use_pre, zero_gravity_kp_floor_[j], zero_gravity_kd_v_[j],
          gravity_g_diag_[0], gravity_g_diag_[1], gravity_g_diag_[2],
          gravity_g_diag_[3], gravity_g_diag_[4], gravity_g_diag_[5],
          gravity_trim_[0], gravity_trim_[1], gravity_trim_[2],
          gravity_trim_[3], gravity_trim_[4], gravity_trim_[5],
          gravity_tau_[0], gravity_tau_[1], gravity_tau_[2],
          gravity_tau_[3], gravity_tau_[4], gravity_tau_[5]);
    }
    // 零重力模式：kp 恒 0（位置命令失效），只留速度阻尼 + 重力前馈
    // G04：统一用实际模式（含服务进入的），速度守护也用这个
    const bool zg_now = (zero_gravity_ || zero_gravity_srv_mode_);
    // G02：kp 按 kp_scale 过渡（阶段 2 才降到 0），不是瞬间归零
    // kp_use：过渡期从满 kp 线性降到 kp_floor（而不是 0）
    //   kp_floor = 0 时行为与旧版完全一致（完全失重）
    const double kp_use = kp_scale * kp_[j] + (1.0 - kp_scale) * zero_gravity_kp_floor_[j];
    double kd_use = zg_now ? (zero_gravity_kd_v_[j] * (1.0 - kp_scale) + kd_[j] * kp_scale)
                           : kd_[j];
    // ── 零重力速度守护（2026-11-28 重做：滞回 + 速率限制）
    //
    // 旧实现的问题：|v| > limit 时 kd 一帧从 3.0 跳到 5.0。v 在阈值附近来回
    // 穿越时 kd 每 8 ms 硬切一次，对 joint6（M=3e-4）相当于 2e4 rad/s^2 的
    // 力矩阶跃 —— 现场就是"掰 j6 时电机剧烈抖动"。
    //
    // 现在：
    //   · 进入阈值 vel_limit，退出阈值 vel_limit * 0.6   → 滞回，不会在边界抖
    //   · kd 每周期最多变 guard_kd_slew_（默认 0.06 @125Hz ≈ 7.5/s）
    //   · 守护 kd 上限受 tau_limit*0.8 约束：kd*v 不许超过额定力矩的 80%
    //     （否则小惯量关节会瞬间饱和，那才是抖振的真正来源）
    const bool guard_over = std::abs(vel_states_[j]) > zero_gravity_vel_limit_;
    const bool guard_under = std::abs(vel_states_[j]) < zero_gravity_vel_limit_ * 0.6;
    if (zg_now) {
      if (!vel_guard_active_[j] && guard_over) {
        vel_guard_active_[j] = true;
        RCLCPP_WARN(logger(),
                    "零重力: joint%zu 速度 %.2f rad/s 超限 %.2f —— 平滑加大 kd 刹车",
                    j + 1, vel_states_[j], zero_gravity_vel_limit_);
      } else if (vel_guard_active_[j] && guard_under) {
        vel_guard_active_[j] = false;
        RCLCPP_INFO(logger(), "零重力: joint%zu 速度已回落 %.2f rad/s —— 守护解除",
                    j + 1, vel_states_[j]);
      }
      // 守护目标 kd：按 |v| 超限倍数线性抬升，但受力矩预算封顶
      double guard_target = 0.0;
      if (vel_guard_active_[j]) {
        const double v = std::abs(vel_states_[j]);
        // 上限：kd*v <= 0.8*tau_limit  =>  kd <= 0.8*tau_limit/v
        const double kd_cap = v > 1e-6 ? 0.8 * tau_limit_[j] / v : 5.0;
        guard_target = std::clamp(std::min(5.0, kd_cap), 0.0, 5.0);
      }
      // 速率限制逼近（这也是唯一改变 kd 的地方 —— 不存在阶跃）
      const double slew = guard_kd_slew_;
      guard_kd_state_[j] += std::clamp(guard_target - guard_kd_state_[j], -slew, slew);
      if (guard_kd_state_[j] > 0.0) {
        kd_use = std::max(kd_use, guard_kd_state_[j]);
      }
    } else {
      vel_guard_active_[j] = false;
      guard_kd_state_[j] = 0.0;
    }
    params.push_back({kp_use, kd_use,
                      d * pos_commands_[j],
                      d * vel_commands_[j],
                      d * tau_cmd});
  }
  openarm_->get_arm().mit_control_all(params);
  openarm_->recv_all(100);

  // 写失败（ENETDOWN / ENOBUFS / 其它 errno）说明链路出问题了
  if (!openarm_->is_bus_healthy()) {
    if (hard_bus_fault(openarm_->get_bus_status())) {
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 2000, "CAN 写失败且总线硬故障，请求去激活");
      log_bus_status(openarm_->get_bus_status(), "write");
      return return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000, "CAN 写异常（非致命）");
    log_bus_status(openarm_->get_bus_status(), "write(warn)");
  }
  return return_type::OK;
}

// ─────────────────────────────────────────────────────── 回零（带插值）
void HwArmHW::return_to_zero() {
  RCLCPP_INFO(logger(), "回零中…");
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& motors = openarm_->get_arm().get_motors();
  std::vector<double> start(active_idx_.size(), 0.0);
  for (size_t k = 0; k < active_idx_.size() && k < motors.size(); ++k) {
    start[k] = motors[k].get_position();
  }

  constexpr int kSteps = 100;
  for (int s = 1; s <= kSteps; ++s) {
    const double a = static_cast<double>(s) / kSteps;
    std::vector<openarm::damiao_motor::MITParam> params;
    params.reserve(active_idx_.size());
    for (size_t k = 0; k < active_idx_.size(); ++k) {
      const size_t j = active_idx_[k];
      params.push_back({kp_[j], kd_[j], start[k] * (1.0 - a), 0.0, 0.0});
    }
    openarm_->get_arm().mit_control_all(params);
    openarm_->recv_all(5);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    pos_commands_[i] = 0.0;
    vel_commands_[i] = 0.0;
    tau_commands_[i] = 0.0;
  }
}

}  // namespace hw_arm_hardware

PLUGINLIB_EXPORT_CLASS(hw_arm_hardware::HwArmHW, hardware_interface::SystemInterface)
