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
rclcpp::Logger logger() { return rclcpp::get_logger("HwArmHW"); }
rclcpp::Clock& throttle_clock() {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

bool to_bool(std::string v, bool fallback) {
  // ::tolower 对负值是 UB（非 ASCII 字符高位），先转 unsigned char
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (v == "true" || v == "1") return true;
  if (v == "false" || v == "0") return false;
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
    return_to_zero_ = to_bool(get("return_to_zero"), false);

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
  }
  if (zero_gravity_) {
    // 零重力必然要求真正施加重力前馈，忽略 dry_run
    gravity_comp_ = true;
    gravity_dry_run_ = false;
    RCLCPP_WARN(logger(),
                "★ 零重力（拖动示教）模式已开启：kp=0 / kd=%.2f / tau_ff=%.2f·g(q)",
                zero_gravity_kd_, gravity_scale_);
    RCLCPP_WARN(logger(),
                "  机械臂会进入'失重'状态 —— 请扶着它，或确认它能自己站稳！");
    RCLCPP_WARN(logger(),
                "  此模式请只启动 joint_state_broadcaster，不要启动轨迹控制器");
  }
  if (!get("urdf_file").empty()) urdf_file_ = get("urdf_file");

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

    RCLCPP_INFO(logger(),
                "重力补偿: 开（scale=%.2f, dry_run=%s）URDF=%s 总质量=%.4f kg",
                gravity_scale_, gravity_dry_run_ ? "true" : "false",
                urdf_file_.c_str(), model_->totalMass());
    {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq());
      Eigen::VectorXd g(model_->nv());
      hw_arm_algorithms::computeGravity(*model_, *ws_, q, g);
      RCLCPP_INFO(logger(),
                  "  g(q=0) = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] N·m"
                  "（可用于核对期望值）",
                  g[0], g[1], g[2], g[3], g[4], g[5]);
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
             !st.is_stale(std::chrono::milliseconds(stale_timeout_ms_)) &&
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
  for (size_t k = 0; k < active_idx_.size(); ++k) {
    const size_t j = active_idx_[k];
    const bool has = (k < motors.size());
    const bool stale =
        !has || openarm_->get_arm()
                    .get_link_stats(static_cast<int>(k))
                    .is_stale(std::chrono::milliseconds(stale_timeout_ms_));

    if (stale) {
      any_stale = true;
      if (!axis_stale_latched_[k]) {
        axis_stale_latched_[k] = true;
        RCLCPP_ERROR(logger(),
                     "%s 反馈过期（>%ld ms 没有新的状态帧）。"
                     "该轴的 pos/vel/tau 状态不再可信。",
                     joint_names_[j].c_str(), (long)stale_timeout_ms_);
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
return_type HwArmHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
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
  if (compute_gravity) {
    Eigen::VectorXd q(static_cast<Eigen::Index>(ARM_DOF));
    for (size_t i = 0; i < ARM_DOF; ++i) q[static_cast<Eigen::Index>(i)] = pos_states_[i];
    Eigen::VectorXd g(model_->nv());
    hw_arm_algorithms::computeGravity(*model_, *ws_, q, g);
    for (size_t i = 0; i < ARM_DOF; ++i)
      gravity_tau_[i] = gravity_scale_ * g[static_cast<Eigen::Index>(i)];

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
    const double tau_cmd = tau_commands_[j] +
                           (apply_gravity ? gravity_tau_[j] : 0.0);
    // 零重力模式：kp 恒 0（位置命令失效），只留速度阻尼 + 重力前馈
    const double kp_use = zero_gravity_ ? 0.0 : kp_[j];
    const double kd_use = zero_gravity_ ? zero_gravity_kd_ : kd_[j];
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
