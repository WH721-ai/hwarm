// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 6 轴单臂 ros2_control 硬件接口
// ─ 参考 OpenArm 的 openarm_simple_hardware（Apache-2.0），改动如下：
//   · 7 轴 -> 6 轴，删掉全部夹爪逻辑
//   · 关节名不再用字符串拼接，直接取 info.joints（修掉上游的 TODO）
//   · read() 检查电机错误码，异常返回 ERROR（上游永远返回 OK）
//   · 支持 require_all_motors:=false：缺电机时告警并继续（单电机调试用）
//   · 回零默认关闭（return_to_zero:=false），避免上电意外动作

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hw_arm_hardware {

class HwArmHW : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HwArmHW)

  HwArmHW() = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  static constexpr size_t ARM_DOF = 6;

  // ── 电机型号（按 joint1..joint6 顺序，必须与实际装配一致）
  //    joint1: DM-J10010L  joint2: DM-J6216P   joint3: DM-J4340
  //    joint4: DM-J4310P   joint5: DM-J4310    joint6: DM-J4310
  //    注意：DM6216P / DM4310P 是本地给 openarm_can 打的补丁（上游无此型号），
  //    其 vMax/tMax 为**待确认占位值**，见 PATCHES.md。
  const std::vector<openarm::damiao_motor::MotorType> MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM10010L,  // joint1
      openarm::damiao_motor::MotorType::DM6216P,   // joint2
      openarm::damiao_motor::MotorType::DM4340,    // joint3
      openarm::damiao_motor::MotorType::DM4310P,   // joint4
      openarm::damiao_motor::MotorType::DM4310,    // joint5
      openarm::damiao_motor::MotorType::DM4310,    // joint6
  };

  // ── CAN ID（与 OpenArm 一致，去掉第 7 组）
  // CAN ID 默认值：joint i ↔ 发送 0x0i / 接收 0x1i
  // 可用参数覆盖（见 send_can_ids_ / recv_can_ids_）。
  const std::vector<uint32_t> SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  const std::vector<uint32_t> RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

  // ── 实际使用的 CAN ID 表（可被参数覆盖，用于"电机装反了但不想改固件 ID"）
  //
  // 场景：joint5 / joint6 的电机物理上装反了，CAN5 的电机在 joint6 位置。
  // 两种修法：
  //   A. 改电机固件 ID（有风险，中途失败会出现两个同 ID 的电机）
  //   B. **改这张映射表**（零风险，一行配置）：joint_can_ids = 1 2 3 4 6 5
  // 推荐 B —— 两边都是 DM4310，型号一致，交换后没有任何副作用。
  std::vector<uint32_t> send_can_ids_ = SEND_CAN_IDS;
  std::vector<uint32_t> recv_can_ids_ = RECV_CAN_IDS;

  // ── 默认增益（URDF 给了就用 URDF 的）
  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6};

  // ── 电机转向（待办 C-02）
  //
  // 达妙电机自己的 +q 和 URDF 的 +q 不一定同向，取决于电机装到关节上的朝向。
  // 两边**各自都满足右手定则**，差的只是"两个轴的指向关系"，只能实测。
  //
  // 实测方法见 scripts/mirror_check.py（实时镜像：手拖真机，看 RViz 跟不跟）。
  // 2026-11-15 在真机上测得：只有 joint2 / joint3 反向。
  //
  // 映射关系（direction[i] = +1 或 -1）：
  //   read()  : joint_pos[i] = direction[i] * motor_pos[i]
  //   write() : motor_cmd[i] = direction[i] * joint_cmd[i]
  // 力/力矩同号处理：MIT 的 tau_ff 也要乘 direction。
  // 增益 kp/kd 不乘 —— 它们是正定的，与方向无关。
  std::vector<double> direction_ = {+1.0, -1.0, -1.0, +1.0, +1.0, +1.0};
  // direction 参数写成 " +1 -1 -1 +1 +1 +1 " 时的缓存（懒解析）
  std::vector<std::string> direction_vec_;

  // ── 配置
  std::string can_interface_ = "can0";
  bool can_fd_ = true;
  bool require_all_motors_ = false;   // 单电机调试时设 false
  bool return_to_zero_ = false;       // 上电自动回零（默认关，安全）

  // ── 硬件
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;
  std::vector<std::string> joint_names_;

  // 实际接了电机的关节下标（0..5）。URDF 可能声明 6 个关节，但硬件只到货一部分，
  // 用 active_joints 参数指定，例如 "1" 表示只接 joint1。
  // 只有这些下标对应的电机才会被 init / enable / refresh / 发 MIT —— 关键：
  // 对不存在的 CAN ID 发帧会导致无 ACK、错误计数飙升、最终 bus-off。
  std::vector<size_t> active_idx_;

  // 状态刷新节奏：**每周期刷新 N 台**，轮转覆盖全部电机。
  // 不用「每 K 周期刷新全部」是因为那样会出现 12 帧的突发，
  // 而 gs_usb 的 URB 池只有 ~10 个 → 突发必丢帧（实测 ENOBUFS 40%）。
  // 轮转后每周期帧数恒定 = 6 MIT + N refresh，无突发。
  int refresh_per_cycle_ = 2;    // 0 = 不刷新（靠电机自动上报）
  uint32_t cycle_count_ = 0;

  // ── 状态 / 命令缓冲（export_* 把指针交给 controller_manager）
  std::vector<double> pos_commands_, vel_commands_, tau_commands_;
  std::vector<double> pos_states_, vel_states_, tau_states_;

  bool parse_config(const hardware_interface::HardwareInfo& info);
  void resolve_joint_names(const hardware_interface::HardwareInfo& info);
  void return_to_zero();
};

}  // namespace hw_arm_hardware
