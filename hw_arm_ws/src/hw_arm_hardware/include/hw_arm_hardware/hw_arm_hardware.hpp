// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 6 轴单臂 ros2_control 硬件接口
// ─ 参考 OpenArm 的 openarm_simple_hardware（Apache-2.0），改动如下：
//   · 7 轴 -> 6 轴，删掉全部夹爪逻辑
//   · 关节名不再用字符串拼接，直接取 info.joints（修掉上游的 TODO）
//   · read() 检查电机错误码，异常返回 ERROR（上游永远返回 OK）
//   · 支持 require_all_motors:=false：缺电机时告警并继续（单电机调试用）
//   · 回零默认开启（return_to_zero=true）：零位是机械结构托住的安全位形，
//     上电先回零便于确认一切正常；不想要可用 return_to_zero:=false 关掉

#pragma once

#include <memory>
#include <string>
#include <vector>

// ── 构建身份（CMake 编译期注入；单独编译这个头时给个兜底）
#ifndef HW_ARM_GIT_COMMIT
#define HW_ARM_GIT_COMMIT "unknown"
#endif
#ifndef HW_ARM_BUILD_TIME
#define HW_ARM_BUILD_TIME "unknown"
#endif
#ifndef HW_ARM_SRC_HASH
#define HW_ARM_SRC_HASH "unknown"
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// 重力补偿（C-04）—— 算法来自 hw_arm_algorithms，这里只做调用
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/gravity_identify.hpp"
#include "hw_arm_algorithms/model.hpp"

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
  // 2026-11-15 真机实测：只有 joint2 / joint3 反向。
  // 同日 joint5/joint6 的电机固件 ID 交换后**重测确认**：j5/j6 均为 +1（见 C-02）。
  //
  // 映射关系（direction[i] = +1 或 -1）：
  //   read()  : joint_pos[i] = direction[i] * motor_pos[i]
  //   write() : motor_cmd[i] = direction[i] * joint_cmd[i]
  // 力/力矩同号处理：MIT 的 tau_ff 也要乘 direction。
  // 增益 kp/kd 不乘 —— 它们是正定的，与方向无关。
  std::vector<double> direction_ = {+1.0, -1.0, -1.0, +1.0, +1.0, +1.0};

  // ── 重力补偿（待办 C-04）
  //
  // tau_ff = gravity_scale * computeGravity(q_actual)
  //
  // 三个开关，默认**全关**，保证不改变现有行为：
  //   gravity_compensation : 总开关（默认 false）
  //   gravity_dry_run      : 只算不执行，把期望力矩打到日志（默认 true）
  //   gravity_scale        : 0.0 ~ 1.5，先 0.3 → 0.6 → 1.0 慢慢加
  //
  // 为什么默认 dry-run：重力补偿方向/幅值一旦错了，机械臂会**主动往下压**，
  // 比不补偿更危险。先看数字对不对，再真正施加。
  bool motors_ready_ = false;   // on_activate 里确认能读到反馈后才为 true

  // ── 逐轴反馈时效（R7）──────────────────────────────────────
  //
  // 为什么必须逐轴：CAN 上其他节点还在 ACK 时，掉线的那一个轴不一定触发
  // bus-off；它只是**不再回状态帧**，而 Motor 缓存里仍是最后一次的值。
  // 于是 read() 会把旧位置当新位置发布，JTC 看到"没在动但也没出错"，
  // 最后可能"时间到了 → 报 SUCCEEDED"。这是最隐蔽的假成功。
  //
  // 阈值取 200 ms：250 Hz 下相当于 50 个周期没回包，
  // 远大于正常抖动（实测丢帧 10~20% 也能在这个窗口内补上）。
  int64_t stale_timeout_ms_ = 200;
  std::vector<bool> axis_stale_latched_;   // 每轴一个，避免刷屏


  // ── 零重力（拖动示教）模式
  //
  // 和「重力补偿」是**两件事**，容易混淆：
  //   重力补偿：kp 照常（70/70/60…）+ tau_ff = g(q)
  //             → 机械臂依然很"硬"，只是静态下垂变小
  //   零重力  ：kp = 0 + tau_ff = g(q) + kd 一点阻尼
  //             → 机械臂"失重"，用手可以轻松推动，松手停在原地
  //
  // 零重力模式下发 kp 恒为 0，位置命令不再起作用，
  // 所以**不要同时启用 JTC**（用 controllers:=joint_state_broadcaster）。
  bool zero_gravity_ = false;
  double zero_gravity_kd_ = 1.0;   // 速度阻尼（0~5，MIT 协议范围）

  bool gravity_comp_ = false;
  bool gravity_dry_run_ = true;
  double gravity_scale_ = 1.0;
  std::string urdf_file_;    // 为空则在 on_init 里按 ament index 找默认路径

  std::unique_ptr<hw_arm_algorithms::RobotModel> model_;
  std::unique_ptr<hw_arm_algorithms::RobotWorkspace> ws_;
  std::vector<double> gravity_tau_;   // 最近一次算出的 g(q)，方便日志/调试

  // ── 拖动示教（零重力）的安全机制（2026-11-26 新增）
  //
  // 零重力模式下 kp=0，机械臂**只靠 tau_ff 支撑**。一旦 tau_ff 算错，
  // 机械臂会直接往下掉。所以加了四道保险：
  //   ① gravity_params_path：用辨识出的 θ̂（rms 0.18 N·m）而不是标称 URDF（0.88）
  //   ② gravity_ramp_time：激活后从 0 线性升到 gravity_scale，
  //      给操作者 3 秒时间扶住/确认，避免一使能就猛地下垂
  //   ③ tau_limit：逐轴力矩限幅（默认按电机连续额定），防止算错时打满
  //   ④ zero_gravity_vel_limit：任一轴超速就加大 kd 当刹车，
  //      并且只打印节流日志（不直接去激活 —— 去激活反而会自由落体）
  std::string gravity_params_path_;          // 空 = 用标称 URDF 惯性参数
  hw_arm_algorithms::GravityParams gparams_; // 读回来的 θ̂ / 偏置 / 摩擦
  double gravity_ramp_time_ = 3.0;           // s，0 = 立即满额
  double ramp_t0_ = 0.0;                     // 激活时刻（steady clock 秒）
  std::vector<double> tau_limit_ = {120.0, 45.0, 28.0, 10.0, 7.0, 7.0};
  double zero_gravity_vel_limit_ = 2.0;      // rad/s
  int vel_guard_count_ = 0;                  // 连续超速周期数（用于日志）

  // ── 恒定力矩偏置修正（gravity_trim）
  //
  // 2026-11-26 现场发现：同一套 θ̂，隔了 12 小时再测，
  //   joint3 的实测力矩系统性比模型低 **0.55 N·m**，
  //   而且残差与位形角的相关性只有 -0.74（斜率 -0.008 N·m/°，
  //   比"零位偏移"应有的 ±0.09 N·m/° 小一个数量级）
  //   ⇒ 不是零位漂移，是**恒定力矩偏置漂移**
  //      （电流传感器零点温漂 / 线缆张力 / 减速器预紧力）
  //
  // 这类漂移任何参数辨识都吸收不了（它会随时间变），只能运行时补：
  //     tau_ff = ramp × (gravity_scale × g(q) + trim)
  // trim 的单位是 N·m，逐轴。
  std::vector<double> gravity_trim_ = {0, 0, 0, 0, 0, 0};

  // ── 运行时切换零重力（~/set_zero_gravity，std_srvs/SetBool）
  //
  // 为什么必须能运行时进出（2026-11-26 实测）：
  //   零重力启动点默认在 q≈0（return_to_zero），但零位是**机械结构托住**的。
  //   在零位附近测到的 joint3 力矩比模型低 0.5~4.6 N·m（模型高估），
  //   一离开结构支撑，多出来的力就把机械臂往上推 —— 实测"往上飘"就是这个。
  //   正确做法：**先位置控制走到自由位形，再切零重力**。
  //
  // 切换是瞬间的（不做渐变），因为位置控制下稳态误差 e=(g−tau_ff)/kp，
  // 若 tau_ff≈g 则 e≈0，切 kp→0 时机械臂几乎不动（误差 0.65° 量级）。
  bool zero_gravity_srv_mode_ = false;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zero_g_srv_;
  // direction 参数写成 " +1 -1 -1 +1 +1 +1 " 时的缓存（懒解析）
  std::vector<std::string> direction_vec_;

  // ── 配置
  std::string can_interface_ = "can0";
  bool can_fd_ = true;
  bool require_all_motors_ = false;   // 单电机调试时设 false
  bool return_to_zero_ = true;        // 上电自动回零（2026-11-26 起默认开：零位是结构托住的安全位形）

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
