// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 笛卡尔阻抗控制器（ros2_control 插件）
// ─────────────────────────────────────────────────────────────
// 分工：
//   硬件层  : 只负责 CAN、重力前馈的**计算**（通过 gravity_torque 状态接口导出）、
//             力矩输出与安全联锁。
//   本控制器: 读关节状态 + gravity_torque → 用 cartesian_impedance 算出
//             完整关节力矩 → 写 effort 命令。
//
// ⚠️ 因此本控制器接入时，硬件必须 **gravity_compensation:=false**
//    （否则硬件会把 g(q) 再加一遍）。硬件侧会打印一行提示。
//
// 控制律（见 hw_arm_algorithms/cartesian_impedance.hpp）：
//   τ = g(q) + Jᵀ( K·(ref − meas) − D·v )
//   本控制器只负责提供 Sample（pose/J/dq/gravity）与目标位姿，
//   所有限幅、激活混合、故障判定都在 cartesian_impedance 里。
//
// 接口：
//   命令 interface : joint1..6/effort
//   状态 interface : joint1..6/position, velocity, gravity_torque
//   订阅           : ~/target_pose (geometry_msgs/PoseStamped) —— 非实时，写入
//                     realtime buffer，由 update() 取出
//   服务           : ~/deactivate_impedance (std_srvs/Trigger) 主动退出到 Inactive
//   发布           : ~/state (std_msgs/Float64MultiArray) 诊断
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hw_arm_algorithms/cartesian_impedance.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity_identify.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  // 把 6 轴状态 + 重力前馈打包成 cartesian_impedance 的 Sample
  bool buildSample(hw_arm_algorithms::impedance::Sample& out) const;
  void publishState();

  // 参数
  std::vector<std::string> joint_names_;
  std::string end_effector_frame_;
  std::string robot_description_key_ = "robot_description";
  std::array<double, 6> stiffness_{};          // [N/m; N·m/rad]
  std::array<double, 6> effective_inertia_{};  // [kg; kg·m²]
  std::array<double, 6> damping_ratio_{};
  std::array<double, 6> wrench_limit_{};
  std::array<double, 6> torque_limit_{};
  std::array<double, 6> torque_slew_{};
  double joint_speed_limit_ = 2.0;
  double filter_time_ = 0.05;
  double ref_linear_speed_ = 0.5;
  double ref_angular_speed_ = 1.0;
  double activation_time_ = 0.5;
  double max_position_error_ = 0.05;
  double max_rotation_error_ = 0.5;
  double max_limited_time_ = 0.5;

  // 模型
  std::unique_ptr<hw_arm_algorithms::RobotModel> model_;
  std::unique_ptr<hw_arm_algorithms::RobotWorkspace> ws_;
  pinocchio::FrameIndex end_frame_ = 0;  // 运行时按 end_effector_frame_ 解析

  // 阻抗
  hw_arm_algorithms::impedance::CartesianImpedance impedance_;

  // 目标位姿（非实时线程写，update 取）
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> target_buffer_;
  // RealtimeBuffer 里永远有一个值（默认构造的 PoseStamped 是"全零 + 四元数 w=0"，
  // 属于非法位姿）。所以**不能**只看 readFromRT() 是否非空就来 setTarget ——
  // 那样每周期都会拿一个零位姿去设目标，被拒并刷警告（2026-11-28 实测现象）。
  // 用序号判断"上次应用之后有没有新消息"。
  std::atomic<uint64_t> target_seq_{0};
  uint64_t applied_seq_ = 0;
  // 上一拍指令的限幅标志：故障时打出来，否则只知道 "SaturationTimeout"
  // 却不知道是 wrench / torque / slew 哪一项在饱和（2026-11-28 现场踩过）。
  bool last_wrench_limited_ = false, last_torque_limited_ = false,
       last_rate_limited_ = false;

  // 接口按下标访问：Jazzy 的 get_value() 是返回值而不是引用，
  // 所以不缓存指针，每周期直接 get_value()/set_value()。
  // state 顺序：joint_i -> [position, velocity, gravity_torque] 三个一组
  // command 顺序：joint_i -> [effort]
  // gravity_torque 来自硬件还是自算
  //
  // 默认 false（自己算）：假硬件的 mock_components 只有 position/velocity/effort，
  // 没有 gravity_torque；硬性依赖它会让控制器在任何 mock 上都无法 activate。
  // 真机上可以设 true，用硬件算的那份（省一次 RNEA，且与硬件严格同源）。
  //
  // 两条路加载的是**同一个 URDF + 同一份 gravity_theta.yaml**，结果应当一致。
  bool use_hardware_gravity_ = false;
  // ⚠️ 每关节的状态接口个数：2（pos,vel）或 3（pos,vel,gravity_torque）。
  //    2026-11-28 崩溃事故：buildSample 里写死了 stride=3，
  //    而 use_hardware_gravity=false 时只声明了 2 个接口 ——
  //    i=4 时索引 12 越界，segfault 在 on_activate。
  int state_stride_ = 2;
  std::string gravity_params_path_;
  Eigen::VectorXd gravity_theta_, gravity_offset_;

  void writeCmd(const std::array<double, 6>& t);
  void writeZero();

  // 上一拍实际下发（用于力矩变化率限制与激活混合）
  hw_arm_algorithms::impedance::Vec6 prev_torque_ =
      hw_arm_algorithms::impedance::Vec6::Zero();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

}  // namespace hw_arm_controllers
