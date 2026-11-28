// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0

#include "hw_arm_controllers/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/kinematics.hpp"

namespace hw_arm_controllers {

namespace ci = hw_arm_algorithms::impedance;
using CallbackReturn = controller_interface::CallbackReturn;
using controller_interface::return_type;

namespace {
/// 把 yaml 列表/标量读成 6 个 double
bool read6(rclcpp_lifecycle::LifecycleNode& node, const std::string& name,
           std::array<double, 6>& out) {
  std::vector<double> v;
  if (!node.get_parameter(name, v)) return false;
  if (v.size() == 1) v.assign(6, v[0]);
  if (v.size() != 6) {
    RCLCPP_ERROR(node.get_logger(), "%s 需要 1 或 6 个值，给了 %zu", name.c_str(), v.size());
    return false;
  }
  std::copy(v.begin(), v.end(), out.begin());
  return true;
}
}  // namespace

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joint_names_) cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  return cfg;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joint_names_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
    // 只有显式要求时才读硬件的 g(q)：mock_components 不导出这个接口，
    // 硬性依赖会让控制器在假硬件上根本无法 activate。
    if (use_hardware_gravity_) cfg.names.push_back(j + "/gravity_torque");
  }
  return cfg;
}

CallbackReturn CartesianImpedanceController::on_init() {
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  auto node = get_node();
  node->declare_parameter("joints", joint_names_);
  node->declare_parameter("end_effector_frame", std::string("tool0"));
  node->declare_parameter("stiffness", std::vector<double>{500., 500., 500., 50., 50., 50.});
  node->declare_parameter("effective_inertia",
                          std::vector<double>{1.0, 1.0, 1.0, 0.1, 0.1, 0.1});
  node->declare_parameter("damping_ratio", std::vector<double>{1., 1., 1., 1., 1., 1.});
  node->declare_parameter("wrench_limit", std::vector<double>{30., 30., 30., 5., 5., 5.});
  node->declare_parameter("torque_limit", std::vector<double>{20., 20., 15., 6., 4., 4.});
  node->declare_parameter("torque_slew", std::vector<double>{200., 200., 200., 100., 100., 100.});
  node->declare_parameter("joint_speed_limit", 3.0);
  node->declare_parameter("filter_time", 0.05);
  node->declare_parameter("reference_linear_speed", 0.5);
  node->declare_parameter("reference_angular_speed", 1.0);
  node->declare_parameter("activation_time", 0.5);
  node->declare_parameter("max_position_error", 0.05);
  node->declare_parameter("max_rotation_error", 0.5);
  node->declare_parameter("max_limited_time", 0.5);
  node->declare_parameter("use_hardware_gravity", false);
  node->declare_parameter("gravity_params_path", std::string(""));

  node->get_parameter("joints", joint_names_);
  if (joint_names_.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints 必须是 6 个");
    return CallbackReturn::ERROR;
  }
  node->get_parameter("end_effector_frame", end_effector_frame_);
  if (!read6(*node, "stiffness", stiffness_) ||
      !read6(*node, "effective_inertia", effective_inertia_) ||
      !read6(*node, "damping_ratio", damping_ratio_) ||
      !read6(*node, "wrench_limit", wrench_limit_) ||
      !read6(*node, "torque_limit", torque_limit_) ||
      !read6(*node, "torque_slew", torque_slew_)) {
    return CallbackReturn::ERROR;
  }
  node->get_parameter("joint_speed_limit", joint_speed_limit_);
  node->get_parameter("filter_time", filter_time_);
  node->get_parameter("reference_linear_speed", ref_linear_speed_);
  node->get_parameter("reference_angular_speed", ref_angular_speed_);
  node->get_parameter("activation_time", activation_time_);
  node->get_parameter("max_position_error", max_position_error_);
  node->get_parameter("max_rotation_error", max_rotation_error_);
  node->get_parameter("max_limited_time", max_limited_time_);
  node->get_parameter("use_hardware_gravity", use_hardware_gravity_);
  node->get_parameter("gravity_params_path", gravity_params_path_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  // URDF：从 robot_description 参数取（与硬件用同一份）
  // robot_description 是 robot_state_publisher 发布的**话题**，不是本节点参数。
  // 拿不到就退回包的 install 里的 URDF —— 它和硬件加载的是同一个文件
  // （构建期由 xacro 展开生成），因此仍然同源。
  std::string urdf;
  if (!node->get_parameter("robot_description", urdf) || urdf.empty()) {
    RCLCPP_INFO(node->get_logger(), "没有 robot_description 参数，改用包内的 hw_arm.urdf");
  }
  // RobotModel 只提供 load(path)。这里用包的 install 里的 URDF ——
  // 它和硬件加载的是**同一个文件**（构建期由 xacro 展开生成），因此一致。
  try {
    const std::string path = ament_index_cpp::get_package_share_directory("hw_arm_description") +
                             "/urdf/hw_arm.urdf";
    model_ = std::make_unique<hw_arm_algorithms::RobotModel>();
    if (!model_->load(path)) {
      RCLCPP_ERROR(node->get_logger(), "模型加载失败: %s", path.c_str());
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "找不到 hw_arm_description: %s", e.what());
    return CallbackReturn::ERROR;
  }
  (void)urdf;
  ws_ = std::make_unique<hw_arm_algorithms::RobotWorkspace>(*model_);

  const auto& pin_model = model_->pin();
  if (pin_model.existFrame(end_effector_frame_)) {
    end_frame_ = pin_model.getFrameId(end_effector_frame_);
  } else {
    RCLCPP_ERROR(node->get_logger(), "URDF 里没有帧 '%s'", end_effector_frame_.c_str());
    return CallbackReturn::ERROR;
  }

  // 自算路径需要 θ̂；显式给路径就必须读成功（不静默退回标称参数）
  if (!gravity_params_path_.empty()) {
    hw_arm_algorithms::GravityParams gp;
    std::string err;
    if (!hw_arm_algorithms::loadGravityParams(gravity_params_path_, gp, &err)) {
      RCLCPP_ERROR(node->get_logger(), "读重力参数失败 %s: %s",
                   gravity_params_path_.c_str(), err.c_str());
      return CallbackReturn::ERROR;
    }
    gravity_theta_ = gp.theta;
    gravity_offset_ = gp.offsets.size() == 6 ? gp.offsets : Eigen::VectorXd::Zero(6);
    RCLCPP_INFO(node->get_logger(), "自算重力：θ̂ rank=%d rms=%.3f（%s）",
                gp.rank, gp.rms, gravity_params_path_.c_str());
  } else if (!use_hardware_gravity_) {
    RCLCPP_WARN(node->get_logger(),
                "gravity_params_path 为空且 use_hardware_gravity=false "
                "-> 梯度前馈恒为 0（只有阻尼+弹簧，机械臂会下沉）");
  }

  ci::Config cfg;
  for (int i = 0; i < 6; ++i) {
    cfg.stiffness[i] = stiffness_[static_cast<size_t>(i)];
    cfg.effective_inertia[i] = effective_inertia_[static_cast<size_t>(i)];
    cfg.damping_ratio[i] = damping_ratio_[static_cast<size_t>(i)];
    cfg.wrench_limit[i] = wrench_limit_[static_cast<size_t>(i)];
    cfg.torque_limit[i] = torque_limit_[static_cast<size_t>(i)];
    cfg.torque_slew[i] = torque_slew_[static_cast<size_t>(i)];
    cfg.joint_speed_limit[i] = joint_speed_limit_;
  }
  cfg.filter_time = filter_time_;
  cfg.reference_linear_speed = ref_linear_speed_;
  cfg.reference_angular_speed = ref_angular_speed_;
  cfg.activation_time = activation_time_;
  cfg.min_dt = 0.001;
  cfg.max_dt = 0.05;
  cfg.max_position_error = max_position_error_;
  cfg.max_rotation_error = max_rotation_error_;
  cfg.max_limited_time = max_limited_time_;
  if (!impedance_.configure(cfg)) {
    RCLCPP_ERROR(node->get_logger(), "impedance.configure 失败");
    return CallbackReturn::ERROR;
  }

  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_buffer_.writeFromNonRT(*msg);
        target_seq_.fetch_add(1, std::memory_order_release);
      });
  state_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);
  stop_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/deactivate_impedance",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        impedance_.deactivate();
        res->success = true;
        res->message = "impedance deactivated";
      });
  RCLCPP_INFO(node->get_logger(), "笛卡尔阻抗控制器已配置：帧=%s", end_effector_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // 根据实际声明了几个接口决定 stride —— 不能写死
  state_stride_ = use_hardware_gravity_ ? 3 : 2;
  if (state_interfaces_.size() != static_cast<size_t>(6 * state_stride_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "状态接口数量不对：期望 %d，实际 %zu",
                 6 * state_stride_, state_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  // 丢弃旧目标
  applied_seq_ = target_seq_.load(std::memory_order_acquire);

  ci::Sample s;
  if (!buildSample(s)) {
    RCLCPP_ERROR(get_node()->get_logger(), "激活时构建 Sample 失败");
    return CallbackReturn::ERROR;
  }
  // ⚠️ handover 必须传 "此刻能让机械臂静止的支撑力矩"，也就是 g(q)，
  //    不能传 0（2026-11-28 现场事故）。
  //    激活混合是 raw = (1-w)*handover + w*desired，
  //    传 0 的话前 0.2~0.3 s 只给了一半甚至更少的重力支撑 ->
  //    机械臂自由下落 -> |dq| 很快超过 joint_speed_limit ->
  //    validSample() 判 InvalidSample(err=2) -> 故障 -> 输出零力矩 -> 直接掉。
  ci::Vec6 handover = s.gravity;
  if (!impedance_.activate(s, handover)) {
    RCLCPP_ERROR(get_node()->get_logger(), "impedance.activate 失败 err=%d",
                 static_cast<int>(impedance_.error()));
    return CallbackReturn::ERROR;
  }
  prev_torque_ = handover;
  // 初始目标 = 当前位姿（激活后先"保持不动"，再等外部给目标）
  ci::Pose p;
  p.p = s.pose.p;
  p.q = s.pose.q;
  impedance_.setTarget(p);  // activate 后 state=Ramping，setTarget 会被拒；见 update
  RCLCPP_INFO(get_node()->get_logger(), "进入 Ramping；目标默认=当前位姿");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  impedance_.deactivate();
  return CallbackReturn::SUCCESS;
}

bool CartesianImpedanceController::buildSample(ci::Sample& out) const {
  if (!model_ || !ws_) return false;
  Eigen::VectorXd q(model_->nq()), dq(model_->nv());
  for (int i = 0; i < 6; ++i) {
    q[i] = state_interfaces_[static_cast<size_t>(i) * static_cast<size_t>(state_stride_) + 0].get_value();
    dq[i] = state_interfaces_[static_cast<size_t>(i) * static_cast<size_t>(state_stride_) + 1].get_value();
  }
  for (int i = 0; i < model_->nq(); ++i) {
    if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) return false;
  }

  pinocchio::SE3 pose;
  hw_arm_algorithms::computeFK(*model_, *ws_, q, end_frame_, pose);
  Eigen::MatrixXd J(6, model_->nv());
  hw_arm_algorithms::computeJacobian(*model_, *ws_, q, end_frame_,
                                     hw_arm_algorithms::JacobianFrame::LOCAL_WORLD_ALIGNED, J);

  out.pose.p = pose.translation();
  out.pose.q = Eigen::Quaterniond(pose.rotation());
  out.J = J;
  out.dq = dq;
  if (use_hardware_gravity_) {
    for (int i = 0; i < 6; ++i)
      out.gravity[i] = state_interfaces_[static_cast<size_t>(i) * static_cast<size_t>(state_stride_) + 2].get_value();
  } else if (gravity_theta_.size() > 0) {
    Eigen::VectorXd g;
    hw_arm_algorithms::computeGravityFromParamsAndOffset(*model_, *ws_, q, gravity_theta_,
                                                        gravity_offset_, g);
    for (int i = 0; i < 6; ++i) out.gravity[i] = g[i];
  }  // 否则保持 0
  out.valid = true;
  return true;
}

return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/,
                                                 const rclcpp::Duration& period) {
  const auto st = impedance_.state();
  if (st != ci::State::Ramping && st != ci::State::Active) {
    writeZero();
    return return_type::OK;
  }

  ci::Sample s;
  if (!buildSample(s)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "状态无效，输出零力矩");
    writeZero();
    return return_type::OK;
  }

  // 只在 Active 之后接受外部目标
  const uint64_t seq = target_seq_.load(std::memory_order_acquire);
  if (seq != applied_seq_ && impedance_.state() == ci::State::Active) {
    applied_seq_ = seq;   // 新目标只处理一次；失败也前进，避免每周期重复报警
    auto* tgt = target_buffer_.readFromRT();
    ci::Pose p;
    p.p = Eigen::Vector3d(tgt->pose.position.x, tgt->pose.position.y, tgt->pose.position.z);
    p.q = Eigen::Quaterniond(tgt->pose.orientation.w, tgt->pose.orientation.x,
                             tgt->pose.orientation.y, tgt->pose.orientation.z);
    if (tgt && !impedance_.setTarget(p)) {
      RCLCPP_WARN(get_node()->get_logger(), "目标位姿被拒（跳变过大或非法）");
    }
  }

  const double dt = period.seconds();
  auto cmd = impedance_.update(s, prev_torque_, dt);
  if (!cmd) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "阻抗故障 err=%d -> 输出零力矩。上一拍限幅: wrench=%d torque=%d slew=%d；"
                 "gravity=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                 static_cast<int>(impedance_.error()),
                 last_wrench_limited_ ? 1 : 0, last_torque_limited_ ? 1 : 0,
                 last_rate_limited_ ? 1 : 0,
                 s.gravity[0], s.gravity[1], s.gravity[2], s.gravity[3], s.gravity[4],
                 s.gravity[5]);
    RCLCPP_ERROR(get_node()->get_logger(),
                 "  dq=[%.2f %.2f %.2f %.2f %.2f %.2f]（限 %.2f rad/s）",
                 s.dq[0], s.dq[1], s.dq[2], s.dq[3], s.dq[4], s.dq[5],
                 joint_speed_limit_);
    writeZero();
    return return_type::OK;
  }
  std::array<double, 6> out{};
  for (int i = 0; i < 6; ++i) out[static_cast<size_t>(i)] = cmd->torque[i];
  writeCmd(out);
  prev_torque_ = cmd->torque;
  publishState();
  return return_type::OK;
}

void CartesianImpedanceController::writeCmd(const std::array<double, 6>& t) {
  for (size_t i = 0; i < 6; ++i) command_interfaces_[i].set_value(t[i]);
}

void CartesianImpedanceController::writeZero() { writeCmd({0, 0, 0, 0, 0, 0}); }

void CartesianImpedanceController::publishState() {
  if (!state_pub_) return;
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(8);
  for (int i = 0; i < 6; ++i) msg.data[static_cast<size_t>(i)] = prev_torque_[i];
  msg.data[6] = static_cast<double>(impedance_.state());
  msg.data[7] = static_cast<double>(impedance_.error());
  state_pub_->publish(msg);
}

}  // namespace hw_arm_controllers

PLUGINLIB_EXPORT_CLASS(hw_arm_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
