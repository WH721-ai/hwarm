// gravity_node.cpp —— Gazebo 里的算法验证节点
//
// 这条回路和 MuJoCo 那边的 launch_mujoco.py 完全同构，只是换成了 ROS 通信：
//
//   订阅 /joint_states            →  q, qd
//   hw_arm_algorithms::computeGravity(q)  →  g          ← 同进程直接 link，无 c_api
//   tau = kp*(q_des - q) - kd*qd + g
//   发布 /hw_arm_effort_controller/commands (Float64MultiArray)
//
// 用法：
//   ros2 run hw_arm_gazebo gravity_node                       # 默认 hold + 算法前馈
//   ros2 run hw_arm_gazebo gravity_node --ros-args -p mode:=gravity   # 自由下坠（ctrl=0）
//   ros2 run hw_arm_gazebo gravity_node --ros-args -p urdf:=/path/to/hw_arm.urdf
//
// 参数（-p name:=value）：
//   urdf          算法用的 URDF（必须是 xacro 展开过、mesh 路径不是 package:// 的）
//   joints        关节名列表，默认 joint1..joint6
//   mode          hold | teach | gravity | zero
//   kp / kd       PD 增益（每个关节一个，或给一个值自动复制）
//   q_des         目标关节角（rad），hold 模式用
//   forcerange    每关节力矩上限（N·m），默认额定 20/10/9/3.5/3/3
//   control_rate  控制频率（Hz），默认 250（与 controller_manager 一致）
//   print_period  打印周期（秒），默认 1.0，0=不打印

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace {

/// 把"给一个值"或"给 n 个值"统一成 n 个值
std::vector<double> expand(const std::vector<double>& v, size_t n, double fallback) {
  if (v.size() == n) return v;
  if (v.size() == 1) return std::vector<double>(n, v.front());
  return std::vector<double>(n, fallback);
}

/// 找一个可用的 URDF 默认值（要求已展开、mesh 路径可解析）
std::string default_urdf() {
  const std::vector<std::pair<std::string, std::string>> candidates = {
      {"hw_arm_mujoco", "mjcf/_hw_arm_from_urdf.urdf"},   // 已展开、package:// 已替换
      {"hw_arm_description", "urdf/hw_arm.urdf"},        // 若你手动导出过一份
  };
  for (const auto& [pkg, rel] : candidates) {
    try {
      const std::string p =
          ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
      if (std::ifstream(p).good()) return p;
    } catch (const std::exception&) {
      // 包没装，换下一个
    }
  }
  return "";
}

}  // namespace

class GravityNode : public rclcpp::Node {
 public:
  GravityNode() : rclcpp::Node("hw_arm_gravity_node") {
    // ── 参数 ──
    joints_ = declare_parameter<std::vector<std::string>>(
        "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    const size_t n = joints_.size();

    mode_ = declare_parameter<std::string>("mode", "hold");
    control_rate_ = declare_parameter<double>("control_rate", 250.0);
    print_period_ = declare_parameter<double>("print_period", 1.0);

    std::string urdf = declare_parameter<std::string>("urdf", "");
    if (urdf.empty()) urdf = default_urdf();
    if (urdf.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "找不到 URDF。请用 -p urdf:=<展开后的 hw_arm.urdf> 指定"
                   "（xacro 文件不行，必须是展开结果；mesh 不能是 package://）");
      throw std::runtime_error("no urdf");
    }

    // ── 算法侧：加载模型（workspace 必须在 load 成功之后构造）──
    if (!model_.load(urdf)) {
      std::string msg = "URDF 加载失败: " + urdf;
      for (const auto& e : model_.validation().errors) msg += "\n  " + e;
      RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
      throw std::runtime_error("urdf load failed");
    }
    ws_ = std::make_unique<hw_arm_algorithms::RobotWorkspace>(model_);

    if (static_cast<size_t>(model_.nq()) != n) {
      RCLCPP_ERROR(get_logger(), "关节数不匹配：URDF nq=%d，参数 joints=%zu",
                   model_.nq(), n);
      throw std::runtime_error("nq mismatch");
    }
    RCLCPP_INFO(get_logger(), "URDF: %s", urdf.c_str());
    RCLCPP_INFO(get_logger(), "算法模型: nq=%d nv=%d 总质量=%.6f kg",
                model_.nq(), model_.nv(), model_.totalMass());

    // ── 增益 / 限幅 / 目标 ──
    kp_ = expand(declare_parameter<std::vector<double>>("kp", {70.0}), n, 70.0);
    kd_ = expand(declare_parameter<std::vector<double>>("kd", {2.75}), n, 2.75);
    kd_teach_ = declare_parameter<double>("kd_teach", 5.0);
    q_des_ = expand(declare_parameter<std::vector<double>>("q_des", {0.0}), n, 0.0);
    forcerange_ = expand(
        declare_parameter<std::vector<double>>("forcerange",
                                              {20.0, 10.0, 9.0, 3.5, 3.0, 3.0}),
        n, 3.0);

    q_.setZero(n);
    qd_.setZero(n);
    g_.setZero(n);
    tau_.setZero(n);

    RCLCPP_INFO(get_logger(), "模式: %s  控制频率: %.0f Hz", mode_.c_str(), control_rate_);
    RCLCPP_INFO(get_logger(), "限幅: [%.1f %.1f %.1f %.1f %.1f %.1f] N·m",
                forcerange_[0], forcerange_[1], forcerange_[2], forcerange_[3],
                forcerange_[4], forcerange_[5]);

    // ── ROS 接口 ──
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hw_arm_effort_controller/commands", 10);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(*msg); });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_), [this]() { onControlTimer(); });

    RCLCPP_INFO(get_logger(), "等待 /joint_states ...");
  }

 private:
  void onJointState(const sensor_msgs::msg::JointState& msg) {
    // 按名字映射，不靠顺序猜（话题里关节顺序不受我们控制）
    bool ok = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto it = std::find(msg.name.begin(), msg.name.end(), joints_[i]);
      if (it == msg.name.end()) {
        if (!warned_missing_) {
          RCLCPP_WARN(get_logger(), "/joint_states 里没有关节 '%s'", joints_[i].c_str());
          warned_missing_ = true;
        }
        ok = false;
        break;
      }
      const size_t idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
      q_[i] = (idx < msg.position.size()) ? msg.position[idx] : 0.0;
      qd_[i] = (idx < msg.velocity.size()) ? msg.velocity[idx] : 0.0;
    }
    have_state_ = ok;
    last_state_time_ = now();
  }

  void onControlTimer() {
    // 没有状态就不发指令（宁可不发，也不要拿旧值乱算）
    if (!have_state_) return;
    if ((now() - last_state_time_).seconds() > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "/joint_states 超过 1 s 没更新，暂停下发");
      return;
    }

    // ① 算法：重力补偿（这是要验证的东西）
    hw_arm_algorithms::computeGravity(model_, *ws_, q_, g_);

    // ② 控制律
    for (size_t i = 0; i < joints_.size(); ++i) {
      double t = 0.0;
      if (mode_ == "hold") {
        t = kp_[i] * (q_des_[i] - q_[i]) - kd_[i] * qd_[i] + g_[i];
      } else if (mode_ == "teach") {
        t = g_[i] - kd_teach_ * qd_[i];          // 失重手感
      } else {                                    // gravity / zero
        t = 0.0;
      }
      tau_[i] = std::clamp(t, -forcerange_[i], forcerange_[i]);   // 模拟额定扭矩限幅
    }

    // ③ 下发
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_.data(), tau_.data() + tau_.size());
    cmd_pub_->publish(cmd);
    ++step_;

    // ④ 诊断
    if (print_period_ > 0.0) {
      const auto t_now = now();
      if ((t_now - last_print_time_).seconds() >= print_period_) {
        last_print_time_ = t_now;
        double err_max = 0.0, tau_max = 0.0;
        for (size_t i = 0; i < joints_.size(); ++i) {
          err_max = std::max(err_max, std::abs(q_des_[i] - q_[i]) * 1000.0);  // mrad
          tau_max = std::max(tau_max, std::abs(tau_[i]));
        }
        RCLCPP_INFO(get_logger(),
                    "step=%d  |err|max=%7.3f mrad  |tau|max=%5.2f N·m  g=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                    step_, err_max, tau_max, g_[0], g_[1], g_[2], g_[3], g_[4], g_[5]);
      }
    }
  }

  // 参数
  std::vector<std::string> joints_;
  std::string mode_;
  double control_rate_ = 250.0;
  double print_period_ = 1.0;
  double kd_teach_ = 5.0;
  std::vector<double> kp_, kd_, q_des_, forcerange_;

  // 算法
  hw_arm_algorithms::RobotModel model_;
  std::unique_ptr<hw_arm_algorithms::RobotWorkspace> ws_;

  // 状态
  Eigen::VectorXd q_, qd_, g_, tau_;
  bool have_state_ = false;
  bool warned_missing_ = false;
  rclcpp::Time last_state_time_;
  rclcpp::Time last_print_time_{0, 0, RCL_ROS_TIME};
  int step_ = 0;

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GravityNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("hw_arm_gravity_node"), "启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
