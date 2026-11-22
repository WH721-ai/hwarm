// ══════════════════════════════════════════════════════════════
// BT 执行器：加载 XML → 10 Hz tick → 打印结果
//
// 它是**决策层**：只调 Action，不碰 CAN、不算 IK、不发控制量。
// ══════════════════════════════════════════════════════════════
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <thread>

#include "mini_manip_bt/bt_nodes.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_executor");

  // ── 1. 建工厂，注册节点 ────────────────────────────────────
  BT::BehaviorTreeFactory factory;

  BT::RosNodeParams params(node);
  params.default_port_value = "/hw_arm_controller/follow_joint_trajectory";
  params.server_timeout = std::chrono::milliseconds(5000);
  params.wait_for_server_timeout = std::chrono::milliseconds(2000);

  factory.registerNodeType<mini_manip_bt::FollowJointTrajectory>(
      "FollowJointTrajectory", params);

  // ── 2. 加载 XML（默认用 install 里的，可命令行覆盖）────────
  std::string xml_file = "goto_home.xml";
  if (argc > 1) xml_file = argv[1];
  else {
    try {
      xml_file = ament_index_cpp::get_package_share_directory("mini_manip_bt") +
                 "/trees/goto_home.xml";
    } catch (...) {}
  }

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_file);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "加载 %s 失败: %s", xml_file.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "已加载行为树: %s", xml_file.c_str());

  BT::StdCoutLogger logger(tree);

  // ── 3. tick 循环（10 Hz，非实时线程）──────────────────────
  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "行为树结束: %s", BT::toStr(status).c_str());
  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
