#include "mini_manip_bt/bt_nodes.hpp"

#include <builtin_interfaces/msg/duration.hpp>

namespace mini_manip_bt {

bool FollowJointTrajectory::setGoal(Goal& goal) {
  auto target = getInput<std::vector<double>>("target");
  auto duration = getInput<double>("duration");

  if (!target || target->size() != 6) {
    throw BT::RuntimeError("FollowJointTrajectory: target 必须是 6 个数");
  }
  if (!duration || duration.value() <= 0.0) {
    throw BT::RuntimeError("FollowJointTrajectory: duration 必须 > 0");
  }
  target_ = target.value();

  goal.trajectory.joint_names = {"joint1", "joint2", "joint3",
                                 "joint4", "joint5", "joint6"};

  trajectory_msgs::msg::JointTrajectoryPoint p0;
  p0.positions.resize(6, 0.0);           // 起点由 JTC 用当前状态覆盖（留空也行）
  p0.velocities.resize(6, 0.0);
  p0.time_from_start.sec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions = target_;
  p1.velocities.assign(6, 0.0);
  const double T = duration.value();
  p1.time_from_start.sec = static_cast<int32_t>(T);
  p1.time_from_start.nanosec =
      static_cast<uint32_t>((T - p1.time_from_start.sec) * 1e9);

  // 只发终点一个点：JTC 会从「当前状态」插值到终点，最稳。
  goal.trajectory.points = {p1};
  return true;
}

BT::NodeStatus FollowJointTrajectory::onResultReceived(const WrappedResult& result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->error_code == 0) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(rclcpp::get_logger("bt"),
               "FollowJointTrajectory 失败: code=%d error_code=%d %s",
               static_cast<int>(result.code),
               static_cast<int>(result.result ? result.result->error_code : -1),
               result.result ? result.result->error_string.c_str() : "");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowJointTrajectory::onFailure(BT::ActionNodeErrorCode error) {
  RCLCPP_ERROR(rclcpp::get_logger("bt"), "FollowJointTrajectory onFailure: %d",
               static_cast<int>(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace mini_manip_bt
