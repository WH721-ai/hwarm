// hw_arm 的 BT 节点定义
#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace mini_manip_bt {

/// 把 6 个关节角发成一条 FollowJointTrajectory（走现有的 JTC）
///
/// XML 用法：
///   <FollowJointTrajectory target="0;0.8;0.4;0;0.6;0"
///                          duration="3.0"
///                          action_name="/hw_arm_controller/follow_joint_trajectory"/>
class FollowJointTrajectory
    : public BT::RosActionNode<control_msgs::action::FollowJointTrajectory> {
 public:
  using ActionT = control_msgs::action::FollowJointTrajectory;

  FollowJointTrajectory(const std::string& name, const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
      : BT::RosActionNode<ActionT>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::vector<double>>("target", "6 个关节目标角 (rad)"),
        BT::InputPort<double>("duration", 3.0, "轨迹时长 (s)"),
    });
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

 private:
  std::vector<double> target_;
};

}  // namespace mini_manip_bt
