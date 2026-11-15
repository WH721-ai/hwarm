#!/usr/bin/env python3
"""在 RViz 里显示 hw_arm（六轴机械臂）。

用法：
    ros2 launch hw_arm_description display.launch.py
    ros2 launch hw_arm_description display.launch.py gui:=false      # 不弹滑条
    ros2 launch hw_arm_description display.launch.py rviz:=false     # 只起 rsp
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("hw_arm_description")
    xacro_file = os.path.join(pkg, "urdf", "hw_arm.urdf.xacro")
    rviz_file = os.path.join(pkg, "rviz", "display.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true",
                                  description="是否启动 joint_state_publisher_gui 滑条"),
            DeclareLaunchArgument("rviz", default_value="true",
                                  description="是否启动 RViz"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_file],
                condition=IfCondition(rviz),
                output="screen",
            ),
        ]
    )
