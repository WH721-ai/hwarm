#!/usr/bin/env python3
"""纯可视化启动（不加载 ros2_control，不碰硬件）

用法：
    ros2 launch hw_arm_bringup display.launch.py
    ros2 launch hw_arm_bringup display.launch.py gui:=false      # 不带关节滑条

为什么单独做一个：
    只改 URDF 看外形时，不需要 controller_manager / 硬件插件。
    这些节点会让 robot_description 有多个来源，之前踩过"双栈冲突"的坑。
    这个 launch 只起 3 个节点，职责唯一：
        robot_state_publisher  （发布 /robot_description + /tf）
        joint_state_publisher  （发布 /joint_states，默认全 0）
        rviz2                  （按 display.rviz 显示）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    desc_pkg = get_package_share_directory("hw_arm_description")
    xacro_file = os.path.join(desc_pkg, "urdf", "hw_arm.urdf.xacro")
    rviz_file = os.path.join(desc_pkg, "rviz", "display.rviz")

    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_fake_hardware:=", use_fake,
        ]),
        value_type=str,
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui" if gui else "joint_state_publisher",
            executable="joint_state_publisher_gui" if gui else "joint_state_publisher",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="true=带关节滑条（需要显示环境）"),
        OpaqueFunction(function=launch_setup),
    ])
