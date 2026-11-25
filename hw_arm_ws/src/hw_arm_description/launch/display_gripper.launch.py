#!/usr/bin/env python3
"""在 RViz 里显示 hw_arm + 滚轮夹爪（roller_gripper_asm7）。

用法：
    ros2 launch hw_arm_description display_gripper.launch.py
    ros2 launch hw_arm_description display_gripper.launch.py gui:=false   # 不弹滑条
    ros2 launch hw_arm_description display_gripper.launch.py rviz:=false  # 只起 rsp

夹爪开关由 xacro 的 with_gripper 参数控制（这里固定为 true）。
夹爪挂载位姿可用 gripper_xyz / gripper_rpy 覆盖。
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
    rviz_file = os.path.join(pkg, "rviz", "display_gripper.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    gxyz = LaunchConfiguration("gripper_xyz")
    grpy = LaunchConfiguration("gripper_rpy")

    robot_description = ParameterValue(
        # ⚠️ gripper_xyz / gripper_rpy 的值含空格，必须**加引号**，
        #    否则 xacro 会把 "0 0 0" 拆成三个参数（报 expected exactly one input file）
        Command([
            "xacro ", xacro_file,
            " with_gripper:=true",
            " gripper_xyz:='", gxyz, "'",
            " gripper_rpy:='", grpy, "'",
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否启动 joint_state_publisher_gui 滑条"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="是否启动 RViz"),
        DeclareLaunchArgument("gripper_xyz", default_value="0 0 0",
                              description="夹爪相对 tool0 的平移"),
        DeclareLaunchArgument("gripper_rpy", default_value="3.14159265 0 0",
                              description="夹爪相对 tool0 的旋转"),
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
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
            condition=IfCondition(rviz),
        ),
    ])
