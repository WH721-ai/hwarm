#!/usr/bin/env python3
"""hw_arm 单臂启动（mock / 真机通用）

用法：
    ros2 launch hw_arm_bringup hw_arm.launch.py                        # mock，带 RViz
    ros2 launch hw_arm_bringup hw_arm.launch.py use_fake_hardware:=false can_interface:=can0
    ros2 launch hw_arm_bringup hw_arm.launch.py rviz:=false            # 无 RViz

参数：
    use_fake_hardware  true=用 mock_components（默认），false=用 hw_arm_hardware/HwArmHW
    can_interface      SocketCAN 接口名，默认 can0
    can_fd             是否 CANFD，默认 true
    rviz               是否启动 RViz，默认 true
    controllers        要加载的控制器（逗号分隔），默认 joint_state_broadcaster,hw_arm_controller
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
    bringup_pkg = get_package_share_directory("hw_arm_bringup")

    xacro_file = os.path.join(desc_pkg, "urdf", "hw_arm.urdf.xacro")
    controllers_file = os.path.join(
        bringup_pkg, "config", "controllers", "hw_arm_controllers.yaml"
    )
    rviz_file = os.path.join(desc_pkg, "rviz", "display.rviz")

    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
    can_iface = LaunchConfiguration("can_interface").perform(context)
    can_fd = LaunchConfiguration("can_fd").perform(context)
    active_joints = LaunchConfiguration("active_joints").perform(context)
    want_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")
    controller_list = [
        c.strip()
        for c in LaunchConfiguration("controllers").perform(context).split(",")
        if c.strip()
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ", xacro_file,
                " use_fake_hardware:=", use_fake,
                " can_interface:=", can_iface,
                " can_fd:=", can_fd,
                " active_joints:=", active_joints,
            ]
        ),
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
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": robot_description}, controllers_file],
        ),
    ]

    # 逐个 spawn 控制器（顺序执行，jsb 先起）
    for name in controller_list:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[name, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    if want_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_file],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "can_fd",
                default_value="false",
                description="达妙电机只认经典 CAN 帧；接口也必须 fd off"),
            DeclareLaunchArgument("active_joints", default_value="1,2,3,4,5,6"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "controllers",
                default_value="joint_state_broadcaster,hw_arm_controller",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
