#!/usr/bin/env python3
"""用 ros2 launch 起 MuJoCo viewer（脚本版见 scripts/launch_mujoco.py）。

    ros2 launch hw_arm_mujoco mujoco.launch.py
    ros2 launch hw_arm_mujoco mujoco.launch.py mode:=torque
    ros2 launch hw_arm_mujoco mujoco.launch.py duration:=2.0

注意：本 launch **不做** URDF→MJCF 转换。若 mjcf/scene.xml 不存在，
先运行：python3 scripts/urdf_to_mjcf.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hw_arm_mujoco")
    script = os.path.join(share, "scripts", "launch_mujoco.py")
    scene = os.path.join(share, "mjcf", "scene.xml")

    mode = LaunchConfiguration("mode")
    duration = LaunchConfiguration("duration")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gravity",
                              choices=["gravity", "torque"],
                              description="gravity=自由下坠；torque=滑条手动给力矩"),
        DeclareLaunchArgument("duration", default_value="0",
                              description=">0 时跑这么多仿真秒后退出"),
        Node(
            executable=script,
            output="screen",
            arguments=[
                "--scene", scene,
                "--mode", mode,
                "--duration", duration,
            ],
        ),
    ])
