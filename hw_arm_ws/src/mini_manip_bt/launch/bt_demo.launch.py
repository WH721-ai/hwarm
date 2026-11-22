"""启动 BT 执行器（假设 ros2_control 已经在跑）

用法：
    # 终端 1：起机械臂（真机或 mock）
    ros2 launch hw_arm_bringup hw_arm.launch.py use_fake_hardware:=true

    # 终端 2：起 BT
    ros2 launch mini_manip_bt bt_demo.launch.py

    # 或者指定别的树文件
    ros2 launch mini_manip_bt bt_demo.launch.py tree:=/path/to/other.xml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("mini_manip_bt")
    default_tree = os.path.join(pkg, "trees", "goto_home.xml")

    return LaunchDescription([
        DeclareLaunchArgument("tree", default_value=default_tree,
                              description="行为树 XML 路径"),
        Node(
            package="mini_manip_bt",
            executable="bt_executor",
            name="bt_executor",
            output="screen",
            arguments=[LaunchConfiguration("tree")],
        ),
    ])
