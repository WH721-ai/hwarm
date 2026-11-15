#!/usr/bin/env python3
"""URDF 可视化验证 —— **转换 MuJoCo 之前必须先跑这个**。

════════════════════════════════════════════════════════════════════
为什么要单独做这一步？
════════════════════════════════════════════════════════════════════
MuJoCo 只能吃「几何 + 惯性 + 关节链」，它**不显示**：
    · 关节轴向（axis）
    · 关节限位（limit lower/upper）
    · 关节原点相对关系（origin xyz/rpy）
    · 惯性张量的旋转（inertial origin rpy）

一旦转成 MJCF 才发现关节轴错、原点偏、限位反，排查成本极高
（MJCF 里的 quat/pos 全被烘焙成数字，看不出原始意图）。

所以流程固定为两步：
    ① ros2 launch hw_arm_mujoco display.launch.py     ← 肉眼确认 URDF
    ② python3 scripts/urdf_to_mjcf.py                 ← 再转 MJCF

在第 ① 步里用 joint_state_publisher_gui 拉各个关节，确认：
    □ 每个关节转动的方向符合直觉（不是反的）
    □ 零位姿态与 CAD 装配一致
    □ 关节到限位附近时没有明显穿模
    □ mesh 完整、没有缺失或错位（缺 mesh 会在终端报错）
    □ 质量/惯量在 RViz 的 Mass Properties 里不是 0 或天文数字

════════════════════════════════════════════════════════════════════
用法
════════════════════════════════════════════════════════════════════
    ros2 launch hw_arm_mujoco display.launch.py
    ros2 launch hw_arm_mujoco display.launch.py rviz:=false   # 只看终端报错
    ros2 launch hw_arm_mujoco display.launch.py use_fake_hardware:=false

节点职责（与 hw_arm_bringup/display.launch.py 相同，放在这里是为了
让「验证 → 转换」两步都在本包内完成，不依赖其他包是否已构建）：
    robot_state_publisher      发布 /robot_description + /tf
    joint_state_publisher_gui  关节滑条 → /joint_states
    rviz2                      按 hw_arm_description 的 display.rviz 显示
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("hw_arm_description")
    xacro_file = os.path.join(desc_share, "urdf", "hw_arm.urdf.xacro")
    rviz_file = os.path.join(desc_share, "rviz", "display.rviz")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_fake_hardware:=", use_fake_hardware]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description="传给 xacro 的开关（mock 硬件为 true）"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="false = 只发 /robot_description 和 /tf，不开 RViz"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(rviz),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
            condition=IfCondition(rviz),
        ),
    ])
