#!/usr/bin/env python3
"""hw_arm_gazebo.launch.py —— 在 gz sim (Harmonic) 里起 hw_arm + ros2_control

用法：
    ros2 launch hw_arm_gazebo hw_arm_gazebo.launch.py
    ros2 launch hw_arm_gazebo hw_arm_gazebo.launch.py gui:=false            # 无界面
    ros2 launch hw_arm_gazebo hw_arm_gazebo.launch.py spawn_controllers:=true

⚠️ 首次使用前，description 里必须加一个 gz 硬件插件分支（见 README）：
      <xacro:if value="${use_gazebo}">
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </xacro:if>
   否则机器人能被 spawn 出来，但 /controller_manager 不会出现，控制器起不来。
   本 launch 已经把 use_gazebo:=true 传进去了，description 补上分支即生效。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = FindPackageShare("hw_arm_gazebo")
    pkg_description = FindPackageShare("hw_arm_description")

    # ── 可调参数 ──
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    controllers_file = LaunchConfiguration("controllers_file")
    spawn_controllers = LaunchConfiguration("spawn_controllers")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_height = LaunchConfiguration("initial_height")

    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "hw_arm.urdf.xacro"])

    # use_fake_hardware:=false → 走真机那支；use_gazebo:=true → 让 description
    # 选择 gz_ros2_control/GazeboSimSystem（需要 description 支持，见 README）
    robot_description = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", xacro_file,
            " use_fake_hardware:=false",
            " use_gazebo:=true",
        ]
    ), value_type=str)

    # ── gz sim ──
    # 注意：ros_gz_sim 的 gz_sim.launch.py **没有** gui 参数，
    # 要靠 gz_args 里的 -s 区分「只跑 server」和「server+GUI」。
    gz_launch_src = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    )

    gz_sim_gui = IncludeLaunchDescription(
        gz_launch_src,
        launch_arguments={
            "gz_args": ["-r -v ", verbose, " ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )
    gz_sim_server = IncludeLaunchDescription(
        gz_launch_src,
        launch_arguments={
            "gz_args": ["-r -s -v ", verbose, " ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(gui),
    )

    # ── 发布 TF / robot_description ──
    # URDF 里的 mesh 是 model://hw_arm_description/meshes/xxx.stl，
    # 需要让 gz 能在 <install>/share 下找到名为 hw_arm_description 的目录
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.dirname(get_package_share_directory("hw_arm_description")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    # ── 把机器人 spawn 进 gz（从 robot_description 话题读）──
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "hw_arm",
            "-z", initial_height,
        ],
    )

    # ── 控制器 spawner：等实体 spawn 完成 + gz 里的 controller_manager 起来 ──
    def spawner(controller: str) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(spawn_controllers),
        )

    jsb_spawner = spawner("joint_state_broadcaster")
    effort_spawner = spawner("hw_arm_effort_controller")

    # gz_ros2_control 的 controller_manager 在 spawn 之后才出现，给 3 秒余量
    delayed_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[jsb_spawner, effort_spawner],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution([pkg_gazebo, "worlds", "hw_arm.sdf"]),
                description="gz sim 世界文件（.sdf）",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [pkg_gazebo, "config", "hw_arm_gazebo_controllers.yaml"]
                ),
                description="控制器配置（由 description 的 gz 插件读取）",
            ),
            DeclareLaunchArgument("gui", default_value="true", description="是否开 gz 图形界面"),
            DeclareLaunchArgument("verbose", default_value="3", description="gz 日志等级 1-4"),
            DeclareLaunchArgument(
                "spawn_controllers",
                default_value="false",
                description="是否自动起控制器（description 补 gz 插件前请保持 false）",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="使用 gz 的仿真时钟"
            ),
            DeclareLaunchArgument(
                "initial_height", default_value="0.0", description="机器人 spawn 的 z 高度"
            ),
            gz_resource_path,
            gz_sim_gui,
            gz_sim_server,
            robot_state_publisher,
            spawn_entity,
            delayed_spawners,
        ]
    )
