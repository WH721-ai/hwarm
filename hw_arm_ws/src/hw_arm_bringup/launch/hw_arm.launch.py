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

    # 重力补偿 / 拖动示教开关（透传给 xacro）
    gcomp = LaunchConfiguration("gravity_compensation").perform(context)
    gdry = LaunchConfiguration("gravity_dry_run").perform(context)
    zg = LaunchConfiguration("zero_gravity").perform(context)
    gscale = LaunchConfiguration("gravity_scale").perform(context)
    gkd = LaunchConfiguration("zero_gravity_kd").perform(context)
    gkpf = LaunchConfiguration("zero_gravity_kp_floor").perform(context)
    gkde = LaunchConfiguration("zero_gravity_kd_each").perform(context)
    gramp = LaunchConfiguration("gravity_ramp_time").perform(context)
    gpath = LaunchConfiguration("gravity_params_path").perform(context)
    update_rate = int(LaunchConfiguration("update_rate").perform(context))
    gtrim = LaunchConfiguration("gravity_trim").perform(context)
    rtz = LaunchConfiguration("return_to_zero").perform(context)

    robot_description = ParameterValue(
        Command(
            [
                "xacro ", xacro_file,
                " use_fake_hardware:=", use_fake,
                " can_interface:=", can_iface,
                " can_fd:=", can_fd,
                " active_joints:=", active_joints,
                " gravity_compensation:=", gcomp,
                " gravity_dry_run:=", gdry,
                " zero_gravity:=", zg,
                " gravity_scale:=", gscale,
                " zero_gravity_kd:=", gkd,
                # ⚠️ 这三个值都可能是"6 个空格分隔的数"，必须加引号
                #    否则 xacro 会被拆成多个参数，报
                #    "expected exactly one input file as argument"
                " zero_gravity_kp_floor:='", gkpf, "'",
                " zero_gravity_kd_each:='", gkde, "'",
                " gravity_ramp_time:=", gramp,
                " gravity_params_path:=", gpath,
                # ⚠️ 值里有空格，必须加引号 —— 否则 xacro 会把
                #    "0 0 0 0 0 0" 拆成 6 个参数，报
                #    "expected exactly one input file as argument"
                " gravity_trim:='", gtrim, "'",
                " return_to_zero:=", rtz,
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
            # update_rate 用命令行 -p 覆盖 params 文件（cmdline 优先级最高）
            arguments=["--ros-args", "-p", f"update_rate:={update_rate}"],
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
        # ── 重力补偿 / 拖动示教（C-04 / C-05）
        DeclareLaunchArgument(
            "gravity_compensation", default_value="false",
            description="启用重力前馈 tau_ff = g(q)（kp 照常，机械臂变'轻'但仍然是硬的）"),
        DeclareLaunchArgument(
            "gravity_dry_run", default_value="true",
            description="只算 tau_ff 不施加（阶段 1 用）"),
        DeclareLaunchArgument(
            "zero_gravity", default_value="false",
            description="零重力拖动示教：kp=0 + tau_ff=g(q) + kd 阻尼。"
                        "此时**不要**启动轨迹控制器"),
        DeclareLaunchArgument(
            "gravity_params_path", default_value="",
            description="gravity_id_cli 产出的 theta YAML；空 = 用 URDF 标称惯性参数"),
        DeclareLaunchArgument("gravity_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "update_rate", default_value="250",
            description="控制环频率 Hz。**USB-CAN 吞吐有限，250 Hz 时总线已接近饱和**"
                        "（6 电机 × 250 Hz × 3 类帧 ≈ 3500 fps，实测上限约 3000 fps），"
                        "表现为偶发 4~6 ms 的 read/write 卡顿。"
                        "零重力拖拽建议 125，对 8 ms 的力矩刷新完全够用"),
        DeclareLaunchArgument("zero_gravity_kd", default_value="1.0"),
        DeclareLaunchArgument("zero_gravity_kp_floor", default_value="0.0"),
        DeclareLaunchArgument("zero_gravity_kd_each", default_value=""),
        DeclareLaunchArgument(
            "gravity_trim", default_value="0 0 0 0 0 0",
            description="恒定力矩偏置修正 N·m，逐轴。抵消电流零点温漂/线缆张力等"
                        "随运行状态漂移的量（辨识拟合不进去）。例如 '0 0.3 -0.55 0 0 0'"),
        DeclareLaunchArgument(
            "return_to_zero", default_value="true",
            description="上电自动回零。位置控制下是安全位形；"
                        "但零重力调试时**必须关掉**，因为零位是结构托住、补偿误差最大的地方"),
        DeclareLaunchArgument("gravity_ramp_time", default_value="3.0",
                              description="激活后重力从 0 升到满额的时间 s"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "controllers",
                default_value="joint_state_broadcaster,hw_arm_controller",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
