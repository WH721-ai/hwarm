#!/usr/bin/env bash
# 统一的环境入口 —— **所有**终端都 source 这个，不要再手敲两个 source
#
#   source ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/env.sh
#
# ══════════════════════════════════════════════════════════════════════
# 为什么要固定 ROS_LOCALHOST_ONLY=1（2026-11-26 实测）
#
# 这台机器是 VMware 虚拟机，只有一块网卡 ens33（192.168.16.50/24，NAT）。
# Fast DDS 默认走**组播在 ens33 上发现**，表现是：
#   · `ros2 node list` 能看到所有节点（发现是通的）
#   · `ros2 control list_controllers` 也能查到
#   · 但 `/joint_states` 这种 250 Hz 的大流量话题会**间歇性收不到**
#     采集脚本因此报 "运动中关节状态中断"，而控制器其实一直是 active
#   实测：同一个采集进程，前两个位形正常，第三个位形突然 9.2 秒收不到任何消息，
#        过一会儿又自己好了。
#
# ROS_LOCALHOST_ONLY=1 让 Fast DDS 只用回环口，完全绕开 VMware 的 NAT 网络，
# 单机场景下这是最稳的配置。
# ⚠️ 必须**所有**节点都设，包括 bringup —— 混合设置会互相看不见。
# ══════════════════════════════════════════════════════════════════════

export ROS_LOCALHOST_ONLY=1

# 只 source 一次，避免重复叠加
if [ -z "${HW_ARM_ENV_LOADED:-}" ]; then
  source /opt/ros/jazzy/setup.bash
  if [ -f "$HOME/hw_arm/hw_arm_ws/install/setup.bash" ]; then
    source "$HOME/hw_arm/hw_arm_ws/install/setup.bash"
  fi
  export HW_ARM_ENV_LOADED=1
fi

# 常用路径
export HW_ARM_WS="$HOME/hw_arm/hw_arm_ws"
export HW_ARM_SCRIPTS="$HW_ARM_WS/src/hw_arm_description/scripts"
export HW_ARM_URDF="$HW_ARM_WS/install/hw_arm_description/share/hw_arm_description/urdf/hw_arm.urdf"
export HW_ARM_BIN="$HW_ARM_WS/build/hw_arm_algorithms"
