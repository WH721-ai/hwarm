#!/usr/bin/env bash
# 只启动**一个** hw_arm bringup
#
# ⚠️ 为什么需要这个脚本（2026-11-26 真机踩过）：
#   两个 ros2_control_node 同时打开 can0 并周期性下发 MIT 帧，
#   会互相覆盖指令 —— 电机在两组 setpoint 之间被抢来抢去。
#   而且 `ros2 control list_controllers` 只会连上其中一个，
#   看到的 "No controllers are currently loaded" 其实是另一个进程。
#
# 用法：
#   ./bringup_once.sh                     # 默认参数
#   ./bringup_once.sh rviz:=true          # 透传给 ros2 launch
set -u

# ⚠️ 用 "[r]os2_control_node" 而不是 "ros2_control_node"：
#    pgrep -f 会匹配到**自己**（以及 pkill 那条命令）的命令行，
#    直接用裸字符串会永远返回 ≥1，脚本就再也启动不了。
# pgrep -c 没有匹配时本身会打印 "0" 并返回 1，直接 || echo 0 会得到两个 0
N=$(pgrep -fc "[r]os2_control_node" 2>/dev/null || true); N=${N:-0}
if [ "${N:-0}" -gt 0 ]; then
  echo "❌ 已经有 $N 个 ros2_control_node 在跑，拒绝再启动一个。"
  echo
  pgrep -fa "[r]os2_control_node|[r]os2 launch hw_arm_bringup" | sed 's/^/   /'
  echo
  echo "先全部停掉："
  echo "  pkill -f '[r]os2_control_node'; pkill -f '[r]os2 launch hw_arm_bringup'; sleep 2"
  exit 1
fi

if ! ip link show can0 >/dev/null 2>&1; then
  echo "❌ can0 不存在。先跑 can_up.sh"
  exit 1
fi
if ! ip -br link show can0 | grep -q "UP"; then
  echo "❌ can0 不是 UP 状态"
  echo "  sudo ip link set can0 up type can bitrate 1000000"
  exit 1
fi

echo "✅ 只有一个 ros2_control_node 在跑（其实是 0 个），can0 UP"
echo "   启动 hw_arm bringup …"
exec ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:=can0 rviz:=false \
    controllers:=joint_state_broadcaster,hw_arm_forward_position_controller \
    "$@"
