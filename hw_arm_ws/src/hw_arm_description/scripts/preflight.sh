#!/usr/bin/env bash
# 采集前的环境自检 —— 每项独立报告 OK / FAIL，**不做任何修改**
#
# ⚠️ 为什么需要（2026-11-26 真机踩到）：
#   电机没上电时启动 bringup，硬件 on_activate 会拒绝使能（这是对的 ——
#   否则会给可能已经掉下来的机械臂发指令），于是 joint_state_broadcaster
#   拿不到 state interface，STRICT 模式把控制器全部释放。
#   结果 `ros2 control list_controllers` 显示 inactive / unconfigured，
#   看起来像"控制器坏了"，其实**只是电机没电**。
#   这个脚本把"电机没电"和"控制器坏了"分开报，避免误判。
#
# 用法：./preflight.sh        全部检查
#       ./preflight.sh --quick  跳过电机探询（约省 5 秒）
# ⚠️ 不要 set -u：source /opt/ros/jazzy/setup.bash 里有未定义变量，
#    开了 -u 会在 source 那一步直接退出，脚本什么也不输出（踩过）。
#    所以下面的变量一律用 ${VAR:-} 形式。
HERE="$(cd "$(dirname "$0")" && pwd)"
set +u
source "$(dirname "$0")/env.sh"

QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

FAIL=0
ok()   { echo "  ✅ $1"; }
bad()  { echo "  ❌ $1"; FAIL=$((FAIL + 1)); }
warn() { echo "  ⚠️  $1"; }

# 用 comm（进程名，15 字符截断）而不是 cmdline 匹配，避免 pgrep -f 匹配到自己
count_node() { ps -eo comm= | grep -cx "ros2_control_no" || true; }
count_rsp()  { ps -eo comm= | grep -cx "robot_state_pub" || true; }
count_launch() {
  ps -eo args= | grep -c "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch hw_arm_bringup" || true
}

echo "════ ① CAN 接口 ════"
if ip link show can0 >/dev/null 2>&1; then
  if ip -br link show can0 | grep -q "UP"; then
    ok "can0 存在且 UP"
  else
    bad "can0 存在但未 UP —— sudo ip link set can0 up type can bitrate 1000000"
  fi
else
  bad "can0 不存在 —— 先跑 can_up.sh"
fi

echo "════ ② 电机是否上电 ════"
# ⚠️ bringup 在跑的时候**不能**用 raw CAN 去扫电机：
#    ros2_control 已经占着 can0 在 250 Hz 收发，扫描进程的查询会被淹没，
#    结果表现为"0/6 在线"，但电机其实是好的（joint_states 一直在更新）。
#    所以分两种情况：
#      没节点 → raw CAN 扫描（能直接判断电机有没有电）
#      有节点 → 看 /joint_states 的 effort 是否非零、position 是否非全 0
N_NODE_EARLY=$(count_node); N_NODE_EARLY=${N_NODE_EARLY:-0}
if [ "$N_NODE_EARLY" -ge 1 ]; then
  JS=$(timeout 10 ros2 topic echo --once /joint_states 2>/dev/null || true)
  if [ -z "$JS" ]; then
    bad "收不到 /joint_states，无法判断电机（bringup 在跑但没数据）"
  else
    EFF=$(printf '%s' "$JS" | sed -n '/^effort:/,/^---/p' | grep -c '^- ' || true)
    NONZERO=$(printf '%s' "$JS" | sed -n '/^effort:/,/^---/p' | grep -E '^- (0\.0+|-0\.0+)$' -c || true)
    if [ "${EFF:-0}" -ge 6 ] && [ "${NONZERO:-6}" -lt 6 ]; then
      ok "effort 有 ${EFF} 个值且不全为 0（电机在出力）"
    else
      bad "effort 全为 0 或数量不足 —— 电机可能失能/没电"
    fi
  fi
elif [ "$QUICK" -eq 0 ] && ip -br link show can0 2>/dev/null | grep -q "UP"; then
  SCAN=$(timeout 40 python3 "$HERE/scan_motors.py" --ids 1,2,3,4,5,6 2>&1 || true)
  N_ON=$(printf '%s\n' "$SCAN" | grep -c "✅ ID" || true)
  if [ "${N_ON:-0}" -ge 6 ]; then
    ok "6 台电机全部在线"
  else
    bad "只有 ${N_ON:-0}/6 台在线 —— **电机电源没开？**（这是最常见的原因）"
  fi
else
  warn "跳过（--quick 或 can0 不可用）"
fi

echo "════ ③ ros2_control_node 数量 ════"
N_NODE=$(count_node); N_NODE=${N_NODE:-0}
case "$N_NODE" in
  0) warn "0 个 —— bringup 还没启动（下一步：./bringup_once.sh）" ;;
  1) ok "恰好 1 个" ;;
  *) bad "$N_NODE 个！多个节点会抢同一块 CAN —— 先全部 kill 再重启" ;;
esac
N_RSP=$(count_rsp); N_RSP=${N_RSP:-0}
# ⚠️ 残留的 robot_state_publisher 是**致命**的：
#    它们一直往 /robot_description 发**旧的** URDF，
#    而 ros2_control_node 只取**第一条**收到的描述。
#    结果：参数时而生效、时而不生效（重力补偿一时开一时关），
#    日志里看不出任何错误。2026-11-26 被这个坑了两个小时。
if [ "$N_RSP" -eq 0 ]; then
  ok "没有残留的 robot_state_publisher"
elif [ "$N_RSP" -eq 1 ]; then
  ok "robot_state_publisher 1 个"
else
  bad "robot_state_publisher ${N_RSP} 个！残留的会发旧 URDF 覆盖参数 —— 必须先 kill 掉"
fi
N_LAUNCH=$(count_launch); N_LAUNCH=${N_LAUNCH:-0}
[ "$N_LAUNCH" -le 1 ] || bad "ros2 launch 有 ${N_LAUNCH} 个（重复启动）"

echo "════ ④ 控制器 ════"
if [ "$N_NODE" -ge 1 ]; then
  CTRL=$(timeout 10 ros2 control list_controllers 2>/dev/null || true)
  if printf '%s' "$CTRL" | grep -q "joint_state_broadcaster.*active"; then
    ok "joint_state_broadcaster active"
  else
    bad "joint_state_broadcaster 不是 active —— 常见原因：**启动时电机没电**"
  fi
  if printf '%s' "$CTRL" | grep -q "hw_arm_forward_position_controller.*active"; then
    ok "hw_arm_forward_position_controller active"
  else
    bad "hw_arm_forward_position_controller 不是 active"
  fi
else
  warn "跳过（bringup 没起）"
fi

echo "════ ⑤ /joint_states ════"
if [ "$N_NODE" -ge 1 ]; then
  NPUB=$(timeout 8 ros2 topic info /joint_states 2>/dev/null \
         | grep -o 'Publisher count: [0-9]*' | grep -o '[0-9]*' || true)
  if [ "${NPUB:-0}" = "1" ]; then
    NAMES=$(timeout 8 ros2 topic echo --once /joint_states 2>/dev/null \
            | sed -n '/^name:/,/^position:/p' | grep -c '^- ' || true)
    if [ "${NAMES:-0}" = "6" ]; then
      ok "1 个发布者、6 个关节"
    else
      bad "发布者正确但关节数 = ${NAMES:-0}（应为 6）"
    fi
  else
    bad "发布者数量 = ${NPUB:-0}（应为 1）"
  fi
else
  warn "跳过（bringup 没起）"
fi

echo
if [ "$FAIL" -eq 0 ]; then
  echo "════ ✅ 全部通过 ════"
  exit 0
else
  echo "════ ❌ $FAIL 项未通过，先修好再采集 ════"
  exit 1
fi
