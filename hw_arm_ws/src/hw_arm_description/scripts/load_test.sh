#!/usr/bin/env bash
# 控制环满载测试 —— 验证"CPU 被压满时 125 Hz 还稳不稳"
#
# 用法：
#   bash load_test.sh                 # 默认压 8 核、跑 60 秒
#   bash load_test.sh --cpus 12 --secs 90
#   bash load_test.sh --no-stress     # 只跑控制节点，不加负载（基线）
#
# ═══════════════════════════════════════════════════════════════════
# 它会做什么
# ═══════════════════════════════════════════════════════════════════
#   1. 检查没有残留的 ros2_control_node
#   2. 后台启动 hw_arm bringup（纯位置控制、关重力补偿、不回零）
#   3. 等控制器真正 active（不是靠 sleep 猜）
#   4. 记录起始时间，按 --cpus 数量压核（绑在 cpu0..N-1，留出 P-core 给控制）
#   5. 压 --secs 秒，期间控制节点持续运行
#   6. 解除负载，统计
#   7. 用 SIGINT 正常关闭节点（这样日志里会出现 shutdown 记录，证明它确实跑过）
#   8. 输出报告：运行时长 / 实际 overrun 次数 / 分解 / 结论
#
# ⚠️ 会**使能电机**并保持当前位置（kp=70）。运行前请确认：
#    · 机械臂已装机、周围清空
#    · 不要用 return_to_zero（脚本已强制 false）
#    · 第一次跑建议有人在场
#
# ⚠️ 不碰电机运动：整场只有 joint_state_broadcaster + forward_position_controller，
#    没有任何轨迹命令，机械臂保持上电时的位形不动。

set -o pipefail

WS="$HOME/hw_arm/hw_arm_ws"
SCRIPTS="$WS/src/hw_arm_description/scripts"
LOG="/tmp/hw_arm_load_test.log"
CPUS=8
SECS=60
NO_STRESS=0
CAN="can0"

C_OK="\033[32m"; C_WARN="\033[33m"; C_ERR="\033[31m"; C_DIM="\033[2m"; C_RST="\033[0m"
ok()   { printf "  ${C_OK}✅${C_RST} %s\n" "$*"; }
bad()  { printf "  ${C_ERR}❌${C_RST} %s\n" "$*"; }
warn() { printf "  ${C_WARN}⚠️${C_RST}  %s\n" "$*"; }
dim()  { printf "  ${C_DIM}%s${C_RST}\n" "$*"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --cpus) CPUS="$2"; shift 2 ;;
    --secs) SECS="$2"; shift 2 ;;
    --can)  CAN="$2"; shift 2 ;;
    --no-stress) NO_STRESS=1; shift ;;
    -h|--help) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1"; exit 2 ;;
  esac
done

# ── 前置检查
echo "══════════════════════════════════════════════════"
echo " 控制环满载测试"
echo "  负载: $([ "$NO_STRESS" -eq 1 ] && echo '不加压（基线）' || echo "压 $CPUS 核")  时长: ${SECS}s"
echo "══════════════════════════════════════════════════"

if ! [ -f "$SCRIPTS/env.sh" ]; then
  bad "找不到 $SCRIPTS/env.sh"
  exit 1
fi
# shellcheck disable=SC1090
source "$SCRIPTS/env.sh" >/dev/null 2>&1

leftover=$(ps -eo comm= | grep -cx "ros2_control_no" || true)
if [ "${leftover:-0}" -gt 0 ]; then
  bad "还有 $leftover 个 ros2_control_node 在跑。先清干净："
  dim "  pkill -9 -x robot_state_pub; pkill -9 -x ros2_control_no; sleep 2"
  exit 1
fi
ok "无残留 ros2_control_node"

if ! ip -br link show "$CAN" >/dev/null 2>&1; then
  bad "$CAN 不存在（适配器插了吗？）"
  exit 1
fi
ok "$CAN 存在"

# ── 启动控制节点（后台）
: > "$LOG"
echo
echo "启动 bringup（$LOG）..."
ros2 launch hw_arm_bringup hw_arm.launch.py \
    use_fake_hardware:=false can_interface:="$CAN" rviz:=false \
    controllers:=joint_state_broadcaster,hw_arm_forward_position_controller \
    gravity_compensation:=false \
    return_to_zero:=false > "$LOG" 2>&1 &
LP=$!

# ── 等控制器真正 active
echo -n "等待控制器 active"
ACTIVE=0
for _ in $(seq 40); do
  sleep 0.5
  if grep -q "Configured and activated hw_arm_forward_position_controller" "$LOG" 2>/dev/null; then
    ACTIVE=1; break
  fi
  if ! kill -0 "$LP" 2>/dev/null; then break; fi
  echo -n "."
done
echo
if [ "$ACTIVE" -ne 1 ]; then
  bad "控制器没起来。日志末尾："
  tail -15 "$LOG" | sed 's/^/     /'
  kill -INT "$LP" 2>/dev/null; sleep 2; kill -9 "$LP" 2>/dev/null
  exit 1
fi
ok "控制器已 active"

# 确认电机使能
if grep -q "已使能 6 台电机" "$LOG"; then
  ok "6 台电机已使能"
else
  warn "没看到'已使能 6 台电机'，可能只有部分就绪"
fi
printf "  ${C_OK}✅${C_RST} 启动阶段 overrun: %s 次\n" \
  "$(grep -cE 'Overrun detected|Overrun might occur' "$LOG" || true)"

# ── 加压
STRESS_PIDS=()
cleanup_stress() {
  for p in "${STRESS_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  STRESS_PIDS=()
}
trap 'cleanup_stress' EXIT

if [ "$NO_STRESS" -eq 0 ]; then
  echo
  echo "开始压 $CPUS 个核（cpu0..$((CPUS-1))），持续 ${SECS}s ..."
  for i in $(seq "$CPUS"); do
    taskset -c $((i-1)) bash -c 'while :; do :; done' >/dev/null 2>&1 &
    STRESS_PIDS+=($!)
  done
  ok "负载已启动（${#STRESS_PIDS[@]} 个进程）"
else
  echo
  echo "（不加负载，只跑基线）"
fi

T0=$(date +%s)
sleep "$SECS"
T1=$(date +%s)
cleanup_stress
ok "负载已解除"

# ── 关闭并统计
echo
echo "关闭控制节点（SIGINT，让它正常 shutdown）..."
kill -INT "$LP" 2>/dev/null
for _ in $(seq 20); do
  kill -0 "$LP" 2>/dev/null || break
  sleep 0.5
done
kill -9 "$LP" 2>/dev/null
# launch 的子进程可能残留
sleep 1
pkill -9 -x robot_state_pub 2>/dev/null || true
pkill -9 -x ros2_control_no 2>/dev/null || true

# ── 报告
DUR=$((T1-T0))
RUN_OVR=$(grep -cE 'Overrun detected|Overrun might occur' "$LOG" || true)
START_OVR=$(grep -nE 'Overrun detected|Overrun might occur' "$LOG" | head -1 | cut -d: -f1 || true)
SHUTDOWN=$(grep -c "Shutdown request received" "$LOG" || true)

echo
echo "══════════════════════════════════════════════════"
echo " 测试报告"
echo "══════════════════════════════════════════════════"
printf "  压测时长      : %d s（目标 %d s）%s\n" "$DUR" "$SECS" \
  "$([ "$DUR" -ge $((SECS-2)) ] && echo '' || echo '  ⚠️ 提前结束')"
printf "  负载          : %s\n" "$([ "$NO_STRESS" -eq 1 ] && echo 无 || echo "${CPUS} 核自旋")"
if [ "${SHUTDOWN:-0}" -gt 0 ]; then
  ok "日志有正常 shutdown 记录 → 证明节点整场都在跑"
else
  warn "日志没有 shutdown 记录（可能被 kill -9），时长以脚本计时为准"
fi
echo
printf "  Overrun 总数  : %s\n" "${RUN_OVR:-0}"
if [ "${RUN_OVR:-0}" -eq 0 ]; then
  ok "整场零 overrun"
else
  echo
  echo "  ── 详情 ──"
  grep -E 'Overrun detected|Overrun might occur' "$LOG" | head -10 | sed 's/^/    /'
  echo
  echo "  ── 分解（Read=收 CAN 慢, Write=发 CAN 慢）──"
  grep "Overrun might occur" "$LOG" | head -5 \
    | grep -oE "Read time : [0-9]+ us.*Write time : [0-9]+ us" | sed 's/^/    /' || true
fi
echo
echo "  完整日志: $LOG"
echo "══════════════════════════════════════════════════"

if [ "${RUN_OVR:-0}" -eq 0 ]; then
  printf " ${C_OK}结论：通信层在%s下稳定${C_RST}\n" \
    "$([ "$NO_STRESS" -eq 1 ] && echo '空载' || echo " ${CPUS} 核满载 ${SECS}s")"
  exit 0
fi
printf " ${C_WARN}结论：%s 次 overrun，看上面分解定位是读还是写慢${C_RST}\n" "$RUN_OVR"
exit 1
