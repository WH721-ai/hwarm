#!/usr/bin/env bash
# hw_arm 实时环境一键配置与验证
#
# 用法：
#   bash rt_setup.sh setup     # 需要 sudo：修 setcap 破坏 + 配 limits + 装开机调优服务
#   sudo reboot                # 重启（limits 只在登录时生效）
#   bash rt_setup.sh verify    # 重启后验证
#
# ═══════════════════════════════════════════════════════════════════
# 为什么要这个脚本
# ═══════════════════════════════════════════════════════════════════
# 2026-11-27 踩过的两个坑：
#
#  ① 在 ROS 二进制上设 file capability（setcap cap_sys_nice+ep）会让
#     glibc 加载器进入 secure execution 模式，**忽略 LD_LIBRARY_PATH**。
#     ROS 的二进制几乎都靠 LD_LIBRARY_PATH 找 libbackward.so 之类的库，
#     结果直接 exit code 127：
#         error while loading shared libraries: libbackward.so
#     ⇒ 正确做法是用 RLIMIT_RTPRIO（/etc/security/limits.d/），不要 setcap。
#
#  ② /etc/security/limits.d 的限制**只在登录时由 PAM 生效**，
#     开新终端、su 都不算，必须真正重新登录或重启。
#
# ⚠️ 本脚本**不碰机械臂**：不使能电机、不发运动命令、不启动控制节点。
#    verify 阶段只做读取和 cyclictest（纯 CPU 测试）。

set -o pipefail

ROS_BIN="/opt/ros/jazzy/lib/controller_manager/ros2_control_node"
LIMITS_FILE="/etc/security/limits.d/99-realtime.conf"
SERVICE_FILE="/etc/systemd/system/hw-arm-rt-tune.service"
GOVERNOR_TARGET="performance"

C_OK="\033[32m"; C_WARN="\033[33m"; C_ERR="\033[31m"; C_DIM="\033[2m"; C_RST="\033[0m"
PASS=0; FAIL=0; SKIP=0

ok()   { printf "  ${C_OK}✅${C_RST} %s\n" "$*"; PASS=$((PASS+1)); }
FAILED_ITEMS=()
bad()  { printf "  ${C_ERR}❌${C_RST} %s\n" "$*"; FAIL=$((FAIL+1)); FAILED_ITEMS+=("$*"); }
warn() { printf "  ${C_WARN}⚠️${C_RST}  %s\n" "$*"; SKIP=$((SKIP+1)); }
dim()  { printf "  ${C_DIM}%s${C_RST}\n" "$*"; }

need_root() {
  if [ "$(id -u)" -ne 0 ]; then
    echo "❌ setup 需要 root。请用： sudo bash $0 setup"
    exit 1
  fi
}

# ─────────────────────────────────────────────────────────────
# setup
# ─────────────────────────────────────────────────────────────
do_setup() {
  need_root
  echo "══════════════════════════════════════════════════"
  echo " 实时环境配置（改完需要重启）"
  echo "══════════════════════════════════════════════════"

  # ① 修 setcap 造成的破坏
  echo
  echo "【1】清理 ROS 二进制上的 file capability"
  if [ -e "$ROS_BIN" ]; then
    if getcap "$ROS_BIN" 2>/dev/null | grep -q .; then
      dim "当前: $(getcap "$ROS_BIN")"
      setcap -r "$ROS_BIN" && ok "已移除 capability（否则加载器会忽略 LD_LIBRARY_PATH）"
    else
      ok "本来就没有 capability，无需处理"
    fi
  else
    bad "找不到 $ROS_BIN"
  fi

  # ② limits.d
  echo
  echo "【2】配置实时权限（RLIMIT_RTPRIO / MEMLOCK）"
  LOCAL_USER="${SUDO_USER:-$USER}"
  cat > "$LIMITS_FILE" <<'LIMEOF'
# hw_arm 实时权限
# 允许任意用户拿 SCHED_FIFO 到 99，并允许锁定内存（避免缺页抖动）
# ⚠️ 这些限制只在**登录时**由 PAM 生效 —— 改完必须重新登录/重启
*  -  rtprio   99
*  -  memlock  unlimited
LIMEOF
  ok "已写入 $LIMITS_FILE"
  dim "rtprio 99 / memlock unlimited"

  # ③ systemd 的系统级默认限制（否则某些启动路径拿不到）
  echo
  echo "【3】放宽 systemd 的默认 RT 限制"
  local changed=0
  for kv in "DefaultLimitRTPRIO=99" "DefaultLimitMEMLOCK=infinity"; do
    local key="${kv%%=*}"
    if grep -qE "^${key}=" /etc/systemd/system.conf 2>/dev/null; then
      if ! grep -qE "^${key}=${kv#*=}$" /etc/systemd/system.conf; then
        sed -i "s|^${key}=.*|${kv}|" /etc/systemd/system.conf && changed=1
      fi
    else
      # 取消注释或追加
      if grep -qE "^#${key}=" /etc/systemd/system.conf 2>/dev/null; then
        sed -i "s|^#${key}=.*|${kv}|" /etc/systemd/system.conf && changed=1
      else
        echo "$kv" >> /etc/systemd/system.conf && changed=1
      fi
    fi
  done
  if [ "$changed" -eq 1 ]; then
    systemctl daemon-reexec 2>/dev/null
    ok "已更新 systemd 默认限制并 daemon-reexec"
  else
    ok "systemd 默认限制已是目标值"
  fi

  # ④ 运行期调优的开机服务（governor / nmi_watchdog 重启会丢）
  echo
  echo "【4】装开机调优服务（governor / NMI watchdog / cpu_dma_latency）"
  cat > "$SERVICE_FILE" <<SVCEOF
[Unit]
Description=hw_arm realtime tuning (governor, nmi_watchdog, cpu_dma_latency)
# 尽量早，但要在 sysfs/cpufreq 就绪之后
After=sysinit.target local-fs.target
Wants=sysinit.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\\
  for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do \\
    [ -w "\$g" ] && echo ${GOVERNOR_TARGET} > "\$g"; done; \\
  command -v cpupower >/dev/null && cpupower frequency-set -g ${GOVERNOR_TARGET} >/dev/null 2>&1 || true; \\
  echo 0 > /proc/sys/kernel/nmi_watchdog 2>/dev/null || true'

[Install]
WantedBy=multi-user.target
SVCEOF
  systemctl daemon-reload
  systemctl enable hw-arm-rt-tune.service 2>/dev/null \
    && ok "已安装并启用 hw-arm-rt-tune.service" \
    || bad "启用服务失败"

  # ⑤ 立即生效一次（不用等重启）
  echo
  echo "【5】立即应用一次（本次登录内）"
  cpupower frequency-set -g "$GOVERNOR_TARGET" >/dev/null 2>&1 \
    && ok "governor → $GOVERNOR_TARGET" || warn "cpupower 设置失败"
  echo 0 > /proc/sys/kernel/nmi_watchdog 2>/dev/null \
    && ok "nmi_watchdog → 0" || warn "nmi_watchdog 设置失败"

  # ⑥ 检查 C-state / preempt（这两个只能改内核启动参数）
  echo
  echo "【6】内核启动参数检查（这两项改不了，只能提示）"
  local mc; mc=$(cat /sys/module/intel_idle/parameters/max_cstate 2>/dev/null)
  if [ "$mc" = "1" ]; then ok "intel_idle.max_cstate = 1"; else
    warn "intel_idle.max_cstate = ${mc:-未知}（建议在 GRUB 里加 intel_idle.max_cstate=1）"
  fi
  if grep -q "idle=poll" /proc/cmdline; then
    ok "idle=poll 已启用（延迟最低，但发热大）"
  else
    dim "未使用 idle=poll（如有需要可加到 GRUB）"
  fi

  # ⑦ 确认二进制能正常加载（这是 setcap 修复的验证）
  echo
  echo "【7】确认 ros2_control_node 能正常加载动态库"
  if [ -e "$ROS_BIN" ]; then
    if ldd "$ROS_BIN" 2>/dev/null | grep -q "not found"; then
      bad "仍有库找不到："
      ldd "$ROS_BIN" 2>/dev/null | grep "not found" | sed 's/^/       /'
    else
      ok "所有依赖库都能解析"
    fi
  fi

  cat <<EOF

══════════════════════════════════════════════════
 ${C_OK}配置完成${C_RST}
══════════════════════════════════════════════════
 下一步：${C_WARN}必须重启${C_RST}（limits 只在登录时生效）

     sudo reboot

 重启后验证：

     bash $0 verify
EOF
}

# ─────────────────────────────────────────────────────────────
# verify
# ─────────────────────────────────────────────────────────────
do_verify() {
  echo "══════════════════════════════════════════════════"
  echo " 实时环境验证"
  echo "══════════════════════════════════════════════════"

  # 【1】limits 是否生效
  echo
  echo "【1】实时权限（limits）"
  local rtp mlk
  rtp=$(ulimit -r 2>/dev/null)
  mlk=$(ulimit -l 2>/dev/null)
  if [ -n "$rtp" ] && [ "$rtp" -ge 99 ] 2>/dev/null; then
    ok "ulimit -r = $rtp"
  else
    bad "ulimit -r = ${rtp:-0}（期望 ≥99）—— 改完 limits 后**重启过吗**？"
  fi
  # memlock 单位是 KB。上限 >=64 MB 对控制环已经足够（不锁 KB 级大内存）
  local mlk_ok=0
  if [ "$mlk" = "unlimited" ]; then
    mlk_ok=1
  elif [ -n "$mlk" ] && [ "$mlk" -ge 65536 ] 2>/dev/null; then
    mlk_ok=1
  fi
  if [ "$mlk_ok" -eq 1 ]; then
    ok "ulimit -l = $mlk（足够）"
  else
    bad "ulimit -l = ${mlk:-?}（期望 unlimited 或 ≥65536 KB）"
  fi

  # 【2】ROS 二进制没有被 capability 破坏
  echo
  echo "【2】ROS 二进制状态"
  if [ -e "$ROS_BIN" ]; then
    if getcap "$ROS_BIN" 2>/dev/null | grep -q .; then
      bad "仍有 file capability：$(getcap "$ROS_BIN")"
      dim "  会让加载器忽略 LD_LIBRARY_PATH → libbackward.so 找不到"
      dim "  修：sudo setcap -r $ROS_BIN"
    else
      ok "没有 file capability（正确）"
    fi
    if ldd "$ROS_BIN" 2>/dev/null | grep -q "not found"; then
      bad "有依赖库找不到："
      ldd "$ROS_BIN" 2>/dev/null | grep "not found" | sed 's/^/       /'
    else
      ok "依赖库全部可解析"
    fi
  else
    warn "找不到 $ROS_BIN"
  fi

  # 【3】CPU 调频
  echo
  echo "【3】CPU 调频与空闲策略"
  local gov bad_gov=0
  for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -e "$f" ] || continue
    gov=$(cat "$f" 2>/dev/null)
    [ "$gov" = "$GOVERNOR_TARGET" ] || bad_gov=$((bad_gov+1))
  done
  if [ "$bad_gov" -eq 0 ]; then
    ok "所有核 governor = $GOVERNOR_TARGET"
  else
    bad "$bad_gov 个核 governor 不是 $GOVERNOR_TARGET"
    dim "  修：sudo cpupower frequency-set -g $GOVERNOR_TARGET"
    dim "      或 systemctl restart hw-arm-rt-tune"
  fi
  local cur max
  cur=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)
  max=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null)
  [ -n "$cur" ] && [ -n "$max" ] && dim "cpu0: $((cur/1000)) / $((max/1000)) MHz"

  local mc; mc=$(cat /sys/module/intel_idle/parameters/max_cstate 2>/dev/null)
  if [ "$mc" = "1" ]; then ok "intel_idle.max_cstate = 1"; else
    warn "intel_idle.max_cstate = ${mc:-未知}（建议 GRUB 加 intel_idle.max_cstate=1）"
  fi
  grep -q "idle=poll" /proc/cmdline \
    && ok "idle=poll 已启用" \
    || dim "未使用 idle=poll"

  # 【4】抢占模式 / NMI watchdog / THP
  echo
  echo "【4】内核实时相关设置"
  local pm=""
  [ -r /sys/kernel/debug/sched/preempt ] && pm=$(cat /sys/kernel/debug/sched/preempt 2>/dev/null)
  if [ -n "$pm" ]; then
    case "$pm" in
      full*) ok "抢占模式 = $pm" ;;
      *) warn "抢占模式 = $pm（建议 GRUB 加 preempt=full）" ;;
    esac
  else
    dim "读不到 /sys/kernel/debug/sched/preempt（需 sudo 或该内核不支持）"
  fi
  local nmi; nmi=$(cat /proc/sys/kernel/nmi_watchdog 2>/dev/null)
  if [ "$nmi" = "0" ]; then ok "nmi_watchdog = 0"; else
    bad "nmi_watchdog = ${nmi:-?}（期望 0）"
    dim "  修：sudo sysctl -w kernel.nmi_watchdog=0"
  fi
  local thp; thp=$(cat /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null)
  case "$thp" in
    *\[never\]*) ok "transparent_hugepage = never" ;;
    *) dim "transparent_hugepage: $thp（影响不大，可选）" ;;
  esac

  # 【5】服务状态
  echo
  echo "【5】开机调优服务"
  if systemctl is-enabled hw-arm-rt-tune.service >/dev/null 2>&1; then
    ok "hw-arm-rt-tune.service 已启用"
    systemctl is-active hw-arm-rt-tune.service >/dev/null 2>&1 \
      && ok "服务当前 active" || warn "服务 enabled 但未 active"
  else
    warn "hw-arm-rt-tune.service 未安装（先跑 setup）"
  fi

  # 【6】cyclictest
  echo
  echo "【6】抖动实测（cyclictest，30 秒）"
  if ! command -v cyclictest >/dev/null; then
    warn "cyclictest 未装：sudo apt install rt-tests"
  else
    dim "（需要 sudo 才能拿 SCHED_FIFO，下面会提示输密码）"
    if sudo -n true 2>/dev/null || [ -t 0 ]; then
      local out
      # -q = 退出时才打印汇总，避免每秒一行刷屏
      out=$(sudo cyclictest -q -t1 -p 99 -i 1000 -D 30s -m -a 8 2>/dev/null | grep -E "^T:" | tail -1)
      if [ -n "$out" ]; then
        echo "  $out"
        local mx
        mx=$(echo "$out" | grep -oP 'Max:\s*\K[0-9]+' | tail -1)
        if [ -n "$mx" ]; then
          if [ "$mx" -le 200 ]; then
            ok "Max = ${mx} µs（优秀，125 Hz 只占 $((mx*100/8000))%）"
          elif [ "$mx" -le 1000 ]; then
            ok "Max = ${mx} µs（良好，125 Hz 可承受）"
          elif [ "$mx" -le 5000 ]; then
            warn "Max = ${mx} µs（勉强）"
          else
            bad "Max = ${mx} µs（过大，需要排查）"
          fi
        fi
      else
        warn "cyclictest 没输出（可能取消密码了）"
      fi
    else
      warn "无 sudo 权限，跳过 cyclictest"
      dim "  手动跑：sudo cyclictest -t1 -p 99 -i 1000 -D 30s -m -a 8"
    fi
  fi

  # 汇总
  echo
  echo "══════════════════════════════════════════════════"
  if [ "$FAIL" -eq 0 ]; then
    printf " ${C_OK}✅ 全部通过${C_RST}（%d 项通过，%d 项提示）\n" "$PASS" "$SKIP"
    echo
    echo " 可以跑控制节点了。建议第一次用纯位置控制："
    echo "   ros2 launch hw_arm_bringup hw_arm.launch.py \\"
    echo "       use_fake_hardware:=false can_interface:=can0 rviz:=false \\"
    echo "       controllers:=joint_state_broadcaster,hw_arm_forward_position_controller \\"
    echo "       gravity_compensation:=false return_to_zero:=false"
    echo
    echo " 期望：不再出现 'Could not enable FIFO RT scheduling policy'"
  else
    printf " ${C_ERR}❌ %d 项未通过${C_RST}（%d 通过，%d 提示）\n" "$FAIL" "$PASS" "$SKIP"
    if [ "${#FAILED_ITEMS[@]}" -gt 0 ]; then
      echo " 未通过项："
      for it in "${FAILED_ITEMS[@]}"; do echo "   · $it"; done
    fi
  fi
  echo "══════════════════════════════════════════════════"
  [ "$FAIL" -eq 0 ]
}

# ─────────────────────────────────────────────────────────────
case "${1:-}" in
  setup)  do_setup  ;;
  verify) do_verify ;;
  *)
    cat <<EOF
用法:
  sudo bash $0 setup     # 一次性配置（会提示重启）
  sudo reboot
  bash $0 verify         # 重启后验证

它做什么:
  setup : 移除 ROS 二进制上的 file capability（那个会破坏动态库加载）
          写 /etc/security/limits.d/99-realtime.conf（rtprio 99 + memlock）
          放宽 systemd 默认 RT 限制
          装 hw-arm-rt-tune.service（开机自动设 governor / nmi_watchdog）
  verify: 检查 ulimit / governor / cstate / preempt / 服务 / 二进制可加载性
          并跑 30 秒 cyclictest

⚠️ 全程不碰机械臂：不使能电机、不发运动命令、不启动控制节点。
EOF
    exit 2
    ;;
esac
