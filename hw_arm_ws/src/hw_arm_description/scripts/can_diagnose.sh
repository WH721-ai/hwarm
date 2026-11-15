#!/usr/bin/env bash
# CAN 总线一键诊断（需要 sudo）
#
#   sudo bash ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/can_diagnose.sh [接口名]
#
# 做四件事：
#   ① 打印接口状态
#   ② 内部回环测试（验证适配器 USB→CAN 控制器通路，不需要外部节点）
#   ③ 正常模式下发 refresh 广播，扫 ID 1..8，看有没有电机应答
#   ④ 恢复接口到 CANFD 1M/5M
set -u

IF=${1:-can0}
PY=python3

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
ok()  { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad() { printf '  \033[31m✗\033[0m %s\n' "$*"; }

# ─────────────────────────────────────────── ① 接口状态
say "① 接口状态"
if ! ip link show "$IF" >/dev/null 2>&1; then
  bad "找不到 $IF —— 模块没插好，或固件不是 gs_usb 模式"
  lsusb | grep -iE "1d50:606f|gs_usb" || echo "  （lsusb 里也没有 gs_usb 设备）"
  exit 1
fi
ip -details link show "$IF" | sed -n '1,4p' | sed 's/^/  /'

# ─────────────────────────────────────────── ② 回环测试
say "② 内部回环测试（不依赖外部节点）"
ip link set "$IF" down 2>/dev/null
ip link set "$IF" type can bitrate 1000000 loopback on 2>/dev/null \
  || { bad "设置 loopback 失败（需要 root）"; }
ip link set "$IF" up 2>/dev/null

LOOP_RESULT=$($PY - "$IF" <<'PYEOF'
import socket, struct, sys
ifname = sys.argv[1]
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((ifname,))
s.settimeout(0.5)
payload = b"\x11\x22\x33\x44\x55\x66\x77\x88"
try:
    s.send(struct.pack("<IB3x8s", 0x123, 8, payload))
except OSError as e:
    print("SEND_FAIL", e); sys.exit(0)
try:
    raw = s.recv(16)
except socket.timeout:
    print("NO_ECHO"); sys.exit(0)
tf = raw[4]
data = raw[8:8+tf]
print("ECHO_OK" if data == payload else f"ECHO_BAD {data.hex(' ')}")
PYEOF
)

case "$LOOP_RESULT" in
  ECHO_OK)   ok "回环收到自己发的帧 → 适配器 USB / CAN 控制器正常" ;;
  NO_ECHO)   bad "回环收不到 → 适配器或 gs_usb 固件有问题" ;;
  SEND_FAIL*) bad "回环发送失败：${LOOP_RESULT#SEND_FAIL }" ;;
  *)         bad "回环异常：$LOOP_RESULT" ;;
esac

# ─────────────────────────────────────────── ③ 正常模式扫描
say "③ 正常模式扫描电机（ID 1..8）"
ip link set "$IF" down 2>/dev/null
# 注意：`type can` 只改指定参数，不指定 loopback off 的话它会一直留着
ip link set "$IF" type can bitrate 1000000 dbitrate 5000000 fd on \
        loopback off listen-only off one-shot off 2>/dev/null \
  || { bad "无法把 $IF 配成 CANFD 1M/5M"; }
ip link set "$IF" up 2>/dev/null
sleep 0.3

# 确认 ctrlmode 已清干净
CTRLMODE=$(ip -details link show "$IF" | sed -n '3p')
if echo "$CTRLMODE" | grep -qi "loopback"; then
  bad "仍然处于 LOOPBACK 模式，扫描结果无意义：$CTRLMODE"
  exit 1
fi
ok "ctrlmode 干净：$(echo "$CTRLMODE" | sed 's/^ *//')"

SCAN=$($PY - "$IF" <<'PYEOF'
import socket, struct, sys, time
ifname = sys.argv[1]
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
s.bind((ifname,)); s.settimeout(0.002)

def send(cid, data):
    s.send(struct.pack("<IBBBB64s", cid, len(data), 0, 0, 0, data.ljust(64, b"\x00")))

def drain(t):
    out, end = [], time.monotonic() + t
    while time.monotonic() < end:
        try: raw = s.recv(72)
        except (socket.timeout, BlockingIOError): continue
        c, d = struct.unpack("<IB", raw[:5])
        out.append((c & 0x1FFFFFFF, d, raw[8:8+d]))
    return out

drain(0.1)
hits, send_fail = 0, 0
for mid in range(1, 9):
    try:
        send(0x7FF, bytes([mid, 0, 0xCC, 0]))
    except OSError:
        send_fail += 1
        break
    for c, d, b in drain(0.08):
        if c == 0x7FF and len(b) >= 3 and b[2] == 0xCC:
            continue        # 自己的 echo
        hits += 1
        print(f"HIT {mid} 0x{c:03X} {b.hex(' ')}")
if send_fail:
    print(f"SENDFAIL {send_fail}")
print(f"SUMMARY hits={hits} sendfail={send_fail}")
PYEOF
)

echo "$SCAN" | grep -E "^HIT" | sed 's/^/  /' || true
if echo "$SCAN" | grep -q "SUMMARY hits=0"; then
  bad "1..8 全部无应答"
  if echo "$SCAN" | grep -q "sendfail=1"; then
    bad "发送队列被打满 → 帧没有被 ACK（总线上没有第二个节点）"
  fi
else
  ok "$(echo "$SCAN" | grep SUMMARY)"
fi

# ─────────────────────────────────────────── ④ 恢复
say "④ 恢复接口到 CANFD 1M/5M"
ip link set "$IF" down 2>/dev/null
ip link set "$IF" type can bitrate 1000000 dbitrate 5000000 fd on \
        loopback off listen-only off 2>/dev/null
ip link set "$IF" up 2>/dev/null
ip -details link show "$IF" | sed -n '3p' | sed 's/^/  /' 

say "结论"
if [ "$LOOP_RESULT" = "ECHO_OK" ]; then
  echo "  适配器自身正常。"
  if echo "$SCAN" | grep -q "SUMMARY hits=0"; then
    echo "  但总线上没有电机应答 → 问题在【接线 / 供电 / 终端电阻】，"
    echo "  请断电后用万用表量 CAN_H ─ CAN_L（应约 60Ω）。"
  else
    echo "  且有电机应答 → 总线正常，可以开始控制。"
  fi
else
  echo "  回环都不通 → 先解决适配器/固件问题，再看接线。"
fi
echo
