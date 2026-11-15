#!/usr/bin/env bash
# 切换到「纯经典 CAN」模式再查询电机（需要 sudo）
#
#   sudo bash ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/can_classic_probe.sh
#
# 背景：达妙电机可能只认经典 CAN 帧。当接口配成 CANFD（fd on）时，
#       gs_usb 可能把 8 字节帧也按 FD 格式发出去，电机不认 → 不应答。
#       本脚本把接口切成 fd off，再用经典帧查询，最后恢复成 CANFD。
set -u
IF=${1:-can0}
say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
[ "$(id -u)" = "0" ] || { echo "需要 root：sudo bash $0"; exit 1; }

say "① 切到纯经典 CAN（bitrate 1000000, fd off）"
ip link set "$IF" down 2>/dev/null
ip link set "$IF" type can bitrate 1000000 fd off one-shot off listen-only off loopback off 2>/dev/null \
  || { echo "  设置失败"; exit 1; }
ip link set "$IF" up 2>/dev/null
ip -details link show "$IF" | sed -n '3p' | sed 's/^/  /'

say "② 用经典帧查询 ID 1..6"
python3 - "$IF" <<'PYEOF'
import socket, struct, sys, time
ifname = sys.argv[1]
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((ifname,)); s.settimeout(0.002)

def drain(t):
    out, end = [], time.monotonic() + t
    while time.monotonic() < end:
        try: raw = s.recv(16)
        except (socket.timeout, BlockingIOError): continue
        c, d = struct.unpack("<IB", raw[:5])
        out.append((c & 0x1FFFFFFF, d, raw[8:8+d]))
    return out

drain(0.2)
hits = 0
for mid in range(1, 7):
    try:
        s.send(struct.pack("<IB3x8s", 0x7FF, 4, bytes([mid, 0, 0xCC, 0, 0, 0, 0, 0])))
    except OSError as e:
        print(f"  ID{mid} 发送失败: {e}"); break
    for c, d, b in drain(0.35):
        if c == 0x7FF: continue
        hits += 1
        err = b[0] >> 4
        pos = ((b[1] << 8) | b[2])
        q = pos * 25.0 / 65535 - 12.5
        print(f"  ★ ID{mid} → 0x{c:03X}  q={q:+.3f} rad  err=0x{err:X}  "
              f"Tmos={b[6]}℃  Trot={b[7]}℃   raw={b.hex(' ')}")
print(f"\n  共 {hits} 条应答")
PYEOF

say "③ 恢复 CANFD 1M/5M"
ip link set "$IF" down 2>/dev/null
ip link set "$IF" type can bitrate 1000000 dbitrate 5000000 fd on \
        loopback off listen-only off one-shot off 2>/dev/null
ip link set "$IF" up 2>/dev/null
ip -details link show "$IF" | sed -n '3p' | sed 's/^/  /'
