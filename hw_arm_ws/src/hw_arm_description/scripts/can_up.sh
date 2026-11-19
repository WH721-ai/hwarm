#!/usr/bin/env bash
# 一次性把 USB-CAN 配好（经典 CAN 1 Mbps），并验证能不能真的发出去。
#
# 为什么需要：每次**拔插 USB-CAN 适配器**或**重启电脑**之后，
#   can0 都会被重新创建成默认状态：
#       state DOWN、bitrate 未设置、txqueuelen=10
#   不重新配置就完全通不了（表现是"电机一个都不响应"）。
#
# 用法（需要密码，会提示一次）：
#     bash scripts/can_up.sh
#     bash scripts/can_up.sh can1        # 指定别的接口
#
# 退出码：0=成功  1=接口不存在  2=配置失败  3=能配但发不出帧
set -u

IFACE="${1:-can0}"
BITRATE="${BITRATE:-1000000}"

if [ ! -e "/sys/class/net/$IFACE" ]; then
    echo "❌ 接口 $IFACE 不存在。"
    echo "   USB-CAN 适配器插好了吗？ lsusb | grep 1d6b"
    echo "   现有接口："; ip -br link show type can 2>/dev/null || echo "     （没有任何 CAN 接口）"
    exit 1
fi

echo "═══ 配置 $IFACE ═══"
sudo ip link set "$IFACE" down 2>/dev/null
sudo ip link set "$IFACE" type can bitrate "$BITRATE" sample-point 0.75 \
     restart-ms 0 fd off || { echo "❌ 设置 bitrate 失败"; exit 2; }
sudo ip link set "$IFACE" txqueuelen 1000
sudo ip link set "$IFACE" up || { echo "❌ 拉起接口失败"; exit 2; }

echo
echo "═══ 当前状态 ═══"
ip -details link show "$IFACE" | grep -E "^[0-9]+:|can state|bitrate" | sed 's/^/  /'
echo "  txqueuelen = $(cat /sys/class/net/$IFACE/tx_queue_len)"

# ── 验证真的能发出去 ──
# 注意：不能看 /sys/class/net/*/statistics/tx_packets —— gs_usb 驱动不更新它！
# 必须看 /proc/net/can/stats 的 TXF，那个才是真实发送计数。
echo
echo "═══ 发送自检 ═══"
python3 - "$IFACE" <<'PY'
import re, socket, struct, sys, time

iface = sys.argv[1]

def txf_rxf():
    t = open('/proc/net/can/stats').read()
    return (int(re.search(r'(\d+) transmitted frames', t).group(1)),
            int(re.search(r'(\d+) received frames', t).group(1)))

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((iface,))
s.setblocking(False)

a = txf_rxf()
for _ in range(20):
    try:
        s.send(struct.pack("<IB3x8s", 0x7FF, 4,
                           bytes([1, 0, 0xCC, 0]).ljust(8, b"\0")))
    except (BlockingIOError, TimeoutError, OSError):
        pass
    time.sleep(0.004)
time.sleep(0.2)
b = txf_rxf()
s.close()

dt, dr = b[0] - a[0], b[1] - a[1]
print(f"  TXF: {a[0]} → {b[0]}  (+{dt})")
print(f"  RXF: {a[1]} → {b[1]}  (+{dr})")
if dt == 0:
    print("  ❌ 发不出去 —— 适配器可能卡死，再拔插一次或重载 gs_usb")
    sys.exit(3)
if dr == 0:
    print("  ✅ 能发；但**收到 0 帧** —— 总线上没有设备应答")
    print("     接下来查：电机供电 / CAN_H-CAN_L 是否接反 / 终端电阻")
else:
    print("  ✅ 能发也能收 —— 电机在线！")
PY
rc=$?
echo
if [ $rc -eq 0 ]; then
    echo "════════════════════════════════════════════════════════"
    echo "  ✅ $IFACE 已就绪。下一步验证电机："
    echo "     python3 scripts/scan_motors.py"
    echo "════════════════════════════════════════════════════════"
fi
exit $rc
