#!/bin/bash
# 由 udev 在 USB-CAN 插入时自动调用（不要手动跑，手动请用 can_up.sh）。
#
# udev 的 RUN 环境很干净，所以全部用绝对路径，并做几次重试
# （设备节点刚创建时可能还不可配置）。
set -u
IFACE="${1:-can0}"
BITRATE=1000000
for _ in $(seq 1 20); do
    [ -e "/sys/class/net/$IFACE" ] && break
    sleep 0.1
done
[ -e "/sys/class/net/$IFACE" ] || exit 0

ip link set "$IFACE" down 2>/dev/null
ip link set "$IFACE" type can bitrate "$BITRATE" sample-point 0.75 restart-ms 0 fd off 2>/dev/null
ip link set "$IFACE" txqueuelen 1000 2>/dev/null
ip link set "$IFACE" up 2>/dev/null
exit 0
