#!/usr/bin/env bash
# 判定总线是否真的被 ACK（需要 sudo）
#
#   sudo bash ~/hw_arm/hw_arm_ws/src/hw_arm_description/scripts/can_ack_test.sh [接口名]
#
# 原理：
#   CAN 控制器默认「自动重发」，没人 ACK 的帧会无限重试 → TX 队列被打满（ENOBUFS）。
#   开 one-shot 模式后，没人 ACK 的帧**只发一次就丢弃**，并计入 TX errors。
#   于是「发 5 帧，看 TX errors」就能干净地判断：
#        TX errors = 0  → 每一帧都被 ACK 了 → 总线上有活节点
#        TX errors = 5  → 每一帧都没人 ACK → 总线上没有节点（接线/供电问题）
set -u
IF=${1:-can0}

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }

[ "$(id -u)" = "0" ] || { echo "需要 root：sudo bash $0 $IF"; exit 1; }

say "① 切到 one-shot（经典 CAN 1M）"
ip link set "$IF" down 2>/dev/null
ip link set "$IF" type can bitrate 1000000 one-shot on loopback off listen-only off 2>/dev/null \
  || { echo "  设置失败"; exit 1; }
ip link set "$IF" up 2>/dev/null
ip -details link show "$IF" | sed -n '3p' | sed 's/^/  /'

before=$(ip -s link show "$IF" | awk '/TX:/{getline; print $3}')
say "② 发 5 帧到 0x7FE（一个不存在的 ID），起始 TX errors = $before"

python3 - "$IF" <<'PYEOF'
import socket, struct, sys, time
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((sys.argv[1],))
for _ in range(5):
    try:
        s.send(struct.pack("<IB3x8s", 0x7FE, 8, b"\x01\x02\x03\x04\x05\x06\x07\x08"))
    except OSError as e:
        print("  发送失败:", e)
    time.sleep(0.05)
PYEOF
sleep 0.5

after=$(ip -s link show "$IF" | awk '/TX:/{getline; print $3}')
say "③ 结果：TX errors $before → $after"

delta=$((after - before))
if [ "$delta" -eq 0 ]; then
  printf '  \033[32m✓ 每一帧都被 ACK\033[0m → 总线上有活的 CAN 节点\n'
else
  printf '  \033[31m✗ %s 帧无人 ACK\033[0m → 总线上没有节点\n' "$delta"
  echo
  echo "  说明帧根本没被任何设备接收。请检查（按顺序）："
  echo "   1) 电机 LED 亮不亮（不亮 = 没上电/电源接错）"
  echo "   2) 适配器 CAN_H ↔ 电机 CAN_H、CAN_L ↔ CAN_L（断电用万用表测通断）"
  echo "   3) 4-pin 排线方向是否插反（V+ 接到了 CAN 针上会烧收发器）"
  echo "   4) 断电测 CAN_H — CAN_L 电阻：应 ≈60Ω（两端各 120Ω）"
fi

say "④ 恢复 CANFD 1M/5M"
ip link set "$IF" down 2>/dev/null
ip link set "$IF" type can bitrate 1000000 dbitrate 5000000 fd on \
        loopback off listen-only off one-shot off 2>/dev/null
ip link set "$IF" up 2>/dev/null
ip -details link show "$IF" | sed -n '3p' | sed 's/^/  /'
