#!/bin/bash
# 让 USB-CAN 适配器「插上就自动配好」，不用每次手动跑 can_up.sh。
#
# 用法：
#     sudo bash scripts/install_can_udev.sh            # 安装
#     sudo bash scripts/install_can_udev.sh --remove   # 卸载
#
# 装了什么：
#     /usr/local/lib/hw_arm_can_setup.sh              配置脚本
#     /etc/udev/rules.d/90-hw-arm-can.rules           udev 规则
#
# 触发条件：USB 的 idVendor=1d50 idProduct=606f（达妙 DM-USB2FDCAN，
#           以及大部分 gs_usb / candleLight 兼容适配器）。
set -eu
[ "$(id -u)" = "0" ] || { echo "需要 root：sudo bash $0"; exit 1; }

HOOK=/usr/local/lib/hw_arm_can_setup.sh
RULE=/etc/udev/rules.d/90-hw-arm-can.rules
SRC="$(cd "$(dirname "$0")" && pwd)/can_auto_setup.sh"

if [ "${1:-}" = "--remove" ]; then
    rm -f "$RULE" "$HOOK"
    udevadm control --reload-rules 2>/dev/null || true
    echo "✅ 已卸载。以后插拔需要手动跑 can_up.sh"
    exit 0
fi

[ -f "$SRC" ] || { echo "❌ 找不到 $SRC"; exit 1; }
install -Dm755 "$SRC" "$HOOK"
cat > "$RULE" <<'RULEEOF'
# hw_arm: 达妙 DM-USB2FDCAN / gs_usb 适配器插入时自动配成经典 CAN 1 Mbps
ACTION=="add", SUBSYSTEM=="net", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", RUN+="/usr/local/lib/hw_arm_can_setup.sh %k"
RULEEOF

udevadm control --reload-rules
# 对已经插着的设备补触发一次
udevadm trigger --action=add --subsystem-match=net 2>/dev/null || true
sleep 1

echo "✅ 安装完成"
echo "   规则 : $RULE"
echo "   脚本 : $HOOK"
echo
echo "以后：拔插 USB-CAN，接口会自动被配成 1 Mbps 经典 CAN 并 UP。"
echo "验证："
echo "     ip -br link show can0        # 期望 UP"
echo "     bash scripts/can_up.sh       # 也可以随时手动补配"
echo
echo "卸载： sudo bash $0 --remove"
