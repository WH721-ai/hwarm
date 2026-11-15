#!/usr/bin/env python3
"""测 USB-CANFD 适配器的**持续**吞吐能力（不是突发）。

为什么不能用「一口气发 N 帧」来测：
    SocketCAN 发送路径上有两级，任一级满了就返回 ENOBUFS：
      ① netdev 队列      txqueuelen（本机 1000，可用 ip link 调）
      ② gs_usb 的 URB 池  约 10 个在途 USB 传输，**不可调**
    突发发送会立刻填满 ②，测出来的只是「池子大小」，和真实吞吐无关。
    正确做法：按目标速率**匀速**发，看哪个速率开始丢帧。

本脚本只发 refresh 广播帧（0x7FF + [id,0,0xCC,0]），只查询状态，不驱动电机，安全。

用法：
    python3 scripts/can_benchmark.py
    python3 scripts/can_benchmark.py --rates 500,1000,2000,3000
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time


def read_qlen(ifname: str):
    try:
        with open(f"/sys/class/net/{ifname}/tx_queue_len") as f:
            return int(f.read().strip())
    except OSError:
        return None


def read_tx_packets(ifname: str):
    try:
        with open(f"/sys/class/net/{ifname}/statistics/tx_packets") as f:
            return int(f.read().strip())
    except OSError:
        return None


def refresh_payload(mid: int) -> bytes:
    return bytes([mid, 0, 0xCC, 0])


class Bus:
    def __init__(self, ifname: str):
        self.ifname = ifname
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.settimeout(0)
        self.last_err = ""

    def send_refresh(self, mid: int) -> bool:
        try:
            self.sock.send(struct.pack("<IB3x8s", 0x7FF, 4,
                                       refresh_payload(mid).ljust(8, b"\x00")))
            return True
        except BlockingIOError:
            self.last_err = "ENOBUFS"
            return False
        except OSError as e:
            self.last_err = str(e)
            return False

    def drain(self, seconds: float) -> int:
        n, end = 0, time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                self.sock.recv(16)
                n += 1
            except (BlockingIOError, socket.timeout):
                pass
        return n


def paced_test(bus: Bus, rate: float, duration: float, motors: int):
    """按 rate 帧/s 匀速发 duration 秒。"""
    tx0 = read_tx_packets(bus.ifname)
    n_total = int(rate * duration)
    interval = 1.0 / rate
    ok = fail = 0
    t_start = time.perf_counter()
    next_t = t_start
    for i in range(n_total):
        next_t += interval
        now = time.perf_counter()
        if next_t > now:
            time.sleep(next_t - now)
        if bus.send_refresh((i % motors) + 1):
            ok += 1
        else:
            fail += 1
    elapsed = time.perf_counter() - t_start
    tx1 = read_tx_packets(bus.ifname)
    return {
        "ok": ok, "fail": fail, "elapsed": elapsed,
        "tx": (tx1 - tx0) if (tx0 is not None and tx1 is not None) else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--motors", type=int, default=6)
    ap.add_argument("--rates", default="300,600,1000,1500,2000,3000,4000")
    ap.add_argument("--duration", type=float, default=1.0)
    args = ap.parse_args()

    rates = [float(x) for x in args.rates.split(",") if x.strip()]

    try:
        bus = Bus(args.interface)
    except OSError as e:
        print(f"❌ 打不开 {args.interface}: {e}")
        return 2

    print(f"接口: {args.interface}   电机数: {args.motors}   "
          f"txqueuelen: {read_qlen(args.interface)}")

    bus.drain(0.3)
    tx_before = read_tx_packets(args.interface)
    ok = 0
    for _ in range(10):
        if bus.send_refresh(1):
            ok += 1
        time.sleep(0.01)
    time.sleep(0.3)
    tx_after = read_tx_packets(args.interface)
    tx_delta = (tx_after - tx_before) if (tx_before is not None and tx_after is not None) else -1
    print(f"预热: send() 成功 {ok}/10，驱动实际发出 {tx_delta} 帧\n")
    if ok > 0 and tx_delta == 0:
        print("❌ send() 能返回成功，但驱动一帧都没发出去。")
        print("   说明控制器卡在重发上 —— 总线上没有任何节点 ACK。")
        print("   排查顺序：")
        print("     1) 电机是否上电（指示灯亮）")
        print("     2) 适配器的 CAN_H / CAN_L 是否接到电机（重插 USB 时容易碰到）")
        print("     3) 断电后量 CAN_H — CAN_L 电阻，应 ≈60Ω")
        print("   恢复接口：")
        print(f"     sudo ip link set {args.interface} down")
        print(f"     sudo ip link set {args.interface} up")
        return 3
    if ok == 0:
        print(f"❌ 队列已满（{bus.last_err}）—— 先执行：")
        print(f"   sudo ip link set {args.interface} down")
        print(f"   sudo ip link set {args.interface} type can bitrate 1000000 "
              f"fd off loopback off one-shot off")
        print(f"   sudo ip link set {args.interface} txqueuelen 1000")
        print(f"   sudo ip link set {args.interface} up")
        return 1

    # ① 单帧延迟
    lat = []
    for _ in range(50):
        t = time.perf_counter()
        bus.send_refresh(1)
        lat.append((time.perf_counter() - t) * 1e6)
        time.sleep(0.005)
    lat.sort()
    print(f"① 单帧 send() 耗时: 中位 {lat[len(lat)//2]:.0f} µs, "
          f"p95 {lat[int(len(lat)*0.95)]:.0f} µs, 最大 {lat[-1]:.0f} µs\n")

    # ② 速率阶梯
    print(f"② 匀速发送测试（每档 {args.duration:.1f}s）")
    print(f"   {'目标帧/s':>9s} {'实发':>6s} {'ENOBUFS':>8s} {'失败率':>7s} "
          f"{'驱动发出':>9s} {'耗时':>7s}  结论")
    ceiling = None
    for r in rates:
        bus.drain(0.05)
        st = paced_test(bus, r, args.duration, args.motors)
        fail_pct = st["fail"] / max(1, st["ok"] + st["fail"]) * 100
        tx = st["tx"] if st["tx"] is not None else -1
        if fail_pct < 1.0:
            verdict, ceiling = "✅", r
        elif fail_pct < 10:
            verdict = "⚠️"
        else:
            verdict = "❌"
        print(f"   {r:>9.0f} {st['ok']:>6d} {st['fail']:>8d} {fail_pct:>6.1f}% "
              f"{tx:>9d} {st['elapsed']:>6.2f}s  {verdict}")
        time.sleep(0.2)

    bus.drain(0.2)

    # ③ 折算
    print("\n③ 折算：各控制频率需要多少帧/s")
    print(f"   {'update_rate':>12s} {'refresh_div':>12s} {'帧/s':>8s} {'占上限':>8s}  结论")
    for hz in (100, 125, 250, 500):
        for div in (1, 4):
            fps = hz * (args.motors + args.motors / div)
            if not ceiling:
                pct, verdict = 0, "（上限未知）"
            else:
                pct = fps / ceiling * 100
                verdict = "✅ 可行" if pct < 60 else ("⚠️ 偏紧" if pct < 85 else "❌ 超出")
            print(f"   {hz:>10d}Hz {div:>12d} {fps:>8.0f} {pct:>7.0f}%  {verdict}")

    print("\n结论：")
    if ceiling:
        need = 250 * (args.motors + args.motors / 4)
        print(f"  匀速无丢帧的最高档 ≈ {ceiling:.0f} 帧/s")
        print(f"  250Hz + refresh_divider=4 需要 {need:.0f} 帧/s → "
              f"{'✅ 可行' if need < ceiling * 0.85 else '❌ 需要降频或减帧'}")
    else:
        print("  所有档位都有丢帧 → 总线或驱动已达极限，或帧无人 ACK。")
        print("  请确认：电机上电？can0 是 fd off？")
    return 0


if __name__ == "__main__":
    sys.exit(main())
