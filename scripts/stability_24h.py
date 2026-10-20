#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IO 板 24h 稳定性轮询 + 工业三件套调试工具（pymodbus 3.x）

用法（仓库根目录）：
  # 10 分钟冒烟（先跑这个）
  python scripts/stability_24h.py --port COM7 --minutes 10

  # 满跑 24 小时，产 CSV + 汇总
  python scripts/stability_24h.py --port COM7 --minutes 1440 --csv run24h.csv

  # 便捷读写（配参数/查状态用）
  python scripts/stability_24h.py --port COM7 rd 0x2000 9
  python scripts/stability_24h.py --port COM7 wr 0x2001 1500
  python scripts/stability_24h.py --port COM7 wr 0x2005 0x5AA5   # 落 Flash

轮询负载：读 AI 镜像寄存器(0x2064-0x2067) 与写线圈 0 交替，250ms 一轮；
每 60s 采一次设备统计区（0x2010-0x2017：有效帧/CRC 错帧/复位原因/空闲栈水位）。
"""

import argparse
import csv
import sys
import time
from datetime import datetime

from pymodbus.client import ModbusSerialClient

SLAVE = 1
STAT_BASE = 0x2010   # 8 regs: rx_lo rx_hi crc_lo crc_hi rst_lo rst_hi stk_lo stk_hi
CFG_BASE = 0x2000
AI_MIRROR = 100      # holding regs 100-103 mirror the 4 AI channels

RCC_CSR_FLAGS = [
    (1 << 26, "PIN 复位"),
    (1 << 27, "上电复位"),
    (1 << 28, "软件复位"),
    (1 << 29, "独立看门狗复位"),
    (1 << 30, "窗口看门狗复位"),
    (1 << 31, "低功耗复位"),
]


def decode_reset_csr(csr: int) -> str:
    names = [n for m, n in RCC_CSR_FLAGS if csr & m]
    return " | ".join(names) if names else "无标志"


def make_client(port: str) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port,
        baudrate=115200,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=1.0,
    )


def read_regs(cl, addr: int, count: int):
    rr = cl.read_holding_registers(addr, count=count, slave=SLAVE)
    if rr.isError():
        raise RuntimeError(f"read error @0x{addr:04X}: {rr}")
    return rr.registers


def write_reg(cl, addr: int, value: int):
    rr = cl.write_register(addr, value, slave=SLAVE)
    if rr.isError():
        raise RuntimeError(f"write error @0x{addr:04X}: {rr}")


def poll_loop(cl, minutes: float, csv_path: str | None) -> int:
    end = time.time() + minutes * 60.0
    cycles = 0
    errors = 0
    timeouts = 0
    lat_max = 0.0
    rst_first = None
    rst_changes = 0
    coil = 0
    rows = []
    print(f"轮询开始 {datetime.now():%Y-%m-%d %H:%M:%S}，时长 {minutes} 分钟 (Ctrl+C 提前结束)")
    t_stat = 0.0
    try:
        while time.time() < end:
            t0 = time.time()
            try:
                if cycles % 2 == 0:
                    read_regs(cl, AI_MIRROR, 4)
                else:
                    rr = cl.write_coil(0, coil, slave=SLAVE)
                    if rr.isError():
                        errors += 1
                    coil ^= 1
                cycles += 1
                lat = time.time() - t0
                lat_max = max(lat_max, lat)
            except Exception as e:
                errors += 1
                if "timed out" in str(e).lower() or "timeout" in str(e).lower():
                    timeouts += 1
                print(f"  [异常] {e}")

            if (time.time() - t_stat) >= 60.0:
                t_stat = time.time()
                try:
                    st = read_regs(cl, STAT_BASE, 8)
                    rx = st[0] | (st[1] << 16)
                    crc = st[2] | (st[3] << 16)
                    rst = st[4] | (st[5] << 16)
                    stk = st[6] | (st[7] << 16)
                    if rst_first is None:
                        rst_first = rst
                    elif rst != rst_first:
                        rst_changes += 1
                        print(f"  [警报] 复位原因寄存器变化 {rst_first:#010x} -> {rst:#010x} ({decode_reset_csr(rst)})")
                    line = (datetime.now().strftime("%H:%M:%S"), rx, crc, rst_changes, stk, errors, timeouts, f"{lat_max*1000:.0f}")
                    print("  %s 帧=%d crc错=%d 复位变=%d 空闲栈min=%d 主机错=%d(超时%d) 时延max=%sms" % line)
                    rows.append((datetime.now().isoformat(), *line))
                except Exception as e:
                    print(f"  [统计读取失败] {e}")
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\n手动停止")

    print("\n===== 汇总 =====")
    print(f"时长       : {minutes} 分钟（计划）")
    print(f"轮询周期   : {cycles} 次，失败 {errors} 次（其中超时 {timeouts}）")
    print(f"设备侧统计 : 请用 rd 0x2010 8 对照（脚本断开前最后一次为准）")
    if rst_first is not None:
        print(f"复位原因   : {rst_first:#010x} = {decode_reset_csr(rst_first)}，运行中变化 {rst_changes} 次")
    print(f"空闲栈最小 : 见上方日志（0 = 从未调度到空闲任务，需关注）")
    print(f"最大时延   : {lat_max*1000:.0f} ms")
    if csv_path:
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["time", "clock", "device_rx_frames", "device_crc_errors",
                        "reset_changes", "idle_stack_min_words", "host_errors",
                        "host_timeouts", "latency_max_ms"])
            for r in rows:
                w.writerow(r)
        print(f"CSV        : {csv_path}")
    return 0 if errors == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--minutes", type=float, default=1440.0)
    ap.add_argument("--csv", default=None)
    ap.add_argument("cmd", nargs="?", choices=["run", "rd", "wr"], default="run",
                    help="run=轮询；rd/wr=便捷读写（跟地址参数）")
    ap.add_argument("addr", nargs="?", type=lambda x: int(x, 0), help="rd/wr 的寄存器地址")
    ap.add_argument("value", nargs="?", type=lambda x: int(x, 0), help="wr 的值 / rd 的数量(默认1)")
    args = ap.parse_args()

    cl = make_client(args.port)
    if not cl.connect():
        print("串口打开失败")
        return 2
    try:
        if args.cmd == "rd":
            n = args.value if args.value else 1
            regs = read_regs(cl, args.addr, n)
            for i, v in enumerate(regs):
                print(f"  0x{args.addr + i:04X} = 0x{v:04X} ({v})")
            if args.addr <= STAT_BASE and (args.addr + n) > (STAT_BASE + 5):
                print("（提示：0x2014-15 为复位原因寄存器，设备仅在启动时快照）")
        elif args.cmd == "wr":
            write_reg(cl, args.addr, args.value)
            print(f"  已写 0x{args.addr:04X} = {args.value}（0x{args.value:04X}）")
            if args.addr == 0x2005:
                print("  SAVE 触发：Flash 保存中，通信会停顿约 1-2 秒属正常")
        else:
            return poll_loop(cl, args.minutes, args.csv)
    finally:
        cl.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
