#!/usr/bin/env python3
"""
test_serial_protocol.py — 串口协议快速诊断
===========================================
用法:
  python3 test_serial_protocol.py

操作:
  P=90     → 发 P=90.0
  P=90.3   → 发 P=90.3
  T=45     → 发 T=45.0
  RST      → 归中
  q        → 退出

观察:
  1. 云台动了吗? 方向对吗?
  2. STM32 回复了 OK 吗? 角度显示有小数点吗?
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mcu_communicator import MCUCommunicator


def find_port():
    candidates = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyS0", "/dev/ttyAMA0"]
    for p in candidates:
        if os.path.exists(p):
            try:
                import serial
                s = serial.Serial(p, 115200, timeout=0.1)
                s.close()
                return p
            except Exception:
                continue
    return None


def main():
    port = find_port()
    if port is None:
        print("❌ 未找到串口")
        return

    mcu = MCUCommunicator(port=port)
    if not mcu.connect():
        print("❌ 连接失败")
        return
    time.sleep(0.3)

    print(f"✅ 已连接 {port}")
    print("=" * 50)
    print("  命令: P=90 | P=91.3 | T=45 | RST | q")
    print("  看云台动不动, STM32回什么")
    print("=" * 50)

    try:
        while True:
            cmd = input("> ").strip()
            if not cmd:
                continue
            if cmd.lower() == 'q':
                break

            # 自动补小数
            if cmd.startswith("P=") or cmd.startswith("T="):
                if '.' not in cmd:
                    cmd = cmd + ".0"

            mcu.send_raw_command(cmd)
            time.sleep(0.05)

    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        mcu.send_raw_command("P=90.0")
        time.sleep(0.05)
        mcu.send_raw_command("T=90.0")
        mcu.disconnect()
        print("已断开")


if __name__ == "__main__":
    main()
