#!/usr/bin/env python3
"""
test_mcu.py — MCU通信诊断测试脚本
功能: 自动检测可用串口，测试与STM32的通信，排查接线问题

用法:
  python3 test_mcu.py              # 自动检测端口
  python3 test_mcu.py --port /dev/ttyS0  # 指定端口
"""

from mcu_communicator import MCUCommunicator
import time
import sys
import os
import argparse


def scan_ports():
    """扫描系统可用串口"""
    ports = []
    for name in ["/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB0", "/dev/ttyACM0"]:
        if os.path.exists(name):
            ports.append(name)
    # 也检查 /dev/serial*
    serial_dir = "/dev/serial/by-id"
    if os.path.isdir(serial_dir):
        for f in os.listdir(serial_dir):
            link = os.path.realpath(os.path.join(serial_dir, f))
            if link not in ports:
                ports.append(link)
    return ports


def print_separator(title=""):
    print(f"\n{'='*60}")
    if title:
        print(f"  {title}")
        print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description="MCU通信诊断测试")
    parser.add_argument("--port", help="MCU串口设备路径 (如 /dev/ttyAMA0)")
    args = parser.parse_args()

    print_separator("MCU通信诊断测试")

    # ── 1. 串口扫描 ─────────────────────
    print("\n[1] 扫描可用串口...")
    ports = scan_ports()
    if ports:
        for p in ports:
            # 显示软链接目标
            real = os.path.realpath(p) if os.path.islink(p) else p
            print(f"    {p} -> {real}")
    else:
        print("    未找到任何串口设备！")
        print("    请检查：")
        print("    - STM32是否上电")
        print("    - 串口线是否插好")
        print("    - 树莓派是否启用UART (sudo raspi-config)")

    # ── 2. 确定测试端口 ─────────────────
    if args.port:
        test_port = args.port
    elif "/dev/ttyS0" in ports:
        test_port = "/dev/ttyS0"   # GPIO mini UART，树莓派4上默认可用
    elif "/dev/ttyAMA0" in ports:
        test_port = "/dev/ttyAMA0"
    elif ports:
        test_port = ports[0]
    else:
        print("\n✗ 没有可用串口，退出。")
        return

    print(f"\n[2] 使用端口: {test_port}")

    # 检查是否是软链接（说明连接正确）
    if os.path.islink(test_port):
        real = os.path.realpath(test_port)
        print(f"    实际设备: {real}")
        if "AMA0" in real:
            print("    ✓ 这是硬件UART (PL011)，波特率精准")
        elif "S0" in real:
            print("    △ 这是mini UART，波特率可能有微小误差（115200够用）")

    # ── 3. 连接MCU ──────────────────────
    print("\n[3] 连接MCU...")
    mcu = MCUCommunicator(port=test_port)

    if not mcu.connect():
        print("    ✗ 连接失败!")
        print("    可能原因：")
        print("    - 用户不在 dialout 组 → 试试: sudo usermod -a -G dialout $USER")
        print("    - 端口被占用 → 试试: sudo lsof", test_port)
        print("    - 权限不够 → 试试: sudo chmod 666", test_port)
        return

    print("    ✓ 串口已打开")

    # ── 4. 归中测试 ─────────────────────
    print("\n[4] 发送归中命令 (RST)...")
    ok = mcu.send_center()
    if ok:
        print("    ✓ 已发送，舵机应该归中")
    else:
        print("    ✗ 发送失败 (Write timeout)")
        print("    可能原因：")
        print("    - STM32端未共地 (树莓派GND ↔ STM32 GND 必须连通)")
        print("    - TX/RX 接反 (树莓派TX→STM32 RX, 树莓派RX→STM32 TX)")
        print("    - STM32未烧录程序或程序未运行")
        print("    - 波特率不匹配 (MCU端默认115200)")
    time.sleep(1.5)

    # ── 5. TRACK命令测试 ────────────────
    print("\n[5] 测试 TRACK 命令...")

    tests = [
        ("Pan右转10步",  10,  0),
        ("Pan左转10步", -10,  0),
        ("Tilt下转10步",  0, 10),
        ("Tilt上转10步",  0,-10),
        ("归零",          0,  0),
    ]

    for label, dp, dt in tests:
        print(f"    {label}...", end=" ")
        ok = mcu.send_track_command(dp, dt)
        if ok:
            print("✓")
        else:
            print("✗ Write timeout")
        time.sleep(0.6)

    # ── 6. P/T 角度命令测试 ──────────────
    print("\n[6] 测试绝对角度命令...")
    for label, cmd in [("Pan=90度", "P=90"), ("Tilt=90度", "T=90")]:
        print(f"    {label}...", end=" ")
        ok = mcu.send_raw_command(cmd)
        if ok:
            print("✓")
        else:
            print("✗")
        time.sleep(0.6)

    # ── 7. 最终归中 ──────────────────────
    print("\n[7] 最终归中...")
    mcu.send_center()
    time.sleep(1)

    mcu.disconnect()

    print_separator("测试完成")
    print()
    print("总结：")
    print("  - 如果全部 Write timeout → 检查串口线、共地、STM32是否运行")
    print("  - 如果发送成功但舵机不动 → 检查MCU端代码是否添加了 TRACK 指令解析")
    print("  - 如果舵机动了 → 一切正常，可以启动 main_controller_usb.py")


if __name__ == "__main__":
    main()
