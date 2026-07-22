#!/usr/bin/env python3
"""
test_pan_direction.py - 测试 Pan 轴方向（带摄像头预览）
========================
交互式判断 P 轴舵机方向与图像 X 轴是否一致。

用法:
  python3 test_pan_direction.py
  python3 test_pan_direction.py --port /dev/ttyS0 --camera 0

操作（在终端输入，回车执行）:
  +数字   → P 增大（右转）     如 +30
  -数字   → P 减小（左转）     如 -30
  数字    → 直接设置 P 角度    如 110
  c       → 归中 P=90
  q       → 退出

判断方法:
  摄像头画面中激光点固定不动。
  输入 +30 (P=120，云台右转):
    → 激光点往画面 LEFT  移动  = 方向正确 ✓
    → 激光点往画面 RIGHT 移动  = 方向反了 ✗
"""

import sys
import os
import time
import argparse
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
from mcu_communicator import MCUCommunicator


def find_mcu_port():
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
    parser = argparse.ArgumentParser(description="Pan 轴方向测试工具（带摄像头）")
    parser.add_argument("--port", default="auto", help="MCU 串口 (默认自动扫描)")
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引 (默认0)")
    args = parser.parse_args()

    # ─── 摄像头（主线程打开） ───
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"❌ 无法打开摄像头 {args.camera}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    w, h = 320, 240
    cx_img, cy_img = w // 2, h // 2
    print(f"📷 摄像头已打开: {w}x{h}")

    # ─── MCU ───
    mcu_port = args.port
    if mcu_port == "auto":
        mcu_port = find_mcu_port()
        if mcu_port is None:
            print("❌ 未找到可用串口！")
            cap.release()
            return

    print(f"MCU 端口: {mcu_port}")
    mcu = MCUCommunicator(port=mcu_port)
    if not mcu.connect():
        print("❌ 串口连接失败！")
        cap.release()
        return
    time.sleep(0.3)

    # 归中
    print("\n归中: P=90 ...")
    mcu.send_raw_command("P=90")
    time.sleep(0.3)

    print("=" * 55)
    print("  Pan 轴方向测试（摄像头窗口 + 终端输入）")
    print("=" * 55)
    print("  终端命令:")
    print("    +30  → P=120 (右转)")
    print("    -30  → P=60  (左转)")
    print("    110  → 直接设 P=110")
    print("    c    → 归中 P=90")
    print("    q    → 退出")
    print()
    print("  💡 判断: 激光笔固定 → 输入 +30 →")
    print("     光斑往 LEFT  移动 = 方向正确 ✓")
    print("     光斑往 RIGHT 移动 = 方向反了 ✗")
    print("=" * 55)
    print("在终端输入命令后回车，然后切回摄像头窗口观察！\n")

    current_p = 90
    overlay_text = "P=90 (归中)"
    overlay_color = (0, 255, 255)

    # ─── 后台线程: 读终端输入进队列 ───
    cmd_queue = []

    def input_thread():
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd_queue.append(line.strip())
            except (EOFError, KeyboardInterrupt):
                break

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    # ─── 主循环: 摄像头 + 处理命令 ───
    try:
        while True:
            # 读一帧
            ret, frame = cap.read()
            if not ret:
                print("⚠️ 读取帧失败")
                time.sleep(0.05)
                continue

            # 画中心十字
            cv2.line(frame, (cx_img, 0), (cx_img, h), (255, 255, 255), 1)
            cv2.line(frame, (0, cy_img), (w, cy_img), (255, 255, 255), 1)
            cv2.circle(frame, (cx_img, cy_img), 5, (0, 255, 255), 1)
            cv2.putText(frame, overlay_text, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, overlay_color, 2)
            cv2.putText(frame, "Terminal: +30/-30/c/q", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow("Pan Direction Test", frame)
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or key == ord('q'):
                print("\n摄像头窗口关闭，退出.")
                break

            # 处理终端输入
            while cmd_queue:
                cmd = cmd_queue.pop(0)

                if cmd == 'q':
                    print("退出.")
                    cap.release()
                    cv2.destroyAllWindows()
                    mcu.send_raw_command("P=90")
                    time.sleep(0.2)
                    mcu.disconnect()
                    return

                if cmd == 'c':
                    current_p = 90
                    mcu.send_raw_command("P=90")
                    overlay_text = "P=90 (归中)"
                    overlay_color = (0, 255, 255)
                    print("  → 已归中 P=90")
                    time.sleep(0.2)
                    continue

                try:
                    if cmd.startswith('+'):
                        delta = int(cmd[1:])
                        target = current_p + delta
                    elif cmd.startswith('-'):
                        delta = int(cmd)
                        target = current_p + delta
                    else:
                        target = int(cmd)
                        delta = target - current_p

                    target = max(0, min(180, target))
                    delta = target - current_p

                    if target == current_p:
                        print(f"  → 角度不变 P={target}，跳过")
                        continue

                    direction_word = "右转 →" if delta > 0 else "左转 ←"
                    print(f"  发送: P={target}  ({direction_word}  Δ={delta:+d})")
                    mcu.send_raw_command(f"P={target}")
                    current_p = target
                    overlay_text = f"P={target} ({direction_word})"
                    overlay_color = (0, 255, 0)
                    time.sleep(0.15)

                except ValueError:
                    print(f"  ❌ 无效输入: '{cmd}'")

    except KeyboardInterrupt:
        print("\n中断.")
    finally:
        print("归中并断开...")
        mcu.send_raw_command("P=90")
        time.sleep(0.2)
        mcu.disconnect()
        cap.release()
        cv2.destroyAllWindows()
        print("完成.")


if __name__ == "__main__":
    main()
