#!/usr/bin/env python3
"""
test_vision_stability.py — 视觉稳定性测试（舵机不动，只看识别）
========================================================
关闭舵机，只开摄像头+红点识别，观察 cx/cy 是否稳定。

用法:
  python3 test_vision_stability.py
  python3 test_vision_stability.py --camera 0

输出每 0.5s 一行:
  cx=108 cy=120 detected=True
  cx=109 cy=121 detected=True
  ...

判断标准:
  - cx 稳定在 ±3px   → 视觉没问题，问题在控制
  - cx 在 ±30px 跳动 → 视觉有问题
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
from usb_camera_tracker import USBCameraTracker


def main():
    parser = argparse.ArgumentParser(description="视觉稳定性测试（不开舵机）")
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引 (默认0)")
    args = parser.parse_args()

    tracker = USBCameraTracker(camera_index=args.camera)
    if not tracker.start():
        print("❌ 摄像头打开失败")
        return

    time.sleep(0.5)
    print(f"画面中心: ({tracker.center_x}, {tracker.center_y})")
    print("=" * 50)
    print("  舵机不开，只观察识别结果。")
    print("  激光笔固定，看 cx/cy 是否稳定。")
    print("  按 ESC 或 q 退出。")
    print("=" * 50)

    # 统计
    cx_samples = []
    cy_samples = []
    start_time = time.time()
    last_print = 0

    try:
        while True:
            if not tracker.update():
                time.sleep(0.01)
                continue

            cx, cy, detected = tracker.get_position()
            ex, ey, _ = tracker.get_error()

            if detected:
                cx_samples.append(cx)
                cy_samples.append(cy)

            # 每 0.5s 打印一次
            now = time.time()
            if now - last_print >= 0.5:
                last_print = now
                tag = "✓" if detected else "✗"
                print(f"[{now - start_time:>5.1f}s] {tag} cx={cx:>4} cy={cy:>4}  ex={ex:>+4} ey={ey:>+4}")

            # 按键退出
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        print()

        # 统计
        if cx_samples:
            import statistics
            print("=" * 50)
            print(f"  样本数: {len(cx_samples)}")
            print(f"  cx: 均值={statistics.mean(cx_samples):.1f}  "
                  f"标准差={statistics.stdev(cx_samples):.1f}  "
                  f"范围=[{min(cx_samples)}, {max(cx_samples)}]")
            print(f"  cy: 均值={statistics.mean(cy_samples):.1f}  "
                  f"标准差={statistics.stdev(cy_samples):.1f}  "
                  f"范围=[{min(cy_samples)}, {max(cy_samples)}]")
            print()
            if statistics.stdev(cx_samples) < 3:
                print("  ✅ 视觉识别非常稳定 → 问题在控制算法")
            elif statistics.stdev(cx_samples) < 8:
                print("  ⚠️ 视觉有轻微抖动 → 可能视觉+控制都有问题")
            else:
                print("  ❌ 视觉识别不稳定 → 先解决识别问题再调控制")
        else:
            print("  ⚠️ 没检测到任何激光，请确认激光笔已打开并指向摄像头")


if __name__ == "__main__":
    main()
