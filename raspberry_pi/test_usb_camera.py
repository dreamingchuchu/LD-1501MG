#!/usr/bin/env python3
"""
test_usb_camera.py - USB摄像头测试脚本
功能: 测试USB摄像头是否能正常工作，并检测红色激光

运行方式:
  python3 test_usb_camera.py
"""

import cv2
import numpy as np
import time

print("=" * 60)
print("USB摄像头测试程序")
print("=" * 60)

print("\n正在检测可用摄像头...")
available_cameras = []
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        available_cameras.append(i)
        cap.release()

if not available_cameras:
    print("错误: 未检测到可用摄像头！")
    print("请检查摄像头是否已连接。")
    exit(1)

print(f"检测到 {len(available_cameras)} 个摄像头: {available_cameras}")

camera_index = available_cameras[0]
print(f"\n使用摄像头 {camera_index}")

cap = cv2.VideoCapture(camera_index)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

print("\n摄像头测试:")
print("- 按 'q' 退出")
print("- 按 's' 保存截图")
print("- 红色激光光斑会显示为绿色圆圈")
print("\n开始测试...")

frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("错误: 无法读取摄像头帧")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    cx, cy = -1, -1

    if contours:
        best_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best_contour)

        if 5 <= area <= 2000:
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected = True

                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)

    center_x, center_y = 160, 120
    cv2.line(frame, (center_x, 0), (center_x, 240), (255, 255, 255), 1)
    cv2.line(frame, (0, center_y), (320, center_y), (255, 255, 255), 1)

    if detected:
        error_x = cx - center_x
        error_y = cy - center_y
        cv2.putText(frame, f"Error: ({error_x}, {error_y})", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y), (cx, cy), (0, 0, 255), 2)

    frame_count += 1
    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0

    status = "DETECTED" if detected else "NO LASER"
    color = (0, 255, 0) if detected else (0, 0, 255)
    cv2.putText(frame, f"{status} | FPS: {fps:.1f}", (10, 230),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    cv2.imshow("USB Camera Test", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\n退出测试")
        break
    elif key == ord('s'):
        filename = f"capture_{int(time.time())}.jpg"
        cv2.imwrite(filename, frame)
        print(f"截图已保存: {filename}")

cap.release()
cv2.destroyAllWindows()

print(f"\n测试结束")
print(f"总帧数: {frame_count}")
print(f"平均帧率: {fps:.1f} FPS")