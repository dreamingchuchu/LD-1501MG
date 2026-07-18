"""
red_laser_tracker.py - OpenMV Cam H7 红色激光光斑检测
功能: 实时检测红色激光光斑位置，通过UART发送质心坐标给树莓派
协议: 每帧发送 "cx,cy\n"，无目标时发送 "-1,-1\n"
"""

import sensor
import image
import time
import math
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

sensor.set_auto_exposure(False, exposure_us=5000)
sensor.set_windowing((0, 0, 320, 240))

clock = time.clock()

uart = UART(1, 115200, timeout_char=10)
uart.init(115200, bits=8, parity=None, stop=1)

RED_THRESHOLD_LAB = (40, 100,
                      30, 127,
                     -20,  50)

MIN_BLOB_AREA = 5
MAX_BLOB_AREA = 2000

CENTER_X = 160
CENTER_Y = 120

while True:
    clock.tick()

    img = sensor.snapshot()

    blobs = img.find_blobs(
        [RED_THRESHOLD_LAB],
        pixels_threshold=MIN_BLOB_AREA,
        area_threshold=MIN_BLOB_AREA,
        merge=True,
        margin=5
    )

    if blobs:
        best_blob = None
        max_area = 0

        for blob in blobs:
            area = blob.area()
            if MIN_BLOB_AREA <= area <= MAX_BLOB_AREA and area > max_area:
                max_area = area
                best_blob = blob

        if best_blob:
            cx = best_blob.cx()
            cy = best_blob.cy()

            img.draw_cross(cx, cy, size=10, color=(0, 255, 0))
            img.draw_rectangle(best_blob.rect(), color=(0, 255, 0))
            img.draw_circle(cx, cy, 5, color=(0, 255, 0))
        else:
            cx, cy = -1, -1
    else:
        cx, cy = -1, -1

    img.draw_cross(CENTER_X, CENTER_Y, size=15, color=(255, 255, 255))

    if cx > 0 and cy > 0:
        img.draw_line(CENTER_X, CENTER_Y, cx, cy, color=(255, 0, 0))

    uart.write(f"{cx},{cy}\n")

    fps = clock.fps()