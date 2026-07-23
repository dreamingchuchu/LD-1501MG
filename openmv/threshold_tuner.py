"""
threshold_tuner.py - 红色阈值交互式标定工具
使用方法: 在OpenMV IDE中运行, 拖动滑块调节阈值, 实时查看效果
"""

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

thresholds = [40, 100, 30, 127, -20, 50]

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs([tuple(thresholds)], pixels_threshold=5, merge=True)

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        img.draw_string(blob.cx() + 5, blob.cy(), f"area={blob.area()}", color=(255,255,255), scale=1.2)

    img.draw_string(0, 0, f"FPS: {clock.fps():.1f}", color=(255,255,255))
    img.draw_string(0, 15, f"THR: L({thresholds[0]},{thresholds[1]}) "
                           f"A({thresholds[2]},{thresholds[3]}) "
                           f"B({thresholds[4]},{thresholds[5]})", color=(255,255,255))
    img.draw_string(0, 30, f"Blobs found: {len(blobs)}", color=(255,255,255))