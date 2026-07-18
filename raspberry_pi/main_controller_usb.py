#!/usr/bin/env python3
"""
main_controller_usb.py - 红色激光追踪主控制程序（USB摄像头版本）
功能: 使用USB摄像头替代OpenMV，实现红色激光追踪

运行方式:
  python3 main_controller_usb.py

依赖:
  pip install opencv-python numpy pyserial
"""

import time
import sys
import logging
import argparse

from usb_camera_tracker import USBCameraTracker
from mcu_communicator import MCUCommunicator
from pid_controller import IncrementalPID, PAN_PID_CONFIG, TILT_PID_CONFIG
from calibration import Calibration

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger("MainController")

CONTROL_PERIOD = 0.020
DATA_STALE_TIMEOUT = 0.5


class TrackingControllerUSB:
    """红色激光追踪主控制器（USB摄像头版本）"""

    def __init__(self, camera_index=0, mcu_port="/dev/ttyAMA0"):
        logger.info("=" * 60)
        logger.info("红色激光追踪系统 v1.0 (USB摄像头版本) 启动中...")
        logger.info("=" * 60)

        self.tracker = USBCameraTracker(camera_index=camera_index)

        self.mcu = MCUCommunicator(port=mcu_port)

        self.pid_pan = IncrementalPID(PAN_PID_CONFIG, name="Pan-PID")
        self.pid_tilt = IncrementalPID(TILT_PID_CONFIG, name="Tilt-PID")

        self.calib = Calibration()
        self.calib.load()

        self._running = False
        self._debug_print = True
        self._debug_counter = 0

        self._loop_count = 0
        self._tracking_count = 0
        self._lost_count = 0
        self._start_time = None

    def start(self):
        """启动追踪"""
        if not self.tracker.start():
            logger.error("无法打开摄像头, 退出")
            return False

        if not self.mcu.connect():
            logger.error("无法连接MCU串口, 退出")
            self.tracker.stop()
            return False

        time.sleep(0.5)

        self.mcu.send_center()
        time.sleep(0.5)

        self._running = True
        self._start_time = time.time()

        logger.info("追踪已启动! 画面中心 = (%d, %d)",
                   self.tracker.center_x, self.tracker.center_y)
        logger.info("按键: q=退出 c=归中 r=重置PID p=调试打印")
        logger.info("OpenCV窗口: 按ESC退出")

        try:
            self._control_loop()
        except KeyboardInterrupt:
            logger.info("收到中断信号, 正在退出...")
        finally:
            self.shutdown()

        return True

    def shutdown(self):
        """安全关闭系统"""
        logger.info("正在关闭系统...")
        self._running = False
        self.tracker.stop()
        self.mcu.send_center()
        self.mcu.disconnect()

        import cv2
        cv2.destroyAllWindows()

        logger.info("系统已关闭")

    def _control_loop(self):
        """主控制循环 (50Hz)"""
        while self._running:
            loop_start = time.time()

            if not self.tracker.update():
                time.sleep(0.01)
                continue

            ex, ey, detected = self.tracker.get_error()

            if self.tracker.is_stale(DATA_STALE_TIMEOUT):
                if self._debug_print and self._loop_count % 50 == 0:
                    logger.warning("摄像头数据超时! 跳过PID更新")
                time.sleep(CONTROL_PERIOD)
                continue

            delta_pan = 0.0
            delta_tilt = 0.0

            if detected:
                delta_pan = self.pid_pan.update(ex)
                delta_tilt = self.pid_tilt.update(ey)

                if abs(delta_pan) > 0.01 or abs(delta_tilt) > 0.01:
                    self.mcu.send_track_command(int(round(delta_pan)), int(round(delta_tilt)))

                self._tracking_count += 1
            else:
                self.pid_pan.reset()
                self.pid_tilt.reset()
                self._lost_count += 1

            if self._debug_print:
                self._debug_counter += 1
                if self._debug_counter >= 25:
                    self._debug_counter = 0
                    self._print_status(ex, ey, detected, delta_pan, delta_tilt)

            import cv2
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                logger.info("按下ESC键，退出...")
                break
            elif key == ord('c'):
                self.mcu.send_center()
                logger.info("舵机归中")
            elif key == ord('r'):
                self.pid_pan.reset()
                self.pid_tilt.reset()
                logger.info("PID已重置")

            self._loop_count += 1
            elapsed = time.time() - loop_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.005:
                if self._loop_count % 250 == 0:
                    logger.warning(f"控制循环超时: {-sleep_time*1000:.1f}ms")

        total_time = time.time() - self._start_time
        logger.info("=" * 60)
        logger.info(f"追踪结束. 总时间: {total_time:.1f}s")
        logger.info(f"总循环: {self._loop_count}, "
                    f"追踪: {self._tracking_count}, "
                    f"丢失: {self._lost_count}")
        logger.info(f"平均帧率: {self._loop_count/total_time:.1f}Hz")
        logger.info("=" * 60)

    def _print_status(self, ex, ey, detected, delta_pan, delta_tilt):
        """打印当前状态"""
        status = "追踪中" if detected else "目标丢失"
        logger.info(
            f"{status} | "
            f"误差=({ex:+4d},{ey:+4d}) | "
            f"修正=({delta_pan:+6.1f},{delta_tilt:+6.1f})"
        )


def main():
    parser = argparse.ArgumentParser(description="红色激光追踪系统（USB摄像头版本）")
    parser.add_argument("--camera", type=int, default=0,
                        help="摄像头索引（默认0）")
    parser.add_argument("--mcu-port", default="/dev/ttyAMA0",
                        help="MCU串口设备路径")
    parser.add_argument("--no-debug", action="store_true",
                        help="关闭调试输出")
    parser.add_argument("--no-gui", action="store_true",
                        help="关闭OpenCV窗口显示")
    args = parser.parse_args()

    controller = TrackingControllerUSB(
        camera_index=args.camera,
        mcu_port=args.mcu_port
    )
    if args.no_debug:
        controller._debug_print = False
    if args.no_gui:
        controller.tracker.set_debug(False)

    controller.start()


if __name__ == "__main__":
    main()