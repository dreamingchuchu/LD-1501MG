"""
usb_camera_tracker.py - 树莓派USB摄像头红色激光检测
功能: 使用USB摄像头实时检测红色激光光斑位置，替代OpenMV
依赖: pip install opencv-python numpy
"""

import cv2
import numpy as np
import time
import logging

logger = logging.getLogger(__name__)


class USBCameraTracker:
    """USB摄像头红色激光追踪器"""

    def __init__(self, camera_index=0, width=320, height=240):
        """
        初始化USB摄像头追踪器

        Args:
            camera_index: 摄像头索引（默认0，如果有多个摄像头可以尝试1,2...）
            width: 图像宽度
            height: 图像高度
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height

        self._cap = None
        self._cx = -1
        self._cy = -1
        self._detected = False
        self._timestamp = 0.0

        self.min_blob_area = 5
        self.max_blob_area = 2000

        self.center_x = width // 2
        self.center_y = height // 2

        self._debug = True

    def start(self):
        """打开摄像头"""
        try:
            self._cap = cv2.VideoCapture(self.camera_index)

            if not self._cap.isOpened():
                logger.error(f"无法打开摄像头 {self.camera_index}")
                return False

            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            logger.info(f"USB摄像头已打开: {self.width}x{self.height}")
            return True

        except Exception as e:
            logger.error(f"摄像头初始化失败: {e}")
            return False

    def stop(self):
        """关闭摄像头"""
        if self._cap:
            self._cap.release()
            logger.info("摄像头已关闭")

    def detect_red_laser(self, frame):
        """
        检测红色激光光斑

        Args:
            frame: BGR格式的图像帧

        Returns:
            (cx, cy, detected): 质心坐标和检测状态
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return -1, -1, False

        best_contour = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_blob_area <= area <= self.max_blob_area and area > max_area:
                max_area = area
                best_contour = contour

        if best_contour is None:
            return -1, -1, False

        M = cv2.moments(best_contour)
        if M["m00"] == 0:
            return -1, -1, False

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return cx, cy, True

    def update(self):
        """
        更新检测状态（每帧调用一次）

        Returns:
            bool: 是否成功获取帧
        """
        if not self._cap or not self._cap.isOpened():
            return False

        ret, frame = self._cap.read()
        if not ret:
            logger.warning("无法读取摄像头帧")
            return False

        cx, cy, detected = self.detect_red_laser(frame)

        self._cx = cx
        self._cy = cy
        self._detected = detected
        self._timestamp = time.time()

        if self._debug:
            self._draw_debug(frame, cx, cy, detected)

        return True

    def _draw_debug(self, frame, cx, cy, detected):
        """绘制调试信息"""
        cv2.line(frame, (self.center_x, 0), (self.center_x, self.height), (255, 255, 255), 1)
        cv2.line(frame, (0, self.center_y), (self.width, self.center_y), (255, 255, 255), 1)

        if detected:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
            cv2.line(frame, (self.center_x, self.center_y), (cx, cy), (0, 0, 255), 2)

            error_x = cx - self.center_x
            error_y = cy - self.center_y
            cv2.putText(frame, f"Error: ({error_x}, {error_y})", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        status = "TRACKING" if detected else "LOST"
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(frame, status, (10, self.height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("Red Laser Tracker", frame)
        cv2.waitKey(1)

    @property
    def cx(self):
        return self._cx

    @property
    def cy(self):
        return self._cy

    @property
    def detected(self):
        return self._detected

    @property
    def timestamp(self):
        return self._timestamp

    def get_position(self):
        """获取最新坐标"""
        return (self._cx, self._cy)

    def get_error(self, center_x=None, center_y=None):
        """
        获取相对于画面中心的误差

        Args:
            center_x: 画面中心X（默认使用self.center_x）
            center_y: 画面中心Y（默认使用self.center_y）

        Returns:
            (error_x, error_y, detected)
        """
        if center_x is None:
            center_x = self.center_x
        if center_y is None:
            center_y = self.center_y

        if self._detected:
            return (self._cx - center_x, self._cy - center_y, True)
        else:
            return (0, 0, False)

    def is_stale(self, max_age=0.5):
        """检查数据是否过期"""
        if self._timestamp == 0:
            return True
        return (time.time() - self._timestamp) > max_age

    def set_debug(self, enable):
        """设置调试模式"""
        self._debug = enable

    def set_threshold(self, min_area=None, max_area=None):
        """设置检测阈值"""
        if min_area is not None:
            self.min_blob_area = min_area
        if max_area is not None:
            self.max_blob_area = max_area