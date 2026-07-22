"""
usb_camera_tracker.py — 树莓派 USB 摄像头红色激光检测 (生产版)
================================================================
6 阶段识别流水线 + 过曝高亮兜底:
  1. HSV 双通道红色筛选
  2. 形态学去噪
  3. 找轮廓 + 计算特征
  4. 面积过滤
  5. 宽高比 + 圆度 + 亮度过滤
  6. 连续性过滤 (★ 比赛最关键)

公开接口 (与旧版完全兼容, 主控代码不需要改):
  tracker.start()
  tracker.update()
  tracker.cx, tracker.cy, tracker.detected
  tracker.get_position()      -> (cx, cy, detected)
  tracker.get_error()         -> (ex, ey, detected)
  tracker.stop()

依赖:
  pip install opencv-python numpy
"""

import cv2
import numpy as np
import time
import logging

from detection_config import DetectionConfig
from laser_detector import LaserDetector

logger = logging.getLogger(__name__)


class USBCameraTracker:
    """USB 摄像头红色激光追踪器 (6 阶段识别版)"""

    def __init__(self, camera_index=0, width=320, height=240, config=None):
        """
        Args:
            camera_index: 摄像头索引
            width:  图像宽度
            height: 图像高度
            config: DetectionConfig 实例 (None = 用默认配置)
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height

        # ─── 内部状态 ───
        self._cap = None
        self._cx = -1
        self._cy = -1
        self._detected = False
        self._timestamp = 0.0

        self.center_x = width // 2
        self.center_y = height // 2

        # ─── 6 阶段识别流水线 ───
        self._cfg = config or DetectionConfig()
        self._detector = LaserDetector(self._cfg)

        # ─── 兜底: 过曝高亮检测 (远距离激光发白) ───
        self._brightness_fallback = True
        self._bright_min_v = 240   # 远距离激光过曝, V>240
        self._bright_max_area = 200  # 兜底只接受很小的亮区 (激光特征)

        # ─── 调试显示 ───
        self._debug = True   # 默认显示 GUI, 无显示器自动关闭

    # ════════════════════════════════════════════════════
    #  摄像头控制
    # ════════════════════════════════════════════════════

    def start(self):
        """打开摄像头"""
        try:
            self._cap = cv2.VideoCapture(self.camera_index)
            if not self._cap.isOpened():
                logger.error(f"无法打开摄像头 {self.camera_index}")
                return False
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            logger.info(f"USB 摄像头已打开: {self.width}x{self.height}")
            return True
        except Exception as e:
            logger.error(f"摄像头初始化失败: {e}")
            return False

    def stop(self):
        """关闭摄像头"""
        if self._cap:
            self._cap.release()
            logger.info("摄像头已关闭")

    # ════════════════════════════════════════════════════
    #  识别主入口
    # ════════════════════════════════════════════════════

    def detect_red_laser(self, frame):
        """
        检测红色激光光斑 (6 阶段流水线 + 过曝高亮兜底)

        Returns:
            (cx, cy, detected)
        """
        # ─── 主通道: 6 阶段流水线 ───
        cx, cy, detected, _info = self._detector.detect(frame)

        # ─── 兜底: 远距离激光过曝发白 (H 信息丢失) ───
        if not detected and self._brightness_fallback:
            cx, cy, detected = self._brightness_detect(frame)

        return cx, cy, detected

    def _brightness_detect(self, frame):
        """
        过曝高亮兜底检测
        远距离激光在廉价摄像头上会过曝发白 (H 信息丢失),
        此时 V 通道 > 240, 而 S 通道 < 50 (饱和度反而下降).
        只接受很小面积的亮区, 排除灯光 / 窗户等大面积高亮.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        _, _, v = cv2.split(hsv)
        _, mask = cv2.threshold(v, self._bright_min_v, 255, cv2.THRESH_BINARY)

        # 去噪
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return -1, -1, False

        # 取最小面积的亮区 (激光点特征: 面积小)
        best = min(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < 1 or area > self._bright_max_area:
            return -1, -1, False

        M = cv2.moments(best)
        if M["m00"] == 0:
            return -1, -1, False

        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), True

    # ════════════════════════════════════════════════════
    #  帧更新
    # ════════════════════════════════════════════════════

    def update(self):
        """
        更新检测状态 (每帧调用一次)

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
            try:
                self._draw_debug(frame, cx, cy, detected)
            except Exception:
                logger.warning("无法显示 GUI 窗口 (可能无显示器), 自动关闭画面预览")
                self._debug = False

        return True

    def _draw_debug(self, frame, cx, cy, detected):
        """绘制调试信息 (中心十字 + 目标点 + 状态)"""
        h, w = frame.shape[:2]
        cv2.line(frame, (self.center_x, 0), (self.center_x, h), (255, 255, 255), 1)
        cv2.line(frame, (0, self.center_y), (w, self.center_y), (255, 255, 255), 1)

        if detected:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
            cv2.line(frame, (self.center_x, self.center_y), (cx, cy), (0, 0, 255), 2)
            error_x = cx - self.center_x
            error_y = cy - self.center_y
            cv2.putText(frame, f"Error: ({error_x:+d}, {error_y:+d})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        status = "TRACKING" if detected else "LOST"
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(frame, status, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("Red Laser Tracker", frame)
        cv2.waitKey(1)

    # ════════════════════════════════════════════════════
    #  公开属性 (兼容旧版)
    # ════════════════════════════════════════════════════

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

    # ════════════════════════════════════════════════════
    #  公开方法 (兼容旧版, 主控代码不需要改)
    # ════════════════════════════════════════════════════

    def get_position(self):
        """获取最新目标位置 -> (cx, cy, detected)"""
        return (self._cx, self._cy, self._detected)

    def get_error(self, center_x=None, center_y=None):
        """
        获取相对于画面中心的误差 -> (ex, ey, detected)
        """
        if center_x is None:
            center_x = self.center_x
        if center_y is None:
            center_y = self.center_y

        if self._detected:
            return (self._cx - center_x, self._cy - center_y, True)
        return (0, 0, False)

    def is_stale(self, max_age=0.5):
        """检查数据是否过期"""
        if self._timestamp == 0:
            return True
        return (time.time() - self._timestamp) > max_age

    def set_debug(self, enable):
        """设置调试模式"""
        self._debug = enable

    # ════════════════════════════════════════════════════
    #  旧版兼容 (不再使用, 保留以防旧代码引用)
    # ════════════════════════════════════════════════════

    @property
    def min_blob_area(self):
        return self._cfg.MIN_AREA

    @property
    def max_blob_area(self):
        return self._cfg.MAX_AREA
