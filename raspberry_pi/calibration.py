"""
calibration.py - 像素→舵机PWM映射校准
简化版: 只需估算 scale = PWM_per_degree / pixels_per_degree
"""

import json
import os
import logging

logger = logging.getLogger(__name__)

CALIB_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")


class Calibration:
    """
    像素→PWM 线性映射参数

    pwm_delta = pixel_error * scale_x
    正像素误差 → 正PWM修正 → 舵机向右/下转
    """

    def __init__(self):
        self.scale_x = 0.05
        self.scale_y = 0.05

        self.center_x = 160
        self.center_y = 120

        self.servo_center_pan = 1500
        self.servo_center_tilt = 1500

        self.pwm_min = 600
        self.pwm_max = 2500

    def pixel_to_pwm_delta(self, pixel_error_x: float, pixel_error_y: float):
        """
        将像素误差转换为PWM修正增量

        Args:
            pixel_error_x: X轴像素误差 (光斑X - 中心X)
            pixel_error_y: Y轴像素误差 (光斑Y - 中心Y)

        Returns:
            (delta_pan, delta_tilt): PWM修正量
        """
        delta_pan = pixel_error_x * self.scale_x
        delta_tilt = pixel_error_y * self.scale_y
        return (delta_pan, delta_tilt)

    def auto_estimate_scale(self, pixel_change, pwm_change):
        """
        自动估算缩放因子

        用法: 手动移动舵机N步，记录像素变化量
              scale = pwm_change / pixel_change

        Args:
            pixel_change: 光斑在画面中移动的像素数
            pwm_change: 对应的舵机PWM变化量
        """
        if pixel_change != 0:
            self.scale_x = abs(pwm_change / pixel_change)
            self.scale_y = abs(pwm_change / pixel_change)
            logger.info(f"估算Scale: {self.scale_x:.4f} PWM/px")
            return self.scale_x
        return 0

    def save(self, filepath=CALIB_FILE):
        """保存校准参数到文件"""
        data = {
            "scale_x": self.scale_x,
            "scale_y": self.scale_y,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "servo_center_pan": self.servo_center_pan,
            "servo_center_tilt": self.servo_center_tilt,
            "pwm_min": self.pwm_min,
            "pwm_max": self.pwm_max,
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
        logger.info(f"校准参数已保存: {filepath}")

    def load(self, filepath=CALIB_FILE):
        """从文件加载校准参数"""
        if not os.path.exists(filepath):
            logger.warning(f"校准文件不存在: {filepath}, 使用默认参数")
            return False

        try:
            with open(filepath, "r") as f:
                data = json.load(f)

            self.scale_x = data.get("scale_x", self.scale_x)
            self.scale_y = data.get("scale_y", self.scale_y)
            self.center_x = data.get("center_x", self.center_x)
            self.center_y = data.get("center_y", self.center_y)
            self.servo_center_pan = data.get("servo_center_pan", self.servo_center_pan)
            self.servo_center_tilt = data.get("servo_center_tilt", self.servo_center_tilt)
            self.pwm_min = data.get("pwm_min", self.pwm_min)
            self.pwm_max = data.get("pwm_max", self.pwm_max)

            logger.info(f"校准参数已加载: {filepath}")
            return True
        except Exception as e:
            logger.error(f"加载校准文件失败: {e}")
            return False