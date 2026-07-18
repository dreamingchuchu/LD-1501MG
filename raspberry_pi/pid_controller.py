"""
pid_controller.py - 增量式PID控制器
参考: 2023电赛E题国二方案PID.c + 目标追踪系统yuntai.c

增量式公式:
  delta = Kp*(e(k) - e(k-1)) + Ki*e(k) + Kd*(e(k) - 2*e(k-1) + e(k-2))

特性:
  - 积分分离: 大误差时禁用积分项
  - 输出限幅
  - 积分限幅 (防饱和)
  - 死区处理
"""

import time
import logging

logger = logging.getLogger(__name__)


class PIDConfig:
    """PID参数配置"""
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 output_max=30.0, output_min=-30.0,
                 integral_max=50.0, integral_min=-50.0,
                 deadband=3.0,
                 integral_separation_threshold=30.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_max = output_max
        self.output_min = output_min
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.deadband = deadband
        self.integral_separation_threshold = integral_separation_threshold


class IncrementalPID:
    """增量式PID控制器"""

    def __init__(self, config: PIDConfig, name="PID"):
        """
        Args:
            config: PID参数配置
            name: 控制器名称 (用于日志)
        """
        self.cfg = config
        self.name = name

        self._e_k = 0.0
        self._e_k_1 = 0.0
        self._e_k_2 = 0.0

        self._integral = 0.0

        self._last_output = 0.0

        self._start_time = time.time()
        self._update_count = 0

    def reset(self):
        """重置PID状态"""
        self._e_k = 0.0
        self._e_k_1 = 0.0
        self._e_k_2 = 0.0
        self._integral = 0.0
        self._last_output = 0.0
        self._update_count = 0
        logger.info(f"{self.name}: PID已重置")

    def update(self, error: float) -> float:
        """
        计算增量式PID输出

        Args:
            error: 当前误差。正=目标在中心右侧/下方

        Returns:
            delta: 输出修正量 (增量, 非绝对值)
                   正=向右/下移动
        """
        self._e_k = error

        if abs(error) <= self.cfg.deadband:
            self._e_k_2 = self._e_k_1
            self._e_k_1 = self._e_k
            self._last_output = 0.0
            return 0.0

        if abs(error) > self.cfg.integral_separation_threshold:
            ki_effective = 0.0
        else:
            ki_effective = self.cfg.ki
            self._integral += error
            self._integral = max(self.cfg.integral_min,
                                 min(self.cfg.integral_max, self._integral))

        p_term = self.cfg.kp * (self._e_k - self._e_k_1)
        i_term = ki_effective * self._e_k
        d_term = self.cfg.kd * (self._e_k - 2 * self._e_k_1 + self._e_k_2)

        delta = p_term + i_term + d_term

        delta = max(self.cfg.output_min,
                    min(self.cfg.output_max, delta))

        self._e_k_2 = self._e_k_1
        self._e_k_1 = self._e_k
        self._last_output = delta
        self._update_count += 1

        return delta

    def update_tuning(self, kp=None, ki=None, kd=None):
        """在线更新PID参数"""
        if kp is not None:
            self.cfg.kp = kp
        if ki is not None:
            self.cfg.ki = ki
        if kd is not None:
            self.cfg.kd = kd
        self.reset()
        logger.info(f"{self.name}: 参数已更新 Kp={self.cfg.kp} Ki={self.cfg.ki} Kd={self.cfg.kd}")


PAN_PID_CONFIG = PIDConfig(
    kp=0.15,
    ki=0.003,
    kd=0.08,
    output_max=25.0,
    output_min=-25.0,
    deadband=3.0,
    integral_separation_threshold=40.0
)

TILT_PID_CONFIG = PIDConfig(
    kp=0.15,
    ki=0.003,
    kd=0.06,
    output_max=25.0,
    output_min=-25.0,
    deadband=3.0,
    integral_separation_threshold=40.0
)