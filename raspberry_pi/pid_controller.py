"""
pid_controller.py - 增量式PID控制器 + 误差低通滤波
==================================================
参考: 2023电赛E题国二方案 (I-D控制器 + 四级低通滤波)
      目标追踪系统 GreenServo (Kp=0.6 Ki=-0.4 Kd=±0.1)

增量式公式:
  delta = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))

增强:
  - 误差一阶低通滤波: 抑制像素抖动, 防微分项过激
  - 积分分离: 大误差时禁用积分项
  - 输出限幅 + 积分限幅
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
    """增量式PID + 误差低通滤波"""

    # 误差低通滤波系数 (参考国二: Err_Now*0.85 + Err_Last*0.15)
    ERROR_FILTER_ALPHA = 0.15

    def __init__(self, config: PIDConfig, name="PID"):
        self.cfg = config
        self.name = name

        self._e_k = 0.0
        self._e_k_1 = 0.0
        self._e_k_2 = 0.0

        self._integral = 0.0
        self._last_output = 0.0

        # 误差低通滤波状态
        self._filtered_error = 0.0
        self._filter_initialized = False

        self._update_count = 0
        self._start_time = time.time()

    def reset(self):
        """重置PID状态"""
        self._e_k = 0.0
        self._e_k_1 = 0.0
        self._e_k_2 = 0.0
        self._integral = 0.0
        self._last_output = 0.0
        self._filtered_error = 0.0
        self._filter_initialized = False
        self._update_count = 0
        logger.info(f"{self.name}: PID已重置")

    def update(self, error: float) -> float:
        """
        增量式PID — 每帧调用, 输出增量(度)

        Args:
            error: 当前误差(像素)。正=目标在中心右侧/下方

        Returns:
            delta: 输出修正量(度), 累加到目标角度
        """
        # ─── 误差低通滤波 ───
        if not self._filter_initialized:
            self._filtered_error = error
            self._filter_initialized = True
        else:
            self._filtered_error = (self._filtered_error * (1.0 - self.ERROR_FILTER_ALPHA)
                                    + error * self.ERROR_FILTER_ALPHA)

        self._e_k = self._filtered_error

        # ─── 死区 ───
        if abs(self._e_k) <= self.cfg.deadband:
            self._e_k_2 = self._e_k_1
            self._e_k_1 = self._e_k
            self._last_output = 0.0
            return 0.0

        # ─── 积分分离 ───
        if abs(self._e_k) > self.cfg.integral_separation_threshold:
            ki_effective = 0.0
        else:
            ki_effective = self.cfg.ki
            self._integral += self._e_k
            self._integral = max(self.cfg.integral_min,
                                 min(self.cfg.integral_max, self._integral))

        # ─── 增量式PID ───
        p_term = self.cfg.kp * (self._e_k - self._e_k_1)
        i_term = ki_effective * self._e_k
        d_term = self.cfg.kd * (self._e_k - 2 * self._e_k_1 + self._e_k_2)

        delta = p_term + i_term + d_term

        # ─── 输出限幅 ───
        delta = max(self.cfg.output_min,
                    min(self.cfg.output_max, delta))

        # ─── 更新历史 ───
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
        logger.info(f"{self.name}: 参数已更新 Kp={self.cfg.kp} "
                    f"Ki={self.cfg.ki} Kd={self.cfg.kd}")


# ════════════════════════════════════════════════════
#  Pan 轴 PID 配置
# ════════════════════════════════════════════════════
PAN_PID_CONFIG = PIDConfig(
    kp=0.60,      # 参考GreenServo: 0.6*(e(k)-e(k-1))
    ki=0.40,      # 参考GreenServo: 0.4*e(k)
    kd=0.10,      # 参考GreenServo: 0.1*(e(k)-2e(k-1)+e(k-2))
    output_max=0.3,    # 单次增量±0.3°, 15°/s舵机能跟上
    output_min=-0.3,
    integral_max=1.0,  # I项累积上限
    integral_min=-1.0,
    deadband=8.0,      # |误差|<8px 不修正
    integral_separation_threshold=999.0  # 不禁用积分(积分上限已防饱和)
)

# ════════════════════════════════════════════════════
#  Tilt 轴 PID 配置 (灵敏度~23px/°, 比Pan高9倍, 参数单独调低)
# ════════════════════════════════════════════════════
TILT_PID_CONFIG = PIDConfig(
    kp=0.20,     # Pan的1/3, 补偿高灵敏度
    ki=0.13,     # Pan的1/3
    kd=0.03,     # Pan的1/3
    output_max=0.12,  # Pan的1/2.5, 每步移动约3px
    output_min=-0.12,
    integral_max=0.4,
    integral_min=-0.4,
    deadband=8.0,
    integral_separation_threshold=999.0
)
