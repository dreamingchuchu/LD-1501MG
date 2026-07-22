"""
detection_config.py — 激光识别阈值配置 (生产版)
================================================
原则: 所有阈值集中在这里, 算法代码里不出现任何魔法数字
调试: 改完保存, 重启 main_controller_usb.py 即可生效

阶段开关 = 1: 启用
阶段开关 = 0: 禁用 (该阶段不参与过滤, 全部通过)
"""
import numpy as np


class DetectionConfig:
    """激光识别阈值配置"""

    # ═══ 阶段开关 (1=开, 0=关) ═══
    ENABLE_HSV          = 1
    ENABLE_MORPH        = 1
    ENABLE_AREA         = 1
    ENABLE_RATIO        = 1
    ENABLE_CIRCULARITY  = 1
    ENABLE_BRIGHTNESS   = 1
    ENABLE_CONTINUITY   = 1

    # ═══ 阶段 1: HSV 红色双通道 ═══
    HUE_LOWER_1  = 0
    HUE_UPPER_1  = 10
    HUE_LOWER_2  = 160
    HUE_UPPER_2  = 180
    SAT_MIN      = 50
    VAL_MIN_HSV  = 80

    # ═══ 阶段 2: 形态学去噪 ═══
    MORPH_KERNEL = 3
    MORPH_OPEN   = 1
    MORPH_CLOSE  = 1

    # ═══ 阶段 3: 面积过滤 ═══
    # 实测激光面积: 100~285 (随距离变化)
    MIN_AREA = 3
    MAX_AREA = 300

    # ═══ 阶段 4: 宽高比过滤 ═══
    MIN_RATIO = 0.6
    MAX_RATIO = 1.6

    # ═══ 阶段 5: 圆度过滤 ═══
    # 实测 0.85~0.91, 0.40 留出余量避免抖动误杀
    MIN_CIRCULARITY = 0.40

    # ═══ 阶段 6: 亮度过滤 ═══
    MIN_MEAN_V = 100
    MIN_MAX_V  = 200

    # ═══ 阶段 7: 连续性过滤 (★ 关键) ═══
    # MAX_MOVE=80        : 允许目标快速移动 / 重新进入画面
    # CONTINUITY_HITS=1  : 一帧就接受, 不需要凑连续帧
    # CONTINUITY_MISS=60 : 2 秒不丢才放弃 (从 5 → 60)
    MAX_MOVE        = 80
    CONTINUITY_HITS = 1
    CONTINUITY_MISS = 60
