"""
search_engines.py — 搜索策略引擎（局部九宫格 + 全局蛇形扫描）
==============================================================
从 main_controller_usb.py 拆分出来, 减少主文件体积。
"""

# 改进的搜索参数
SEARCH_PAN_MIN = 20          # Pan最小角度（扩大范围）
SEARCH_PAN_MAX = 160         # Pan最大角度（扩大范围）
SEARCH_PAN_STEP = 5          # Pan每步度数（减小步长，提高命中率）
SEARCH_TILT_MIN = 0          # Tilt最小角度（从0开始，覆盖正前方）
SEARCH_TILT_MAX = 150        # Tilt最大角度

# Tilt扫描序列：从T=0开始逐级向上扫描（蛇形扫描的行坐标）
SEARCH_TILT_LEVELS = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150]


class LocalSearchEngine:
    """局部搜索引擎：多轮扩大九宫格搜索"""

    # 搜索范围逐轮扩大: 第1轮±8° → 第2轮±16° → 第3轮±24°
    SEARCH_RANGES = [8, 16, 24]

    def __init__(self):
        self._grid_positions = []
        self._current_idx = 0
        self._current_round = 0   # 当前搜索轮次
        self._frames_at_position = 0
        self._step_start_time = 0.0

    def generate_grid(self, center_pan, center_tilt):
        """生成多轮扩大的九宫格位置"""
        self._grid_positions = []
        self._current_round = 0
        self._current_idx = 0
        self._center_pan = center_pan
        self._center_tilt = center_tilt

        # 生成第1轮位置
        self._add_round_positions(0)

    def _add_round_positions(self, round_idx):
        """生成指定轮次的九宫格位置"""
        if round_idx >= len(self.SEARCH_RANGES):
            return
        d = self.SEARCH_RANGES[round_idx]
        offsets = [-d, 0, +d]
        for dp in offsets:
            for dt in offsets:
                pan = max(0, min(180, self._center_pan + dp))
                tilt = max(0, min(180, self._center_tilt + dt))
                self._grid_positions.append((pan, tilt))

    def get_next_position(self):
        """返回下一个搜索位置，当前轮扫完自动扩大"""
        if self._current_idx >= len(self._grid_positions):
            # 当前轮扫完，尝试下一轮
            if self._current_round + 1 < len(self.SEARCH_RANGES):
                self._current_round += 1
                self._add_round_positions(self._current_round)
            else:
                return None, None  # 所有轮次扫完

        if self._current_idx >= len(self._grid_positions):
            return None, None

        pan, tilt = self._grid_positions[self._current_idx]
        self._current_idx += 1
        return pan, tilt

    def is_complete(self):
        """判断所有轮次是否扫描完成"""
        return (self._current_idx >= len(self._grid_positions)
                and self._current_round >= len(self.SEARCH_RANGES) - 1)

    def reset(self):
        """重置搜索引擎状态"""
        self._current_idx = 0
        self._current_round = 0
        self._frames_at_position = 0


class GlobalSearchEngine:
    """全局搜索引擎：蛇形扫描"""

    def __init__(self):
        self._tilt_levels = SEARCH_TILT_LEVELS
        self._current_tilt_idx = 0
        self._current_pan = SEARCH_PAN_MIN
        self._scan_forward = True
        self._search_step = 0  # 0=MOVE, 1=WAIT, 2=CAPTURE, 3=DECIDE
        self._step_start_time = 0.0
        self._frames_at_position = 0

    def get_next_position(self):
        """按蛇形扫描策略生成下一个扫描位置"""
        if self._current_tilt_idx >= len(self._tilt_levels):
            return None, None

        tilt = self._tilt_levels[self._current_tilt_idx]
        pan = self._current_pan

        # 更新下一个位置（蛇形扫描）
        if self._scan_forward:
            self._current_pan += SEARCH_PAN_STEP
            if self._current_pan >= SEARCH_PAN_MAX:
                self._current_pan = SEARCH_PAN_MAX
                self._scan_forward = False
        else:
            self._current_pan -= SEARCH_PAN_STEP
            if self._current_pan <= SEARCH_PAN_MIN:
                self._current_pan = SEARCH_PAN_MIN
                self._current_tilt_idx += 1
                self._scan_forward = True

        return pan, tilt

    def is_complete(self):
        """判断全图扫描是否完成"""
        return self._current_tilt_idx >= len(self._tilt_levels)

    def reset(self):
        """重置搜索引擎状态"""
        self._current_tilt_idx = 0
        self._current_pan = SEARCH_PAN_MIN
        self._scan_forward = True
        self._frames_at_position = 0
        self._search_step = 0
