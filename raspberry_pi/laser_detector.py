"""
laser_detector.py — 6 阶段激光识别流水线
=========================================
Stage 1: HSV 双通道红色筛选
Stage 2: 形态学去噪 (开/闭运算)
Stage 3: 找轮廓 + 计算特征 (area, ratio, circularity, brightness)
Stage 4: 面积过滤
Stage 5: 宽高比 + 圆度 + 亮度过滤
Stage 6: 连续性过滤 (★ 比赛最关键)

原则:
  - 没有任何魔法数字, 全部读自 DetectionConfig
  - 每阶段可以独立开关
  - Debug 模式显示所有候选和它们的特征
"""
import cv2
import numpy as np
from detection_config import DetectionConfig


# ═══ 单个候选目标 ═══════════════════════════════════════
class LaserCandidate:
    """一个轮廓对应的所有特征, 以及它在各阶段是否通过"""

    def __init__(self, contour, frame):
        self.contour = contour

        # ── 基础特征 ──
        M = cv2.moments(contour)
        if M["m00"] == 0:
            raise ValueError("zero area")
        self.cx = int(M["m10"] / M["m00"])
        self.cy = int(M["m01"] / M["m00"])
        self.area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        self.x, self.y, self.width, self.height = x, y, w, h
        self.ratio = w / h if h > 0 else 0
        self.perimeter = cv2.arcLength(contour, True)
        self.circularity = (4 * np.pi * self.area / (self.perimeter ** 2)
                            if self.perimeter > 0 else 0)

        # ── 亮度特征 (在原图的 mask 区域统计 HSV V 通道) ──
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        contour_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        v_pixels = hsv[:, :, 2][contour_mask == 255]
        self.mean_v = int(np.mean(v_pixels)) if len(v_pixels) > 0 else 0
        self.max_v  = int(np.max(v_pixels))  if len(v_pixels) > 0 else 0

        # ── 各阶段通过状态 ──
        self.pass_area       = False
        self.pass_ratio      = False
        self.pass_circularity = False
        self.pass_brightness = False
        self.pass_continuity = False
        self.score = 0.0

        # ── 诊断字段 (不参与过滤, 仅用于日志) ──
        self.reject_reason   = None    # 第一失败的阶段名
        self.reject_detail   = None    # 失败原因 (e.g. "61<5" 或 "Distance=54/40")
        self.dist_to_last    = None    # 距上一帧目标的距离 (None 表示无上一帧)
        self.is_target       = False   # 是否最终被选中

    def feature_str(self):
        return (f"A={self.area:5.0f} "
                f"r={self.ratio:.2f} "
                f"C={self.circularity:.2f} "
                f"V={self.mean_v:3d}/{self.max_v:3d}")

    def filter_str(self):
        flags = []
        flags.append("A" if self.pass_area       else "x")
        flags.append("R" if self.pass_ratio      else "x")
        flags.append("C" if self.pass_circularity else "x")
        flags.append("B" if self.pass_brightness else "x")
        flags.append("T" if self.pass_continuity else "x")
        return "[" + "".join(flags) + "]"

    def build_report(self, max_move, dist_to_last=None):
        """
        生成候选目标的多阶段报告 (用于诊断日志)
        返回多行字符串
        """
        lines = []
        lines.append(f"  Candidate #{self.cx},{self.cy}")
        # ── Area ──
        area_pass = "PASS" if self.pass_area else "FAIL"
        lines.append(f"    Area         : {self.area:5.0f}            [{area_pass}]")
        # ── Ratio ──
        ratio_pass = "PASS" if self.pass_ratio else "FAIL"
        lines.append(f"    Ratio        : {self.ratio:.2f}              [{ratio_pass}]")
        # ── Circularity ──
        circ_pass = "PASS" if self.pass_circularity else "FAIL"
        lines.append(f"    Circularity  : {self.circularity:.2f}              [{circ_pass}]")
        # ── Brightness ──
        brt_pass = "PASS" if self.pass_brightness else "FAIL"
        lines.append(f"    Brightness   : Mean={self.mean_v:3d}  Max={self.max_v:3d}   [{brt_pass}]")
        # ── Continuity ──
        d_str = f"{dist_to_last:.0f}" if dist_to_last is not None else "-"
        cont_pass = "PASS" if self.pass_continuity else "FAIL"
        lines.append(f"    Continuity   : Distance={d_str}  Threshold={max_move:.0f}   [{cont_pass}]")
        if self.reject_reason:
            lines.append(f"    Reject Reason: {self.reject_reason}  ({self.reject_detail})")
        return "\n".join(lines)


# ═══ 主检测器 ═══════════════════════════════════════════
class LaserDetector:
    """6 阶段激光识别流水线"""

    def __init__(self, config=None):
        self.cfg = config or DetectionConfig()

        # ── 状态 ──
        self.last_target = None      # 上一帧确认的目标 (cx, cy)
        self.target_buffer = []      # 最近 N 帧通过的候选 (用于连续性)
        self.miss_count = 0          # 连续丢失帧数

        # ── Debug 用 ──
        self.last_info = None        # 最近一帧的检测信息
        self.last_mask = None        # 最近一帧的 HSV mask

    def detect(self, frame):
        """
        主入口

        Returns:
            (cx, cy, detected, info)
            info = {
                "candidates": 全部候选,
                "passed":     通过严格过滤的,
                "target":     最终选中的 (或 None),
                "mask":       HSV mask
            }
        """
        h, w = frame.shape[:2]

        # ── Stage 1: HSV 筛选 ──
        if self.cfg.ENABLE_HSV:
            mask = self._stage1_hsv(frame)
        else:
            # HSV 关闭时, 用亮度阈值兜底
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # ── Stage 2: 形态学 ──
        if self.cfg.ENABLE_MORPH:
            mask = self._stage2_morph(mask)
        self.last_mask = mask

        # ── Stage 3: 找轮廓 + 算特征 ──
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for cnt in contours:
            try:
                c = LaserCandidate(cnt, frame)
                candidates.append(c)
            except (ValueError, ZeroDivisionError):
                continue

        # ── Stage 4 & 5: 逐层过滤 ──
        for c in candidates:
            c.pass_area        = self._stage4_area(c)
            c.pass_ratio       = self._stage5_ratio(c)
            c.pass_circularity = self._stage5_circularity(c)
            c.pass_brightness  = self._stage5_brightness(c)
            # ★ 诊断: 记录第一失败的阶段 (不改变过滤逻辑)
            self._annotate_reject(c)

        # 通过"硬过滤"的候选 (Stage 4 + 5)
        passed = [c for c in candidates if
                  c.pass_area and c.pass_ratio and
                  c.pass_circularity and c.pass_brightness]

        # ── Stage 6: 连续性 ──
        if self.cfg.ENABLE_CONTINUITY:
            target = self._stage6_continuity(passed)
        else:
            # 没开连续性, 直接选第一个通过的
            target = passed[0] if passed else None
            if target:
                target.pass_continuity = True
            # ★ 诊断: 没开连续性时, 也要给其他候选标 reject
            for c in passed[1:]:
                c.reject_reason = "Continuity"
                c.reject_detail = "DISABLED (filter not running)"

        # ── 标记最终目标 ──
        for c in candidates:
            c.is_target = (c is target)

        # ★ 诊断: 给每个通过的候选算距离 (用于连续性可视化)
        if self.last_target is not None:
            for c in passed:
                c.dist_to_last = self._dist(c, self.last_target)

        # ★ 诊断: 给没有成为 target 的通过候选标 Continuity
        for c in passed:
            if c is not target and c.reject_reason is None:
                c.reject_reason = "Continuity"
                if c.dist_to_last is not None:
                    c.reject_detail = f"dist={c.dist_to_last:.0f} > MAX_MOVE={self.cfg.MAX_MOVE}"
                else:
                    c.reject_detail = "no last_target"
                c.pass_continuity = False

        # ── 更新状态 ──
        if target is not None:
            self.last_target = (target.cx, target.cy)
            self.miss_count = 0
        else:
            self.miss_count += 1
            if self.miss_count >= self.cfg.CONTINUITY_MISS:
                # 连续 N 帧都没有, 真正放弃
                self.last_target = None
                self.target_buffer = []

        info = {
            "candidates": candidates,
            "passed":     passed,
            "target":     target,
            "mask":       mask,
            # ★ 诊断字段
            "frame_reject_reason": None,    # 本帧最终失败原因
            "frame_reject_detail": None,    # 本帧最终失败详情
            "best_candidate":     None,     # 最好的候选 (面积最大), 用于详细日志
        }
        # ── 决定本帧的最终失败原因 ──
        info = self._build_frame_failure_info(info)
        self.last_info = info

        if target is not None:
            return target.cx, target.cy, True, info
        return -1, -1, False, info

    # ─── Stage 1: HSV ─────────────────────────────────────
    def _stage1_hsv(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1 = np.array([self.cfg.HUE_LOWER_1, self.cfg.SAT_MIN, self.cfg.VAL_MIN_HSV])
        upper1 = np.array([self.cfg.HUE_UPPER_1, 255, 255])
        lower2 = np.array([self.cfg.HUE_LOWER_2, self.cfg.SAT_MIN, self.cfg.VAL_MIN_HSV])
        upper2 = np.array([self.cfg.HUE_UPPER_2, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        return mask1 + mask2

    # ─── Stage 2: 形态学 ──────────────────────────────────
    def _stage2_morph(self, mask):
        k = self.cfg.MORPH_KERNEL
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        if self.cfg.MORPH_OPEN:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if self.cfg.MORPH_CLOSE:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    # ─── Stage 4: 面积 ────────────────────────────────────
    def _stage4_area(self, c):
        return self.cfg.MIN_AREA <= c.area <= self.cfg.MAX_AREA

    # ─── Stage 5: 宽高比 ─────────────────────────────────
    def _stage5_ratio(self, c):
        return self.cfg.MIN_RATIO <= c.ratio <= self.cfg.MAX_RATIO

    # ─── Stage 5: 圆度 ───────────────────────────────────
    def _stage5_circularity(self, c):
        return c.circularity >= self.cfg.MIN_CIRCULARITY

    # ─── Stage 5: 亮度 ───────────────────────────────────
    def _stage5_brightness(self, c):
        return (c.mean_v >= self.cfg.MIN_MEAN_V and
                c.max_v  >= self.cfg.MIN_MAX_V)

    # ─── 诊断: 标注候选目标的第一失败阶段 (不改变过滤) ──
    def _annotate_reject(self, c):
        """
        按 Stage 4→5 顺序, 找到第一个失败的阶段并记录原因.
        这一步不影响过滤, 只填充 c.reject_reason / c.reject_detail.
        """
        # ── 4: 面积 ──
        if not c.pass_area:
            c.reject_reason = "Area"
            if c.area < self.cfg.MIN_AREA:
                c.reject_detail = f"{c.area:.0f} < MIN={self.cfg.MIN_AREA}"
            else:
                c.reject_detail = f"{c.area:.0f} > MAX={self.cfg.MAX_AREA}"
            return
        # ── 5a: 宽高比 ──
        if not c.pass_ratio:
            c.reject_reason = "Ratio"
            if c.ratio < self.cfg.MIN_RATIO:
                c.reject_detail = f"{c.ratio:.2f} < MIN={self.cfg.MIN_RATIO}"
            else:
                c.reject_detail = f"{c.ratio:.2f} > MAX={self.cfg.MAX_RATIO}"
            return
        # ── 5b: 圆度 ──
        if not c.pass_circularity:
            c.reject_reason = "Circle"
            c.reject_detail = f"{c.circularity:.2f} < MIN={self.cfg.MIN_CIRCULARITY}"
            return
        # ── 5c: 亮度 ──
        if not c.pass_brightness:
            c.reject_reason = "Brightness"
            reasons = []
            if c.mean_v < self.cfg.MIN_MEAN_V:
                reasons.append(f"mean={c.mean_v}<{self.cfg.MIN_MEAN_V}")
            if c.max_v < self.cfg.MIN_MAX_V:
                reasons.append(f"max={c.max_v}<{self.cfg.MIN_MAX_V}")
            c.reject_detail = ", ".join(reasons) if reasons else "too dim"
            return
        # 全部通过, 不设 reject

    def _build_frame_failure_info(self, info):
        """
        根据 info 算出本帧的最终失败原因.
        优先级: HSV (0 候选) > 硬过滤 (取第一个失败) > Continuity.
        """
        candidates = info["candidates"]
        target     = info["target"]

        # ── 情况 1: 成功 ──
        if target is not None:
            return info

        # ── 情况 2: HSV 没找到任何候选 ──
        if not candidates:
            info["frame_reject_reason"] = "HSV"
            info["frame_reject_detail"] = "0 candidates after HSV"
            return info

        # ── 情况 3: 有候选但全部被硬过滤拒 ──
        if info["passed"]:
            # 至少有一个通过硬过滤 → 失败原因是 Continuity
            info["frame_reject_reason"] = "Continuity"
            # 找距离 last_target 最近的
            best = self._closest_to_last(info["passed"])
            if best is not None and self.last_target is not None:
                d = self._dist(best, self.last_target)
                info["frame_reject_detail"] = (
                    f"closest dist={d:.0f} > MAX_MOVE={self.cfg.MAX_MOVE} "
                    f"(cx={best.cx},cy={best.cy})"
                )
            else:
                info["frame_reject_detail"] = (
                    f"{len(info['passed'])} passed, but no continuity match "
                    f"(last_target={self.last_target})"
                )
            info["best_candidate"] = best
            return info

        # ── 情况 4: 有候选但都被硬过滤拒 → 取 reject 列表中第一个 ──
        # 按面积从大到小排序, 第一个就是"最像激光的"
        sorted_cands = sorted(candidates, key=lambda x: x.area, reverse=True)
        first_reject = sorted_cands[0]
        info["frame_reject_reason"] = first_reject.reject_reason or "Unknown"
        info["frame_reject_detail"] = first_reject.reject_detail or ""
        info["best_candidate"]      = first_reject
        return info

    def _closest_to_last(self, candidates):
        if self.last_target is None:
            return candidates[0] if candidates else None
        best, best_d = None, float("inf")
        for c in candidates:
            d = self._dist(c, self.last_target)
            if d < best_d:
                best, best_d = c, d
        return best

    def _dist(self, c, last):
        return ((c.cx - last[0]) ** 2 + (c.cy - last[1]) ** 2) ** 0.5

    # ─── Stage 6: 连续性 (★ 核心) ───────────────────────
    def _stage6_continuity(self, candidates):
        """
        逻辑:
          1) 如果上一帧没有目标 → 这一帧的候选先入缓冲区
          2) 如果上一帧有目标 → 找距离最近的候选, 距离 < MAX_MOVE 才接受
          3) 缓冲区满 CONTINUITY_HITS 帧 → 确认目标
          4) 任何不符合的, 标 pass_continuity=False
        """
        # ── 情况 1: 上一帧没目标 ──
        if self.last_target is None:
            if candidates:
                # 第一个候选先入缓冲
                self.target_buffer.append(candidates[0])
                if len(self.target_buffer) >= self.cfg.CONTINUITY_HITS:
                    # 缓冲满了, 确认
                    target = self.target_buffer[-1]
                    target.pass_continuity = True
                    self.target_buffer = [target]
                    return target
            return None

        # ── 情况 2: 上一帧有目标 ──
        best = None
        best_dist = float('inf')
        for c in candidates:
            dx = c.cx - self.last_target[0]
            dy = c.cy - self.last_target[1]
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best = c

        if best is None or best_dist > self.cfg.MAX_MOVE:
            # 距离超了, 这次不算
            # 但不要清 last_target, 留给下一帧 (物体可能短暂消失)
            return None

        # 距离 OK, 接受
        best.pass_continuity = True
        self.target_buffer.append(best)
        if len(self.target_buffer) > self.cfg.CONTINUITY_HITS:
            self.target_buffer = self.target_buffer[-self.cfg.CONTINUITY_HITS:]
        return best

    # ─── Debug 可视化 ────────────────────────────────────
    def draw_debug(self, frame, info,
                   exposure=None, exposure_trend=None):
        """
        画所有候选 + 特征 + 最终目标
        颜色编码:
          绿色 (0,255,0)   = 最终目标
          黄色 (0,255,255) = 通过硬过滤但被连续性拒 (候选)
          红色 (0,0,255)   = 硬过滤拒掉的 (面积/比/圆/亮)
          灰色 (128,128,128) = 没有任何拒绝信息 (异常)
        """
        if self.cfg.DEBUG_LEVEL == 0:
            return frame

        vis = frame.copy()
        h, w = vis.shape[:2]

        # ── 画所有候选 ──
        for i, c in enumerate(info["candidates"]):
            # ── 决定颜色 ──
            if c.is_target:
                color = (0, 255, 0)        # 绿色: 最终目标
            elif c.pass_continuity:
                color = (0, 255, 255)      # 黄色: 通过但不是最终
            elif c in info["passed"]:
                color = (0, 200, 255)      # 橙色: 通过硬过滤, 被连续性拒
            elif c.reject_reason:
                color = (0, 0, 255)        # 红色: 硬过滤拒
            else:
                color = (128, 128, 128)    # 灰色: 异常

            cv2.rectangle(vis, (c.x, c.y), (c.x + c.width, c.y + c.height),
                          color, 1)
            # ── 标注 (前 2 阶段) ──
            label1 = f"#{i} A:{c.area:.0f} r:{c.ratio:.2f} C:{c.circularity:.2f}"
            cv2.putText(vis, label1, (c.x, c.y - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1)
            label2 = f"V:{c.mean_v}/{c.max_v}"
            cv2.putText(vis, label2, (c.x, c.y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1)
            # ── 拒绝原因 ──
            if c.reject_reason:
                cv2.putText(vis, f"REJ:{c.reject_reason}", (c.x, c.y + c.height + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

        # ── 高亮最终目标 ──
        if info["target"]:
            t = info["target"]
            cv2.rectangle(vis, (t.x, t.y), (t.x + t.width, t.y + t.height),
                          (0, 255, 0), 2)
            cv2.circle(vis, (t.cx, t.cy), 5, (0, 255, 0), -1)
            cv2.putText(vis, f"TARGET ({t.cx},{t.cy})", (t.x, t.y + t.height + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # ── 连续性可视化: 从 last_target 画线到所有通过候选 ──
        if self.last_target is not None:
            lx, ly = self.last_target
            cv2.circle(vis, (lx, ly), 3, (255, 0, 255), -1)   # 紫点: 上一帧
            for c in info["passed"]:
                d = c.dist_to_last if c.dist_to_last is not None else 0
                line_color = (0, 255, 0) if d <= self.cfg.MAX_MOVE else (0, 0, 255)
                cv2.line(vis, (lx, ly), (c.cx, c.cy), line_color, 1)
                mid_x = (lx + c.cx) // 2
                mid_y = (ly + c.cy) // 2
                cv2.putText(vis, f"d={d:.0f}/{self.cfg.MAX_MOVE:.0f}",
                            (mid_x + 3, mid_y - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, line_color, 1)

        # ── 顶部状态条 ──
        status = f"DBG={self.cfg.DEBUG_LEVEL}  "
        status += f"cands={len(info['candidates'])}  "
        status += f"passed={len(info['passed'])}  "
        status += f"miss={self.miss_count}"
        # ★ 失败原因直接显示
        if info.get("frame_reject_reason"):
            status += f"  REJ={info['frame_reject_reason']}"
        cv2.putText(vis, status, (5, 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # ── 第二行: 失败原因详情 ──
        if info.get("frame_reject_detail"):
            cv2.putText(vis, info["frame_reject_detail"], (5, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 100, 255), 1)

        # ── DEBUG_LEVEL >= 2: 显示 mask ──
        if self.cfg.DEBUG_LEVEL >= 2 and self.last_mask is not None:
            mask_color = cv2.cvtColor(self.last_mask, cv2.COLOR_GRAY2BGR)
            mask_small = cv2.resize(mask_color, (w // 3, h // 3))
            vis[h - mask_small.shape[0]:, w - mask_small.shape[1]:] = mask_small
            cv2.putText(vis, "HSV MASK", (w - mask_small.shape[1] + 5, h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # ── 曝光诊断条 (DEBUG_LEVEL >= 1 都显示) ──
        if exposure is not None and 'mean_v' in exposure:
            bar_x, bar_y, bar_w, bar_h = 5, h - 20, 100, 12
            cv2.rectangle(vis, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                          (50, 50, 50), -1)
            fill = int(bar_w * exposure['mean_v'] / 255)
            color = (0, 0, 255) if exposure.get('jumped') else (0, 255, 0)
            cv2.rectangle(vis, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h),
                          color, -1)
            trend = f" {exposure_trend[0]}" if exposure_trend else ""
            cv2.putText(vis,
                        f"V:{exposure['mean_v']:.0f} S:{exposure.get('mean_s', 0):.0f}{trend}",
                        (bar_x + bar_w + 5, bar_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            if exposure.get('jumped'):
                cv2.putText(vis, "EXPOSURE JUMP", (bar_x, bar_y - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        return vis
