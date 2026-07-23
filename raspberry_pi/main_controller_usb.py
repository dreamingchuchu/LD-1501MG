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
from enum import Enum

import cv2

from usb_camera_tracker import USBCameraTracker
from mcu_communicator import MCUCommunicator
from pid_controller import IncrementalPID, PAN_PID_CONFIG, TILT_PID_CONFIG
from search_engines import LocalSearchEngine, GlobalSearchEngine
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
LOST_RESET_FRAMES = 5        # 连续丢帧N次后才真正重置PID（改为5帧）
SEARCH_SPEED_MS = 0.05       # 搜索扫描时每步停留时间(秒) — 加快扫描
MCU_CMD_INTERVAL = 0.08      # 80ms发一次, 0.3°×4帧=1.2°/次, 小步不甩飞
SERVO_WAIT_TIME = 0.200      # 舵机稳定等待时间(秒) — 搜索时舵机到位等待

# 舵机方向修正: 图像坐标轴与舵机运动方向可能相反
PAN_DIRECTION = -1    # 实测: P增大=云台左转, ex正→PID正→翻负→P减小→右转→回中 ✓
TILT_DIRECTION = -1   # Tilt: 正delta=T增大=仰头, ey正→PID正→翻负→低头→回中 ✓

# TRACK调试
TRACK_DEBUG_INTERVAL = 5       # 每N帧打印一次控制链日志

# 连续检测确认参数
DETECT_THRESHOLD = 3         # 连续3帧检测到才确认
LOSS_THRESHOLD = 15          # 连续15帧丢失才退出TRACK (舵机有运动延迟, 不能刚开始转就退出)


class SystemState(Enum):
    """系统状态枚举"""
    INIT = 0           # 初始化状态
    TRACK = 1          # 追踪状态
    LOCAL_SEARCH = 2   # 局部搜索状态
    GLOBAL_SEARCH = 3  # 全局搜索状态


class TrackingControllerUSB:
    """红色激光追踪主控制器（USB摄像头版本）"""

    def __init__(self, camera_index=0, mcu_port="/dev/ttyAMA0", no_gui=False):
        logger.info("=" * 60)
        logger.info("红色激光追踪系统 v1.0 (USB摄像头版本) 启动中...")
        logger.info("=" * 60)

        self._no_gui = no_gui

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

        # ─── 状态机架构 ──────────────────────────
        self._state = SystemState.INIT
        self._detect_count = 0
        self._lost_count_state = 0
        self._last_known_pan = 90.0
        self._last_known_tilt = 90.0
        self._init_start_time = 0.0
        self._init_cooling_off = False   # 搜索完毕冷却标志
        
        # 搜索引擎
        self._local_search = LocalSearchEngine()
        self._global_search = GlobalSearchEngine()
        
        # 搜索状态机变量
        self._search_step = 0  # 0=MOVE, 1=WAIT, 2=CAPTURE, 3=DECIDE
        self._step_start_time = 0.0
        self._frames_at_position = 0
        self._current_target_pan = 90.0    # 当前目标Pan角度
        self._current_target_tilt = 90.0   # 当前目标Tilt角度
        self._search_tilt_idx = 0          # 当前Tilt序列索引
        self._search_pan_angle = 0         # 当前Pan角度
        self._search_forward = True        # Pan扫描方向
        self._last_mcu_cmd_time = 0.0      # 上次发送MCU命令的时间戳
        self._search_detect_confirm = 0    # 搜索时当前位置连续检测计数
        self._track_debug_counter = 0      # TRACK调试日志计数器
        self._track_frame_count = 0        # TRACK帧计数（软启动+日志）

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

        # 启动第一件事: T=0 (home) + P=90 (中位)
        # T=0 让摄像头看正前方 (云台正立, 摄像头装在 T 轴)
        # 两条命令之间留足够间隔, 确保 MCU 逐个处理
        logger.info("归位: T=0 (home) + P=90 (中位)...")
        self.mcu.send_raw_command("T=0")
        time.sleep(0.3)  # 等 MCU 处理 T=0
        self.mcu.send_raw_command("T=0")  # 再发一次确保到位
        time.sleep(0.05)
        self.mcu.send_raw_command("P=90")
        time.sleep(0.3)  # 等舵机到 home
        self.mcu.send_raw_command("P=90")  # 再确认一次
        time.sleep(0.3)  # 等舵机稳定

        # 初始化状态机
        self._current_target_pan = 90.0
        self._current_target_tilt = 0.0    # 跟 T=0 一致
        self._init_start_time = time.time()
        self._last_mcu_cmd_time = 0.0

        self._running = True
        self._start_time = time.time()

        logger.info("追踪已启动! 画面中心 = (%d, %d)",
                   self.tracker.center_x, self.tracker.center_y)
        logger.info("初始状态: %s", self._state.name)
        logger.info("按键: c=归中 r=重置PID")
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

        if not self._no_gui:
            cv2.destroyAllWindows()

        logger.info("系统已关闭")
    
    def get_state(self):
        """获取当前系统状态"""
        return self._state
    
    def _change_state(self, new_state, reason=""):
        """统一状态切换函数"""
        logger.info(f"状态: {self._state.name} → {new_state.name}  ({reason})")
        self._state = new_state

        # 进入状态时的初始化
        if new_state == SystemState.TRACK:
            # ★ 进入TRACK时Reset PID, 清除搜索阶段残留的积分项
            self.pid_pan.reset()
            self.pid_tilt.reset()
            # ★ 冻结搜索锚点: 记录发现激光时的位置, TRACK期间不再更新
            #    LOCAL_SEARCH用它做网格中心, 而不是TRACK推飞后的位置
            self._last_known_pan = self._current_target_pan
            self._last_known_tilt = self._current_target_tilt
            logger.info(f"PID Reset (进入TRACK) | 锚点冻结: P={self._last_known_pan:.0f} T={self._last_known_tilt:.0f}")
            self._detect_count = 0
            self._lost_count_state = 0
            self._track_debug_counter = 0  # 进入TRACK立即打印第一条调试日志
            self._track_frame_count = 0    # 软启动 + 逐帧日志计数
        elif new_state == SystemState.LOCAL_SEARCH:
            # ★ 先用绝对位置归位到last_known, 消除增量/绝对坐标系偏差
            logger.info(f"LOCAL_SEARCH归位: P={self._last_known_pan:.0f} T={self._last_known_tilt:.0f}")
            self.mcu.send_raw_command(f"P={int(self._last_known_pan)}")
            time.sleep(0.05)
            self.mcu.send_raw_command(f"T={int(self._last_known_tilt)}")
            time.sleep(0.15)  # 等舵机到位
            self._current_target_pan = self._last_known_pan
            self._current_target_tilt = self._last_known_tilt

            self._local_search.generate_grid(self._last_known_pan, self._last_known_tilt)
            self._search_step = 0
            self._step_start_time = time.time()
            self._frames_at_position = 0
            self._search_detect_confirm = 0  # 当前位置连续检测计数
        elif new_state == SystemState.GLOBAL_SEARCH:
            self._global_search.reset()
            self._search_step = 0
            self._step_start_time = time.time()
            self._frames_at_position = 0
            self._search_detect_confirm = 0  # 当前位置连续检测计数
        elif new_state == SystemState.INIT:
            self._init_start_time = time.time()
            self._init_cooling_off = False
    
    def update(self):
        """状态机主更新函数"""
        if self._state == SystemState.INIT:
            self._update_init()
        elif self._state == SystemState.TRACK:
            self._update_track()
        elif self._state == SystemState.LOCAL_SEARCH:
            self._update_local_search()
        elif self._state == SystemState.GLOBAL_SEARCH:
            self._update_global_search()
    
    def _update_init(self):
        """INIT状态处理 -- 不再重复发送归位命令, 避免覆盖 start() 的 T=0"""
        # 冷却模式等5秒, 首次启动等0.5秒 (舵机已在 start() 归位)
        wait_time = 5.0 if self._init_cooling_off else 0.5
        if time.time() - self._init_start_time >= wait_time:
            self._change_state(SystemState.GLOBAL_SEARCH, "初始化完成, 开始全局扫描")
    
    def _update_track(self):
        """TRACK状态处理 — 增量PID每帧累加 + 80ms发绝对角度"""
        cx, cy, detected = self.tracker.get_position()

        if detected:
            self._lost_count_state = 0
            self._detect_count += 1
            self._track_frame_count += 1

            # ─── 每帧: 读误差 → PID输出增量 → 累加到目标角度 ───
            ex, ey, _ = self.tracker.get_error()
            delta_pan = self.pid_pan.update(ex)
            delta_tilt = self.pid_tilt.update(ey)

            self._current_target_pan += delta_pan * PAN_DIRECTION
            self._current_target_tilt += delta_tilt * TILT_DIRECTION
            self._current_target_pan = max(0, min(180, self._current_target_pan))
            self._current_target_tilt = max(0, min(180, self._current_target_tilt))

            # ─── 每80ms发送一次绝对角度 ───
            now = time.time()
            cmd_sent = False

            if (now - self._last_mcu_cmd_time) >= MCU_CMD_INTERVAL:
                # ★ 连续发送, 不留sleep(STM32有硬件RX buffer, 不会丢)
                self.mcu.send_raw_command(f"P={self._current_target_pan:.1f}")
                self.mcu.send_raw_command(f"T={self._current_target_tilt:.1f}")
                self._last_mcu_cmd_time = now
                cmd_sent = True

            # ─── TRACK调试日志: 每N帧 ───
            if self._track_debug_counter % TRACK_DEBUG_INTERVAL == 0:
                send_tag = "SEND" if cmd_sent else "wait"
                logger.info(
                    f"TRACK f={self._track_frame_count:>4} | "
                    f"cx={cx:>4} cy={cy:>4} | "
                    f"ex={ex:>+5.1f} ey={ey:>+5.1f} | "
                    f"ΔP={delta_pan:>+6.2f} ΔT={delta_tilt:>+6.2f} | "
                    f"P={self._current_target_pan:>6.1f} T={self._current_target_tilt:>6.1f} | "
                    f"{send_tag}"
                )
            self._track_debug_counter += 1

        else:
            self._detect_count = 0
            self._lost_count_state += 1

            # 连续N帧丢失 → LOCAL_SEARCH
            if self._lost_count_state >= LOSS_THRESHOLD:
                self._change_state(SystemState.LOCAL_SEARCH,
                                   f"连续丢失{self._lost_count_state}帧")
    
    def _update_local_search(self):
        """LOCAL_SEARCH状态处理"""
        if self._search_step == 0:  # MOVE
            # 获取下一个九宫格位置
            pan, tilt = self._local_search.get_next_position()
            if pan is None:
                # 九宫格扫描完成，进入GLOBAL_SEARCH
                self._change_state(SystemState.GLOBAL_SEARCH, "九宫格扫描完毕未找到")
                return
            
            # 发送舵机命令 (两条命令间隔足够, 确保MCU逐个处理)
            self.mcu.send_raw_command(f"P={int(pan)}")
            time.sleep(0.05)
            self.mcu.send_raw_command(f"T={int(tilt)}")
            
            self._current_target_pan = pan
            self._current_target_tilt = tilt
            self._search_step = 1
            self._step_start_time = time.time()
            self._frames_at_position = 0
            
        elif self._search_step == 1:  # WAIT
            # 等待200ms
            if time.time() - self._step_start_time >= SERVO_WAIT_TIME:
                self._search_step = 2
                
        elif self._search_step == 2:  # CAPTURE
            # 检测多帧
            _, _, detected = self.tracker.get_position()
            self._frames_at_position += 1
            
            if detected:
                self._search_detect_confirm += 1
            else:
                self._search_detect_confirm = 0

            if self._search_detect_confirm >= DETECT_THRESHOLD:
                self._change_state(SystemState.TRACK, f"九宫格连续检测{self._search_detect_confirm}帧确认")
                return

            if self._frames_at_position >= 5:
                # 进入DECIDE
                self._search_step = 3

        elif self._search_step == 3:  # DECIDE
            # 继续下一位置
            self._search_step = 0

    def _update_global_search(self):
        """GLOBAL_SEARCH状态处理"""
        if self._search_step == 0:  # MOVE
            # 获取下一个蛇形扫描位置
            pan, tilt = self._global_search.get_next_position()
            if pan is None:
                # 全图扫描完成，归中等待，歇5秒再重新搜
                logger.warning("全图扫描完成未找到目标，舵机归中，等5秒后重新搜索...")
                self.mcu.send_raw_command("P=90")
                time.sleep(0.05)
                self.mcu.send_raw_command("T=90")
                self._init_cooling_off = True
                self._change_state(SystemState.INIT, "全局扫描完成未找到, 冷却等待")
                return
            
            # 发送舵机命令 (两条命令间隔足够, 确保MCU逐个处理)
            self.mcu.send_raw_command(f"T={int(tilt)}")
            time.sleep(0.05)
            self.mcu.send_raw_command(f"P={int(pan)}")
            
            self._current_target_pan = pan
            self._current_target_tilt = tilt
            self._search_step = 1
            self._step_start_time = time.time()
            self._frames_at_position = 0
            
        elif self._search_step == 1:  # WAIT
            # 等待200ms
            if time.time() - self._step_start_time >= SERVO_WAIT_TIME:
                self._search_step = 2
                
        elif self._search_step == 2:  # CAPTURE
            # 检测多帧
            _, _, detected = self.tracker.get_position()
            self._frames_at_position += 1
            
            if detected:
                self._search_detect_confirm += 1
            else:
                self._search_detect_confirm = 0

            if self._search_detect_confirm >= DETECT_THRESHOLD:
                self._change_state(SystemState.TRACK, f"全局扫描连续检测{self._search_detect_confirm}帧确认")
                return

            if self._frames_at_position >= 5:
                # 进入DECIDE
                self._search_step = 3

        elif self._search_step == 3:  # DECIDE
            # 继续下一位置
            self._search_step = 0

    def _control_loop(self):
        """主控制循环 (50Hz) - 简化版"""
        while self._running:
            loop_start = time.time()

            if not self.tracker.update():
                time.sleep(0.01)
                continue

            if self.tracker.is_stale(DATA_STALE_TIMEOUT):
                if self._debug_print and self._loop_count % 50 == 0:
                    logger.warning("摄像头数据超时! 跳过更新")
                time.sleep(CONTROL_PERIOD)
                continue

            # 状态机更新
            self.update()

            # 调试输出
            if self._debug_print:
                self._debug_counter += 1
                if self._debug_counter >= 25:
                    self._debug_counter = 0
                    ex, ey, detected = self.tracker.get_error()
                    logger.info(f"状态: {self._state.name} | 检测: {detected}")

            # GUI键盘处理
            if not self._no_gui:
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
        logger.info(f"总循环: {self._loop_count}")
        logger.info(f"平均帧率: {self._loop_count/total_time:.1f}Hz")
        logger.info("=" * 60)




def find_mcu_port():
    """自动扫描可用串口，返回第一个可打开的端口"""
    import os
    # 按优先级排序：ttyUSB (USB转串口) > ttyS0 (GPIO mini UART) > ttyAMA0 (硬件UART)
    candidates = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyS0", "/dev/ttyAMA0"]
    for p in candidates:
        if os.path.exists(p):
            # 测试能否打开
            try:
                import serial
                s = serial.Serial(p, 115200, timeout=0.1)
                s.close()
                return p
            except Exception:
                continue
    return None


def main():
    parser = argparse.ArgumentParser(description="红色激光追踪系统（USB摄像头版本）")
    parser.add_argument("--camera", type=int, default=0,
                        help="摄像头索引（默认0）")
    parser.add_argument("--mcu-port", default="auto",
                        help="MCU串口设备路径 (默认自动扫描)")
    parser.add_argument("--no-debug", action="store_true",
                        help="关闭调试输出")
    parser.add_argument("--no-gui", action="store_true",
                        help="关闭OpenCV窗口显示")
    args = parser.parse_args()

    # 自动扫描串口
    mcu_port = args.mcu_port
    if mcu_port == "auto":
        mcu_port = find_mcu_port()
        if mcu_port is None:
            print("错误: 未找到可用的MCU串口！")
            print("请检查：")
            print("  1. STM32是否上电、串口线是否连接")
            print("  2. 用户是否在dialout组: sudo usermod -a -G dialout pi && newgrp dialout")
            print("  3. 或手动指定: python3 main_controller_usb.py --mcu-port /dev/ttyS0")
            return
        print(f"自动扫描MCU端口: {mcu_port}")
    else:
        print(f"使用指定MCU端口: {mcu_port}")

    controller = TrackingControllerUSB(
        camera_index=args.camera,
        mcu_port=mcu_port,
        no_gui=args.no_gui,
    )
    if args.no_debug:
        controller._debug_print = False
    if args.no_gui:
        controller.tracker.set_debug(False)

    controller.start()


if __name__ == "__main__":
    main()