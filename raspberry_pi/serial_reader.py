"""
serial_reader.py - 树莓派端串口接收模块
功能: 从OpenMV读取红色激光光斑坐标，线程安全地更新全局状态
"""

import serial
import threading
import time
import logging

logger = logging.getLogger(__name__)


class LaserTracker:
    """红色激光追踪数据接收器 (线程安全)"""

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.1):
        """
        初始化串口接收器

        Args:
            port: 串口设备路径
                  - /dev/ttyACM0: OpenMV通过USB连接
                  - /dev/ttyAMA0: 树莓派硬件串口(GPIO14/15)
            baudrate: 波特率
            timeout: 读取超时(秒)
        """
        self.port = port
        self.baudrate = baudrate

        self._lock = threading.Lock()
        self._cx = -1
        self._cy = -1
        self._detected = False
        self._timestamp = 0.0
        self._running = False

        self._serial = None

        self._thread = None

    @property
    def cx(self):
        with self._lock:
            return self._cx

    @property
    def cy(self):
        with self._lock:
            return self._cy

    @property
    def detected(self):
        with self._lock:
            return self._detected

    @property
    def timestamp(self):
        with self._lock:
            return self._timestamp

    @property
    def is_running(self):
        with self._lock:
            return self._running

    def start(self):
        """打开串口并启动接收线程"""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            logger.info(f"串口已打开: {self.port} @ {self.baudrate}bps")

            self._running = True
            self._thread = threading.Thread(
                target=self._read_loop,
                name="LaserSerialReader",
                daemon=True
            )
            self._thread.start()
            logger.info("串口接收线程已启动")
            return True

        except serial.SerialException as e:
            logger.error(f"无法打开串口 {self.port}: {e}")
            return False

    def stop(self):
        """停止接收线程并关闭串口"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial:
            self._serial.close()
            logger.info("串口已关闭")

    def _read_loop(self):
        """串口读取线程 (后台运行)"""
        buf = ""

        while self._running:
            try:
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    try:
                        text = data.decode("ascii")
                    except UnicodeDecodeError:
                        continue

                    buf += text

                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self._parse_line(line.strip())

            except serial.SerialException as e:
                logger.error(f"串口错误: {e}")
                time.sleep(0.5)
                try:
                    self._serial.close()
                    self._serial.open()
                    logger.info("串口已重连")
                except Exception:
                    pass

            except Exception as e:
                logger.error(f"接收线程错误: {e}")
                time.sleep(0.1)

    def _parse_line(self, line):
        """解析一行数据: 'cx,cy'"""
        if not line:
            return

        if ":" in line or not "," in line:
            return

        parts = line.split(",")
        if len(parts) < 2:
            return

        try:
            cx = int(parts[0].strip())
            cy = int(parts[1].strip())
        except ValueError:
            return

        with self._lock:
            self._cx = cx
            self._cy = cy
            self._detected = (cx >= 0 and cy >= 0)
            self._timestamp = time.time()

    def get_position(self):
        """获取最新坐标 (返回元组)"""
        with self._lock:
            return (self._cx, self._cy)

    def get_error(self, center_x=160, center_y=120):
        """
        获取相对于画面中心的误差

        Args:
            center_x: 画面中心X (QVGA=160)
            center_y: 画面中心Y (QVGA=120)

        Returns:
            (error_x, error_y, detected)
            正误差 = 光斑在中心右侧/下方
            负误差 = 光斑在中心左侧/上方
        """
        with self._lock:
            if self._detected:
                return (self._cx - center_x, self._cy - center_y, True)
            else:
                return (0, 0, False)

    def is_stale(self, max_age=0.5):
        """
        检查数据是否过期

        Args:
            max_age: 最大允许延迟(秒)

        Returns:
            True 表示超过max_age未收到新数据
        """
        with self._lock:
            if self._timestamp == 0:
                return True
            return (time.time() - self._timestamp) > max_age