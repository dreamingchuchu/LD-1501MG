"""
mcu_communicator.py - 树莓派与MCU通信模块
功能: 通过串口发送TRACK命令给MCU，控制舵机运动
"""

import serial
import logging
import time

logger = logging.getLogger(__name__)


class MCUCommunicator:
    """MCU串口通信器"""

    def __init__(self, port="/dev/ttyAMA0", baudrate=115200, timeout=0.1):
        """
        初始化MCU通信器

        Args:
            port: MCU串口设备路径
                  - /dev/ttyAMA0: 树莓派硬件串口(GPIO14/15)
                  - /dev/ttyUSB0: USB转串口
            baudrate: 波特率
            timeout: 写入超时(秒)
        """
        self.port = port
        self.baudrate = baudrate
        self._serial = None

    def connect(self):
        """连接MCU串口"""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
            logger.info(f"MCU串口已连接: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            logger.error(f"无法连接MCU串口 {self.port}: {e}")
            return False

    def disconnect(self):
        """断开MCU串口"""
        if self._serial:
            self._serial.close()
            logger.info("MCU串口已断开")

    def send_track_command(self, delta_pan, delta_tilt):
        """
        发送TRACK命令给MCU

        Args:
            delta_pan: Pan舵机修正量（正=右转）
            delta_tilt: Tilt舵机修正量（正=下转）

        Returns:
            bool: 发送成功返回True
        """
        if not self._serial:
            logger.error("MCU串口未连接")
            return False

        try:
            cmd = f"TRACK,{int(delta_pan)},{int(delta_tilt)}\n"
            self._serial.write(cmd.encode('ascii'))
            logger.debug(f"发送TRACK命令: {cmd.strip()}")
            return True
        except serial.SerialException as e:
            logger.error(f"发送TRACK命令失败: {e}")
            return False

    def send_raw_command(self, command):
        """
        发送原始命令给MCU

        Args:
            command: 命令字符串（如 "P=90", "T=45", "SCAN", "RST"）

        Returns:
            bool: 发送成功返回True
        """
        if not self._serial:
            logger.error("MCU串口未连接")
            return False

        try:
            cmd = f"{command}\n"
            self._serial.write(cmd.encode('ascii'))
            logger.debug(f"发送原始命令: {command}")
            return True
        except serial.SerialException as e:
            logger.error(f"发送命令失败: {e}")
            return False

    def send_pan_angle(self, angle):
        """设置Pan舵机角度"""
        return self.send_raw_command(f"P={int(angle)}")

    def send_tilt_angle(self, angle):
        """设置Tilt舵机角度"""
        return self.send_raw_command(f"T={int(angle)}")

    def send_center(self):
        """发送归中命令"""
        return self.send_raw_command("RST")

    def send_scan(self):
        """发送扫描命令"""
        return self.send_raw_command("SCAN")

    def send_stop(self):
        """发送停止命令"""
        return self.send_raw_command("STOP")