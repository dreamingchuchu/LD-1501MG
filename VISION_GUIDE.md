# 树莓派4B + OpenMV 红色激光追踪 — 开发指导手册

> **目标**：在现有云台控制项目基础上，增加树莓派4B + OpenMV摄像头，实现红色激光笔光斑的实时检测与画面中心对准。
>
> **写给Agent**：本文档是一份完整的技术规格书，Agent可按章节顺序实现代码。每个章节包含"需求描述 → 技术决策 → 接口定义 → 参考代码"，确保Agent无需额外猜测。

---

## 目录

1. [系统架构总览](#1-系统架构总览)
2. [硬件连接](#2-硬件连接)
3. [OpenMV端 — 红色激光检测](#3-openmv端--红色激光检测)
4. [树莓派与OpenMV通信协议](#4-树莓派与openmv通信协议)
5. [树莓派端 — 串口通信模块](#5-树莓派端--串口通信模块)
6. [树莓派端 — PID控制器](#6-树莓派端--pid控制器)
7. [树莓派端 — 坐标映射与校准](#7-树莓派端--坐标映射与校准)
8. [树莓派端 — 主控制循环](#8-树莓派端--主控制循环)
9. [与现有云台控制的集成](#9-与现有云台控制的集成)
10. [调试与测试](#10-调试与测试)
11. [实施步骤清单](#11-实施步骤清单)

---

## 1. 系统架构总览

### 1.1 数据流

```
┌──────────────────────────────────────────────────────────────┐
│                        物理世界                                │
│  ┌──────────┐         ┌──────────┐                           │
│  │ 红色激光笔 │ ──投影──▶│  投影屏幕  │                           │
│  └──────────┘         └──────────┘                           │
│                            │                                  │
│                            │ 光斑可见                           │
│                            ▼                                  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │            OpenMV Cam H7 Plus                         │    │
│  │  - 320x240 RGB 图像采集                               │    │
│  │  - 红色色块检测 (find_blobs)                          │    │
│  │  - 计算光斑质心 (cx, cy)                              │    │
│  │  - 通过UART发送坐标到树莓派                            │    │
│  └──────────────────────────────────────────────────────┘    │
│                            │                                  │
│                            │ UART (115200bps)                 │
│                            ▼                                  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │            树莓派 4B (Raspberry Pi OS)                │    │
│  │                                                       │    │
│  │  ┌──────────┐  ┌──────────┐  ┌───────────────────┐  │    │
│  │  │ 串口接收  │  │ PID控制器 │  │ 舵机PWM输出       │  │    │
│  │  │ (UART)   │─▶│ (误差计算)│─▶│ (UART→STM32)    │  │    │
│  │  └──────────┘  └──────────┘  └───────────────────┘  │    │
│  │                                                       │    │
│  │  输入: 光斑像素坐标 (cx, cy)                            │    │
│  │  目标: 画面中心 (160, 120)                              │    │
│  │  输出: 舵机角度修正量                                   │    │
│  │  周期: 20ms (50Hz)                                     │    │
│  └──────────────────────────────────────────────────────┘    │
│                            │                                  │
│                            │ PWM (50Hz, 500-2500us)           │
│                            ▼                                  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │  云台舵机 (LD-1501MG x2)                               │    │
│  │  - Pan (水平/X轴)                                      │    │
│  │  - Tilt (垂直/Y轴)                                     │    │
│  └──────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 核心控制回路

```
每个控制周期 (20ms):
  1. 从OpenMV读取光斑坐标 (cx, cy)
  2. 计算误差: err_x = cx - 160, err_y = cy - 120
  3. PID计算 (X/Y轴独立):
     - Pan修正量  = PID_x(err_x)
     - Tilt修正量 = PID_y(err_y)
  4. 更新舵机PWM:
     - pan_pwm  += pan_correction
     - tilt_pwm += tilt_correction
  5. 写入舵机硬件
```

### 1.3 关键设计决策

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 视觉计算位置 | **OpenMV端** | 减轻树莓派负载，OpenMV硬件加速颜色检测 |
| PID计算位置 | **树莓派端** | 便于调参，Python灵活，可复现参考项目的增量式PID |
| 串口协议 | **简单文本行** (CSV + `\n`) | 参考项目2已验证，调试友好 |
| 舵机驱动 | **已有STM32F103 (UART)** | 复用现有MCU输出PWM，树莓派通过串口发送修正量，无需额外硬件 |
| 控制频率 | **50Hz (20ms)** | 与舵机标准频率一致，足够追踪用 |
| 编程语言 | **Python 3** | 树莓派原生支持，OpenMV端用MicroPython |

---

## 2. 硬件连接

### 2.1 物料清单

| 组件 | 型号/规格 | 数量 | 用途 |
|------|----------|------|------|
| 主控 | 树莓派 4B | 1 | 运行PID、接收OpenMV数据、下发舵机修正量 |
| 舵机驱动MCU | STM32F103C8T6 (Blue Pill) | 1 | **已有**，接收树莓派指令，输出PWM驱动舵机 |
| 摄像头 | OpenMV Cam H7 Plus | 1 | 红色激光光斑检测，通过USB发坐标给树莓派 |
| 云台舵机 | LD-1501MG | 2 | Pan (水平) + Tilt (垂直) |
| 激光笔 | 红色激光笔 (650nm) | 1 | 追踪目标 |
| 开发PC | Windows (需USB口) | 1 | 安装OpenMV IDE，标定阈值、烧录脚本。也用CCS/CubeIDE烧录MCU |
| 电源 | 5V/3A (树莓派) + 外部舵机电源(6V/2A) | 各1 | 舵机必须独立供电 |
| 连线 | 杜邦线 (母母) ×6 + USB数据线 ×2 | - | OpenMV用、树莓派供电用 |

### 2.2 系统拓扑

```
┌──────────┐   USB      ┌──────────────┐   UART      ┌──────────────┐   PWM(50Hz)   ┌────────────┐
│  OpenMV  │══════════►│  树莓派 4B    │══════════►│ STM32F103C8T6 │═══════════►│ LD-1501MG │
│  Cam H7  │ 坐标数据   │  PID计算      │  修正指令   │ 舵机PWM输出    │  PA0=Pan    │  PA1=Tilt  │
│          │ /dev/ttyACM0│              │ /dev/ttyAMA0│ (TEST/项目)    │            │            │
└──────────┘           └──────────────┘            └──────────────┘            └────────────┘
     ▲                       ▲
     │                       │
     │  开发时USB连PC          │  STM32的UART1(PA9/PA10)
     │  (OpenMV IDE标定阈值)   │  本身也已连PC用于调试
     │                       │  (115200bps)
```

> **一句话**：树莓派是"大脑"（视觉处理+PID计算），STM32 是"手脚"（PWM输出驱动舵机），OpenMV 是"眼睛"（检测红色光斑坐标）。三者串行工作。

### 2.3 OpenMV 的两种连接模式

OpenMV 在开发阶段和运行阶段连的设备不同：

#### 开发模式：PC ↔ OpenMV（标定阈值、烧录脚本，只需做一次）

```
┌──────────┐    USB 数据线     ┌──────────┐
│  开发PC   │◄══════════════►│  OpenMV  │
│ (Windows) │  供电 + 通信     │  Cam H7  │
└──────────┘                  └──────────┘

操作步骤:
  1. PC 安装 OpenMV IDE (https://openmv.io/pages/download)
  2. OpenMV 用 USB 连 PC
  3. IDE 左下角点 Connect
  4. 打开 red_laser_tracker.py ▶ 运行, 确认画面能看到红色光斑
  5. Tools → Machine Vision → Threshold Editor → 拖动滑块标定红色阈值
  6. 把标定好的阈值填回 red_laser_tracker.py
  7. Tools → Save open script to OpenMV Cam → Yes (上电自动运行)
  8. 拔掉 USB，OpenMV 端开发完成
```

#### 运行模式：树莓派 ↔ OpenMV（实际追踪时）

```
┌──────────────┐    USB 数据线     ┌──────────┐
│  树莓派 4B    │◄══════════════►│  OpenMV  │
│  /dev/ttyACM0 │  供电 + 串口通信  │  Cam H7  │
└──────────────┘                  └──────────┘

无需任何配置，插上后 /dev/ttyACM0 自动出现。
```

### 2.4 接线表（运行模式，全部连线）

```
OpenMV Cam H7 ──USB──▶ 树莓派4B USB口
   (供电 + 数据, 设备路径 /dev/ttyACM0)

树莓派4B GPIO              STM32F103C8T6
─────────────────────────────────────────────
Pin 6  (GND)       ──── GND (必须共地!)
Pin 8  (TX / GPIO14) ──── PA10 (USART1 RX)
Pin 10 (RX / GPIO15) ──── PA9  (USART1 TX)

STM32F103C8T6            舵机
─────────────────────────────────────────────
PA0 (TIM2 CH1)     ──── Pan舵机 信号线 (黄/白)
PA1 (TIM2 CH2)     ──── Tilt舵机 信号线 (黄/白)
GND                ──── 舵机电源 GND (共地)

舵机电源
─────────────────────────────────────────────
外部6V正极          ──── Pan舵机 电源线 (红) + Tilt舵机 电源线 (红)
外部6V负极          ──── 舵机电源 GND + STM32 GND (共地)
```

> **注意**：STM32 的 USART1 (PA9/PA10) 在开发时连着 PC 用于调试下载，运行时改接树莓派。如果不想来回拔插，可以改用 STM32 的 USART2 (PA2/PA3)。

### 2.5 供电注意事项

| 设备 | 供电方式 | 说明 |
|------|---------|------|
| 树莓派4B | USB-C 5V/3A 适配器 | 独立供电 |
| STM32F103 | 树莓派 USB 口 或 独立 5V | STM32 功耗很低，可从树莓派 USB 取电 |
| OpenMV | 树莓派 USB 口 | 供电 + 通信一线搞定 |
| 舵机 ×2 | **外部 6V/2A 独立电源** | 必须独立！堵转电流大，会烧树莓派 USB |

- **必须共地**：树莓派 GND、STM32 GND、舵机电源 GND 三者连在一起，否则串口通信不稳、舵机乱抖。
- **舵机绝不能从树莓派 5V 引脚取电**：LD-1501MG 两个同时堵转可到 1.6A，树莓派 5V 引脚上限约 1A。

---

## 3. OpenMV端 — 红色激光检测

### 3.1 需求规格

| 需求 | 规格 |
|------|------|
| 分辨率 | QVGA (320×240) |
| 帧率 | ≥25fps |
| 输出 | 光斑质心坐标 (cx, cy) |
| 无光斑时 | 输出 (-1, -1) |
| 输出接口 | UART 115200bps, 8N1 |
| 输出格式 | `"cx,cy\n"` (ASCII, 每帧一行) |

### 3.2 红色阈值标定方法

红色激光笔在摄像头中呈现为明亮且饱和度极高的红色区域。在LAB色彩空间中进行检测效果最好。

**标定步骤**（在OpenMV IDE中操作）：

1. 连接OpenMV，打开IDE中的Tools → Machine Vision → Threshold Editor
2. 选择LAB色彩空间
3. 将红色激光对准墙面，在实时画面中用矩形框选取红色光斑
4. 记录阈值范围，典型值如下：

```python
# 红色激光笔典型LAB阈值 (需根据实际环境微调)
# 参考: 2023国二方案使用 (80, 100, -7, 127, -7, 127) - RGB565格式
# 参考: 目标追踪系统使用 (50, 100, 10, 127, -20, 127)

RED_THRESHOLD = [
    (30, 100,    # L: 亮度阈值, 红色激光很亮
     20, 127,    # A: 正值=偏红, 激光笔A值通常 >40
     -10, 50)    # B: 不太重要, 激光偏暖
]
```

### 3.3 OpenMV代码 (`red_laser_tracker.py`)

```python
"""
red_laser_tracker.py — OpenMV Cam H7 红色激光光斑检测
功能: 实时检测红色激光光斑位置，通过UART发送质心坐标给树莓派
协议: 每帧发送 "cx,cy\n"，无目标时发送 "-1,-1\n"
"""

import sensor
import image
import time
import math
from pyb import UART

# ─── 初始化 ────────────────────────────────────────────

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320×240
sensor.skip_frames(time=2000)          # 等待摄像头稳定

# 自动调节：关闭自动白平衡和自动增益以稳定颜色
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 固定曝光和增益（根据环境调节）
sensor.set_auto_exposure(False, exposure_us=5000)
sensor.set_windowing((0, 0, 320, 240))

# 时钟用于帧率统计
clock = time.clock()

# ─── UART 初始化 ────────────────────────────────────────

# UART 1: P1(TX), P0(RX)
# 波特率 115200, 8N1
uart = UART(1, 115200, timeout_char=10)
uart.init(115200, bits=8, parity=None, stop=1)

# ─── 颜色阈值 ───────────────────────────────────────────

# LAB色彩空间阈值: (L_min, L_max, A_min, A_max, B_min, B_max)
# 红色激光笔通常非常亮(L高)，A通道明显偏正(红)，B通道偏暖
#
# ⚠️ 实际值需要现场标定！在OpenMV IDE中用Threshold Editor工具调节。
# 以下为室内白墙场景的参考值：

RED_THRESHOLD_LAB = (40, 100,    # L: 亮度 — 激光点很亮,L应较高
                      30, 127,    # A: 红-绿轴 — 正值=偏红
                     -20,  50)    # B: 黄-蓝轴 — 正值=偏黄

# 备选: RGB565阈值 (简单场景可用)
# RED_THRESHOLD_RGB = (80, 100, -7, 127, -7, 127)

# 最小色块面积 (过滤噪点)
MIN_BLOB_AREA = 5

# 最大色块面积 (如果激光点充满画面则异常)
MAX_BLOB_AREA = 2000

# ─── 主循环 ────────────────────────────────────────────

# 画面中心坐标
CENTER_X = 160
CENTER_Y = 120

while True:
    clock.tick()

    img = sensor.snapshot()

    # LAB空间红色色块检测
    blobs = img.find_blobs(
        [RED_THRESHOLD_LAB],
        pixels_threshold=MIN_BLOB_AREA,
        area_threshold=MIN_BLOB_AREA,
        merge=True,             # 合并相邻色块
        margin=5
    )

    if blobs:
        # 找到色块 → 取面积最大的(排除过大的异常区域)
        best_blob = None
        max_area = 0

        for blob in blobs:
            area = blob.area()
            if MIN_BLOB_AREA <= area <= MAX_BLOB_AREA and area > max_area:
                max_area = area
                best_blob = blob

        if best_blob:
            cx = best_blob.cx()
            cy = best_blob.cy()

            # 在图像上绘制 (调试用)
            img.draw_cross(cx, cy, size=10, color=(0, 255, 0))
            img.draw_rectangle(best_blob.rect(), color=(0, 255, 0))
            img.draw_circle(cx, cy, 5, color=(0, 255, 0))
        else:
            cx, cy = -1, -1
    else:
        cx, cy = -1, -1

    # ─── 绘制中心十字线 (调试用) ──────────────────────────
    img.draw_cross(CENTER_X, CENTER_Y, size=15, color=(255, 255, 255))

    # ─── 绘制误差向量 (调试用) ────────────────────────────
    if cx > 0 and cy > 0:
        img.draw_line(CENTER_X, CENTER_Y, cx, cy, color=(255, 0, 0))

    # ─── 串口发送 ────────────────────────────────────────
    uart.write(f"{cx},{cy}\n")

    # 帧率控制: 如果要限制帧率
    # time.sleep_ms(20)  # 50Hz

    # 打印帧率到LCD (OpenMV屏幕)
    fps = clock.fps()
    # print(f"FPS: {fps:.1f}")  # 如果连接了IDE可以取消注释
```

### 3.4 阈值标定方法

#### 方法一（推荐）：OpenMV IDE 内置 Threshold Editor

在 OpenMV IDE 菜单栏中：

```
Tools → Machine Vision → Threshold Editor → Frame Buffer
```

这会打开**图形化调参窗口**：
- 左侧：摄像头实时画面
- 右侧：**L/A/B 六个滑块**，拖动时色块检测结果实时更新
- 找到能稳定框住红色激光点的阈值后，把 6 个数字记下来，填入 `red_laser_tracker.py` 的 `RED_THRESHOLD_LAB` 变量

#### 方法二：简易辅助脚本 (`threshold_tuner.py`)

如果没有 IDE 完整环境，可用以下脚本作为备选。它会把当前阈值和检测结果显示在画面上（**没有滑块**，需要手动改 `thresholds` 数组后重新运行）：

```python
"""
threshold_tuner.py — 简易阈值辅助脚本
使用方法: 手动修改thresholds数组的值, 重新运行脚本, 观察画面上的检测效果
"""

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 初始阈值 (从参考值开始)
thresholds = [40, 100, 30, 127, -20, 50]  # L_Min, L_Max, A_Min, A_Max, B_Min, B_Max

while True:
    img = sensor.snapshot()

    # 应用当前阈值
    blobs = img.find_blobs([tuple(thresholds)], pixels_threshold=5, merge=True)

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        img.draw_string(blob.cx() + 5, blob.cy(), f"area={blob.area()}", color=(255,255,255), scale=1.2)

    # 画面信息
    img.draw_string(0, 0, f"FPS: {clock.fps():.1f}", color=(255,255,255))
    img.draw_string(0, 15, f"THR: L({thresholds[0]},{thresholds[1]}) "
                           f"A({thresholds[2]},{thresholds[3]}) "
                           f"B({thresholds[4]},{thresholds[5]})", color=(255,255,255))
    img.draw_string(0, 30, f"Blobs found: {len(blobs)}", color=(255,255,255))

    # 在OpenMV IDE的终端中可以手动修改thresholds数组
    # 找到最佳值后记录并填入 red_laser_tracker.py
```

---

## 4. 树莓派与OpenMV通信协议

### 4.1 物理层

| 参数 | 值 |
|------|-----|
| 接口 | UART (ttyAMA0 或 ttyACM0) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验位 | 无 |
| 停止位 | 1 |
| 流控 | 无 |

### 4.2 OpenMV → 树莓派 (数据帧)

```
格式:  cx,cy\n
示例:  145,98\n

cx: 光斑质心X坐标, 整数, 范围[-1, 319]
cy: 光斑质心Y坐标, 整数, 范围[-1, 239]
    当cx或cy为 -1 时表示未检测到光斑

特殊值:
  -1,-1\n  → 视野中无红色光斑
```

### 4.3 树莓派 → OpenMV (命令帧) [可选]

如需双向通信（例如发送校准命令或修改阈值）：

```
格式:  CMD:ARG\n
示例:
  CALIBRATE\n       → 进入校准模式
  THR:40,100,30,127,-20,50\n → 更新色块检测阈值
  LED:ON\n / LED:OFF\n       → 控制OpenMV板载LED
```

**第一阶段可先不做双向通信**，只用单向数据（OpenMV → 树莓派）。

---

## 5. 树莓派端 — 串口通信模块

### 5.1 需求规格

| 需求 | 规格 |
|------|------|
| 接收线程 | 独立线程，非阻塞读取 |
| 数据解析 | 分割逗号，转为int |
| 异常处理 | 数据不完整时丢弃当前帧 |
| 接口 | 提供 `latest_x`, `latest_y`, `target_detected` 全局状态 |

### 5.2 代码 (`serial_reader.py`)

```python
"""
serial_reader.py — 树莓派端串口接收模块
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

        # ─── 线程安全的状态变量 ───
        self._lock = threading.Lock()
        self._cx = -1          # 最新光斑X坐标
        self._cy = -1          # 最新光斑Y坐标
        self._detected = False # 是否检测到光斑
        self._timestamp = 0.0  # 上次更新时间
        self._running = False  # 接收线程运行标志

        # ─── 串口对象 ───
        self._serial = None

        # ─── 接收线程 ───
        self._thread = None

    # ─── 属性访问 (线程安全) ──────────────────────────

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

    # ─── 启动/停止 ────────────────────────────────────

    def start(self):
        """打开串口并启动接收线程"""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # 100ms超时
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

    # ─── 接收线程主循环 ────────────────────────────────

    def _read_loop(self):
        """串口读取线程 (后台运行)"""
        buf = ""

        while self._running:
            try:
                if self._serial.in_waiting > 0:
                    # 读取可用字节
                    data = self._serial.read(self._serial.in_waiting)
                    try:
                        text = data.decode("ascii")
                    except UnicodeDecodeError:
                        # 丢弃非ASCII数据
                        continue

                    buf += text

                    # 按行分割
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self._parse_line(line.strip())

            except serial.SerialException as e:
                logger.error(f"串口错误: {e}")
                time.sleep(0.5)
                # 尝试重新打开串口
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

        # 忽略非数据行 (命令响应等)
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

        # 更新全局状态 (线程安全)
        with self._lock:
            self._cx = cx
            self._cy = cy
            self._detected = (cx >= 0 and cy >= 0)
            self._timestamp = time.time()

    # ─── 辅助方法 ─────────────────────────────────────

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
```

---

## 6. 树莓派端 — PID控制器

### 6.1 需求规格

| 需求 | 规格 |
|------|------|
| PID类型 | 增量式 (velocity form) |
| 独立轴控制 | X/Y轴各一个PID实例 |
| 输出限幅 | [-30, 30] (PWM步长/周期) |
| 积分限幅 | [-50, 50] |
| 死区 | ±3像素 (激光点小抖动时不响应) |

### 6.2 为什么用增量式PID

参考两个国赛方案的经验：

1. **增量式PID** 天然防积分饱和，适合舵机这种执行器位置依赖历史状态的场景。
2. 参考项目1(国二)使用**增量式+微分先行+积分分离**；参考项目2使用**增量式PID**。
3. 增量式的输出是"修正量"而非"绝对位置"，直接加到当前PWM值上，代码更简洁。

### 6.3 代码 (`pid_controller.py`)

```python
"""
pid_controller.py — 增量式PID控制器
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

        # 误差历史
        self._e_k = 0.0      # e(k): 当前误差
        self._e_k_1 = 0.0    # e(k-1): 上次误差
        self._e_k_2 = 0.0    # e(k-2): 上上次误差

        # 积分项累加
        self._integral = 0.0

        # 输出历史
        self._last_output = 0.0

        # 统计
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

        # ─── 死区处理 ──────────────────────────────────
        if abs(error) <= self.cfg.deadband:
            # 误差在死区内，不输出修正
            self._e_k_2 = self._e_k_1
            self._e_k_1 = self._e_k
            self._last_output = 0.0
            return 0.0

        # ─── 积分分离 ──────────────────────────────────
        if abs(error) > self.cfg.integral_separation_threshold:
            # 大误差 → 只用PD, 防止积分饱和
            ki_effective = 0.0
        else:
            ki_effective = self.cfg.ki
            # 积分累加
            self._integral += error
            # 积分限幅
            self._integral = max(self.cfg.integral_min,
                                 min(self.cfg.integral_max, self._integral))

        # ─── 增量式PID公式 ──────────────────────────────
        # delta = Kp*(ek - ek_1) + Ki*ek + Kd*(ek - 2*ek_1 + ek_2)

        p_term = self.cfg.kp * (self._e_k - self._e_k_1)
        i_term = ki_effective * self._e_k
        d_term = self.cfg.kd * (self._e_k - 2 * self._e_k_1 + self._e_k_2)

        delta = p_term + i_term + d_term

        # ─── 输出限幅 ──────────────────────────────────
        delta = max(self.cfg.output_min,
                    min(self.cfg.output_max, delta))

        # ─── 更新历史 ──────────────────────────────────
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


# ─── 预设参数 (基于参考项目经验) ────────────────────────

# Pan轴(X轴) 参数 — 初始保守值，需根据实际调参
# 参考: 国二方案X轴 Kp=0, Ki=-0.0018, Kd=-0.005
#       追踪方案 Pan: Kp=0.6*比例项, Ki=-0.4, Kd=0.1
PAN_PID_CONFIG = PIDConfig(
    kp=0.15,     # 比例 (根据像素误差→PWM映射调参)
    ki=0.003,    # 积分 (小值, 消除稳态误差)
    kd=0.08,     # 微分 (抑制振荡)
    output_max=25.0,
    output_min=-25.0,
    deadband=3.0,
    integral_separation_threshold=40.0
)

# Tilt轴(Y轴) 参数
# 注意: Tilt轴由于重力影响，正反参数可能不对称
TILT_PID_CONFIG = PIDConfig(
    kp=0.15,
    ki=0.003,
    kd=0.06,
    output_max=25.0,
    output_min=-25.0,
    deadband=3.0,
    integral_separation_threshold=40.0
)
```

### 6.4 PID调参指南

```
调参步骤 (按顺序):

1. 死区设置: 让系统不输出修正, 观察光斑自然抖动范围, 设为该值的1.5倍
   例如: 自然抖动±2px → deadband=3

2. Kp粗调 (Ki=0, Kd=0):
   - 从小值开始 (0.05)
   - 每次加倍直到出现超调
   - 取超调前值的60%

3. Kd微调:
   - 从小值开始 (0.01)
   - 出现高频抖动 → Kd过大
   - 找到消除超调又不引入抖动的值

4. Ki最后:
   - 从极小值开始 (0.001)
   - 看稳态是否能收敛到0
   - 出现低频摆动 → Ki过大

5. 在线调参命令 (通过SSH或串口):
   发送 "PID:X,Kp=0.2"  修改X轴Kp
   发送 "PID:Y,Ki=0.005" 修改Y轴Ki
```

---

## 7. 树莓派端 — 坐标映射与校准

### 7.1 坐标映射原理

```
摄像头像素坐标 (cx, cy) → 舵机PWM角度空间

需要建立两个映射:
  1. 像素增量 → PWM增量 (用于PID)
  2. 绝对像素坐标 → 绝对PWM值 (用于归位)

简化方案 (参考国二方案):
  - 像素→PWM映射 = 简单线性缩放
  - scale = (PWM_max - PWM_min) / (pixel_max - pixel_min)
  - 校正量(PWM) = 像素误差 * scale

详细校准方案 (参考追踪系统方案):
  - 在屏幕四角做标记
  - 记录每个角的(像素, PWM)对
  - 线性插值映射任意像素到PWM
```

### 7.2 简化版实现（推荐第一阶段使用）

由于我们只需"画面中心对准激光点"而非"将激光移动到某个特定像素坐标"，可以跳过复杂的全画面校准。只需要知道**1像素误差对应多少PWM修正**即可。

```python
"""
calibration.py — 像素→舵机PWM映射校准
简化版: 只需估算 scale = PWM_per_degree / pixels_per_degree
"""

import json
import os
import logging

logger = logging.getLogger(__name__)

# 默认配置文件路径
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")


class Calibration:
    """
    像素→PWM 线性映射参数

    pwm_delta = pixel_error * scale_x
    正像素误差 → 正PWM修正 → 舵机向右/下转
    """

    def __init__(self):
        # ─── 默认参数 (需实测校准) ────────────────────
        self.scale_x = 0.05          # X轴: 1px误差 → N步PWM修正
        self.scale_y = 0.05          # Y轴: 1px误差 → N步PWM修正

        # 画面中心像素坐标
        self.center_x = 160          # QVGA宽度/2
        self.center_y = 120          # QVGA高度/2

        # 舵机中位PWM值
        self.servo_center_pan = 1500   # Pan中位(us)
        self.servo_center_tilt = 1500  # Tilt中位(us)

        # 舵机PWM范围
        self.pwm_min = 500
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
            self.scale_y = abs(pwm_change / pixel_change)  # 初始假设XY相同
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
```

---

## 8. 树莓派端 — STM32串口通信模块

### 8.1 需求规格

| 需求 | 规格 |
|------|------|
| 芯片 | STM32F103C8T6 (已有，TEST/项目) |
| 通信接口 | UART (树莓派 GPIO14/15 ↔ STM32 PA9/PA10) |
| 波特率 | 115200 bps, 8N1 |
| 协议 | 文本行, 以 `\n` 结尾 |
| 指令格式 | `TRACK,delta_pan,delta_tilt\n` |
| 舵机PWM | 由STM32端 TIM2 CH1(PA0)/CH2(PA1) 输出, 50Hz |

### 8.2 为什么复用STM32而不是树莓派直驱

| 方案 | 问题 |
|------|------|
| 树莓派GPIO软件PWM | Linux非实时系统，电平切换可能被中断，舵机抖动、异响 |
| 树莓派GPIO硬件PWM | 只有2路，且被音频驱动占用，需要禁用音频 |
| PCA9685 I2C模块 | **你没有这个模块** |
| **复用现有STM32 (采用)** | 已有硬件和代码，PWM由TIM硬件生成，波形干净无抖动 |

### 8.3 STM32端需要新增的代码

在 `TEST/Core/Src/Serial.c` 的 `ParseCommand()` 函数中增加一条指令解析：

```c
// 在 ParseCommand() 函数的命令匹配区添加:

// 树莓派追踪修正指令: TRACK,delta_pan,delta_tilt
if (strncmp(cmd, "TRACK,", 6) == 0) {
    int delta_pan = 0, delta_tilt = 0;
    if (sscanf(cmd, "TRACK,%d,%d", &delta_pan, &delta_tilt) >= 2) {
        Servo_AdjustPan(delta_pan);
        Servo_AdjustTilt(delta_tilt);
    }
}
```

对应在 `Servo.c` 中新增两个函数：

```c
// Servo_AdjustPan: Pan舵机在当前角度上叠加修正量
// delta > 0 → 右转, delta < 0 → 左转
void Servo_AdjustPan(int16_t delta) {
    uint16_t new_pwm = Servo_GetPanPWM() + delta;
    // 限幅保护
    if (new_pwm < 600)  new_pwm = 600;
    if (new_pwm > 2400) new_pwm = 2400;
    Servo_SetPanPWM(new_pwm);
}

// Servo_AdjustTilt: Tilt舵机在当前角度上叠加修正量
// delta > 0 → 下转, delta < 0 → 上转
void Servo_AdjustTilt(int16_t delta) {
    uint16_t new_pwm = Servo_GetTiltPWM() + delta;
    if (new_pwm < 600)  new_pwm = 600;
    if (new_pwm > 2400) new_pwm = 2400;
    Servo_SetTiltPWM(new_pwm);
}
```

> 完整参考：TEST/ 项目现有的 `Servo_SetAngle()` 和 `TIM2->CCR1/CCR2` 操作。

### 8.4 树莓派端代码 (`stm32_comm.py`)

```python
"""
stm32_comm.py — 树莓派→STM32 串口通信模块
功能: 通过UART发送舵机修正指令给STM32
协议: "TRACK,delta_pan,delta_tilt\n"
"""

import serial
import threading
import time
import logging

logger = logging.getLogger(__name__)


class STM32Servo:
    """通过串口控制STM32驱动的两轴舵机"""

    # 舵机安全限位 (PWM脉冲宽度, us)
    PAN_MIN = 600
    PAN_MAX = 2400
    TILT_MIN = 600
    TILT_MAX = 2400
    CENTER = 1500

    def __init__(self, port="/dev/ttyAMA0", baudrate=115200, timeout=0.1):
        """
        Args:
            port: STM32串口设备路径
                  - /dev/ttyAMA0: 树莓派硬件串口 (GPIO14/15)
                  - /dev/ttyUSB0: USB转串口 (如果用USB-TTL)
            baudrate: 波特率, 需与STM32端一致
        """
        self.port = port
        self.baudrate = baudrate

        # 跟踪当前舵机位置 (软件镜像, 不精确但够用)
        self._lock = threading.Lock()
        self._pan_pwm = self.CENTER
        self._tilt_pwm = self.CENTER

        self._serial = None

    # ─── 启动/停止 ────────────────────────────────────

    def start(self):
        """打开串口连接STM32"""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            logger.info(f"STM32串口已打开: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            logger.error(f"无法打开STM32串口 {self.port}: {e}")
            return False

    def stop(self):
        """关闭串口"""
        if self._serial:
            self._serial.close()
            logger.info("STM32串口已关闭")

    # ─── 舵机控制 ────────────────────────────────────

    def adjust_pan(self, delta: int):
        """Pan舵机相对移动 (正=右, 负=左)"""
        new_pwm = self._pan_pwm + delta
        new_pwm = max(self.PAN_MIN, min(self.PAN_MAX, new_pwm))
        self._pan_pwm = new_pwm
        self._send_cmd(f"TRACK,{delta},0")

    def adjust_tilt(self, delta: int):
        """Tilt舵机相对移动 (正=下, 负=上)"""
        new_pwm = self._tilt_pwm + delta
        new_pwm = max(self.TILT_MIN, min(self.TILT_MAX, new_pwm))
        self._tilt_pwm = new_pwm
        self._send_cmd(f"TRACK,0,{delta}")

    def adjust_both(self, delta_pan: int, delta_tilt: int):
        """两轴同时移动"""
        new_pan = max(self.PAN_MIN, min(self.PAN_MAX, self._pan_pwm + delta_pan))
        new_tilt = max(self.TILT_MIN, min(self.TILT_MAX, self._tilt_pwm + delta_tilt))
        self._pan_pwm = new_pan
        self._tilt_pwm = new_tilt
        self._send_cmd(f"TRACK,{delta_pan},{delta_tilt}")

    def center(self):
        """归中"""
        self._pan_pwm = self.CENTER
        self._tilt_pwm = self.CENTER
        self._send_cmd("P=90")
        self._send_cmd("T=90")

    # ─── 内部方法 ─────────────────────────────────────

    def _send_cmd(self, cmd: str):
        """发送一行指令 (自动加换行)"""
        if self._serial and self._serial.is_open:
            try:
                line = cmd + "\n"
                self._serial.write(line.encode("ascii"))
            except serial.SerialException as e:
                logger.error(f"串口发送失败: {e}")

    # ─── 属性 ────────────────────────────────────────

    @property
    def pan_pwm(self):
        with self._lock:
            return self._pan_pwm

    @property
    def tilt_pwm(self):
        with self._lock:
            return self._tilt_pwm
```

---

## 9. 树莓派端 — 主控制循环

### 9.1 需求规格

| 需求 | 规格 |
|------|------|
| 主循环频率 | 50Hz (20ms周期) |
| 错误处理 | 数据超时时停止PID输出 |
| 安全机制 | 急停按钮 (GPIO输入或键盘中断) |
| 日志 | 关键状态打印到控制台/文件 |
| PID在线调参 | 支持运行时修改参数 |

### 9.2 代码 (`main_controller.py`)

```python
#!/usr/bin/env python3
"""
main_controller.py — 红色激光追踪主控制程序
功能: 整合OpenMV通信 + PID控制 + 舵机驱动, 实现闭环追踪

运行方式:
  python3 main_controller.py

运行时按键:
  q / Ctrl+C  → 退出
  c           → 归中
  r           → 重置PID
  p           → 切换调试打印
  + / -       → 在线调节Kp
  [ / ]       → 在线调节Ki
  ; / '       → 在线调节Kd
"""

import time
import signal
import sys
import logging
import threading

from serial_reader import LaserTracker
from pid_controller import IncrementalPID, PIDConfig, PAN_PID_CONFIG, TILT_PID_CONFIG
from calibration import Calibration
from stm32_comm import STM32Servo

# ─── 日志配置 ──────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        # logging.FileHandler("tracker.log")  # 可选: 同时写入文件
    ]
)
logger = logging.getLogger("MainController")

# ─── 画面参数 ──────────────────────────────────────────

FRAME_WIDTH = 320      # QVGA
FRAME_HEIGHT = 240
CENTER_X = FRAME_WIDTH // 2    # 160
CENTER_Y = FRAME_HEIGHT // 2   # 120

# ─── 控制参数 ──────────────────────────────────────────

CONTROL_PERIOD = 0.020   # 20ms = 50Hz
DATA_STALE_TIMEOUT = 0.5  # 数据超过0.5秒视为过期


class TrackingController:
    """红色激光追踪主控制器"""

    def __init__(self, serial_port="/dev/ttyACM0"):
        # ─── 模块初始化 ────────────────────────────────
        logger.info("=" * 60)
        logger.info("红色激光追踪系统 v1.0 启动中...")
        logger.info("=" * 60)

        # 串口接收 (OpenMV)
        self.tracker = LaserTracker(port=serial_port)

        # PID控制器 (X=Pan, Y=Tilt)
        self.pid_pan = IncrementalPID(PAN_PID_CONFIG, name="Pan-PID")
        self.pid_tilt = IncrementalPID(TILT_PID_CONFIG, name="Tilt-PID")

        # 校准
        self.calib = Calibration()
        self.calib.load()

        # 舵机驱动 (通过UART→STM32)
        self.servo = STM32Servo(port="/dev/ttyAMA0", baudrate=115200)
        self.servo.start()
        self.servo.center()
        time.sleep(0.5)

        # ─── 运行状态 ───────────────────────────────────
        self._running = False
        self._debug_print = True
        self._debug_counter = 0

        # 统计
        self._loop_count = 0
        self._tracking_count = 0
        self._lost_count = 0
        self._start_time = None

    def start(self):
        """启动追踪"""
        # 打开串口
        if not self.tracker.start():
            logger.error("无法打开串口, 退出")
            return False

        time.sleep(1.0)  # 等待OpenMV数据稳定

        self._running = True
        self._start_time = time.time()

        logger.info("追踪已启动! 画面中心 = (%d, %d)", CENTER_X, CENTER_Y)
        logger.info("按键: q=退出 c=归中 r=重置PID p=调试打印")

        # ─── 主循环 ────────────────────────────────────
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
        self.servo.center()
        logger.info("系统已关闭")

    def _control_loop(self):
        """主控制循环 (50Hz)"""
        last_time = time.time()

        while self._running:
            loop_start = time.time()

            # ─── 步骤1: 读取OpenMV数据 ──────────────────
            ex, ey, detected = self.tracker.get_error(CENTER_X, CENTER_Y)

            # ─── 步骤2: 数据有效性检查 ──────────────────
            if self.tracker.is_stale(DATA_STALE_TIMEOUT):
                # 数据过期, 跳过PID更新 (防止传感器断连时乱动)
                if self._debug_print and self._loop_count % 50 == 0:
                    logger.warning("OpenMV数据超时! 跳过PID更新")
                time.sleep(CONTROL_PERIOD)
                continue

            if not detected:
                # 无光斑, 保持当前位置
                self._lost_count += 1
                # 如果长时间丢失, 可以进入搜索模式 (不在此实现)
                pass

            # ─── 步骤3: PID计算 ─────────────────────────
            if detected:
                # 像素误差 → 直接给PID (增量式PID内部处理缩放)
                delta_pan = self.pid_pan.update(ex)
                delta_tilt = self.pid_tilt.update(ey)

                # 或者: 像素误差 → PWM修正 → PID
                # delta_pwm_x, delta_pwm_y = self.calib.pixel_to_pwm_delta(ex, ey)
                # delta_pan = self.pid_pan.update(delta_pwm_x)
                # delta_tilt = self.pid_tilt.update(delta_pwm_y)

                # ─── 步骤4: 更新舵机 ─────────────────────
                if abs(delta_pan) > 0.01:  # 有意义的修正量
                    self.servo.adjust_pan(int(round(delta_pan)))
                if abs(delta_tilt) > 0.01:
                    self.servo.adjust_tilt(int(round(delta_tilt)))

                self._tracking_count += 1
            else:
                # 丢失目标 → 保持当前位置 (或进入搜索模式)
                self.pid_pan.reset()  # 清空积分项
                self.pid_tilt.reset()
                self._lost_count += 1

            # ─── 步骤5: 调试输出 ─────────────────────────
            if self._debug_print:
                self._debug_counter += 1
                if self._debug_counter >= 25:  # 每0.5秒打印一次
                    self._debug_counter = 0
                    self._print_status(ex, ey, detected, delta_pan, delta_tilt)

            # ─── 步骤6: 键盘输入 ─────────────────────────
            self._check_keyboard()

            # ─── 步骤7: 频率控制 ─────────────────────────
            self._loop_count += 1
            elapsed = time.time() - loop_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.005:  # 超过5ms延迟
                if self._loop_count % 250 == 0:
                    logger.warning(f"控制循环超时: {-sleep_time*1000:.1f}ms")

        # ─── 循环结束, 打印统计 ─────────────────────────
        total_time = time.time() - self._start_time
        logger.info("=" * 60)
        logger.info(f"追踪结束. 总时间: {total_time:.1f}s")
        logger.info(f"总循环: {self._loop_count}, "
                    f"追踪: {self._tracking_count}, "
                    f"丢失: {self._lost_count}")
        logger.info(f"平均帧率: {self._loop_count/total_time:.1f}Hz")
        logger.info("=" * 60)

    def _print_status(self, ex, ey, detected, delta_pan, delta_tilt):
        """打印当前状态"""
        pan_pwm = self.servo.pan_pwm
        tilt_pwm = self.servo.tilt_pwm

        status = "🔴追踪中" if detected else "⚫目标丢失"
        logger.info(
            f"{status} | "
            f"误差=({ex:+4d},{ey:+4d}) | "
            f"ΔPWM=({delta_pan:+6.1f},{delta_tilt:+6.1f}) | "
            f"PWM=({pan_pwm:4d},{tilt_pwm:4d})"
        )

    def _check_keyboard(self):
        """检查键盘输入 (非阻塞)"""
        # 注意: 非阻塞键盘输入在Linux上较复杂
        # 推荐使用 select.select 或 curses
        # 这里提供接口, 实际实现根据需求选择
        #
        # 替代方案: 使用GPIO按钮作为急停/归中按钮
        pass


# ─── 可选: 非阻塞键盘输入 ─────────────────────────────

class KeyboardInput(threading.Thread):
    """键盘输入线程 (非阻塞)"""
    def __init__(self, controller: TrackingController):
        super().__init__(daemon=True, name="KeyboardInput")
        self.ctrl = controller

    def run(self):
        """可选: 使用 pynput / getch 实现非阻塞按键"""
        pass


# ─── 入口 ─────────────────────────────────────────────

def main():
    import argparse

    parser = argparse.ArgumentParser(description="红色激光追踪系统")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="OpenMV串口设备路径")
    parser.add_argument("--no-debug", action="store_true",
                        help="关闭调试输出")
    args = parser.parse_args()

    controller = TrackingController(serial_port=args.port)
    if args.no_debug:
        controller._debug_print = False

    controller.start()


if __name__ == "__main__":
    main()
```

---

## 10. 与现有云台控制的集成

### 10.1 现有项目能复用的部分

| 现有代码 | 位置 | 是否复用 | 说明 |
|----------|------|---------|------|
| 串口命令解析 (P=n, T=n) | `empty/src/Serial.c`, `TEST/Core/Src/Serial.c` | **部分复用** | Serial协议可用于调试命令通道 |
| 舵机PWM计算 (角度→脉冲) | `empty/src/Servo.c` | **数学逻辑复用** | PWM=500+angle*2000/180 公式在Python端重写 |
| Motion模块 (线性插值) | `TEST/Core/Src/Motion.c` | **逻辑复用** | 用于搜索模式和校准扫描，在Python端重写 |
| SquareDraw (状态机) | `TEST/Core/Src/SquareDraw.c` | **暂时不用** | 原有方形绘制功能保留在MCU端 |
| PID实现 (增量式) | `TEST/2023-电赛E题-目标追踪系统/GreenServo/STM32/User/YunTai/yuntai.c` | **算法复用** | 参考其增量式PID公式和参数取值范围 |
| 专家PID | `TEST/2023-电赛E题-国二-全套代码/Application/Src/PID.c` | **高级优化时可参考** | 变速积分、微分先行等特性 |

### 10.2 通信架构

```
            ┌─────────────────────────────┐
            │        树莓派 4B              │
            │                              │
            │  ┌────────────────────────┐  │
            │  │  追踪主程序              │  │
            │  │  main_controller.py    │  │
            │  └───────┬────────────────┘  │
            │          │                    │
            │    ┌─────┴─────┐             │
            │    │ 串口0      │ 串口1(USB)  │
            │    │ /dev/ttyAMA0│/dev/ttyACM0│
            │    └─────┬─────┘     │        │
            │          │           │         │
            └──────────┼───────────┼─────────┘
                       │           │
                       │           │
         ┌─────────────┴─┐   ┌────┴──────────┐
         │  现有MCU       │   │   OpenMV       │
         │  (MSPM0/STM32) │   │   Cam H7       │
         │                │   │                 │
         │ 接收高级命令:    │   │ 发送光斑坐标:    │
         │  SCAN, RST等   │   │ "cx,cy\n"      │
         └──────┬─────────┘   └────────────────┘
                │
          ┌─────┴─────┐
          │  舵机云台   │
          └───────────┘
```

### 10.3 集成策略

**方案：树莓派+STM32 串联（采用）**
1. STM32 继续做舵机 PWM 输出（TIM2 CH1/CH2 → PA0/PA1），响应串口指令
2. 树莓派通过 UART 发送 `TRACK,delta_pan,delta_tilt\n` 修正指令
3. 树莓派做视觉（读 OpenMV）+ PID 计算，STM32 只管执行
4. 原有 `P=n,T=n,SCAN,SQUARE` 等指令仍可从 PC 串口助手发出给 STM32 做独立控制

### 10.4 MCU侧需要添加的代码（方案B）

如果MCU端使用 [TEST/](TEST/) 下的STM32代码，需要在串口命令解析中增加：

```c
// 在 Serial.c 的 ParseCommand() 中添加:

// 接收树莓派的追踪修正量
if (strncmp(cmd, "TRACK,", 6) == 0) {
    int pan_correction, tilt_correction;
    if (sscanf(cmd, "TRACK,%d,%d", &pan_correction, &tilt_correction) == 2) {
        // 在当前舵机角度上叠加修正量
        Servo_AdjustPan(pan_correction);
        Servo_AdjustTilt(tilt_correction);
    }
}

// 树莓派 → MCU 的追踪控制
// 格式: "TRACK,delta_pan,delta_tilt\n"
// 示例: "TRACK,5,-3\n" → Pan舵机右转5步, Tilt舵机上转3步
```

---

## 11. 调试与测试

### 11.1 分步验证清单

```
□ 步骤1: OpenMV独立测试
  - 在OpenMV IDE中运行 red_laser_tracker.py
  - 确认画面中能看到红色光斑上的绿色十字
  - 确认串口终端输出 "cx,cy\n" 格式数据

□ 步骤2: 树莓派串口接收测试
  - 运行一个简单的Python脚本, 读取/dev/ttyACM0
  - 确认能收到OpenMV发送的坐标

□ 步骤3: STM32串口通信测试
  - 用USB-TTL连接PC→STM32, 串口助手发送 `TRACK,10,0\n`, 确认Pan舵机响应
  - 发送 `P=90\n` 确认归中正常

□ 步骤4: PID离线调参
  - 用模拟数据(固定误差序列)测试PID输出是否合理
  - 调参脚本: test_pid.py

□ 步骤5: 系统联调 (开环)
  - 先只读取OpenMV数据, 打印误差, 不控制舵机
  - 手动移动激光笔, 观察误差变化

□ 步骤6: 系统联调 (闭环)
  - PID输出接到舵机
  - 观察激光点是否能收敛到画面中心
  - 可能需要调节PWM→像素的scale

□ 步骤7: 鲁棒性测试
  - 遮挡激光 → 应立即停止舵机移动
  - 快速移动激光 → 应能跟上且不振荡
  - 重新打开串口 → 应能自动重连
```

### 11.2 常见问题排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| OpenMV不发送数据 | 波特率不匹配 | 检查OpenMV和树莓派波特率都是115200 |
| 收不到数据 | 串口设备路径不对 | `ls /dev/tty*` 确认OpenMV设备名 |
| 光斑检测不到 | 颜色阈值不合适 | 用threshold_tuner.py标定红色阈值 |
| 舵机抖动严重 | Kp过大或Kd过小 | 减小Kp到当前值的50%再试 |
| 收敛太慢 | Kp过小 | 增大Kp |
| 振荡(来回摆动) | Kp过大或缺少D | 先加大Kd, 无效则减小Kp |
| 稳态有偏差 | Ki太小 | 增大Ki |
| 积分饱和导致过冲 | 积分分离阈值不合理 | 降低integral_separation_threshold |
| 激光点卡在边缘不动 | 舵机达到物理限位 | 检查PAN_LIMIT/TILT_LIMIT设置 |
| 串口数据乱码 | 地线未连接 | 确保OpenMV和树莓派共地 |

### 11.3 性能基准

| 指标 | 目标值 | 优秀值 |
|------|--------|--------|
| 静态稳态误差 | <5像素 | <2像素 |
| 动态跟踪延迟 | <200ms | <100ms |
| 控制帧率 | ≥40Hz | 50Hz稳定 |
| 抗干扰(遮挡恢复) | <500ms重新锁定 | <200ms |
| CPU使用率(树莓派) | <30% | <15% |

---

## 12. 实施步骤清单

以下是Agent应按顺序执行的实施步骤：

### Phase 1: 环境准备

- [ ] **1.1** 树莓派安装依赖: `pip3 install pyserial`
- [ ] **1.2** 树莓派启用UART: `sudo raspi-config` → Interface Options → Serial (enable, disable login shell)
- [ ] **1.3** 在OpenMV IDE中将 `red_laser_tracker.py` 刷入OpenMV Cam H7
- [ ] **1.4** 烧录STM32 (确保包含新增的 `TRACK` 指令解析 + `Servo_AdjustPan/Tilt` 函数)
- [ ] **1.5** 连接硬件: 树莓派UART(GPIO14/15)↔STM32(PA9/PA10), OpenMV↔树莓派USB

### Phase 2: 单模块测试

- [ ] **2.1** 运行 `red_laser_tracker.py` 在OpenMV IDE中验证红色检测
- [ ] **2.2** 在树莓派上用minicom/PuTTY验证能收到OpenMV串口数据
- [ ] **2.3** STM32串口指令测试: PC串口助手→STM32, 发送 `TRACK,5,-3\n` 确认舵机响应

### Phase 3: 集成代码编写

- [ ] **3.1** 创建 `serial_reader.py` — 串口接收模块
- [ ] **3.2** 创建 `pid_controller.py` — PID控制器
- [ ] **3.3** 创建 `calibration.py` — 校准模块
- [ ] **3.4** 创建 `stm32_comm.py` — STM32串口通信模块
- [ ] **3.5** 创建 `main_controller.py` — 主控制循环
- [ ] **3.6** (可选) 创建 `threshold_tuner.py` — OpenMV阈值标定工具

### Phase 4: 系统联调

- [ ] **4.1** 开环测试: 移动激光笔, 观察误差读数
- [ ] **4.2** 闭环测试: 启用PID, 调参数
- [ ] **4.3** 校准scale_x/scale_y
- [ ] **4.4** 鲁棒性测试(遮挡、快速移动、断线重连)

### Phase 5: 与现有MCU集成（可选）

- [ ] **5.1** MCU端增加 `TRACK,delta_pan,delta_tilt` 命令解析
- [ ] **5.2** 树莓派端增加MCU串口通信模块
- [ ] **5.3** 联调: 树莓派(视觉+PID) → MCU(舵机驱动)

---

## 附录A: 依赖清单 (requirements.txt)

```
# 树莓派端Python依赖 (仅一个!)
pyserial>=3.5
```

> STM32 端不需要额外依赖——用的是现有 TEST/ 项目 + 新增两行 `Servo_AdjustPan/Tilt` 函数。

## 附录B: 参考项目关键文件索引

| 文件 | 参考价值 |
|------|---------|
| [TEST/2023-电赛E题-目标追踪系统/GreenServo/K210/green.py](TEST/2023-电赛E题-目标追踪系统/GreenServo/K210/green.py) | 红绿双色检测、UART通信协议 |
| [TEST/2023-电赛E题-目标追踪系统/GreenServo/STM32/User/YunTai/yuntai.c](TEST/2023-电赛E题-目标追踪系统/GreenServo/STM32/User/YunTai/yuntai.c) | 增量式PID实现、舵机插值运动 |
| [TEST/2023-电赛E题-国二-全套代码/Application/Src/PID.c](TEST/2023-电赛E题-国二-全套代码/Application/Src/PID.c) | 专家PID、积分分离、微分先行 |
| [TEST/2023-电赛E题-国二-全套代码/Application/Src/platformTask.c](TEST/2023-电赛E题-国二-全套代码/Application/Src/platformTask.c) | 跟踪主循环、线性插值目标点生成 |
| [TEST/2023-电赛E题-国二-全套代码/Application/Src/communication.c](TEST/2023-电赛E题-国二-全套代码/Application/Src/communication.c) | CRC8校验的串口协议实现 |
| [TEST/Core/Src/Motion.c](TEST/Core/Src/Motion.c) | 平滑轨迹插值 (可用于搜索模式) |
| [TEST/Docs/SquareDraw_StateMachine.md](TEST/Docs/SquareDraw_StateMachine.md) | 状态机设计文档 |

## 附录C: 快速启动命令

```bash
# 1. 树莓派端启动追踪
cd /home/pi/laser_tracker
python3 main_controller.py --port /dev/ttyACM0

# 2. 调试模式 (更多日志)
python3 main_controller.py --port /dev/ttyACM0 2>&1 | tee tracker.log

# 3. 仅测试串口接收
python3 -c "
from serial_reader import LaserTracker
t = LaserTracker()
t.start()
import time
for _ in range(100):
    print(f'({t.cx}, {t.cy}) detected={t.detected}')
    time.sleep(0.2)
t.stop()
"
```
