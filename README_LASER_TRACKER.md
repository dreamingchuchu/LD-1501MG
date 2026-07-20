# 红色激光追踪系统

基于树莓派4B + OpenMV + STM32/MSPM0 实现的红色激光笔光斑实时检测与追踪系统。

## 项目结构

```
task2/
├── openmv/                    # OpenMV端代码
│   ├── red_laser_tracker.py   # 红色激光检测主程序
│   └── threshold_tuner.py     # 阈值标定工具
│
├── raspberry_pi/              # 树莓派端代码
│   ├── serial_reader.py       # OpenMV串口接收模块
│   ├── mcu_communicator.py    # MCU串口通信模块
│   ├── pid_controller.py      # PID控制器
│   ├── calibration.py         # 校准模块
│   └── main_controller.py     # 主控制程序
│
├── TEST/                      # MCU端代码（STM32）
│   └── Core/
│       ├── Src/
│       │   ├── Serial.c       # 串口命令解析（已添加TRACK命令）
│       │   └── Servo.c        # 舵机控制（已添加增量调整函数）
│       └── Inc/
│           └── Servo.h        # 舵机头文件
│
└── VISION_GUIDE.md            # 开发指导手册
```

## 系统架构

```
OpenMV Cam H7 → 树莓派4B → MCU(STM32) → 舵机云台
    │              │            │
    │              │            └─ PWM输出
    │              └─ PID计算 + 发送TRACK命令
    └─ 红色激光检测 + 发送坐标
```

## 通信协议

### OpenMV → 树莓派
- 格式：`cx,cy\n`
- 示例：`145,98\n`
- 无光斑：`-1,-1\n`

### 树莓派 → MCU
- TRACK命令：`TRACK,delta_pan,delta_tilt\n`
- 示例：`TRACK,5,-3\n`

## 快速启动

### 1. OpenMV端
1. 在OpenMV IDE中打开 `openmv/red_laser_tracker.py`
2. 根据实际环境调节LAB色彩阈值
3. 运行程序

### 2. MCU端
1. 编译并烧录更新后的代码到STM32
2. 确认串口通信正常

### 3. 树莓派端
```bash
# 安装依赖
pip install pyserial

# 运行主程序
cd raspberry_pi
python main_controller.py --openmv-port /dev/ttyACM0 --mcu-port /dev/ttyAMA0
```

## 接线说明

### 系统连接图

```
┌─────────────┐         ┌─────────────┐         ┌─────────────┐         ┌─────────────┐
│  OpenMV H7  │         │  树莓派 4B  │         │  STM32 MCU  │         │  舵机云台   │
│             │         │             │         │             │         │             │
│         USB ├─────────┤ USB口       │         │             │         │             │
│             │         │             │         │             │         │             │
│             │         │ Pin8  (TX)  ├─────────┤ RX          │         │             │
│             │         │ Pin10 (RX)  ├─────────┤ TX          │         │             │
│             │         │ GND         ├─────────┤ GND         │         │             │
│             │         │             │         │             │         │             │
│             │         │             │         │ PWM1        ├─────────┤ Pan舵机信号 │
│             │         │             │         │ PWM2        ├─────────┤ Tilt舵机信号│
│             │         │             │         │ GND         ├─────────┤ 舵机GND    │
│             │         │             │         │ 5V/6V       ├─────────┤ 舵机电源    │
└─────────────┘         └─────────────┘         └─────────────┘         └─────────────┘
```

### 详细接线表

#### 1. OpenMV ← → 树莓派（USB连接）

| OpenMV | 树莓派 | 说明 |
|--------|--------|------|
| USB口 | USB口 | USB连接，同时供电和通信 |
| | | 设备名：/dev/ttyACM0 |

**或使用串口连接（可选）：**

| OpenMV | 树莓派 | 说明 |
|--------|--------|------|
| P4 (RX) | Pin8 (GPIO14/TX) | 数据传输 |
| P5 (TX) | Pin10 (GPIO15/RX) | 数据传输 |
| GND | GND | 共地 |

#### 2. 树莓派 ← → MCU（串口连接）

| 树莓派 GPIO | MCU | 说明 |
|-------------|-----|------|
| Pin8 (GPIO14/TX) | RX (PA10) | 树莓派发送 → MCU接收 |
| Pin10 (GPIO15/RX) | TX (PA9) | MCU发送 → 树莓派接收 |
| Pin6 (GND) | GND | 共地（必须连接） |

**设备名：/dev/ttyAMA0**

#### 3. MCU ← → 舵机

| MCU | 舵机 | 说明 |
|-----|------|------|
| PA0 (TIM2_CH1) | Pan舵机信号线 | 水平舵机PWM |
| PA1 (TIM2_CH2) | Tilt舵机信号线 | 垂直舵机PWM |
| GND | 舵机GND（棕色线） | 共地 |
| 5V/6V | 舵机VCC（红色线） | 外部电源供电 |

### 树莓派引脚图

```
树莓派 4B GPIO 引脚：
┌─────────────────────────────┐
│  3.3V  (Pin1)   (Pin2)  5V  │
│  SDA1  (Pin3)   (Pin4)  5V  │
│  SCL1  (Pin5)   (Pin6)  GND │ ← 连接MCU GND
│  GPIO4 (Pin7)   (Pin8)  TX  │ ← 连接MCU RX
│  GND   (Pin9)   (Pin10) RX  │ ← 连接MCU TX
│  ...                         │
└─────────────────────────────┘
```

### 供电注意事项

**⚠️ 重要：舵机必须独立供电！**

```
方案1：外部电源模块
┌─────────────┐
│  6V/2A电源  │───┬───→ 舵机VCC（红色线）
│             │   │
└─────────────┘   └───→ 舵机GND（棕色线）───→ MCU GND

方案2：锂电池
┌─────────────┐
│  2S锂电池   │───┬───→ 舵机VCC
│  (7.4V)     │   │    （需加降压模块到6V）
└─────────────┘   └───→ 舵机GND ───→ MCU GND
```

### 接线步骤

**步骤1：OpenMV连接**
```
1. 用USB线连接OpenMV到树莓派USB口
2. 检查设备：ls /dev/ttyACM*
3. 应看到 /dev/ttyACM0
```

**步骤2：MCU串口连接**
```
1. 连接树莓派Pin8 (TX) → MCU RX (PA10)
2. 连接树莓派Pin10 (RX) → MCU TX (PA9)
3. 连接树莓派GND → MCU GND（必须共地）
4. 检查设备：ls /dev/ttyAMA*
5. 应看到 /dev/ttyAMA0
```

**步骤3：舵机连接**
```
1. 连接MCU PA0 → Pan舵机信号线（橙色/黄色）
2. 连接MCU PA1 → Tilt舵机信号线（橙色/黄色）
3. 连接外部电源6V → 舵机VCC（红色）
4. 连接外部电源GND → 舵机GND（棕色）
5. 连接外部电源GND → MCU GND（共地）
```

### 验证接线

```bash
# 在树莓派上检查设备
ls /dev/ttyACM0   # OpenMV
ls /dev/ttyAMA0   # MCU

# 检查串口权限
sudo usermod -a -G dialout $USER
# 需要重新登录生效

# 测试串口通信
python -c "
import serial
# 测试OpenMV
try:
    s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print('OpenMV串口OK')
    s.close()
except: print('OpenMV串口失败')

# 测试MCU
try:
    s = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    print('MCU串口OK')
    s.close()
except: print('MCU串口失败')
"
```

### 常见问题

| 问题 | 原因 | 解决方法 |
|------|------|----------|
| 找不到ttyACM0 | OpenMV未连接 | 检查USB线，重新插拔 |
| 找不到ttyAMA0 | 串口未启用 | `sudo raspi-config`启用串口 |
| 串口权限拒绝 | 用户不在dialout组 | `sudo usermod -a -G dialout $USER` |
| 舵机抖动 | 电源不足 | 使用外部6V/2A电源供电 |
| 通信乱码 | 未共地 | 连接所有设备GND |

## 参数调节

### PID参数
- Pan轴：Kp=0.15, Ki=0.003, Kd=0.08
- Tilt轴：Kp=0.15, Ki=0.003, Kd=0.06

### 校准参数
- scale_x = 0.05 (像素→PWM映射系数)
- scale_y = 0.05

## 性能指标

- 控制频率：50Hz
- 稳态误差：<5像素
- 响应延迟：<200ms
- 检测帧率：≥25fps