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

### 树莓派 ← → OpenMV
- OpenMV USB → 树莓派USB口（/dev/ttyACM0）

### 树莓派 ← → MCU
- 树莓派 Pin8 (TX/GPIO14) → MCU RX
- 树莓派 Pin10 (RX/GPIO15) → MCU TX
- 树莓派 GND → MCU GND

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