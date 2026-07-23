# USB摄像头版本快速测试指南

## 系统要求

- 树莓派4B（或其他运行Python的设备）
- USB摄像头
- STM32 MCU（已烧录最新代码）
- 红色激光笔

## 安装依赖

```bash
cd raspberry_pi
pip install -r requirements.txt
```

## 测试步骤

### 步骤1：测试USB摄像头

```bash
python test_usb_camera.py
```

**预期结果：**
- 打开摄像头窗口
- 显示实时画面
- 用红色激光笔照射墙面，应看到绿色圆圈标记光斑
- 画面中心显示白色十字线
- 左上角显示误差值
- 按 'q' 退出，按 's' 保存截图

**如果看不到画面：**
- 检查摄像头是否已连接
- 尝试不同的摄像头索引：`python test_usb_camera.py`（会自动检测）
- 检查摄像头权限

**如果检测不到红色激光：**
- 调整环境光照
- 尝试不同颜色的背景（白色墙面效果最好）
- 检查激光笔是否开启

### 步骤2：测试MCU通信

```bash
python -c "
from mcu_communicator import MCUCommunicator
import time

mcu = MCUCommunicator(port='/dev/ttyAMA0')
if mcu.connect():
    print('MCU连接成功')
    mcu.send_center()
    time.sleep(1)
    mcu.send_track_command(10, 0)
    print('发送TRACK命令: 10, 0')
    time.sleep(0.5)
    mcu.send_track_command(-10, 0)
    print('发送TRACK命令: -10, 0')
    mcu.disconnect()
else:
    print('MCU连接失败')
"
```

**预期结果：**
- MCU连接成功
- 舵机归中
- Pan舵机先右转再左转

### 步骤3：运行完整追踪系统

```bash
python main_controller_usb.py
```

**命令行参数：**
```bash
# 指定摄像头索引
python main_controller_usb.py --camera 0

# 指定MCU串口
python main_controller_usb.py --mcu-port /dev/ttyAMA0

# 关闭调试输出
python main_controller_usb.py --no-debug

# 关闭OpenCV窗口（无头模式）
python main_controller_usb.py --no-gui
```

**运行时控制：**
- **OpenCV窗口：**
  - ESC：退出
  - c：舵机归中
  - r：重置PID

- **终端：**
  - Ctrl+C：退出

**预期结果：**
- 摄像头窗口显示实时画面
- 终端显示追踪状态
- 用激光笔照射墙面，舵机应该追踪激光点
- 移动激光笔，舵机跟随移动

## 接线说明

```
USB摄像头 ──→ 树莓派USB口

树莓派              MCU
Pin8 (TX)  ────→   RX
Pin10(RX)  ←────   TX
GND        ────→   GND

MCU        ────→   舵机云台
```

## 常见问题

### 1. 摄像头打不开

```bash
# 检查摄像头设备
ls /dev/video*

# 检查摄像头权限
sudo usermod -a -G video $USER
# 重新登录生效
```

### 2. MCU串口连接失败

```bash
# 检查串口设备
ls /dev/ttyAMA0

# 检查串口权限
sudo usermod -a -G dialout $USER
# 重新登录生效
```

### 3. 检测不到红色激光

**可能原因：**
- 环境光照太强或太弱
- 激光笔功率太低
- 背景颜色干扰

**解决方法：**
- 在较暗的环境测试
- 使用更高功率的激光笔
- 在白色墙面上测试

### 4. 舵机不追踪

**检查步骤：**
1. 确认MCU已烧录最新代码（包含TRACK命令）
2. 确认串口通信正常
3. 确认摄像头能检测到激光
4. 查看终端输出的误差值是否正确

### 5. 追踪不稳定

**调节PID参数：**
编辑 `pid_controller.py` 中的参数：

```python
PAN_PID_CONFIG = PIDConfig(
    kp=0.15,    # 增大→响应更快，但可能振荡
    ki=0.003,   # 增大→消除稳态误差，但可能过冲
    kd=0.08,    # 增大→抑制振荡，但可能抖动
    ...
)
```

## 性能调优

### 1. 提高帧率

```python
# 在 usb_camera_tracker.py 中调整分辨率
self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # 降低分辨率
self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
```

### 2. 提高检测灵敏度

```python
# 在 usb_camera_tracker.py 中调整阈值
lower_red1 = np.array([0, 80, 80])    # 降低饱和度和亮度阈值
upper_red1 = np.array([10, 255, 255])
```

### 3. 调节控制频率

```python
# 在 main_controller_usb.py 中调整
CONTROL_PERIOD = 0.020  # 20ms = 50Hz
```

## 下一步

测试成功后，可以：
1. 调节PID参数优化追踪性能
2. 校准像素到角度的映射
3. 部署到实际应用场景