# 性能指标文档

## 1. 性能指标概述

本文档定义Motion模块和SquareDraw模块的性能指标和验收标准。

## 2. 时间性能指标

### 2.1 主循环周期

**指标：** 主循环执行周期

**目标值：** 20ms±1ms

**测量方法：**
```c
uint32_t start_tick = HAL_GetTick();
// 执行一次主循环
uint32_t end_tick = HAL_GetTick();
uint32_t period = end_tick - start_tick;
```

**验收标准：**
- 周期在19ms~21ms范围内
- 周期抖动<1ms

---

### 2.2 Motion_Update执行时间

**指标：** Motion_Update函数执行时间

**目标值：** <1ms

**测量方法：**
```c
uint32_t start_tick = HAL_GetTick();
Motion_Update();
uint32_t end_tick = HAL_GetTick();
uint32_t execution_time = end_tick - start_tick;
```

**验收标准：**
- 执行时间<1ms
- 不影响主循环周期

---

### 2.3 SquareDraw_Execute执行时间

**指标：** SquareDraw_Execute函数执行时间

**目标值：** <1ms

**测量方法：**
```c
uint32_t start_tick = HAL_GetTick();
SquareDraw_Execute();
uint32_t end_tick = HAL_GetTick();
uint32_t execution_time = end_tick - start_tick;
```

**验收标准：**
- 执行时间<1ms
- 不影响主循环周期

---

### 2.4 主循环总执行时间

**指标：** 主循环总执行时间（包括所有任务）

**目标值：** <5ms

**测量方法：**
```c
uint32_t start_tick = HAL_GetTick();
// 执行主循环所有任务
Motion_Update();
SquareDraw_Execute();
// OLED更新等
uint32_t end_tick = HAL_GetTick();
uint32_t total_time = end_tick - start_tick;
```

**验收标准：**
- 总执行时间<5ms
- 不超过主循环周期（20ms）

## 3. 运动性能指标

### 3.1 运动速度

**指标：** 舵机运动速度

**默认值：** 1.0°/20ms（约50°/s）

**范围：** 0.1°/20ms ~ 5.0°/20ms（约5°/s ~ 250°/s）

**测量方法：**
```c
// 设置速度
Motion_SetSpeed(2.0f);

// 测量实际速度
float start_angle = Servo_GetPanAngle();
Delay_ms(1000);  // 等待1秒
float end_angle = Servo_GetPanAngle();
float actual_speed = (end_angle - start_angle) / 1.0f;  // °/s
```

**验收标准：**
- 实际速度与设定速度误差<5%
- 速度稳定，无抖动

---

### 3.2 运动连续性

**指标：** 相邻两次Update调用之间的角度差

**目标值：** ≤speed（默认1.0°）

**测量方法：**
```c
float angle_history[100];
for (int i = 0; i < 100; i++) {
    angle_history[i] = Servo_GetPanAngle();
    Delay_ms(20);
}

// 计算相邻角度差
float max_delta = 0;
for (int i = 1; i < 100; i++) {
    float delta = fabsf(angle_history[i] - angle_history[i-1]);
    if (delta > max_delta) max_delta = delta;
}
```

**验收标准：**
- 最大角度差≤speed
- 无角度跳变（角度差>2×speed）

---

### 3.3 到达精度

**指标：** 到达目标角度的误差

**目标值：** <1.0°

**测量方法：**
```c
Motion_MoveTo(120.0f, 90.0f);
while (Motion_IsBusy()) {
    Motion_Update();
    Delay_ms(20);
}

float actual_pan = Servo_GetPanAngle();
float error = fabsf(actual_pan - 120.0f);
```

**验收标准：**
- 到达误差<1.0°
- 到达后角度稳定

## 4. 正方形绘制性能指标

### 4.1 绘制时间

**指标：** 绘制完整正方形的时间

**目标值：** <10秒（20cm×20cm正方形）

**测量方法：**
```c
uint32_t start_time = HAL_GetTick();
SquareDraw_Start(20, 20);
while (g_SquareMode) {
    Motion_Update();
    SquareDraw_Execute();
    Delay_ms(20);
}
uint32_t end_time = HAL_GetTick();
uint32_t draw_time = (end_time - start_time) / 1000;  // 秒
```

**验收标准：**
- 绘制时间<10秒
- 绘制时间与正方形尺寸成正比

---

### 4.2 边段长度精度

**指标：** 实际绘制的边长与设定边长的误差

**目标值：** <5%

**测量方法：**
- 使用标尺或激光测距仪测量实际边长
- 计算误差：`error = |actual_length - target_length| / target_length`

**验收标准：**
- 边长误差<5%
- 四条边长度一致

---

### 4.3 角点位置精度

**指标：** 实际角点位置与理论位置的误差

**目标值：** <2cm

**测量方法：**
- 使用标尺测量角点位置
- 计算与理论位置的偏差

**验收标准：**
- 角点位置误差<2cm
- 四个角点位置对称

## 5. 响应性能指标

### 5.1 STOP命令响应时间

**指标：** 从发送STOP命令到运动停止的时间

**目标值：** <100ms

**测量方法：**
```c
SquareDraw_Start(20, 20);
Delay_ms(500);  // 运动中

uint32_t start_time = HAL_GetTick();
// 发送STOP命令
SquareDraw_Stop();
uint32_t end_time = HAL_GetTick();
uint32_t response_time = end_time - start_time;
```

**验收标准：**
- 响应时间<100ms
- 运动立即停止，无延迟

---

### 5.2 命令响应时间

**指标：** 从发送命令到开始执行的时间

**目标值：** <50ms

**测量方法：**
```c
uint32_t start_time = HAL_GetTick();
// 发送SQUARE命令
SquareDraw_Start(20, 20);
uint32_t end_time = HAL_GetTick();
uint32_t response_time = end_time - start_time;
```

**验收标准：**
- 响应时间<50ms
- 命令立即执行，无延迟

## 6. 可靠性指标

### 6.1 运动连续性

**指标：** 运动过程中无卡顿、无跳变

**验收标准：**
- 运动过程连续平滑
- 无明显停顿
- 无角度跳变

---

### 6.2 状态一致性

**指标：** 状态机状态与实际运动状态一致

**验收标准：**
- 状态转换正确
- 无状态冲突
- 无死锁

---

### 6.3 异常恢复

**指标：** 异常情况下的恢复能力

**验收标准：**
- STOP命令能立即停止运动
- 错误状态能通过RST命令恢复
- 角度超出范围时返回错误码

## 7. 资源占用指标

### 7.1 CPU占用率

**指标：** Motion模块的CPU占用率

**目标值：** <10%

**测量方法：**
- 使用逻辑分析仪测量空闲时间
- 计算CPU占用率

**验收标准：**
- CPU占用率<10%
- 不影响其他任务

---

### 7.2 内存占用

**指标：** Motion模块的内存占用

**目标值：** <100字节

**测量方法：**
```c
// MotionState_t结构体大小
sizeof(MotionState_t) = 4*4 + 1 + 4 = 21字节
```

**验收标准：**
- 内存占用<100字节
- 无内存泄漏

---

### 7.3 栈占用

**指标：** Motion模块的栈占用

**目标值：** <50字节

**验收标准：**
- 栈占用<50字节
- 无栈溢出

## 8. 测试方法

### 8.1 性能测试工具

**硬件工具：**
- 逻辑分析仪：测量时序
- 示波器：测量PWM信号
- 激光测距仪：测量距离

**软件工具：**
- HAL_GetTick()：测量时间
- Serial输出：记录数据

### 8.2 测试环境

**环境要求：**
- 舵机供电稳定（5V±0.1V）
- 负载稳定（激光笔重量一致）
- 温度稳定（25°C±5°C）

### 8.3 测试流程

1. **准备阶段：**
   - 初始化系统
   - 设置测试参数
   - 准备测试工具

2. **执行阶段：**
   - 执行测试用例
   - 记录测试数据
   - 观察运动过程

3. **分析阶段：**
   - 分析测试数据
   - 计算性能指标
   - 判断是否达标

4. **报告阶段：**
   - 生成测试报告
   - 记录问题和改进建议

## 9. 验收标准汇总

| 指标类别 | 指标名称 | 目标值 | 验收标准 |
|---------|---------|--------|---------|
| 时间性能 | 主循环周期 | 20ms±1ms | 周期在19ms~21ms范围内 |
| 时间性能 | Motion_Update执行时间 | <1ms | 不影响主循环周期 |
| 时间性能 | 主循环总执行时间 | <5ms | 不超过主循环周期 |
| 运动性能 | 运动速度 | 50°/s | 误差<5% |
| 运动性能 | 运动连续性 | ≤1.0° | 无角度跳变 |
| 运动性能 | 到达精度 | <1.0° | 到达后角度稳定 |
| 绘制性能 | 绘制时间 | <10秒 | 与尺寸成正比 |
| 绘制性能 | 边段长度精度 | <5% | 四条边长度一致 |
| 绘制性能 | 角点位置精度 | <2cm | 四个角点位置对称 |
| 响应性能 | STOP命令响应时间 | <100ms | 运动立即停止 |
| 响应性能 | 命令响应时间 | <50ms | 命令立即执行 |
| 可靠性 | 运动连续性 | - | 无卡顿、无跳变 |
| 可靠性 | 状态一致性 | - | 无状态冲突、无死锁 |
| 资源占用 | CPU占用率 | <10% | 不影响其他任务 |
| 资源占用 | 内存占用 | <100字节 | 无内存泄漏 |