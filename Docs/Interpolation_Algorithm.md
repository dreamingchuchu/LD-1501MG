# 插值算法文档

## 1. 算法概述

Motion模块使用线性插值算法实现舵机的连续平滑运动。

## 2. 算法原理

### 2.1 线性插值

线性插值是最简单的插值方法，在当前点和目标点之间进行匀速运动。

**公式：**
```
current_position = current_position + step
```

其中：
- `step = speed`（向目标方向）
- `speed`为速度参数（默认1.0°/20ms）

### 2.2 插值方向判定

根据当前角度与目标角度的差值判定运动方向：

```c
float delta = target_angle - current_angle;

if (delta > 0) {
    // 向正方向运动
    step = speed;
    if (step > delta) step = delta;  // 不超过剩余距离
} else if (delta < 0) {
    // 向负方向运动
    step = -speed;
    if (step < delta) step = delta;  // 不超过剩余距离
}
```

### 2.3 到达判定

当当前角度与目标角度的差值小于阈值时，判定为已到达：

```c
if (fabsf(delta_pan) < 1.0f && fabsf(delta_tilt) < 1.0f) {
    // 已到达目标
    current_pan = target_pan;
    current_tilt = target_tilt;
    is_busy = 0;
}
```

**阈值：** 1.0°（可调整）

## 3. 算法实现

### 3.1 Motion_Update函数

```c
void Motion_Update(void)
{
    // 1. 检查运动状态
    if (!g_MotionState.is_busy) {
        return;
    }
    
    // 2. 计算差值
    float delta_pan = g_MotionState.target_pan - g_MotionState.current_pan;
    float delta_tilt = g_MotionState.target_tilt - g_MotionState.current_tilt;
    
    // 3. 判断是否到达
    if (fabsf(delta_pan) < 1.0f && fabsf(delta_tilt) < 1.0f) {
        g_MotionState.current_pan = g_MotionState.target_pan;
        g_MotionState.current_tilt = g_MotionState.target_tilt;
        g_MotionState.is_busy = 0;
        return;
    }
    
    // 4. 计算步长
    float step_pan = 0.0f;
    float step_tilt = 0.0f;
    
    if (delta_pan > 0.0f) {
        step_pan = g_MotionState.speed;
        if (step_pan > delta_pan) step_pan = delta_pan;
    } else if (delta_pan < 0.0f) {
        step_pan = -g_MotionState.speed;
        if (step_pan < delta_pan) step_pan = delta_pan;
    }
    
    if (delta_tilt > 0.0f) {
        step_tilt = g_MotionState.speed;
        if (step_tilt > delta_tilt) step_tilt = delta_tilt;
    } else if (delta_tilt < 0.0f) {
        step_tilt = -g_MotionState.speed;
        if (step_tilt < delta_tilt) step_tilt = delta_tilt;
    }
    
    // 5. 更新当前位置
    g_MotionState.current_pan += step_pan;
    g_MotionState.current_tilt += step_tilt;
    
    // 6. 角度钳位
    if (g_MotionState.current_pan < 0.0f) g_MotionState.current_pan = 0.0f;
    if (g_MotionState.current_pan > 180.0f) g_MotionState.current_pan = 180.0f;
    if (g_MotionState.current_tilt < 0.0f) g_MotionState.current_tilt = 0.0f;
    if (g_MotionState.current_tilt > 180.0f) g_MotionState.current_tilt = 180.0f;
    
    // 7. 设置舵机角度
    Servo_SetPanAngle((uint16_t)g_MotionState.current_pan);
    Servo_SetTiltAngle((uint8_t)g_MotionState.current_tilt);
}
```

## 4. 精度分析

### 4.1 角度精度

**影响因素：**
1. 插值步长：由速度参数决定（默认1.0°）
2. 到达阈值：1.0°
3. 舵机分辨率：取决于PWM精度

**理论精度：**
- 最大误差：1.0°（到达阈值）
- 实际精度：取决于舵机硬件精度

### 4.2 时间精度

**影响因素：**
1. 主循环周期：20ms
2. 定时器精度：HAL_GetTick()精度为1ms

**理论精度：**
- 周期误差：±1ms
- 实际精度：取决于系统负载

### 4.3 运动连续性

**定义：** 相邻两次Update调用之间的角度差

**理论值：**
```
Δangle = speed × 20ms
```

**默认值：**
```
Δangle = 1.0°/20ms × 20ms = 1.0°
```

**最大值：**
```
Δangle_max = 5.0°/20ms × 20ms = 5.0°
```

## 5. 性能优化

### 5.1 减少计算量

**优化前：**
```c
float delta_pan = target_pan - current_pan;
float delta_tilt = target_tilt - current_tilt;
```

**优化后：**
```c
// 无需优化，减法运算很快
```

### 5.2 减少函数调用

**优化前：**
```c
Servo_SetPanAngle((uint16_t)current_pan);
Servo_SetTiltAngle((uint8_t)current_tilt);
```

**优化后：**
```c
// 可以考虑内联Servo_SetXXX函数
// 但实际影响很小，不推荐
```

### 5.3 减少浮点运算

**当前实现：** 使用float类型

**替代方案：** 使用定点数（不推荐）
- 优点：减少浮点运算开销
- 缺点：精度降低，代码复杂度增加

**结论：** STM32F103有硬件FPU，浮点运算性能足够

## 6. 算法扩展

### 6.1 其他插值算法

**S曲线插值：**
- 优点：运动更平滑，加速度连续
- 缺点：计算复杂，需要存储更多状态

**梯形插值：**
- 优点：有加减速过程，运动平滑
- 缺点：计算较复杂

**结论：** 当前线性插值已满足需求，无需扩展

### 6.2 多轴协调

**当前实现：** Pan和Tilt独立插值

**问题：** 两轴到达时间可能不同

**解决方案：** 协调插值（不推荐）
- 计算两轴的最大差值
- 根据最大差值调整两轴速度
- 使两轴同时到达

**结论：** 当前独立插值已满足需求

## 7. 测试验证

### 7.1 单元测试

**测试用例1：** 正向运动
```c
Motion_Init();
Motion_MoveTo(100.0f, 90.0f);  // Pan从90°运动到100°

// 期望：每次Update，Pan增加1°
// 10次Update后，到达目标
```

**测试用例2：** 负向运动
```c
Motion_Init();
Motion_MoveTo(80.0f, 90.0f);  // Pan从90°运动到80°

// 期望：每次Update，Pan减少1°
// 10次Update后，到达目标
```

**测试用例3：** 同时运动
```c
Motion_Init();
Motion_MoveTo(100.0f, 100.0f);  // Pan和Tilt同时运动

// 期望：两轴同时到达
```

### 7.2 精度测试

**测试方法：**
1. 记录运动过程中的角度序列
2. 计算相邻角度差
3. 验证角度差≤speed

**期望结果：**
- 所有相邻角度差≤1.0°（默认速度）
- 到达误差<1.0°

## 8. 注意事项

1. **速度参数：** 速度参数影响运动平滑度和时间，建议根据实际需求调整
2. **到达阈值：** 阈值过小会导致无法到达，过大会导致精度降低
3. **角度钳位：** 目标角度会被自动钳位到0~180°，无需手动检查
4. **定时调用：** Update必须每20ms调用一次，否则运动不连续
5. **浮点精度：** STM32F103的float精度足够，无需使用double