# Motion模块接口文档

## 1. 模块概述

Motion模块负责舵机的连续平滑运动控制，实现非阻塞式的轨迹插值计算。

## 2. 数据结构

### 2.1 MotionState_t

```c
typedef struct {
    float current_pan;      // 当前Pan角度（0~180°）
    float current_tilt;     // 当前Tilt角度（0~180°）
    float target_pan;       // 目标Pan角度（0~180°）
    float target_tilt;      // 目标Tilt角度（0~180°）
    uint8_t is_busy;        // 运动状态标志（0=空闲，1=运动中）
    float speed;            // 速度参数（°/20ms，范围0.1~5.0）
} MotionState_t;
```

## 3. 接口函数

### 3.1 Motion_Init

**功能：** 初始化Motion模块，记录当前舵机位置

**原型：** `void Motion_Init(void)`

**参数：** 无

**返回值：** 无

**调用时机：** 系统启动时，在Servo初始化之后调用

**示例：**
```c
Servo_SetPanAngle(90);
Servo_SetTiltAngle(90);
Motion_Init();  // 记录当前位置为(90, 90)
```

---

### 3.2 Motion_MoveTo

**功能：** 设置目标位置，启动运动（非阻塞）

**原型：** `int8_t Motion_MoveTo(float target_pan, float target_tilt)`

**参数：**
- `target_pan`: 目标Pan角度（0~180°）
- `target_tilt`: 目标Tilt角度（0~180°）

**返回值：**
- `0`: 成功
- `-1`: 失败（当前正在运动中）

**注意事项：**
- 该函数为非阻塞调用，立即返回
- 目标角度会被自动钳位到0~180°范围
- 如果当前正在运动中，会返回错误码-1

**示例：**
```c
int8_t result = Motion_MoveTo(120.0f, 90.0f);
if (result == 0) {
    // 成功启动运动
} else {
    // 当前正在运动中，无法启动新运动
}
```

---

### 3.3 Motion_Update

**功能：** 执行插值计算，更新舵机位置（每20ms调用一次）

**原型：** `void Motion_Update(void)`

**参数：** 无

**返回值：** 无

**调用时机：** 主循环中每20ms调用一次

**算法原理：**
1. 检查`is_busy`标志，如果为0则直接返回
2. 计算当前角度与目标角度的差值
3. 判断是否到达目标（误差<1°）
4. 如果未到达，按速度参数计算步长
5. 更新当前位置并调用Servo_SetXXX设置舵机角度

**示例：**
```c
// 主循环中
if (HAL_GetTick() - last_tick >= 20) {
    last_tick = HAL_GetTick();
    Motion_Update();  // 每20ms调用一次
}
```

---

### 3.4 Motion_IsBusy

**功能：** 查询运动状态

**原型：** `uint8_t Motion_IsBusy(void)`

**参数：** 无

**返回值：**
- `0`: 空闲（已到达目标或未启动运动）
- `1`: 运动中

**使用场景：** SquareDraw模块通过该函数判断当前边段是否完成

**示例：**
```c
if (!Motion_IsBusy()) {
    // 当前运动已完成，可以启动下一段运动
    Motion_MoveTo(next_pan, next_tilt);
}
```

---

### 3.5 Motion_Stop

**功能：** 停止当前运动

**原型：** `void Motion_Stop(void)`

**参数：** 无

**返回值：** 无

**注意事项：**
- 立即停止运动，不等待到达目标
- 舵机保持在当前位置

**示例：**
```c
// STOP命令处理
void SquareDraw_Stop(void) {
    Motion_Stop();  // 停止运动
    // ...
}
```

---

### 3.6 Motion_SetSpeed

**功能：** 设置运动速度

**原型：** `int8_t Motion_SetSpeed(float speed)`

**参数：**
- `speed`: 速度参数（°/20ms），范围0.1~5.0

**返回值：**
- `0`: 成功
- `-1`: 失败（速度超出范围）

**速度说明：**
- 默认速度：1.0°/20ms（约50°/s）
- 最小速度：0.1°/20ms（约5°/s）
- 最大速度：5.0°/20ms（约250°/s）

**示例：**
```c
// 设置速度为2.0°/20ms（约100°/s）
int8_t result = Motion_SetSpeed(2.0f);
if (result == 0) {
    Serial_SendString("速度设置成功\r\n");
} else {
    Serial_SendString("速度超出范围\r\n");
}
```

---

### 3.7 Motion_GetSpeed

**功能：** 获取当前运动速度

**原型：** `float Motion_GetSpeed(void)`

**参数：** 无

**返回值：** 当前速度参数（°/20ms）

**示例：**
```c
float current_speed = Motion_GetSpeed();
Serial_SendString("当前速度：");
Serial_SendNum((uint32_t)current_speed);
Serial_SendString("°/20ms\r\n");
```

## 4. 使用流程

### 4.1 基本使用流程

```c
// 1. 初始化
Motion_Init();

// 2. 设置目标位置
Motion_MoveTo(120.0f, 90.0f);

// 3. 主循环中持续调用Update
while (1) {
    if (HAL_GetTick() - last_tick >= 20) {
        last_tick = HAL_GetTick();
        Motion_Update();
    }
    
    // 4. 检查是否到达
    if (!Motion_IsBusy()) {
        // 到达目标，执行下一步操作
        break;
    }
}
```

### 4.2 连续轨迹绘制

```c
// 绘制正方形：依次移动到四个角点
Motion_MoveTo(corner[0].pan, corner[0].tilt);  // 起点
while (Motion_IsBusy()) Motion_Update();

Motion_MoveTo(corner[1].pan, corner[1].tilt);  // 角点1
while (Motion_IsBusy()) Motion_Update();

Motion_MoveTo(corner[2].pan, corner[2].tilt);  // 角点2
while (Motion_IsBusy()) Motion_Update();

Motion_MoveTo(corner[3].pan, corner[3].tilt);  // 角点3
while (Motion_IsBusy()) Motion_Update();
```

## 5. 注意事项

1. **非阻塞设计：** 所有接口均为非阻塞调用，不会等待运动完成
2. **定时调用：** `Motion_Update()`必须每20ms调用一次，否则运动不连续
3. **状态检查：** 调用`Motion_MoveTo()`前应检查`Motion_IsBusy()`，避免覆盖当前运动
4. **角度范围：** 目标角度会被自动钳位到0~180°，无需手动检查
5. **速度调整：** 速度参数影响所有后续运动，建议在初始化时设置