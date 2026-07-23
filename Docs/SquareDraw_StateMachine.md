# SquareDraw状态机文档

## 1. 状态机概述

SquareDraw模块采用状态机驱动，实现非阻塞式的正方形轨迹绘制。

## 2. 状态定义

### 2.1 状态枚举

```c
typedef enum {
    SQUARE_STATE_IDLE = 0,              // 空闲状态
    SQUARE_STATE_MOVING_TO_START = 1,   // 移动到起点状态
    SQUARE_STATE_MOVING_EDGE = 2,       // 边段运动状态
    SQUARE_STATE_ERROR = 3              // 错误状态
} SquareDrawStateEnum;
```

### 2.2 状态说明

| 状态 | 说明 | 进入条件 | 退出条件 |
|------|------|----------|----------|
| IDLE | 空闲状态，等待SQUARE命令 | 系统启动、绘制完成、STOP命令 | 收到SQUARE命令 |
| MOVING_TO_START | 移动到起点（原点） | 收到SQUARE命令，角点计算成功 | Motion到达起点 |
| MOVING_EDGE | 边段运动，依次绘制四条边 | 到达起点 | 四条边全部完成 |
| ERROR | 错误状态 | 角度超出范围 | 无（需复位） |

## 3. 状态转换图

```
            SQUARE命令
            ↓
    ┌───────────────────┐
    │       IDLE        │◄─────────────┐
    └───────────────────┘              │
            │                          │
            │ 角点计算成功              │
            ↓                          │
    ┌───────────────────┐              │
    │  MOVING_TO_START  │              │
    └───────────────────┘              │
            │                          │
            │ Motion到达起点            │
            ↓                          │
    ┌───────────────────┐              │
    │   MOVING_EDGE     │──────────────┤
    └───────────────────┘  四条边完成  │
            │                          │
            │ STOP命令                 │
            └──────────────────────────┘
```

## 4. 状态处理逻辑

### 4.1 IDLE状态

**进入条件：**
- 系统启动时
- 绘制完成后
- STOP命令后

**处理逻辑：**
- 无操作，等待SQUARE命令

**退出条件：**
- 收到SQUARE命令，转换为MOVING_TO_START状态

---

### 4.2 MOVING_TO_START状态

**进入条件：**
- 收到SQUARE命令
- 角点计算成功
- 调用`Motion_MoveTo()`移动到起点

**处理逻辑：**
```c
if (!Motion_IsBusy()) {
    // 到达起点
    Serial_SendString("到达起点，开始绘制第一条边\r\n");
    state = SQUARE_STATE_MOVING_EDGE;
    current_edge = 0;
    Motion_MoveTo(corners[1].pan, corners[1].tilt);  // 启动第一条边
}
```

**退出条件：**
- Motion到达起点，转换为MOVING_EDGE状态

---

### 4.3 MOVING_EDGE状态

**进入条件：**
- 从MOVING_TO_START状态转换
- `current_edge`初始化为0

**处理逻辑：**
```c
if (!Motion_IsBusy()) {
    // 当前边段完成
    输出"到达角点N"信息
    
    if (current_edge < 3) {
        // 还有边段未完成
        current_edge++;
        next_corner = current_edge + 1;
        if (next_corner > 3) next_corner = 0;
        Motion_MoveTo(corners[next_corner].pan, corners[next_corner].tilt);
    } else {
        // 四条边全部完成
        state = SQUARE_STATE_IDLE;
        g_SquareMode = 0;
        输出"绘制完成"信息
    }
}
```

**退出条件：**
- 四条边全部完成，转换为IDLE状态
- STOP命令，转换为IDLE状态

---

### 4.4 ERROR状态

**进入条件：**
- 角度超出范围（角点计算失败）

**处理逻辑：**
- 无操作，等待复位

**退出条件：**
- RST命令或系统复位

## 5. 边段绘制顺序

### 5.1 角点定义

| 角点索引 | 坐标 | Pan角度 | Tilt角度 |
|---------|------|---------|----------|
| 0（原点） | (0, 0) | origin_pan | origin_tilt |
| 1（右上） | (width, 0) | origin_pan + Δθ_pan | origin_tilt |
| 2（右下） | (width, height) | origin_pan + Δθ_pan | origin_tilt + Δθ_tilt |
| 3（左下） | (0, height) | origin_pan | origin_tilt + Δθ_tilt |

### 5.2 边段绘制顺序

```
角点0（原点）→ 角点1（右上）→ 角点2（右下）→ 角点3（左下）→ 角点0（原点）
     ↑                                                                              ↓
     └──────────────────────────────────────────────────────────────────────────────┘
```

**边段顺序：**
1. 边段0：角点0 → 角点1（向右）
2. 边段1：角点1 → 角点2（向下）
3. 边段2：角点2 → 角点3（向左）
4. 边段3：角点3 → 角点0（向上）

### 5.3 current_edge与角点关系

| current_edge | 起始角点 | 目标角点 | 运动方向 |
|-------------|---------|---------|---------|
| 0 | 0 | 1 | 向右（Pan增加） |
| 1 | 1 | 2 | 向下（Tilt增加） |
| 2 | 2 | 3 | 向左（Pan减少） |
| 3 | 3 | 0 | 向上（Tilt减少） |

## 6. 时序图

### 6.1 正常绘制流程

```
用户            Serial          SquareDraw           Motion            Servo
 │                │                 │                  │                 │
 │──SQUARE 20──→│                 │                  │                 │
 │                │──ParseCommand──→│                  │                 │
 │                │                 │──CalculateCorners│                 │
 │                │                 │                  │                 │
 │                │                 │──MoveTo(角点0)──→│                 │
 │                │                 │  (MOVING_TO_START)│                 │
 │                │                 │                  │──SetPanAngle──→│
 │                │                 │                  │──SetTiltAngle──→│
 │                │                 │                  │                 │
 │                │                 │                  │←─Update(20ms)──│
 │                │                 │                  │──SetPanAngle──→│
 │                │                 │                  │──SetTiltAngle──→│
 │                │                 │                  │                 │
 │                │                 │←─IsBusy=false───│                 │
 │                │                 │  (到达起点)       │                 │
 │                │                 │                  │                 │
 │                │                 │──MoveTo(角点1)──→│                 │
 │                │                 │  (MOVING_EDGE)   │                 │
 │                │                 │                  │──SetPanAngle──→│
 │                │                 │                  │──SetTiltAngle──→│
 │                │                 │                  │                 │
 │                │                 │                  │←─Update(20ms)──│
 │                │                 │                  │──SetPanAngle──→│
 │                │                 │                  │                 │
 │                │                 │←─IsBusy=false───│                 │
 │                │                 │  (到达角点1)     │                 │
 │                │                 │                  │                 │
 │                │                 │──MoveTo(角点2)──→│                 │
 │                │                 │                  │                 │
 ...              ...               ...                ...               ...
 │                │                 │                  │                 │
 │                │                 │←─IsBusy=false───│                 │
 │                │                 │  (到达角点0)     │                 │
 │                │                 │  (绘制完成)      │                 │
 │←─"绘制完成"───│                 │                  │                 │
```

### 6.2 STOP命令处理

```
用户            Serial          SquareDraw           Motion            Servo
 │                │                 │                  │                 │
 │──SQUARE 20──→│                 │                  │                 │
 │                │                 │──MoveTo(角点0)──→│                 │
 │                │                 │  (MOVING_EDGE)   │                 │
 │                │                 │                  │                 │
 │──STOP───────→│                 │                  │                 │
 │                │──ParseCommand──→│                  │                 │
 │                │                 │──Stop()─────────→│                 │
 │                │                 │  (IDLE)          │  (is_busy=0)    │
 │←─"绘制已停止"─│                 │                  │                 │
```

## 7. 数据结构

### 7.1 SquareDrawState_t

```c
typedef struct {
    SquareDrawStateEnum state;      // 当前状态
    uint8_t current_corner;         // 当前角点索引（保留，未使用）
    uint8_t current_edge;           // 当前边段索引（0~3）
    float origin_pan;               // 原点Pan角度
    float origin_tilt;              // 原点Tilt角度
    uint16_t width_cm;              // 正方形宽度（cm）
    uint16_t height_cm;             // 正方形高度（cm）
    CornerPoint_t corners[4];       // 四个角点坐标
    uint32_t start_time_ms;         // 开始时间（ms）
} SquareDrawState_t;
```

### 7.2 CornerPoint_t

```c
typedef struct {
    float pan_angle;    // Pan角度
    float tilt_angle;   // Tilt角度
    float x_cm;         // X坐标（cm）
    float y_cm;         // Y坐标（cm）
} CornerPoint_t;
```

## 8. 注意事项

1. **非阻塞设计：** 所有状态处理均为非阻塞，不使用Delay_ms()
2. **状态一致性：** 状态转换必须通过SquareDraw_Execute()，不能直接修改state
3. **Motion同步：** 状态转换依赖Motion_IsBusy()的返回值
4. **错误处理：** 角度超出范围时返回错误码-2，进入ERROR状态
5. **模式互斥：** g_SquareMode标志用于防止与SCAN模式冲突