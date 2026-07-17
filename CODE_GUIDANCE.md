# STM32F103C8T6 二维云台串口控制器 + OLED — CubeIDE/HAL 代码指导文档

> **目标**：基于 STM32CubeIDE + HAL 库，实现串口指令控制二维云台（双 LD-1501MG 舵机），OLED 实时显示 Pan/Tilt 角度。将此文档发送给 Agent 即可生成完整代码。

---

## 一、项目概述

### 1.1 硬件清单

| 硬件 | 型号/参数 | 数量 | 用途 |
|------|-----------|------|------|
| 主控 | STM32F103C8T6 | 1 | 系统核心 |
| 舵机 | LD-1501MG (0°~180°) | 2 | Pan 水平 + Tilt 俯仰 |
| 云台支架 | 二维云台机械套件 | 1 | 承载两舵机 |
| 屏幕 | SSD1306 128×64 OLED (I2C, 4脚) | 1 | 实时显示 |
| 电源 | 实验室直流稳压电源 6~7.4V | 1 | 舵机独立供电 |
| USB转TTL | CH340 / CP2102 | 1 | 串口通信 |

### 1.2 功能列表

| # | 功能 | 串口指令 |
|---|------|----------|
| 1 | 单轴角度控制 | `P=90` → Pan 90°, `T=45` → Tilt 45° |
| 2 | 双轴同时控制 | `P=90,T=45` |
| 3 | 全扫描 | `SCAN` → 两轴 0°↔180° 往复 |
| 4 | 单轴扫描 | `SCANP` 仅Pan, `SCANT` 仅Tilt |
| 5 | 限位探测 | `LIMIT` → 慢速扫描，帮你找出云台实际机械范围 |
| 6 | 停止/复位 | `STOP` 停扫描, `RST` 回中 (90°,90°) |
| 7 | OLED 显示 | Pan角度+PWM脉宽, Tilt角度+PWM脉宽, 模式, 最近指令 |
| 8 | 角度限幅 | 每轴独立可配，防止撞机械限位 |

---

## 二、硬件接线

```
┌────────────────────────────────────────────────────────────┐
│                     STM32F103C8T6                          │
│                                                            │
│  PB8  ──── OLED SCL  (I2C 软件模拟)                        │
│  PB9  ──── OLED SDA  (I2C 软件模拟)                        │
│                                                            │
│  PA0  ──── 舵机1 信号线 (Pan 水平)  [TIM2_CH1, PWM]       │
│  PA1  ──── 舵机2 信号线 (Tilt 俯仰) [TIM2_CH2, PWM]       │
│                                                            │
│  PA9  ──── USB转TTL RX  [USART1_TX]                       │
│  PA10 ─── USB转TTL TX  [USART1_RX]                        │
│                                                            │
│  GND  ──── OLED GND + 舵机1 GND + 舵机2 GND + USB转TTL GND│
│  3.3V ──── OLED VCC                                        │
│                                                            │
│  ⚠️ 两舵机 VCC (红线) ──── 实验室直流电源 + (6.5V / 5A)   │
│  ⚠️ 两舵机 GND (黑线) ──── 直流电源 - + STM32 GND (共地!) │
└────────────────────────────────────────────────────────────┘
```

---

## 三、CubeIDE 工程配置（CubeMX .ioc 文件）

### 3.1 新建工程

1. **File → New → STM32 Project**
2. 搜索芯片 `STM32F103C8`，选 `STM32F103C8Tx`，点 Next
3. 工程名 `PanTiltGimbal`，使用默认路径

### 3.2 Pinout 配置

进入 `.ioc` 文件的 **Pinout & Configuration** 标签页：

#### System Core → SYS
| 选项 | 值 |
|------|-----|
| Debug | **Serial Wire** |

#### System Core → RCC
| 选项 | 值 |
|------|-----|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator** |

#### Timers → TIM2
| 选项 | 值 |
|------|-----|
| Clock Source | **Internal Clock** |
| Channel1 | **PWM Generation CH1** |
| Channel2 | **PWM Generation CH2** |

配置后 PA0 自动分配为 TIM2_CH1，PA1 自动分配为 TIM2_CH2。

TIM2 参数配置（在下方的 Configuration 面板）：
| 参数 | 值 | 说明 |
|------|-----|------|
| Prescaler (PSC) | **71** | 72MHz / 72 = 1MHz (1 tick = 1μs) |
| Counter Mode | Up | |
| Counter Period (ARR) | **19999** | 20000 tick = 20ms = 50Hz |
| Auto-reload preload | Enable | |
| **CH1/CH2 参数** | | |
| Mode | PWM Mode 1 | |
| Pulse (初始/* ====== 角度限幅（用户根据云台实际机械范围修改）====== */
#define SERVO_PAN_MIN      0    // ← Pan 最小角度
#define SERVO_PAN_MAX      225  // ← Pan 最大角度（写死为 180）
#define SERVO_TILT_MIN     0    // ← Tilt 最小角度
#define SERVO_TILT_MAX     225  // ← Tilt 最大角度) | **1500** | 对应 90° 中位 |
| CH Polarity | High | |

> **验证**：PWM 频率 = 72MHz / 72 / 20000 = **50Hz** ✅

#### Connectivity → USART1
| 选项 | 值 |
|------|-----|
| Mode | **Asynchronous** |

PA9 自动 = TX，PA10 自动 = RX。

USART1 参数配置：
| 参数 | 值 |
|------|-----|
| Baud Rate | **115200** |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| NVIC Settings → USART1 global interrupt | **✅ Enable** |

#### GPIO（OLED 用，手动配置 PB8/PB9）

在 Pinout 视图找到 PB8 和 PB9：

| 引脚 | 配置 |
|------|------|
| PB8 | **GPIO_Output** → 右键 Pinout 图上的 PB8, 或手动设 (OLED SCL) |
| PB9 | **GPIO_Output** → (OLED SDA) |

PB8/PB9 参数（在 GPIO 配置面板）：
| 参数 | 值 |
|------|-----|
| GPIO output level | High |
| GPIO mode | **Output Open Drain** |
| Maximum output speed | High |
| User Label | `OLED_SCL` / `OLED_SDA`（可选，方便代码可读） |

> ⚠️ **关键**：OLED 用软件模拟 I2C，SDA 必须设为 **开漏输出**，这样才能双向通信（虽然本工程只写不读，但 OLED 的应答位需要释放总线）。

### 3.3 Clock Configuration 时钟树

```
HSE: 8 MHz (外部晶振)
PLL Source: HSE
PLL Mul: ×9  → 72MHz
SYSCLK: 72MHz
HCLK:   72MHz
APB1:   36MHz (最大) → TIM2 时钟 = APB1×2 = 72MHz ⚠️
APB2:   72MHz
```

> ⚠️ **重要：APB1 Timer Clocks**：CubeMX 时钟树下方有个 "APB1 Timer Clocks" 显示条，确认是 **72MHz**（因为 APB1 预分频不为 1 时定时器时钟 = APB1×2）。如果显示 36MHz，需要调整 APB1 prescaler 使定时器时钟达到 72MHz，否则 PWM 计算全部要重算。

实际上：APB1 prescaler = /2 时，APB1 = 36MHz，但 TIM2 时钟 = 36MHz × 2 = 72MHz。CubeMX 会自动处理这个。

### 3.4 Project Manager 设置

| 选项 | 值 |
|------|-----|
| Project Name | PanTiltGimbal |
| Toolchain / IDE | STM32CubeIDE |
| ☑️ Generate peripheral initialization as a pair of '.c/.h' files | ✅ 勾选 |
| ☑️ Keep User Code when re-generating | ✅ 勾选（重要！） |

### 3.5 生成代码

按 `Ctrl+S` 保存 `.ioc` → CubeMX 询问 "Generate Code?" → **Yes**。

---

## 四、需要新建/移植的文件

### 4.1 OLED 驱动 — `Core/Inc/OLED.h` + `Core/Src/OLED.c`

将你参考工程中的 OLED 驱动移植过来。**只需改动一处**：

#### OLED.c 中的 GPIO 操作替换

原 SPL 代码：
```c
#define OLED_W_SCL(x)  GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)  GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))
```

改为 HAL 代码：
```c
#define OLED_W_SCL(x)  HAL_GPIO_WritePin(GPIOB, GPIO_Pin_8, (GPIO_PinState)(x))
#define OLED_W_SDA(x)  HAL_GPIO_WritePin(GPIOB, GPIO_Pin_9, (GPIO_PinState)(x))
```

`OLED_I2C_Init()` 中删除 `RCC_APB2PeriphClockCmd()` 调用（HAL 在 `HAL_Init()` 中已处理时钟）。GPIO 初始化也需要改为 HAL 写法：

```c
void OLED_I2C_Init(void)
{
    // HAL 版本：CubeMX 已生成 GPIO 初始化，但我们需要确保是开漏模式。
    // 如果 CubeMX 已配置好 PB8/PB9 → 此函数可留空，或仅做拉高复位。
    // 为安全起见，CubeMX 的 MX_GPIO_Init() 会处理。
    // 这里只做总线复位：
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}
```

> CubeMX 生成的 `MX_GPIO_Init()` 在 `Core/Src/gpio.c` 中会自动初始化 PB8/PB9。

其余 `OLED_WriteCommand()`, `OLED_WriteData()`, `OLED_ShowChar()` 等全部不动。

**需复制的文件清单**：

| 原工程文件 | 放入 CubeIDE |
|------------|-------------|
| `Hardware/OLED.h` | `Core/Inc/OLED.h` |
| `Hardware/OLED.c` | `Core/Src/OLED.c`（改 GPIO 宏 + 删 RCC 使能） |
| `Hardware/OLED_Font.h` | `Core/Inc/OLED_Font.h` |
| `System/Delay.h` | `Core/Inc/Delay.h` |
| `System/Delay.c` | `Core/Src/Delay.c` |

> **Delay.c**：HAL 自带的 `HAL_Delay()` 只提供毫秒延时。你的 `Delay_us()` 用 SysTick 实现，在 HAL 环境仍然可用，但注意 HAL 也用了 SysTick（`uwTick`）。建议保留 `Delay_us()` / `Delay_ms()`，或者直接用 `HAL_Delay()` 替代 `Delay_ms()`，`Delay_us()` 保留原有实现。

### 4.2 舵机驱动 — `Core/Inc/Servo.h` + `Core/Src/Servo.c`

#### Servo.h

```c
#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f1xx_hal.h"

/* ====== 角度限幅（用户根据云台实际机械范围修改）====== */
#define SERVO_PAN_MIN      0
#define SERVO_PAN_MAX      180
#define SERVO_TILT_MIN     0
#define SERVO_TILT_MAX     180

void Servo_SetPanAngle(uint8_t Angle);
void Servo_SetTiltAngle(uint8_t Angle);
uint8_t Servo_GetPanAngle(void);
uint8_t Servo_GetTiltAngle(void);
uint16_t Servo_AngleToPulse(uint8_t Angle);

#endif
```

#### Servo.c — 实现要点

**关键 PWM 计算**：

```
TIM2 时钟 = 72MHz
PSC = 71  →  计数器频率 = 1MHz (1 tick = 1μs)
ARR = 19999  →  周期 = 20000μs = 20ms = 50Hz

LD-1501MG 脉宽 ↔ 角度：
  500μs  → 0°    CCR = 500
  1500μs → 90°   CCR = 1500
  2500μs → 180°  CCR = 2500

线性公式：CCR = 500 + (Angle * 2000) / 180
```

**实现代码**：

```c
#include "Servo.h"

static uint8_t g_PanAngle  = 90;
static uint8_t g_TiltAngle = 90;

extern TIM_HandleTypeDef htim2;  // CubeMX 生成在 tim.c 中

uint16_t Servo_AngleToPulse(uint8_t Angle)
{
    return 500 + (uint32_t)Angle * 2000 / 180;
}

void Servo_SetPanAngle(uint8_t Angle)
{
    if (Angle < SERVO_PAN_MIN) Angle = SERVO_PAN_MIN;
    if (Angle > SERVO_PAN_MAX) Angle = SERVO_PAN_MAX;

    g_PanAngle = Angle;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_AngleToPulse(Angle));
}

void Servo_SetTiltAngle(uint8_t Angle)
{
    if (Angle < SERVO_TILT_MIN) Angle = SERVO_TILT_MIN;
    if (Angle > SERVO_TILT_MAX) Angle = SERVO_TILT_MAX;

    g_TiltAngle = Angle;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Servo_AngleToPulse(Angle));
}

uint8_t Servo_GetPanAngle(void)  { return g_PanAngle; }
uint8_t Servo_GetTiltAngle(void) { return g_TiltAngle; }
```

> ⚠️ 启动 PWM 输出需要调用 `HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)` 和 `HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2)`。这个调用放在 `main.c` 的初始化区域（`USER CODE BEGIN 2`）。

### 4.3 串口指令解析 — `Core/Inc/Serial.h` + `Core/Src/Serial.c`

#### Serial.h

```c
#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f1xx_hal.h"

void Serial_Init(void);
void Serial_SendString(char *str);
void Serial_SendNum(uint32_t num);
void Serial_ProcessByte(uint8_t byte);   // 由 HAL UART 回调调用

/* 解析结果（供 main.c 消费） */
extern volatile uint8_t  g_CommandReady;
extern volatile uint8_t  g_ParsedPanAngle;
extern volatile uint8_t  g_ParsedTiltAngle;
extern volatile uint8_t  g_ScanMode;     // 0=停止 1=全扫 2=Pan扫 3=Tilt扫 4=限位
extern char   g_RawCommand[32];

#endif
```

#### Serial.c — 实现要点

**接收方式**：使用 HAL 的 `HAL_UART_Receive_IT()` + 回调函数 `HAL_UART_RxCpltCallback()`。每次接收一个字节，收到 `\r` 或 `\n` 时触发行解析。

```c
#include "Serial.h"
#include <string.h>
#include "Servo.h"   // 用于 SERVO_PAN_MIN/MAX 等宏

/* 全局变量定义 */
volatile uint8_t g_CommandReady = 0;
volatile uint8_t g_ParsedPanAngle  = 0;
volatile uint8_t g_ParsedTiltAngle = 0;
volatile uint8_t g_ScanMode = 0;
char   g_RawCommand[32] = {0};

extern UART_HandleTypeDef huart1;  // CubeMX 生成

static char   g_RxLine[32];
static uint8_t g_RxLineIdx = 0;
static uint8_t g_RxByte;  // HAL_UART_Receive_IT 的目标缓冲区

/* ====== 初始化：启动接收中断 ====== */
void Serial_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &g_RxByte, 1);
}

/* ====== 发送 API ====== */
void Serial_SendString(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

void Serial_SendNum(uint32_t num)
{
    char buf[12];
    uint8_t i = 0;
    if (num == 0) { buf[i++] = '0'; }
    else {
        while (num) { buf[i++] = (num % 10) + '0'; num /= 10; }
        // 反转
        for (uint8_t j = 0; j < i / 2; j++) {
            char t = buf[j]; buf[j] = buf[i-1-j]; buf[i-1-j] = t;
        }
    }
    buf[i] = '\0';
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, i, 100);
}

/* ====== 字节处理（在 main.c 的 HAL_UART_RxCpltCallback 中调用） ====== */
void Serial_ProcessByte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (g_RxLineIdx > 0)
        {
            g_RxLine[g_RxLineIdx] = '\0';
            ParseCommand(g_RxLine);
            g_RxLineIdx = 0;
        }
    }
    else if (g_RxLineIdx < sizeof(g_RxLine) - 1)
    {
        g_RxLine[g_RxLineIdx++] = byte;
    }
    // 重新启动下一次接收
    HAL_UART_Receive_IT(&huart1, &g_RxByte, 1);
}

/* ====== 指令解析 ====== */
static void ParseCommand(char *cmd)
{
    strncpy(g_RawCommand, cmd, sizeof(g_RawCommand) - 1);
    g_RawCommand[sizeof(g_RawCommand) - 1] = '\0';

    /* 关键词指令 */
    if      (strcmp(cmd, "SCAN")  == 0) { g_ScanMode = 1; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "SCANP") == 0) { g_ScanMode = 2; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "SCANT") == 0) { g_ScanMode = 3; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "LIMIT") == 0) { g_ScanMode = 4; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "STOP")  == 0) { g_ScanMode = 0; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "RST")   == 0) { g_ScanMode = 0; g_ParsedPanAngle = 90; g_ParsedTiltAngle = 90; g_CommandReady = 2; return; }
    // g_CommandReady = 2 表示 RESET 指令

    /* 数值指令 P=xxx 和/或 T=xxx */
    int panVal = -1, tiltVal = -1;
    char *p = cmd;
    while (*p)
    {
        if (strncmp(p, "P=", 2) == 0) {
            p += 2; panVal = 0;
            while (*p >= '0' && *p <= '9') { panVal = panVal * 10 + (*p - '0'); p++; }
            if      (panVal < SERVO_PAN_MIN)  panVal = SERVO_PAN_MIN;
            else if (panVal > SERVO_PAN_MAX)  panVal = SERVO_PAN_MAX;
        }
        else if (strncmp(p, "T=", 2) == 0) {
            p += 2; tiltVal = 0;
            while (*p >= '0' && *p <= '9') { tiltVal = tiltVal * 10 + (*p - '0'); p++; }
            if      (tiltVal < SERVO_TILT_MIN) tiltVal = SERVO_TILT_MIN;
            else if (tiltVal > SERVO_TILT_MAX) tiltVal = SERVO_TILT_MAX;
        }
        else { p++; }
    }

    if (panVal >= 0 || tiltVal >= 0)
    {
        g_ParsedPanAngle  = (panVal  >= 0) ? (uint8_t)panVal  : 0x00;  // 0x00 = "不变"
        g_ParsedTiltAngle = (tiltVal >= 0) ? (uint8_t)tiltVal : 0x00;
        g_CommandReady = 1;
    }
}
```

---

## 五、main.c 主程序

CubeMX 生成的 `Core/Src/main.c` 中有大量的 `/* USER CODE BEGIN X */` / `/* USER CODE END X */` 注释块。我们在对应位置填入代码。

### 5.1 头文件包含区 — `/* USER CODE BEGIN Includes */`

```c
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "Delay.h"
#include "Servo.h"
#include "Serial.h"
#include <string.h>
/* USER CODE END Includes */
```

### 5.2 全局变量区 — `/* USER CODE BEGIN PV */`

```c
/* USER CODE BEGIN PV */
uint8_t panAngle    = 90;     // 当前 Pan 角度
uint8_t tiltAngle   = 90;     // 当前 Tilt 角度
uint8_t scanRunning = 0;      // 0=停止 1=全扫 2=Pan扫 3=Tilt扫 4=限位
uint8_t scanPanDir  = 0;      // 0=递增 1=递减
uint8_t scanTiltDir = 0;
/* USER CODE END PV */
```

### 5.3 初始化区 — `/* USER CODE BEGIN 2 */`

```c
/* USER CODE BEGIN 2 */

/* OLED 初始化 */
OLED_Init();
OLED_Clear();

/* 舵机 PWM 启动（TIM2 时基已在 MX_TIM2_Init() 配置，但未启动 PWM 输出） */
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // PA0 → Pan
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   // PA1 → Tilt
Servo_SetPanAngle(90);
Servo_SetTiltAngle(90);

/* 串口初始化 */
Serial_Init();

/* OLED 开机画面 */
OLED_ShowString(1, 1, "Pan-Tilt Gimbal");
OLED_ShowString(2, 1, "LD-1501MG x2");
OLED_ShowString(3, 1, "Init done!     ");
OLED_ShowString(4, 1, "UART 115200bps");
HAL_Delay(1500);

/* 串口开机信息 */
Serial_SendString("\r\n========================================\r\n");
Serial_SendString("  2-Axis Pan-Tilt Gimbal Controller\r\n");
Serial_SendString("  LD-1501MG x2  |  STM32F103C8T6\r\n");
Serial_SendString("========================================\r\n");
Serial_SendString("Commands:\r\n");
Serial_SendString("  P=0~180      Pan angle\r\n");
Serial_SendString("  T=0~180      Tilt angle\r\n");
Serial_SendString("  P=90,T=45    Both axes\r\n");
Serial_SendString("  SCAN         Scan all axes\r\n");
Serial_SendString("  SCANP/SCANT  Scan single axis\r\n");
Serial_SendString("  LIMIT        Slow sweep (find limits)\r\n");
Serial_SendString("  STOP / RST   Stop / Reset to 90deg\r\n");
Serial_SendString("========================================\r\n\r\n");

/* USER CODE END 2 */
```

### 5.4 主循环 — `/* USER CODE BEGIN WHILE */`

```c
/* USER CODE BEGIN WHILE */
while (1)
{
    /* ===== 指令处理 ===== */
    if (g_CommandReady)
    {
        uint8_t cmdType = g_CommandReady;   // 1=普通指令 2=RESET
        g_CommandReady = 0;

        if (cmdType == 2)  /* RESET */
        {
            scanRunning = 0;
            panAngle  = 90;
            tiltAngle = 90;
            Servo_SetPanAngle(panAngle);
            Servo_SetTiltAngle(tiltAngle);
            Serial_SendString("OK RESET -> (P=90, T=90)\r\n");
        }
        else if (g_ScanMode > 0)  /* 扫描类指令 */
        {
            scanRunning = g_ScanMode;
            g_ScanMode  = 0;
            scanPanDir  = 0;
            scanTiltDir = 0;

            const char *modeName[] = {"", "SCAN ALL", "SCAN PAN", "SCAN TILT", "LIMIT DETECT"};
            Serial_SendString("OK ");
            Serial_SendString(modeName[scanRunning]);
            Serial_SendString("\r\n");
        }
        else  /* 角度设置指令 */
        {
            // g_ParsedPanAngle / g_ParsedTiltAngle 中，0x00 特殊表示"不改变"
            // 注意：角度=0° 是合法值，但这里假设 0x00 确实是不变标记。
            // 更健壮的方案见下方说明↓
            if (g_ParsedPanAngle > 0 || (g_ParsedPanAngle == 0 && strstr(g_RawCommand, "P=0") != NULL))
            {
                panAngle = g_ParsedPanAngle;
                Servo_SetPanAngle(panAngle);
            }
            if (g_ParsedTiltAngle > 0 || (g_ParsedTiltAngle == 0 && strstr(g_RawCommand, "T=0") != NULL))
            {
                tiltAngle = g_ParsedTiltAngle;
                Servo_SetTiltAngle(tiltAngle);
            }
            scanRunning = 0;

            Serial_SendString("OK P=");
            Serial_SendNum(panAngle);
            Serial_SendString(", T=");
            Serial_SendNum(tiltAngle);
            Serial_SendString("\r\n");

            g_ParsedPanAngle  = 0;
            g_ParsedTiltAngle = 0;
        }
    }

    /* ===== 扫描步进（每轮 20ms） ===== */
    if (scanRunning)
    {
        uint8_t step = (scanRunning == 4) ? 1 : 2;  // LIMIT 模式慢速

        if (scanRunning == 1 || scanRunning == 2 || scanRunning == 4)
        {
            if (scanPanDir == 0) {
                panAngle += step;
                if (panAngle >= SERVO_PAN_MAX) { panAngle = SERVO_PAN_MAX; scanPanDir = 1; }
            } else {
                panAngle -= step;
                if (panAngle <= SERVO_PAN_MIN) { panAngle = SERVO_PAN_MIN; scanPanDir = 0; }
            }
            Servo_SetPanAngle(panAngle);
        }

        if (scanRunning == 1 || scanRunning == 3 || scanRunning == 4)
        {
            if (scanTiltDir == 0) {
                tiltAngle += step;
                if (tiltAngle >= SERVO_TILT_MAX) { tiltAngle = SERVO_TILT_MAX; scanTiltDir = 1; }
            } else {
                tiltAngle -= step;
                if (tiltAngle <= SERVO_TILT_MIN) { tiltAngle = SERVO_TILT_MIN; scanTiltDir = 0; }
            }
            Servo_SetTiltAngle(tiltAngle);
        }
    }

    /* ===== OLED 刷新（4行，每轮刷新） ===== */
    uint16_t panPulse  = Servo_AngleToPulse(panAngle);
    uint16_t tiltPulse = Servo_AngleToPulse(tiltAngle);

    /* 第1行: Pan */
    OLED_ShowString(1, 1, "P:");
    OLED_ShowNum(1, 3, panAngle, 3);
    OLED_ShowString(1, 6, "deg ");
    OLED_ShowNum(1, 10, panPulse, 4);
    OLED_ShowString(1, 15, "us");

    /* 第2行: Tilt */
    OLED_ShowString(2, 1, "T:");
    OLED_ShowNum(2, 3, tiltAngle, 3);
    OLED_ShowString(2, 6, "deg ");
    OLED_ShowNum(2, 10, tiltPulse, 4);
    OLED_ShowString(2, 15, "us");

    /* 第3行: 模式 + 扫描方向 */
    OLED_ShowString(3, 1, "M:");
    if (scanRunning == 0)      OLED_ShowString(3, 3, "MANUAL   ");
    else if (scanRunning == 1) OLED_ShowString(3, 3, "SCAN-ALL ");
    else if (scanRunning == 2) OLED_ShowString(3, 3, "SCAN-PAN ");
    else if (scanRunning == 3) OLED_ShowString(3, 3, "SCAN-TILT");
    else if (scanRunning == 4) OLED_ShowString(3, 3, "LIMIT    ");

    if (scanRunning) {
        OLED_ShowChar(3, 13, scanPanDir  ? '<' : '>');
        OLED_ShowChar(3, 15, scanTiltDir ? 'v' : '^');
    } else {
        OLED_ShowString(3, 13, "  ");
    }

    /* 第4行: 最近指令（超长截断） */
    OLED_ShowString(4, 1, "                ");   // 清空
    char dispCmd[17];
    strncpy(dispCmd, g_RawCommand, 16);
    dispCmd[16] = '\0';
    OLED_ShowString(4, 1, dispCmd);

    HAL_Delay(20);  // 50Hz 主循环
    /* USER CODE END WHILE */
}
```

> ⚠️ **关于 0x00 判定的说明**：如果 Pan=0° 是合法角度，用 0x00 做"不变"标记会和 P=0 冲突。Agent 实现时建议用独立标志位（如 `uint8_t g_PanUpdated; uint8_t g_TiltUpdated;`）来区分"有P指令"和"无P指令"，比用角度值判断更健壮。

### 5.5 UART 回调函数 — `/* USER CODE BEGIN 4 */`

```c
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Serial_ProcessByte(g_RxByte);  // 见 Serial.c
    }
}
/* USER CODE END 4 */
```

> 注意：`HAL_UART_RxCpltCallback` 是 HAL 定义的弱函数（weak），CubeMX 生成在 `stm32f1xx_hal_uart.c` 中有 `__weak` 版本。我们在 main.c 中重写它即可覆盖。

---

## 六、文件清单与工程结构

最终 CubeIDE 工程结构：

```
PanTiltGimbal/
├── PanTiltGimbal.ioc          ← CubeMX 配置文件
├── Core/
│   ├── Inc/
│   │   ├── main.h             ← 自动生成
│   │   ├── stm32f1xx_it.h     ← 自动生成
│   │   ├── gpio.h             ← 自动生成
│   │   ├── tim.h              ← 自动生成
│   │   ├── usart.h            ← 自动生成
│   │   ├── OLED.h             ← 新建/移植
│   │   ├── OLED_Font.h        ← 移植
│   │   ├── Delay.h            ← 移植
│   │   ├── Servo.h            ← 新建
│   │   └── Serial.h           ← 新建
│   └── Src/
│       ├── main.c             ← 部分重写（只在 USER CODE 区域填代码）
│       ├── stm32f1xx_it.c     ← 自动生成（无需手动改 USART1 IRQ）
│       ├── gpio.c             ← 自动生成（含 PB8/PB9 OLED 引脚初始化）
│       ├── tim.c              ← 自动生成（含 TIM2 PWM 配置）
│       ├── usart.c            ← 自动生成（含 USART1 配置）
│       ├── OLED.c             ← 移植（改 GPIO 宏）
│       ├── Delay.c            ← 移植
│       ├── Servo.c            ← 新建
│       └── Serial.c           ← 新建
└── Drivers/
    └── STM32F1xx_HAL_Driver/  ← HAL 库（自动）
```

### Agent 需要生成/修改的文件清单

| # | 文件 | 操作 | 要点 |
|---|------|------|------|
| 1 | `PanTiltGimbal.ioc` | **说明配置** | 见第三节，Agent 只需说明，用户自己点 CubeMX |
| 2 | `Core/Inc/OLED.h` | **移植** | 原样复制，或改头文件 include 为 `<stm32f1xx_hal.h>` |
| 3 | `Core/Src/OLED.c` | **移植+改** | GPIO 宏改为 `HAL_GPIO_WritePin`，`OLED_I2C_Init()` 简化 |
| 4 | `Core/Inc/OLED_Font.h` | **移植** | 原样复制 |
| 5 | `Core/Inc/Delay.h` | **移植** | 原样复制 |
| 6 | `Core/Src/Delay.c` | **移植** | 原样复制（`Delay_ms` 可保留或用 `HAL_Delay` 替代） |
| 7 | `Core/Inc/Servo.h` | **新建** | 见 4.2 节 |
| 8 | `Core/Src/Servo.c` | **新建** | 见 4.2 节 |
| 9 | `Core/Inc/Serial.h` | **新建** | 见 4.3 节 |
| 10 | `Core/Src/Serial.c` | **新建** | 见 4.3 节 |
| 11 | `Core/Src/main.c` | **部分填充** | 只写 USER CODE 区域，见第五节 |

---

## 七、使用指南

### 7.1 首次上电测试

```
1. 舵机先不接电源！只连 STM32 + OLED + USB转TTL
2. CubeIDE 编译下载 → 确认 OLED 显示 "Pan-Tilt Gimbal" → "Init done!"
3. PuTTY 开串口 115200 → 应能看到开机信息
4. 发送 RST → 收到 "OK RESET -> (P=90, T=90)"
5. 关闭 STM32 电源
6. 接舵机电源 6.5V，共地，接信号线
7. 重新上电 → 舵机应回中
8. 发 P=45 → Pan 转 45°
9. 发 T=120 → Tilt 转 120°
10. 发 SCAN → 两轴扫描
11. 发 STOP → 停止
```

### 7.2 限位探测

```
1. 发送 LIMIT → 慢速扫描（1°/20ms = 50°/s）
2. 盯着云台，听声音
3. 某轴到机械极限时会有滋滋声（堵转）
4. 立刻发 STOP
5. 记下 OLED 上的角度值 → 改 Servo.h 中的 MAX/MIN
6. 重新编译下载
```

### 7.3 PuTTY 配置

```
Connection type: Serial
Serial line:     COM?（设备管理器查）
Speed:           115200

Terminal:
  Local echo:         Force on
  Local line editing: Force on
```

---

## 八、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| OLED 不亮 | I2C 接线, GPIO 未初始化为开漏 | 检查 CubeMX PB8/PB9 是否设为 Output Open Drain |
| 舵机不动 | PWM 未启动, 电源未开, 未共地 | 确认 `HAL_TIM_PWM_Start()` 已调用, 检查共地 |
| 舵机抖动 | 供电电流不足 | 提高实验室电源电流限制到 5A |
| 舵机发热严重 | 机械限位 > 配置限位 | 立即断电, 用 LIMIT 模式测实际范围 |
| 串口无响应 | TX/RX 接反, 波特率不匹配 | 交换 PA9/PA10 试试, 确认 115200 |
| `__HAL_TIM_SET_COMPARE` 无效 | TIM2 时钟配置有问题 | 检查 CubeMX 时钟树, APB1 Timer Clock 应为 72MHz |

---

*文档版本：v3.0 | CubeIDE + HAL | STM32F103C8T6 + LD-1501MG ×2 + SSD1306 OLED | 2026-07-16*
