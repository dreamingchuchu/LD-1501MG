#include "Serial.h"
#include <string.h>
#include "Servo.h"
#include "SquareDraw.h"

volatile uint8_t  g_CommandReady = 0;
volatile uint16_t g_ParsedPanAngle10  = 0xFFFF;   // 0.1°单位, 0xFFFF=未设置
volatile uint16_t g_ParsedTiltAngle10 = 0xFFFF;   // 0.1°单位
volatile uint8_t  g_ScanMode = 0;
volatile uint8_t  g_CalibMode = 0;
volatile uint16_t g_CalibValue = 0;
char   g_RawCommand[32] = {0};

volatile uint16_t g_SquareSize_cm = 0;
volatile uint8_t  g_ConfigParamType = 0;
volatile uint16_t g_ConfigParamValue = 0;

extern UART_HandleTypeDef huart1;

static char   g_RxLine[32];
static uint8_t g_RxLineIdx = 0;
uint8_t g_RxByte;

void Serial_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &g_RxByte, 1);
}

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
        for (uint8_t j = 0; j < i / 2; j++) {
            char t = buf[j]; buf[j] = buf[i-1-j]; buf[i-1-j] = t;
        }
    }
    buf[i] = '\0';
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, i, 100);
}

// 解析 "91.3" → 913 (0.1°单位)
static uint16_t ParseAngle10(char **pp)
{
    char *p = *pp;
    uint16_t integer = 0;
    uint8_t  decimal = 0;

    // 整数部分
    while (*p >= '0' && *p <= '9') {
        integer = integer * 10 + (*p - '0');
        p++;
    }
    // 小数部分: 取两位, 第二位用于四舍五入
    if (*p == '.') {
        p++;
        if (*p >= '0' && *p <= '9') {
            decimal = (*p - '0');
            p++;
        }
        if (*p >= '0' && *p <= '9') {
            uint8_t decimal2 = (*p - '0');
            p++;
            if (decimal2 >= 5) {
                decimal++;
                if (decimal >= 10) { decimal = 0; integer++; }
            }
        }
    }

    *pp = p;
    return integer * 10 + decimal;
}

static void ParseCommand(char *cmd)
{
    strncpy(g_RawCommand, cmd, sizeof(g_RawCommand) - 1);
    g_RawCommand[sizeof(g_RawCommand) - 1] = '\0';

    if (g_SquareMode == 1) {
        if (strcmp(cmd, "STOP") == 0) {
            g_SquareMode = 0;
            SquareDraw_Stop();
            g_CommandReady = 1;
            return;
        } else {
            Serial_SendString("错误：正在绘制正方形，只响应STOP命令\r\n");
            return;
        }
    }

    if (strncmp(cmd, "TRACK,", 6) == 0) {
        int delta_pan = 0, delta_tilt = 0;
        char *p = cmd + 6;

        int sign = 1;
        if (*p == '-') { sign = -1; p++; }
        while (*p >= '0' && *p <= '9') { delta_pan = delta_pan * 10 + (*p - '0'); p++; }
        delta_pan *= sign;

        if (*p == ',') {
            p++;
            sign = 1;
            if (*p == '-') { sign = -1; p++; }
            while (*p >= '0' && *p <= '9') { delta_tilt = delta_tilt * 10 + (*p - '0'); p++; }
            delta_tilt *= sign;
        }

        if (delta_pan < -100) delta_pan = -100;
        if (delta_pan > 100) delta_pan = 100;
        if (delta_tilt < -100) delta_tilt = -100;
        if (delta_tilt > 100) delta_tilt = 100;

        // TRACK delta 是整数度, 转换为 ×10
        Servo_AdjustPan10((int16_t)delta_pan * 10);
        Servo_AdjustTilt10((int16_t)delta_tilt * 10);

        Serial_SendString("OK TRACK\r\n");
        return;
    }

    if      (strcmp(cmd, "SCAN")  == 0) { g_ScanMode = 1; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "STOP")  == 0) { g_ScanMode = 0; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "RST")   == 0) { g_ScanMode = 0; g_ParsedPanAngle10 = 900; g_ParsedTiltAngle10 = 900; g_CommandReady = 2; return; }
    else if (strcmp(cmd, "INFO")  == 0) { g_CommandReady = 3; return; }

    else if (strcmp(cmd, "CP")  == 0) { g_CalibMode = 1; g_CommandReady = 4; return; }
    else if (strncmp(cmd, "CP+", 3) == 0) {
        g_CalibMode = 2;
        g_CalibValue = 0;
        char *p = cmd + 3;
        while (*p >= '0' && *p <= '9') { g_CalibValue = g_CalibValue * 10 + (*p - '0'); p++; }
        if (g_CalibValue == 0) g_CalibValue = 5;
        g_CommandReady = 4;
        return;
    }
    else if (strncmp(cmd, "CP-", 3) == 0) {
        g_CalibMode = 3;
        g_CalibValue = 0;
        char *p = cmd + 3;
        while (*p >= '0' && *p <= '9') { g_CalibValue = g_CalibValue * 10 + (*p - '0'); p++; }
        if (g_CalibValue == 0) g_CalibValue = 5;
        g_CommandReady = 4;
        return;
    }
    else if (strcmp(cmd, "CT")  == 0) { g_CalibMode = 4; g_CommandReady = 4; return; }
    else if (strncmp(cmd, "CT+", 3) == 0) {
        g_CalibMode = 5;
        g_CalibValue = 0;
        char *p = cmd + 3;
        while (*p >= '0' && *p <= '9') { g_CalibValue = g_CalibValue * 10 + (*p - '0'); p++; }
        if (g_CalibValue == 0) g_CalibValue = 5;
        g_CommandReady = 4;
        return;
    }
    else if (strncmp(cmd, "CT-", 3) == 0) {
        g_CalibMode = 6;
        g_CalibValue = 0;
        char *p = cmd + 3;
        while (*p >= '0' && *p <= '9') { g_CalibValue = g_CalibValue * 10 + (*p - '0'); p++; }
        if (g_CalibValue == 0) g_CalibValue = 5;
        g_CommandReady = 4;
        return;
    }

    else if (strncmp(cmd, "SQUARE", 6) == 0) {
        g_SquareSize_cm = SQUARE_DEFAULT_SIZE;
        char *p = cmd + 6;
        while (*p == ' ') p++;

        if (*p >= '0' && *p <= '9') {
            uint16_t size = 0;
            while (*p >= '0' && *p <= '9') {
                size = size * 10 + (*p - '0');
                p++;
            }
            if (size < 1 || size > 100) {
                Serial_SendString("错误：尺寸超出范围（1~100cm）\r\n");
                return;
            }
            g_SquareSize_cm = size;
        } else if (*p != '\0') {
            Serial_SendString("错误：参数格式无效，请输入正整数（单位：cm）\r\n");
            return;
        }

        g_CommandReady = 5;
        return;
    }

    else if (strncmp(cmd, "CONFIG", 6) == 0) {
        char *p = cmd + 6;
        while (*p == ' ') p++;

        if (strncmp(p, "ARM", 3) == 0) {
            p += 3;
            while (*p == ' ') p++;

            uint16_t value = 0;
            while (*p >= '0' && *p <= '9') {
                value = value * 10 + (*p - '0');
                p++;
            }

            if (value < 1 || value > 50) {
                Serial_SendString("错误：舵机臂长超出范围（1~50cm）\r\n");
                return;
            }

            g_ConfigParamType = 1;
            g_ConfigParamValue = value;
            g_CommandReady = 6;
            return;
        }
        else if (strncmp(p, "DIST", 4) == 0) {
            p += 4;
            while (*p == ' ') p++;

            uint16_t value = 0;
            while (*p >= '0' && *p <= '9') {
                value = value * 10 + (*p - '0');
                p++;
            }

            if (value < 10 || value > 200) {
                Serial_SendString("错误：安装距离超出范围（10~200cm）\r\n");
                return;
            }

            g_ConfigParamType = 2;
            g_ConfigParamValue = value;
            g_CommandReady = 6;
            return;
        }
        else if (strncmp(p, "SHOW", 4) == 0) {
            g_ConfigParamType = 3;
            g_CommandReady = 6;
            return;
        }
        else {
            Serial_SendString("错误：CONFIG命令格式无效\r\n");
            Serial_SendString("用法：CONFIG ARM n | CONFIG DIST n | CONFIG SHOW\r\n");
            return;
        }
    }

    // ─── P= / T= 解析 (支持小数: P=91.3 → 913) ───
    int16_t panVal10  = -1;
    int16_t tiltVal10 = -1;
    char *p = cmd;
    while (*p)
    {
        if (strncmp(p, "P=", 2) == 0) {
            p += 2;
            panVal10 = (int16_t)ParseAngle10(&p);
            if      (panVal10 < SERVO_PAN_MIN)  panVal10 = SERVO_PAN_MIN;
            else if (panVal10 > SERVO_PAN_MAX)  panVal10 = SERVO_PAN_MAX;
        }
        else if (strncmp(p, "T=", 2) == 0) {
            p += 2;
            tiltVal10 = (int16_t)ParseAngle10(&p);
            if      (tiltVal10 < SERVO_TILT_MIN) tiltVal10 = SERVO_TILT_MIN;
            else if (tiltVal10 > SERVO_TILT_MAX) tiltVal10 = SERVO_TILT_MAX;
        }
        else { p++; }
    }

    if (panVal10 >= 0 || tiltVal10 >= 0)
    {
        // ★ 只更新有值的轴, 不覆盖另一个轴 (防止P=和T=连续发送时互相清空)
        if (panVal10  >= 0) g_ParsedPanAngle10  = (uint16_t)panVal10;
        if (tiltVal10 >= 0) g_ParsedTiltAngle10 = (uint16_t)tiltVal10;
        g_CommandReady = 1;
    }
}

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
    HAL_UART_Receive_IT(&huart1, &g_RxByte, 1);
}
