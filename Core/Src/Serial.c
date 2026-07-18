#include "Serial.h"
#include <string.h>
#include "Servo.h"

volatile uint8_t g_CommandReady = 0;
volatile uint16_t g_ParsedPanAngle  = 0;
volatile uint8_t g_ParsedTiltAngle = 0;
volatile uint8_t g_ScanMode = 0;
volatile uint8_t g_CalibMode = 0;
volatile uint16_t g_CalibValue = 0;
char   g_RawCommand[32] = {0};

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

static void ParseCommand(char *cmd)
{
    strncpy(g_RawCommand, cmd, sizeof(g_RawCommand) - 1);
    g_RawCommand[sizeof(g_RawCommand) - 1] = '\0';

    if      (strcmp(cmd, "SCAN")  == 0) { g_ScanMode = 1; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "STOP")  == 0) { g_ScanMode = 0; g_CommandReady = 1; return; }
    else if (strcmp(cmd, "RST")   == 0) { g_ScanMode = 0; g_ParsedPanAngle = 90; g_ParsedTiltAngle = 90; g_CommandReady = 2; return; }
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
        g_ParsedPanAngle  = (panVal  >= 0) ? (uint16_t)panVal  : 0x00;
        g_ParsedTiltAngle = (tiltVal >= 0) ? (uint8_t)tiltVal : 0x00;
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