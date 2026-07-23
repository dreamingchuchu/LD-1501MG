#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f1xx_hal.h"

void Serial_Init(void);
void Serial_SendString(char *str);
void Serial_SendNum(uint32_t num);
void Serial_ProcessByte(uint8_t byte);

extern volatile uint8_t  g_CommandReady;
extern volatile uint16_t g_ParsedPanAngle10;   // 0.1°单位, 913=91.3°
extern volatile uint16_t g_ParsedTiltAngle10;  // 0.1°单位
extern volatile uint8_t  g_ScanMode;
extern volatile uint8_t  g_CalibMode;
extern volatile uint16_t g_CalibValue;
extern char   g_RawCommand[32];
extern uint8_t g_RxByte;

extern volatile uint16_t g_SquareSize_cm;
extern volatile uint8_t  g_ConfigParamType;
extern volatile uint16_t g_ConfigParamValue;

#endif
