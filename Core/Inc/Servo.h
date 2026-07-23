#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f1xx_hal.h"

// 角度范围 (0.1°单位, 913=91.3°)
#define SERVO_PAN_MIN      0
#define SERVO_PAN_MAX      1800    // 180.0°
#define SERVO_TILT_MIN     0
#define SERVO_TILT_MAX     900     // 90.0°

// 角度统一用 0.1° 单位
uint16_t Servo_Angle10ToPulse_Pan(uint16_t Angle10);
uint16_t Servo_Angle10ToPulse_Tilt(uint16_t Angle10);

void Servo_SetPanAngle10(uint16_t Angle10);
void Servo_SetTiltAngle10(uint16_t Angle10);

uint16_t Servo_GetPanAngle10(void);
uint16_t Servo_GetTiltAngle10(void);

void Servo_AdjustPan10(int16_t Delta10);
void Servo_AdjustTilt10(int16_t Delta10);

#endif
