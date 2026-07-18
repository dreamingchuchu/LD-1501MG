#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f1xx_hal.h"

#define SERVO_PAN_MIN      0
#define SERVO_PAN_MAX      180
#define SERVO_TILT_MIN     0
#define SERVO_TILT_MAX     180

void Servo_SetPanAngle(uint16_t Angle);
void Servo_SetTiltAngle(uint8_t Angle);
uint16_t Servo_GetPanAngle(void);
uint8_t Servo_GetTiltAngle(void);
uint16_t Servo_AngleToPulse_Pan(uint16_t Angle);
uint16_t Servo_AngleToPulse_Tilt(uint8_t Angle);

#endif