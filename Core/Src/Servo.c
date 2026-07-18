#include "Servo.h"

static uint16_t g_PanAngle  = 90;
static uint8_t g_TiltAngle = 90;

extern TIM_HandleTypeDef htim2;

uint16_t Servo_AngleToPulse_Pan(uint16_t Angle)
{
    return 500 + (uint32_t)Angle * 2000 / 180;
}

uint16_t Servo_AngleToPulse_Tilt(uint8_t Angle)
{
    return 544 + (uint32_t)Angle * 1030 / 90;
}

void Servo_SetPanAngle(uint16_t Angle)
{
    if (Angle < SERVO_PAN_MIN) Angle = SERVO_PAN_MIN;
    if (Angle > SERVO_PAN_MAX) Angle = SERVO_PAN_MAX;

    g_PanAngle = Angle;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_AngleToPulse_Pan(Angle));
}

void Servo_SetTiltAngle(uint8_t Angle)
{
    if (Angle < SERVO_TILT_MIN) Angle = SERVO_TILT_MIN;
    if (Angle > SERVO_TILT_MAX) Angle = SERVO_TILT_MAX;

    g_TiltAngle = Angle;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Servo_AngleToPulse_Tilt(Angle));
}

uint16_t Servo_GetPanAngle(void)  { return g_PanAngle; }
uint8_t Servo_GetTiltAngle(void) { return g_TiltAngle; }