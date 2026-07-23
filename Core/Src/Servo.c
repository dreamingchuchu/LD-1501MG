#include "Servo.h"

// 角度存储: 0.1° 单位 (913 = 91.3°)
static uint16_t g_PanAngle10  = 900;   // 90.0°
static uint16_t g_TiltAngle10 = 900;   // 90.0°

extern TIM_HandleTypeDef htim2;

// Pan: 500~2500us → 0.0~180.0°
// pulse = 500 + (angle10/10) * 2000 / 180 = 500 + angle10 * 2000 / 1800
uint16_t Servo_Angle10ToPulse_Pan(uint16_t Angle10)
{
    return 500 + (uint32_t)Angle10 * 2000 / 1800;
}

// Tilt: 544~1574us → 0.0~90.0°
// pulse = 544 + (angle10/10) * 1030 / 90 = 544 + angle10 * 1030 / 900
uint16_t Servo_Angle10ToPulse_Tilt(uint16_t Angle10)
{
    return 544 + (uint32_t)Angle10 * 1030 / 900;
}

void Servo_SetPanAngle10(uint16_t Angle10)
{
    if (Angle10 < SERVO_PAN_MIN) Angle10 = SERVO_PAN_MIN;
    if (Angle10 > SERVO_PAN_MAX) Angle10 = SERVO_PAN_MAX;

    g_PanAngle10 = Angle10;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_Angle10ToPulse_Pan(Angle10));
}

void Servo_SetTiltAngle10(uint16_t Angle10)
{
    if (Angle10 < SERVO_TILT_MIN) Angle10 = SERVO_TILT_MIN;
    if (Angle10 > SERVO_TILT_MAX) Angle10 = SERVO_TILT_MAX;

    g_TiltAngle10 = Angle10;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Servo_Angle10ToPulse_Tilt(Angle10));
}

uint16_t Servo_GetPanAngle10(void)  { return g_PanAngle10; }
uint16_t Servo_GetTiltAngle10(void) { return g_TiltAngle10; }

// delta 也是 0.1° 单位
void Servo_AdjustPan10(int16_t Delta10)
{
    int32_t newAngle = (int32_t)g_PanAngle10 + Delta10;

    if (newAngle < SERVO_PAN_MIN)  newAngle = SERVO_PAN_MIN;
    if (newAngle > SERVO_PAN_MAX)  newAngle = SERVO_PAN_MAX;

    Servo_SetPanAngle10((uint16_t)newAngle);
}

void Servo_AdjustTilt10(int16_t Delta10)
{
    int32_t newAngle = (int32_t)g_TiltAngle10 + Delta10;

    if (newAngle < SERVO_TILT_MIN) newAngle = SERVO_TILT_MIN;
    if (newAngle > SERVO_TILT_MAX) newAngle = SERVO_TILT_MAX;

    Servo_SetTiltAngle10((uint16_t)newAngle);
}
