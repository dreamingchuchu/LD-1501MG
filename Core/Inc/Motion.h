#ifndef __MOTION_H
#define __MOTION_H
#include "stm32f1xx_hal.h"

typedef struct {
    float current_pan;
    float current_tilt;
    float target_pan;
    float target_tilt;
    uint8_t is_busy;
    float speed;
} MotionState_t;

void Motion_Init(void);
int8_t Motion_MoveTo(float target_pan, float target_tilt);
void Motion_Update(void);
uint8_t Motion_IsBusy(void);
void Motion_Stop(void);
int8_t Motion_SetSpeed(float speed);
float Motion_GetSpeed(void);

extern MotionState_t g_MotionState;

#endif