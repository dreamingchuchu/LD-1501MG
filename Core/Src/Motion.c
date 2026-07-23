#include "Motion.h"
#include "Servo.h"
#include <math.h>

MotionState_t g_MotionState;

void Motion_Init(void)
{
    g_MotionState.current_pan = (float)Servo_GetPanAngle10() / 10.0f;
    g_MotionState.current_tilt = (float)Servo_GetTiltAngle10() / 10.0f;
    g_MotionState.target_pan = g_MotionState.current_pan;
    g_MotionState.target_tilt = g_MotionState.current_tilt;
    g_MotionState.is_busy = 0;
    g_MotionState.speed = 1.0f;
}

int8_t Motion_MoveTo(float target_pan, float target_tilt)
{
    if (g_MotionState.is_busy) {
        return -1;
    }
    
    if (target_pan < 0.0f) target_pan = 0.0f;
    if (target_pan > 180.0f) target_pan = 180.0f;
    if (target_tilt < 0.0f) target_tilt = 0.0f;
    if (target_tilt > 180.0f) target_tilt = 180.0f;
    
    g_MotionState.target_pan = target_pan;
    g_MotionState.target_tilt = target_tilt;
    g_MotionState.is_busy = 1;
    
    return 0;
}

void Motion_Update(void)
{
    if (!g_MotionState.is_busy) {
        return;
    }
    
    float delta_pan = g_MotionState.target_pan - g_MotionState.current_pan;
    float delta_tilt = g_MotionState.target_tilt - g_MotionState.current_tilt;
    
    if (fabsf(delta_pan) < 1.0f && fabsf(delta_tilt) < 1.0f) {
        g_MotionState.current_pan = g_MotionState.target_pan;
        g_MotionState.current_tilt = g_MotionState.target_tilt;
        g_MotionState.is_busy = 0;
        return;
    }
    
    float step_pan = 0.0f;
    float step_tilt = 0.0f;
    
    if (delta_pan > 0.0f) {
        step_pan = g_MotionState.speed;
        if (step_pan > delta_pan) step_pan = delta_pan;
    } else if (delta_pan < 0.0f) {
        step_pan = -g_MotionState.speed;
        if (step_pan < delta_pan) step_pan = delta_pan;
    }
    
    if (delta_tilt > 0.0f) {
        step_tilt = g_MotionState.speed;
        if (step_tilt > delta_tilt) step_tilt = delta_tilt;
    } else if (delta_tilt < 0.0f) {
        step_tilt = -g_MotionState.speed;
        if (step_tilt < delta_tilt) step_tilt = delta_tilt;
    }
    
    g_MotionState.current_pan += step_pan;
    g_MotionState.current_tilt += step_tilt;
    
    if (g_MotionState.current_pan < 0.0f) g_MotionState.current_pan = 0.0f;
    if (g_MotionState.current_pan > 180.0f) g_MotionState.current_pan = 180.0f;
    if (g_MotionState.current_tilt < 0.0f) g_MotionState.current_tilt = 0.0f;
    if (g_MotionState.current_tilt > 180.0f) g_MotionState.current_tilt = 180.0f;
    
    Servo_SetPanAngle10((uint16_t)(g_MotionState.current_pan * 10.0f));
    Servo_SetTiltAngle10((uint16_t)(g_MotionState.current_tilt * 10.0f));
}

uint8_t Motion_IsBusy(void)
{
    return g_MotionState.is_busy;
}

void Motion_Stop(void)
{
    g_MotionState.is_busy = 0;
}

int8_t Motion_SetSpeed(float speed)
{
    if (speed < 0.1f || speed > 5.0f) {
        return -1;
    }
    
    g_MotionState.speed = speed;
    return 0;
}

float Motion_GetSpeed(void)
{
    return g_MotionState.speed;
}