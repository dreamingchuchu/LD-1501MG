#include "SquareDraw.h"
#include "Servo.h"
#include "Serial.h"
#include "Motion.h"
#include <math.h>

volatile SquareDrawState_t g_SquareDrawState;
PhysicalConfig_t g_PhysicalConfig = {ARM_LENGTH_DEFAULT, MOUNT_DISTANCE_DEFAULT};
volatile uint8_t g_SquareMode = 0;

extern volatile uint8_t g_ScanMode;

float Config_GetArmLength(void)
{
    return g_PhysicalConfig.arm_length_cm;
}

float Config_GetMountDistance(void)
{
    return g_PhysicalConfig.mount_distance_cm;
}

int8_t Config_SetArmLength(float arm_length_cm)
{
    if (arm_length_cm < 1.0f || arm_length_cm > 50.0f) {
        return -1;
    }
    g_PhysicalConfig.arm_length_cm = arm_length_cm;
    return 0;
}

int8_t Config_SetMountDistance(float mount_distance_cm)
{
    if (mount_distance_cm < 10.0f || mount_distance_cm > 200.0f) {
        return -1;
    }
    g_PhysicalConfig.mount_distance_cm = mount_distance_cm;
    return 0;
}

int8_t Coord_ValidateAngle(float angle_deg)
{
    if (angle_deg < 0.0f || angle_deg > 180.0f) {
        return 0;
    }
    return 1;
}

int8_t Coord_ConvertCmToAngle(float distance_cm, float mount_distance_cm, float current_angle_deg, float *pTarget_angle_deg)
{
    if (mount_distance_cm <= 0.0f) {
        return -1;
    }
    
    float delta_angle = atanf(distance_cm / mount_distance_cm) * (180.0f / 3.14159265f);
    float target_angle = current_angle_deg + delta_angle;
    
    if (!Coord_ValidateAngle(target_angle)) {
        return -1;
    }
    
    *pTarget_angle_deg = target_angle;
    return 0;
}

void SquareDraw_Init(void)
{
    g_SquareDrawState.state = SQUARE_STATE_IDLE;
    g_SquareDrawState.current_corner = 0;
    g_SquareDrawState.current_edge = 0;
    g_SquareMode = 0;
}

static int8_t SquareDraw_CalculateCorners(uint16_t width_cm, uint16_t height_cm)
{
    float origin_pan = (float)Servo_GetPanAngle10() / 10.0f;
    float origin_tilt = (float)Servo_GetTiltAngle10() / 10.0f;
    float mount_distance = Config_GetMountDistance();
    
    g_SquareDrawState.origin_pan = origin_pan;
    g_SquareDrawState.origin_tilt = origin_tilt;
    g_SquareDrawState.width_cm = width_cm;
    g_SquareDrawState.height_cm = height_cm;
    
    g_SquareDrawState.corners[0].x_cm = 0.0f;
    g_SquareDrawState.corners[0].y_cm = 0.0f;
    g_SquareDrawState.corners[0].pan_angle = origin_pan;
    g_SquareDrawState.corners[0].tilt_angle = origin_tilt;
    
    float target_pan;
    if (Coord_ConvertCmToAngle((float)width_cm, mount_distance, origin_pan, &target_pan) != 0) {
        return -2;
    }
    g_SquareDrawState.corners[1].x_cm = (float)width_cm;
    g_SquareDrawState.corners[1].y_cm = 0.0f;
    g_SquareDrawState.corners[1].pan_angle = target_pan;
    g_SquareDrawState.corners[1].tilt_angle = origin_tilt;
    
    float target_tilt;
    if (Coord_ConvertCmToAngle((float)height_cm, mount_distance, origin_tilt, &target_tilt) != 0) {
        return -2;
    }
    g_SquareDrawState.corners[2].x_cm = (float)width_cm;
    g_SquareDrawState.corners[2].y_cm = (float)height_cm;
    g_SquareDrawState.corners[2].pan_angle = target_pan;
    g_SquareDrawState.corners[2].tilt_angle = target_tilt;
    
    g_SquareDrawState.corners[3].x_cm = 0.0f;
    g_SquareDrawState.corners[3].y_cm = (float)height_cm;
    g_SquareDrawState.corners[3].pan_angle = origin_pan;
    g_SquareDrawState.corners[3].tilt_angle = target_tilt;
    
    return 0;
}


int8_t SquareDraw_Start(uint16_t width_cm, uint16_t height_cm)
{
    if (width_cm == 0 || height_cm == 0) {
        return -1;
    }
    
    if (g_ScanMode > 0) {
        return -3;
    }
    
    int8_t calc_result = SquareDraw_CalculateCorners(width_cm, height_cm);
    if (calc_result != 0) {
        return calc_result;
    }
    
    g_SquareDrawState.state = SQUARE_STATE_MOVING_TO_START;
    g_SquareDrawState.current_corner = 0;
    g_SquareDrawState.current_edge = 0;
    g_SquareMode = 1;
    g_SquareDrawState.start_time_ms = HAL_GetTick();
    
    Motion_MoveTo(g_SquareDrawState.corners[0].pan_angle, g_SquareDrawState.corners[0].tilt_angle);
    
    Serial_SendString("开始绘制正方形：尺寸");
    Serial_SendNum(width_cm);
    Serial_SendString("cm×");
    Serial_SendNum(height_cm);
    Serial_SendString("cm，原点(");
    Serial_SendNum((uint32_t)g_SquareDrawState.origin_pan);
    Serial_SendString(",");
    Serial_SendNum((uint32_t)g_SquareDrawState.origin_tilt);
    Serial_SendString(")，安装距离");
    Serial_SendNum((uint32_t)Config_GetMountDistance());
    Serial_SendString("cm\r\n");
    
    return 0;
}

void SquareDraw_Stop(void)
{
    Motion_Stop();
    g_SquareDrawState.state = SQUARE_STATE_IDLE;
    g_SquareMode = 0;
    
    uint16_t current_pan = Servo_GetPanAngle10() / 10;
    uint16_t current_tilt = Servo_GetTiltAngle10() / 10;
    
    Serial_SendString("绘制已停止，当前位置：Pan=");
    Serial_SendNum(current_pan);
    Serial_SendString(", Tilt=");
    Serial_SendNum(current_tilt);
    Serial_SendString("\r\n");
}

void SquareDraw_Execute(void)
{
    if (g_SquareDrawState.state == SQUARE_STATE_MOVING_TO_START) {
        if (!Motion_IsBusy()) {
            Serial_SendString("到达起点，开始绘制第一条边\r\n");
            g_SquareDrawState.state = SQUARE_STATE_MOVING_EDGE;
            g_SquareDrawState.current_edge = 0;
            Motion_MoveTo(g_SquareDrawState.corners[1].pan_angle, g_SquareDrawState.corners[1].tilt_angle);
        }
    }
    else if (g_SquareDrawState.state == SQUARE_STATE_MOVING_EDGE) {
        if (!Motion_IsBusy()) {
            uint8_t next_corner = g_SquareDrawState.current_edge + 1;
            Serial_SendString("到达角点");
            Serial_SendNum(next_corner + 1);
            Serial_SendString("：Pan=");
            Serial_SendNum((uint32_t)g_SquareDrawState.corners[next_corner].pan_angle);
            Serial_SendString(", Tilt=");
            Serial_SendNum((uint32_t)g_SquareDrawState.corners[next_corner].tilt_angle);
            Serial_SendString("\r\n");
            
            if (g_SquareDrawState.current_edge < 3) {
                g_SquareDrawState.current_edge++;
                uint8_t next_corner_idx = g_SquareDrawState.current_edge + 1;
                if (next_corner_idx > 3) next_corner_idx = 0;
                Motion_MoveTo(g_SquareDrawState.corners[next_corner_idx].pan_angle, g_SquareDrawState.corners[next_corner_idx].tilt_angle);
            } else {
                g_SquareDrawState.state = SQUARE_STATE_IDLE;
                g_SquareMode = 0;
                
                uint32_t elapsed_ms = HAL_GetTick() - g_SquareDrawState.start_time_ms;
                uint32_t elapsed_sec = elapsed_ms / 1000;
                
                Serial_SendString("绘制完成，耗时");
                Serial_SendNum(elapsed_sec);
                Serial_SendString("秒，实际尺寸");
                Serial_SendNum(g_SquareDrawState.width_cm);
                Serial_SendString("cm×");
                Serial_SendNum(g_SquareDrawState.height_cm);
                Serial_SendString("cm\r\n");
            }
        }
    }
}