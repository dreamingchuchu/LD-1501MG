#ifndef __SQUAREDRAW_H
#define __SQUAREDRAW_H
#include "stm32f1xx_hal.h"

#define SQUARE_DEFAULT_SIZE      20
#define ARM_LENGTH_DEFAULT       10.0f
#define MOUNT_DISTANCE_DEFAULT   20.0f
#define CORNER_DELAY_MS          100

typedef enum {
    SQUARE_STATE_IDLE = 0,
    SQUARE_STATE_MOVING_TO_START = 1,
    SQUARE_STATE_MOVING_EDGE = 2,
    SQUARE_STATE_ERROR = 3
} SquareDrawStateEnum;

typedef struct {
    float pan_angle;
    float tilt_angle;
    float x_cm;
    float y_cm;
} CornerPoint_t;

typedef struct {
    SquareDrawStateEnum state;
    uint8_t current_corner;
    uint8_t current_edge;
    float origin_pan;
    float origin_tilt;
    uint16_t width_cm;
    uint16_t height_cm;
    CornerPoint_t corners[4];
    uint32_t start_time_ms;
} SquareDrawState_t;

typedef struct {
    float arm_length_cm;
    float mount_distance_cm;
} PhysicalConfig_t;

void SquareDraw_Init(void);
int8_t SquareDraw_Start(uint16_t width_cm, uint16_t height_cm);
void SquareDraw_Stop(void);
void SquareDraw_Execute(void);

float Config_GetArmLength(void);
float Config_GetMountDistance(void);
int8_t Config_SetArmLength(float arm_length_cm);
int8_t Config_SetMountDistance(float mount_distance_cm);

int8_t Coord_ValidateAngle(float angle_deg);
int8_t Coord_ConvertCmToAngle(float distance_cm, float mount_distance_cm, float current_angle_deg, float *pTarget_angle_deg);

extern volatile SquareDrawState_t g_SquareDrawState;
extern PhysicalConfig_t g_PhysicalConfig;
extern volatile uint8_t g_SquareMode;

#endif