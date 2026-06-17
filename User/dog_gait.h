#ifndef __DOG_GAIT_H
#define __DOG_GAIT_H

#include <stdint.h>

typedef struct
{
    float h;
    float r;
    float old_r;
    float l1;
    float l2;
    float bias_angle;
    float bias_hip;
    float bias_knee;
    float x;
    float y;
    float hip_angle;
    float knee_angle;
} DogGaitInfo_t;

typedef enum
{
    DOG_GAIT_LOAD_NONE = 0,
    DOG_GAIT_LOAD_WITH_PAYLOAD,
} DogGaitLoadMode_t;

#define DOG_GAIT_STAIR_LEG_COUNT 4U

typedef enum
{
    DOG_GAIT_STAIR_LEG_LF = 0,
    DOG_GAIT_STAIR_LEG_RF,
    DOG_GAIT_STAIR_LEG_LB,
    DOG_GAIT_STAIR_LEG_RB,
} DogGaitStairLeg_t;

typedef enum
{
    DOG_GAIT_STAIR_BASE_STAND = 0,
    DOG_GAIT_STAIR_BASE_WALK,
} DogGaitStairBase_t;

typedef struct
{
    float x_offset_mm;
    float y_offset_mm;
    float hip_bias_deg;
} DogGaitStairTarget_t;

void DogGait_Init(void);
void DogGait_SetLoadMode(DogGaitLoadMode_t mode);
void DogGait_SetTrotParams(float step_height_mm, float step_length_mm, float speed_freq);
void DogGait_SetTrackParams(float step_height_mm,
                            float left_forward_step_mm,
                            float right_forward_step_mm,
                            float steer_step_mm,
                            float speed_freq);
void DogGait_SetTurnLeftParams(float step_height_mm, float turn_step_mm, float speed_freq);
void DogGait_SetTurnRightParams(float step_height_mm, float turn_step_mm, float speed_freq);
void DogGait_GotoStandPose(uint16_t time_ms);
void DogGait_UpdateTrot(uint16_t time_ms);
void DogGait_SetStairPose(const DogGaitStairTarget_t targets[DOG_GAIT_STAIR_LEG_COUNT],
                          uint16_t time_ms);
void DogGait_SetStairPoseWithBase(const DogGaitStairTarget_t targets[DOG_GAIT_STAIR_LEG_COUNT],
                                  DogGaitStairBase_t base,
                                  uint16_t time_ms);
void DogGait_AllStand(uint16_t time_ms);

#endif
