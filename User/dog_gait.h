#ifndef __DOG_GAIT_H
#define __DOG_GAIT_H

#include <stdint.h>

typedef enum
{
    DOG_GAIT_FOOT_BASE_STAND = 0,
    DOG_GAIT_FOOT_BASE_WALK,
    DOG_GAIT_FOOT_BASE_TURN,
    DOG_GAIT_FOOT_BASE_SHIFT_LEFT,
    DOG_GAIT_FOOT_BASE_SHIFT_RIGHT,
} DogGaitFootBase_t;

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

void DogGait_Init(void);
void DogGait_SetLoadMode(DogGaitLoadMode_t mode);
void DogGait_SetTrotParams(float step_height_mm, float step_length_mm, float speed_freq);
void DogGait_SetTrackParams(float step_height_mm,
                            float left_forward_step_mm,
                            float right_forward_step_mm,
                            float steer_step_mm,
                            float speed_freq);
void DogGait_SetStepInPlaceParams(float step_height_mm, float speed_freq);
void DogGait_SetShiftLeftParams(float step_height_mm, float *step_length_mm, float speed_freq,DogGaitFootBase_t base);
void DogGait_SetShiftRightParams(float step_height_mm, float *step_length_mm, float speed_freq,DogGaitFootBase_t base);
void DogGait_SetShiftStepR(const float r[4]);
void DogGait_UpdateShift(uint16_t time_ms,DogGaitFootBase_t base);
void DogGait_SetTurnLeftParams(float step_height_mm, float turn_step_mm, float speed_freq);
void DogGait_SetTurnRightParams(float step_height_mm, float turn_step_mm, float speed_freq);
void DogGait_ResetWalk(void);
void DogGait_SetWalkParams(float step_height_mm,
                           float step_length_mm,
                           float speed_freq,
                           float cg_base_x_mm,
                           float imu_gain_mm);
void DogGait_UpdateWalk(uint16_t time_ms, float pitch_deg, float roll_deg);
uint8_t DogGait_IsWalkCycleDone(void);
uint8_t DogGait_IsWalkLeftFrontPreSwing(void);
void DogGait_StartWalkSupportPhase(void);
void DogGait_ResumeWalkFromSupportPhase(void);
void DogGait_UpdateWalkSupport(uint16_t time_ms);
uint8_t DogGait_IsWalkSupportReady(void);
uint8_t DogGait_GetWalkFrontRearAverageReach(float *front_reach_mm,
                                              float *rear_reach_mm);
void DogGait_GotoStandPose(uint16_t time_ms);
void DogGait_UpdateTrot(uint16_t time_ms);
void DogGait_AllStand(uint16_t time_ms);
#endif
