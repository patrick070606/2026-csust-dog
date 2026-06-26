#include "dog_gait.h"
#include "dog_servo.h"
#include <math.h>

/*
 * 基础步态参数。
 * 腿序：左前、右前、左后、右后。
 */
#define DOG_GAIT_PI                        3.14159265358979323846f
#define DOG_GAIT_DEFAULT_H_MM              15.0f
#define DOG_GAIT_DEFAULT_R_MM              15.0f
#define DOG_GAIT_DEFAULT_L1_MM             100.0f
#define DOG_GAIT_DEFAULT_L2_MM             100.0f
#define DOG_GAIT_DEFAULT_SPEED_FREQ        0.20f
#define DOG_GAIT_ZERO_FOOT_X_MM            100.0f
#define DOG_GAIT_ZERO_FOOT_Y_MM            100.0f

/*
 * 足端基准坐标策略。
 * 统一 stand/walk/turn 的基准来源，切换步态时足端位置更连续。
 *
 * 0: 该状态复用 stand 的基准坐标。
 * 1: 该状态使用独立的基准坐标。
 */
#define DOG_GAIT_STAND_FOOT_BASE_ENABLE    1U
#define DOG_GAIT_WALK_FOOT_BASE_ENABLE     0U
#define DOG_GAIT_TURN_FOOT_BASE_ENABLE     0U

/* stand 基准坐标，x 偏移用于调整有负荷/无负荷时的重心。 */
#define DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM -50.0f
#define DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM    -50.0f
#define DOG_GAIT_STAND_FOOT_Y_MM                (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)

#if (DOG_GAIT_WALK_FOOT_BASE_ENABLE != 0U)
#define DOG_GAIT_WALK_FOOT_X_OFFSET_NO_LOAD_MM  -50.0f
#define DOG_GAIT_WALK_FOOT_X_OFFSET_LOAD_MM     -50.0f
#define DOG_GAIT_WALK_FOOT_Y_MM                 (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)
#else
#define DOG_GAIT_WALK_FOOT_X_OFFSET_NO_LOAD_MM  DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM
#define DOG_GAIT_WALK_FOOT_X_OFFSET_LOAD_MM     DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM
#define DOG_GAIT_WALK_FOOT_Y_MM                 DOG_GAIT_STAND_FOOT_Y_MM
#endif

#if (DOG_GAIT_TURN_FOOT_BASE_ENABLE != 0U)
#define DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM  -50.0f
#define DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM     -50.0f
#define DOG_GAIT_TURN_FOOT_Y_MM                 (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)
#else
#define DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM  DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM
#define DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM     DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM
#define DOG_GAIT_TURN_FOOT_Y_MM                 DOG_GAIT_STAND_FOOT_Y_MM
#endif

#define DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG         180.0f
#define DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG        180.0f

typedef enum
{
    DOG_GAIT_LEG_LF = 0,
    DOG_GAIT_LEG_RF,
    DOG_GAIT_LEG_LB,
    DOG_GAIT_LEG_RB,
    DOG_GAIT_LEG_COUNT,
} DogGaitLeg_t;

typedef enum
{
    DOG_GAIT_FOOT_BASE_STAND = 0,
    DOG_GAIT_FOOT_BASE_WALK,
    DOG_GAIT_FOOT_BASE_TURN,
} DogGaitFootBase_t;

typedef struct
{
    float x;
    float y;
} DogGaitFootBaseCoord_t;

static DogGaitInfo_t s_gait[DOG_GAIT_LEG_COUNT];
static float s_trot_phase;
static float s_trot_speed_freq = DOG_GAIT_DEFAULT_SPEED_FREQ;
static DogGaitLoadMode_t s_load_mode = DOG_GAIT_LOAD_WITH_PAYLOAD;
static DogGaitFootBase_t s_foot_base = DOG_GAIT_FOOT_BASE_STAND;
static uint8_t s_is_initialized;

static float DogGait_ClampFloat(float value, float min_value, float max_value)
{
    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
}

static void DogGait_InitLeg(DogGaitInfo_t *gait, float h, float r, float l1, float l2, float bias_angle)
{
    gait->h = h;
    gait->r = r;
    gait->old_r = r;
    gait->l1 = l1;
    gait->l2 = l2;
    gait->bias_angle = bias_angle;
    gait->bias_hip = 0.0f;
    gait->bias_knee = 0.0f;
    gait->x = 0.0f;
    gait->y = 0.0f;
    gait->hip_angle = 0.0f;
    gait->knee_angle = 0.0f;
}

static DogGaitFootBaseCoord_t DogGait_GetFootBaseCoord(DogGaitFootBase_t base)
{
    DogGaitFootBaseCoord_t coord;

    switch (base)
    {
    case DOG_GAIT_FOOT_BASE_WALK:
        coord.x = (s_load_mode == DOG_GAIT_LOAD_WITH_PAYLOAD) ?
                  DOG_GAIT_WALK_FOOT_X_OFFSET_LOAD_MM :
                  DOG_GAIT_WALK_FOOT_X_OFFSET_NO_LOAD_MM;
        coord.y = DOG_GAIT_WALK_FOOT_Y_MM;
        break;

    case DOG_GAIT_FOOT_BASE_TURN:
        coord.x = (s_load_mode == DOG_GAIT_LOAD_WITH_PAYLOAD) ?
                  DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM :
                  DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM;
        coord.y = DOG_GAIT_TURN_FOOT_Y_MM;
        break;

    case DOG_GAIT_FOOT_BASE_STAND:
    default:
        coord.x = (s_load_mode == DOG_GAIT_LOAD_WITH_PAYLOAD) ?
                  DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM :
                  DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM;
        coord.y = DOG_GAIT_STAND_FOOT_Y_MM;
        break;
    }

    return coord;
}

/*
 * 根据摆线方程获取足端坐标.
 * t: 当前运动时间t.
 * h: 足端坐标最大高度.
 * r：足端运动最远距离 (r > 0 正方向 r < 0 反方向).
 * x: 输出足端x轴坐标.
 * y: 输出足端y轴坐标.
*/
static void DogGait_GetPosByCycloidalEquation(float bias_angle_deg,
                                              float t,
                                              float h,
                                              float r,
                                              float *x,
                                              float *y)
{
    float th = (2.0f * DOG_GAIT_PI * t) / 0.5f;
    float raw_x;
    float raw_y;
    float angle_rad;

    if (r >= 0.0f)
    {
        raw_x = ((th - sinf(th)) / (2.0f * DOG_GAIT_PI)) * r;
    }
    else
    {
        raw_x = (1.0f - ((th - sinf(th)) / (2.0f * DOG_GAIT_PI))) * (-r);
    }

    raw_y = h * (1.0f - cosf(th)) * 0.5f;
    angle_rad = bias_angle_deg * DOG_GAIT_PI / 180.0f;

    *x = raw_x * cosf(angle_rad) + raw_y * sinf(angle_rad);
    *y = raw_y * cosf(angle_rad) - raw_x * sinf(angle_rad);
}

static void DogGait_CalcAbsoluteAngleByPos(float x,
                                           float y,
                                           float l1,
                                           float l2,
                                           float *hip_angle,
                                           float *knee_angle)
{
    float dy = l1 + l2 - y;
    float ll2 = x * x + dy * dy;
    float ll = sqrtf(ll2);
    float hip_base;
    float hip_link;
    float knee_link;

    if (ll < 0.001f)
    {
        *hip_angle = 0.0f;
        *knee_angle = 0.0f;
        return;
    }

    hip_base = atan2f(x, dy);
    hip_link = acosf(DogGait_ClampFloat((ll2 + l1 * l1 - l2 * l2) / (2.0f * l1 * ll), -1.0f, 1.0f));
    knee_link = acosf(DogGait_ClampFloat((l1 * l1 + l2 * l2 - ll2) / (2.0f * l1 * l2), -1.0f, 1.0f));

    *hip_angle = (hip_base - hip_link) * 180.0f / DOG_GAIT_PI;
    *knee_angle = (DOG_GAIT_PI - knee_link) * 180.0f / DOG_GAIT_PI;
}

/*
相比于绝对角度，计算相对于站立姿势的关节角度偏移，方便在小跑步态中叠加在站立姿势的基础上进行调整。
*/
static void DogGait_CalcAngleByPos(float x,
                                   float y,
                                   float l1,
                                   float l2,
                                   float *hip_angle,
                                   float *knee_angle)
{
    float zero_hip;
    float zero_knee;
    float target_hip;
    float target_knee;

    DogGait_CalcAbsoluteAngleByPos(DOG_GAIT_ZERO_FOOT_X_MM,
                                   DOG_GAIT_ZERO_FOOT_Y_MM,
                                   l1,
                                   l2,
                                   &zero_hip,
                                   &zero_knee);
    DogGait_CalcAbsoluteAngleByPos(x, y, l1, l2, &target_hip, &target_knee);

    *hip_angle = target_hip - zero_hip;
    *knee_angle = target_knee - zero_knee;
}

static void DogGait_SetStandFootPos(void)
{
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(DOG_GAIT_FOOT_BASE_STAND);

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].x = base_coord.x;
        s_gait[i].y = base_coord.y;
    }
}

static void DogGait_ClearLegBiases(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].bias_hip = 0.0f;
        s_gait[i].bias_knee = 0.0f;
    }
}

static void DogGait_ApplySideSteps(float step_height_mm,
                                   float left_r,
                                   float right_r,
                                   float speed_freq,
                                   DogGaitFootBase_t base)
{
    s_trot_speed_freq = speed_freq;
    s_foot_base = base;
    DogGait_ClearLegBiases();

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].h = step_height_mm;
    }

    s_gait[DOG_GAIT_LEG_LF].r = left_r;
    s_gait[DOG_GAIT_LEG_LB].r = left_r;
    s_gait[DOG_GAIT_LEG_RF].r = right_r;
    s_gait[DOG_GAIT_LEG_RB].r = right_r;

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].old_r = s_gait[i].r;
    }
}

static void DogGait_UpdateLegAngles(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        DogGait_CalcAngleByPos(s_gait[i].x,
                               s_gait[i].y,
                               s_gait[i].l1,
                               s_gait[i].l2,
                               &s_gait[i].hip_angle,
                               &s_gait[i].knee_angle);
    }
}

/**
 * 填充舵机角度
 * @param angles 舵机角度数组
 */
static void DogGait_FillServoAngles(float angles[DOG_SERVO_COUNT]) // 根据当前步态信息计算每条腿的关节角度，并填充到舵机角度数组中，准备发送给舵机控制器。
{
    angles[DOG_SERVO_LF_HIP] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_LF].hip_angle + s_gait[DOG_GAIT_LEG_LF].bias_hip,
                                                  -DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG,
                                                  DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG);
    angles[DOG_SERVO_LF_KNEE] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_LF].knee_angle + s_gait[DOG_GAIT_LEG_LF].bias_knee,
                                                   -DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG,
                                                   DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG);

    angles[DOG_SERVO_RF_HIP] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_RF].hip_angle + s_gait[DOG_GAIT_LEG_RF].bias_hip,
                                                  -DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG,
                                                  DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG);
    angles[DOG_SERVO_RF_KNEE] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_RF].knee_angle + s_gait[DOG_GAIT_LEG_RF].bias_knee,
                                                   -DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG,
                                                   DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG);

    angles[DOG_SERVO_LB_HIP] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_LB].hip_angle + s_gait[DOG_GAIT_LEG_LB].bias_hip,
                                                  -DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG,
                                                  DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG);
    angles[DOG_SERVO_LB_KNEE] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_LB].knee_angle + s_gait[DOG_GAIT_LEG_LB].bias_knee,
                                                   -DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG,
                                                   DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG);

    angles[DOG_SERVO_RB_HIP] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_RB].hip_angle + s_gait[DOG_GAIT_LEG_RB].bias_hip,
                                                  -DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG,
                                                  DOG_GAIT_MAX_HIP_TEST_ANGLE_DEG);
    angles[DOG_SERVO_RB_KNEE] = DogGait_ClampFloat(s_gait[DOG_GAIT_LEG_RB].knee_angle + s_gait[DOG_GAIT_LEG_RB].bias_knee,
                                                   -DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG,
                                                   DOG_GAIT_MAX_KNEE_TEST_ANGLE_DEG);
}

void DogGait_SetTrotParams(float step_height_mm, float step_length_mm, float speed_freq) // 设置小跑步态的参数，step_height_mm 是步高，step_length_mm 是步长，speed_freq 是速度频率，单位是每次步态更新时相位增加的值，过大可能导致步态不够流畅，需要根据实际情况调整。
{
    float safe_h = DogGait_ClampFloat(step_height_mm, 0.0f, 60.0f);
    float safe_r = DogGait_ClampFloat(step_length_mm, -60.0f, 60.0f);
    float safe_speed = DogGait_ClampFloat(speed_freq, 0.0f, 0.4f);

    DogGait_ApplySideSteps(safe_h, safe_r, safe_r, safe_speed, DOG_GAIT_FOOT_BASE_WALK);
}

void DogGait_SetLoadMode(DogGaitLoadMode_t mode)
{
    if (mode == DOG_GAIT_LOAD_WITH_PAYLOAD)
    {
        s_load_mode = DOG_GAIT_LOAD_WITH_PAYLOAD;
    }
    else
    {
        s_load_mode = DOG_GAIT_LOAD_NONE;
    }
}

void DogGait_SetTrackParams(float step_height_mm,
                            float left_forward_step_mm,
                            float right_forward_step_mm,
                            float steer_step_mm,
                            float speed_freq) // 设置循迹步态的参数，step_height_mm 是步高，forward_step_mm 是前进时的步长，steer_step_mm 是转向时内侧腿的步长调整值，speed_freq 是速度频率，单位是每次步态更新时相位增加的值，过大可能导致步态不够流畅，需要根据实际情况调整。
{
    float safe_h = DogGait_ClampFloat(step_height_mm, 0.0f, 80.0f);
    float safe_left_forward = DogGait_ClampFloat(left_forward_step_mm, 0.0f, 80.0f);
    float safe_right_forward = DogGait_ClampFloat(right_forward_step_mm, 0.0f, 80.0f);
    float safe_steer = DogGait_ClampFloat(steer_step_mm, -20.0f, 20.0f);
    float left_r = DogGait_ClampFloat(safe_left_forward + safe_steer, -80.0f, 80.0f);
    float right_r = DogGait_ClampFloat(safe_right_forward - safe_steer, -80.0f, 80.0f);
    float safe_speed = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(safe_h, left_r, right_r, safe_speed, DOG_GAIT_FOOT_BASE_WALK);
}

void DogGait_SetTurnLeftParams(float step_height_mm, float turn_step_mm, float speed_freq) // 设置转弯步态的参数，step_height_mm 是步高，turn_step_mm 是转弯时内侧腿的步长，speed_freq 是速度频率，单位是每次步态更新时相位增加的值，过大可能导致步态不够流畅，需要根据实际情况调整。
{
    float safe_h = DogGait_ClampFloat(step_height_mm, 0.0f, 60.0f);
    float safe_r = DogGait_ClampFloat(turn_step_mm, 0.0f, 60.0f);
    float safe_speed = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(safe_h, -safe_r, safe_r, safe_speed, DOG_GAIT_FOOT_BASE_TURN);
}

void DogGait_SetTurnRightParams(float step_height_mm, float turn_step_mm, float speed_freq)
{
    float safe_h = DogGait_ClampFloat(step_height_mm, 0.0f, 60.0f);
    float safe_r = DogGait_ClampFloat(turn_step_mm, 0.0f, 60.0f);
    float safe_speed = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(safe_h, safe_r, -safe_r, safe_speed, DOG_GAIT_FOOT_BASE_TURN);
}

void DogGait_Init(void)
{
    DogGait_InitLeg(&s_gait[DOG_GAIT_LEG_LF], DOG_GAIT_DEFAULT_H_MM, DOG_GAIT_DEFAULT_R_MM, DOG_GAIT_DEFAULT_L1_MM, DOG_GAIT_DEFAULT_L2_MM, 0.0f);
    DogGait_InitLeg(&s_gait[DOG_GAIT_LEG_RF], DOG_GAIT_DEFAULT_H_MM, DOG_GAIT_DEFAULT_R_MM, DOG_GAIT_DEFAULT_L1_MM, DOG_GAIT_DEFAULT_L2_MM, 0.0f);
    DogGait_InitLeg(&s_gait[DOG_GAIT_LEG_LB], DOG_GAIT_DEFAULT_H_MM, DOG_GAIT_DEFAULT_R_MM, DOG_GAIT_DEFAULT_L1_MM, DOG_GAIT_DEFAULT_L2_MM, 0.0f);
    DogGait_InitLeg(&s_gait[DOG_GAIT_LEG_RB], DOG_GAIT_DEFAULT_H_MM, DOG_GAIT_DEFAULT_R_MM, DOG_GAIT_DEFAULT_L1_MM, DOG_GAIT_DEFAULT_L2_MM, 0.0f);

    s_trot_phase = 0.0f;
    s_foot_base = DOG_GAIT_FOOT_BASE_STAND;
    s_trot_speed_freq = DOG_GAIT_DEFAULT_SPEED_FREQ;
    s_is_initialized = 1U;
}

void DogGait_GotoStandPose(uint16_t time_ms)
{
    float angles[DOG_SERVO_COUNT] = {0.0f};

    if (s_is_initialized == 0U)
    {
        DogGait_Init();
    }

    s_trot_phase = 0.0f;
    s_foot_base = DOG_GAIT_FOOT_BASE_STAND;
    DogGait_ClearLegBiases();
    DogGait_SetStandFootPos();
    DogGait_UpdateLegAngles();
    DogGait_FillServoAngles(angles);
    DogServo_SetAngles(angles, time_ms);
}

void DogGait_UpdateTrot(uint16_t time_ms)
{
    float angles[DOG_SERVO_COUNT] = {0.0f};
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(s_foot_base);
    float dx;
    float lift;

    if (s_is_initialized == 0U)
    {
        DogGait_Init();
    }

    if (s_trot_phase <= 0.5f)
    {
        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_LF].h, s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_RB].h, s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift;
    }
    else
    {
        float phase = s_trot_phase - 0.5f;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, phase, s_gait[DOG_GAIT_LEG_RF].h, s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, phase, s_gait[DOG_GAIT_LEG_LB].h, s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift;
    }

    DogGait_UpdateLegAngles();
    DogGait_FillServoAngles(angles);
    DogServo_SetAngles(angles, time_ms);

    s_trot_phase += s_trot_speed_freq;
    if (s_trot_phase >= 1.0f)
    {
        s_trot_phase -= 1.0f;
    }
}

void DogGait_AllStand(uint16_t time_ms)
{
    DogGait_GotoStandPose(time_ms);
}
