/*
 * dog_gait 模块结构概览：
 * 1. 基础参数：默认步高/步长、连杆长度、站立足端基准、角度限幅。
 * 2. 状态数据：四条腿的足端坐标、关节角、步态相位、负载模式和足端基准模式。
 * 3. 内部计算：摆线足端轨迹、足端坐标到髋/膝角的逆运动学、舵机角度填充。
 * 4. 对外接口：初始化、站立、小跑、循迹、左右转参数设置，以及周期性步态更新。
 *
 * 基本流程：设置步态参数 -> DogGait_UpdateTrot() 推进相位 ->
 * 计算足端坐标 -> 反解关节角 -> DogServo_SetAngles() 下发舵机角度。
 */

#include "dog_gait.h"
#include "dog_servo.h"
#include <math.h>
#include <stdint.h>

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

/* 舵机输入角为 0 度时对应的机构零位足端坐标，用于把逆解绝对角转换为舵机相对角。 */
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

volatile uint32_t g_dog_gait_update_count;
volatile float g_dog_gait_phase_before;
volatile float g_dog_gait_phase_after;
volatile float g_dog_gait_lf_hip_angle;
volatile float g_dog_gait_lf_knee_angle;
volatile float g_dog_gait_rf_hip_angle;
volatile float g_dog_gait_rf_knee_angle;

/*
 * 名称：DogGait_ClampFloat
 * 作用：将浮点数限制在指定范围内。
 * 输入：value 待限制的值；min_value 最小值；max_value 最大值。
 * 输出：限制后的浮点数。
 */
static float DogGait_ClampFloat(float value, float min_value, float max_value)
{
    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
}

/*
 * 名称：DogGait_InitLeg
 * 作用：初始化单条腿的步态参数和运行状态。
 * 输入：gait 腿状态指针；h 步高；r 步长；l1/l2 连杆长度；bias_angle 足端轨迹偏转角。
 * 输出：无返回值，更新 gait 指向的腿状态。
 */
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

/*
 * 名称：DogGait_GetFootBaseCoord
 * 作用：根据当前负载模式和足端基准模式，获取站立/行走/转弯的足端基准坐标。
 * 输入：base 足端基准模式。
 * 输出：足端基准坐标 DogGaitFootBaseCoord_t。
 */
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
 * 名称：DogGait_GetPosByCycloidalEquation
 * 作用：根据摆线方程计算当前相位下的足端轨迹偏移。
 * 输入：bias_angle_deg 轨迹偏转角；t 相位时间；h 抬脚高度；r 步长。
 * 输出：通过 x/y 指针输出足端 x/y 偏移坐标。
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

/*
 * 名称：DogGait_CalcAbsoluteAngleByPos
 * 作用：根据足端绝对坐标和两段连杆长度，计算髋关节和膝关节的绝对角度。
 * 输入：x/y 足端坐标；l1/l2 连杆长度。
 * 输出：通过 hip_angle/knee_angle 指针输出关节角度。
 */
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
 * 名称：DogGait_CalcAngleByPos
 * 作用：计算目标足端位置相对于机构零位足端坐标的髋/膝关节角度偏移。
 * 输入：x/y 目标足端坐标；l1/l2 连杆长度。
 * 输出：通过 hip_angle/knee_angle 指针输出给 DogServo_SetAngles() 使用的相对关节角度。
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

/*
 * 名称：DogGait_SetStandFootPos
 * 作用：将四条腿的足端坐标设置到站立基准位置。
 * 输入：无。
 * 输出：无返回值，更新 s_gait 中四条腿的 x/y。
 */
static void DogGait_SetStandFootPos(void)
{
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(DOG_GAIT_FOOT_BASE_STAND);

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].x = base_coord.x;
        s_gait[i].y = base_coord.y;
    }
}

/*
 * 名称：DogGait_ClearLegBiases
 * 作用：清除四条腿的髋/膝关节角度补偿。
 * 输入：无。
 * 输出：无返回值，更新 s_gait 中四条腿的 bias_hip/bias_knee。
 */
static void DogGait_ClearLegBiases(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].bias_hip = 0.0f;
        s_gait[i].bias_knee = 0.0f;
    }
}

/*
 * 名称：DogGait_ApplySideSteps
 * 作用：统一设置左右侧腿的步高、步长、速度和足端基准模式。
 * 输入：step_height_mm 步高；left_r 左侧步长；right_r 右侧步长；speed_freq 相位速度；base 足端基准模式。
 * 输出：无返回值，更新全局步态参数和四条腿状态。
 */
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

/*
 * 名称：DogGait_UpdateLegAngles
 * 作用：根据当前四条腿足端坐标，重新计算每条腿的髋/膝关节角度。
 * 输入：无。
 * 输出：无返回值，更新 s_gait 中四条腿的 hip_angle/knee_angle。
 */
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
 * 名称：DogGait_FillServoAngles
 * 作用：根据当前步态信息计算每条腿的关节角度，并填充到舵机角度数组中。
 * 输入：angles 舵机角度数组。
 * 输出：无返回值，更新 angles 数组。
 */
static void DogGait_FillServoAngles(float angles[DOG_SERVO_COUNT])
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

/*
 * 名称：DogGait_SetTrotParams
 * 作用：设置普通小跑步态参数。
 * 输入：step_height_mm 步高；step_length_mm 步长；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新小跑步态参数。
 */
void DogGait_SetTrotParams(float step_height_mm, float step_length_mm, float speed_freq)
{
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 60.0f);
    float clamped_step_length_mm = DogGait_ClampFloat(step_length_mm, -60.0f, 60.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.4f);

    DogGait_ApplySideSteps(clamped_step_height_mm,
                           clamped_step_length_mm,
                           clamped_step_length_mm,
                           clamped_speed_freq,
                           DOG_GAIT_FOOT_BASE_WALK);
}

/*
 * 名称：DogGait_SetLoadMode
 * 作用：设置当前是否带负载，用于选择不同的足端基准坐标。
 * 输入：mode 负载模式。
 * 输出：无返回值，更新 s_load_mode。
 */
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

/*
 * 名称：DogGait_SetTrackParams
 * 作用：设置循迹行走步态参数，可通过左右步长和转向步长实现偏航调整。
 * 输入：step_height_mm 步高；left_forward_step_mm 左侧前进步长；right_forward_step_mm 右侧前进步长；
 *       steer_step_mm 转向修正步长；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新循迹步态参数。
 */
void DogGait_SetTrackParams(float step_height_mm,
                            float left_forward_step_mm,
                            float right_forward_step_mm,
                            float steer_step_mm,
                            float speed_freq)
{
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 80.0f);
    float clamped_left_forward_step_mm = DogGait_ClampFloat(left_forward_step_mm, 0.0f, 80.0f);
    float clamped_right_forward_step_mm = DogGait_ClampFloat(right_forward_step_mm, 0.0f, 80.0f);
    float clamped_steer_step_mm = DogGait_ClampFloat(steer_step_mm, -20.0f, 20.0f);
    float left_r = DogGait_ClampFloat(clamped_left_forward_step_mm + clamped_steer_step_mm, -80.0f, 80.0f);
    float right_r = DogGait_ClampFloat(clamped_right_forward_step_mm - clamped_steer_step_mm, -80.0f, 80.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(clamped_step_height_mm, left_r, right_r, clamped_speed_freq, DOG_GAIT_FOOT_BASE_WALK);
}

/*
 * 名称：DogGait_SetTurnLeftParams
 * 作用：设置左转步态参数，左侧腿反向、右侧腿正向形成原地/小半径左转。
 * 输入：step_height_mm 步高；turn_step_mm 转向步长；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新左转步态参数。
 */
void DogGait_SetTurnLeftParams(float step_height_mm, float turn_step_mm, float speed_freq)
{
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 60.0f);
    float clamped_turn_step_mm = DogGait_ClampFloat(turn_step_mm, 0.0f, 60.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(clamped_step_height_mm,
                           -clamped_turn_step_mm,
                           clamped_turn_step_mm,
                           clamped_speed_freq,
                           DOG_GAIT_FOOT_BASE_TURN);
}

/*
 * 名称：DogGait_SetTurnRightParams
 * 作用：设置右转步态参数，左侧腿正向、右侧腿反向形成原地/小半径右转。
 * 输入：step_height_mm 步高；turn_step_mm 转向步长；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新右转步态参数。
 */
void DogGait_SetTurnRightParams(float step_height_mm, float turn_step_mm, float speed_freq)
{
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 60.0f);
    float clamped_turn_step_mm = DogGait_ClampFloat(turn_step_mm, 0.0f, 60.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(clamped_step_height_mm,
                           clamped_turn_step_mm,
                           -clamped_turn_step_mm,
                           clamped_speed_freq,
                           DOG_GAIT_FOOT_BASE_TURN);
}

/*
 * 名称：DogGait_Init
 * 作用：初始化四条腿的默认步态参数和模块状态。
 * 输入：无。
 * 输出：无返回值，更新 s_gait、相位、速度和初始化标志。
 */
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

/*
 * 名称：DogGait_GotoStandPose
 * 作用：切换到站立姿态，并将计算出的站立角度下发给舵机。
 * 输入：time_ms 舵机动作过渡时间。
 * 输出：无返回值，通过 DogServo_SetAngles() 输出舵机目标角度。
 */
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

/*
 * 名称：DogGait_UpdateTrot
 * 作用：推进一次小跑相位，更新四条腿足端轨迹、关节角和舵机目标角度。
 * 输入：time_ms 舵机动作过渡时间。
 * 输出：无返回值，通过 DogServo_SetAngles() 输出舵机目标角度，并更新调试变量。
 */
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

    g_dog_gait_update_count++;
    g_dog_gait_phase_before = s_trot_phase;

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
    g_dog_gait_lf_hip_angle = angles[DOG_SERVO_LF_HIP];
    g_dog_gait_lf_knee_angle = angles[DOG_SERVO_LF_KNEE];
    g_dog_gait_rf_hip_angle = angles[DOG_SERVO_RF_HIP];
    g_dog_gait_rf_knee_angle = angles[DOG_SERVO_RF_KNEE];
    DogServo_SetAngles(angles, time_ms);

    s_trot_phase += s_trot_speed_freq;
    if (s_trot_phase >= 1.0f)
    {
        s_trot_phase -= 1.0f;
    }
    g_dog_gait_phase_after = s_trot_phase;
}

/*
 * 名称：DogGait_AllStand
 * 作用：让所有腿进入站立姿态。
 * 输入：time_ms 舵机动作过渡时间。
 * 输出：无返回值，通过 DogGait_GotoStandPose() 下发站立角度。
 */
void DogGait_AllStand(uint16_t time_ms)
{
    DogGait_GotoStandPose(time_ms);
}

void DogGait_UpdateLeft(uint16_t time_ms)
{
    float log_low = 10.0f;
    float angles[DOG_SERVO_COUNT] = {0.0f};
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(s_foot_base);
    float lift;
    float dx;

    if (s_is_initialized == 0U)
    {
        DogGait_Init();
    }

    g_dog_gait_update_count++;
    g_dog_gait_phase_before = s_trot_phase;

    if (s_trot_phase <= 0.5f)
    {
        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_LF].h, s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift - log_low;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_RB].h, s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift - log_low;
    }
    else
    {
        float phase = s_trot_phase - 0.5f;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift - log_low;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x ;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, phase, s_gait[DOG_GAIT_LEG_RF].h, s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, phase, s_gait[DOG_GAIT_LEG_LB].h, s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift - log_low;
    }

    DogGait_UpdateLegAngles();
    DogGait_FillServoAngles(angles);
    g_dog_gait_lf_hip_angle = angles[DOG_SERVO_LF_HIP];
    g_dog_gait_lf_knee_angle = angles[DOG_SERVO_LF_KNEE];
    g_dog_gait_rf_hip_angle = angles[DOG_SERVO_RF_HIP];
    g_dog_gait_rf_knee_angle = angles[DOG_SERVO_RF_KNEE];
    DogServo_SetAngles(angles, time_ms);

    s_trot_phase += s_trot_speed_freq;
    if (s_trot_phase >= 1.0f)
    {
        s_trot_phase -= 1.0f;
    }
    g_dog_gait_phase_after = s_trot_phase;
}

void DogGait_UpdateRight(uint16_t time_ms)
{
    float log_low = 10.0f;
    float angles[DOG_SERVO_COUNT] = {0.0f};
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(s_foot_base);
    float lift;
    float dx;

    if (s_is_initialized == 0U)
    {
        DogGait_Init();
    }

    g_dog_gait_update_count++;
    g_dog_gait_phase_before = s_trot_phase;

    if (s_trot_phase <= 0.5f)
    {
        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_LF].h, s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift ;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_RB].h, s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift - log_low;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift - log_low;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift;
    }
    else
    {
        float phase = s_trot_phase - 0.5f;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x ;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift - log_low;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, phase, s_gait[DOG_GAIT_LEG_RF].h, s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift - log_low;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, phase, s_gait[DOG_GAIT_LEG_LB].h, s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift;
    }

    DogGait_UpdateLegAngles();
    DogGait_FillServoAngles(angles);
    g_dog_gait_lf_hip_angle = angles[DOG_SERVO_LF_HIP];
    g_dog_gait_lf_knee_angle = angles[DOG_SERVO_LF_KNEE];
    g_dog_gait_rf_hip_angle = angles[DOG_SERVO_RF_HIP];
    g_dog_gait_rf_knee_angle = angles[DOG_SERVO_RF_KNEE];
    DogServo_SetAngles(angles, time_ms);

    s_trot_phase += s_trot_speed_freq;
    if (s_trot_phase >= 1.0f)
    {
        s_trot_phase -= 1.0f;
    }
    g_dog_gait_phase_after = s_trot_phase;
}
