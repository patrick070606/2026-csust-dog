/*
 * dog_gait 模块结构概览：
 * 1. 基础参数：默认步高/步长、连杆长度、站立足端基准、角度限幅。
 * 2. 状态数据：四条腿的足端坐标、关节角、步态相位、负载模式和足端基准模式。
 * 3. 内部计算：摆线足端轨迹、足端坐标到髋/膝角的逆运动学、舵机角度填充。
 * 4. 对外接口：初始化、站立、小跑、原地踏步、循迹、左右转/侧移参数设置，以及周期性步态更新。
 *
 * 基本流程：设置步态参数 -> DogGait_UpdateTrot() 推进相位 ->
 * 计算足端坐标 -> 反解关节角 -> DogServo_SetAngles() 下发舵机角度。
 */

#include "dog_gait.h"
#include "dog_servo.h"
#include <math.h>
#include <stdint.h>

//腿部步长编号
#define DOG_GAIT_dx_LF 1
#define DOG_GAIT_dx_RB 2
#define DOG_GAIT_dx_RF 3
#define DOG_GAIT_dx_LB 0
/*
 * 基础步态参数。
 * 腿序：左前、右前、左后、右后。
 */
#define DOG_GAIT_PI                        3.14159265358979323846f
#define DOG_GAIT_DEFAULT_H_MM              15.0f
#define DOG_GAIT_DEFAULT_R_MM              15.0f
#define DOG_GAIT_DEFAULT_L1_MM             125.0f
#define DOG_GAIT_DEFAULT_L2_MM             100.0f
#define DOG_GAIT_DEFAULT_SPEED_FREQ        0.125f

/* 舵机输入角为 0 度时对应的机构零位足端坐标，用于把逆解绝对角转换为舵机相对角。 */
#define DOG_GAIT_ZERO_FOOT_X_MM            125.0f
#define DOG_GAIT_ZERO_FOOT_Y_MM            100.0f

/*
 * 足端基准坐标策略。
 * 统一 stand/walk/turn 的基准来源，切换步态 时足端位置更连续。
 *
 * 0: 该状态复用 stand 的基准坐标。
 * 1: 该状态使用独立的基准坐标。
 */
#define DOG_GAIT_STAND_FOOT_BASE_ENABLE    1U
#define DOG_GAIT_WALK_FOOT_BASE_ENABLE     1U
#define DOG_GAIT_TURN_FOOT_BASE_ENABLE     1U
#define DOG_GAIT_SHIFT_FOOT_BASE_LEFT_ENABLE    1U //左右平移步态
#define DOG_GAIT_SHIFT_FOOT_BASE_RIGHT_ENABLE    1U //左右平移步态

/* stand 基准坐标，x 偏移用于调整有负荷/无负荷时的重心。 */
#define DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM -35.0f
#define DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM    -35.0f
#define DOG_GAIT_STAND_FOOT_Y_MM                (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 160.0f)

#if (DOG_GAIT_WALK_FOOT_BASE_ENABLE != 0U)
#define DOG_GAIT_WALK_FOOT_X_OFFSET_NO_LOAD_MM  -30.0f
#define DOG_GAIT_WALK_FOOT_X_OFFSET_LOAD_MM     -30.0f
#define DOG_GAIT_WALK_FOOT_Y_MM                 (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 160.0f)
#define DOG_GAIT_WALK_FOOT_Y_MM                 DOG_GAIT_STAND_FOOT_Y_MM
#endif

#if (DOG_GAIT_TURN_FOOT_BASE_ENABLE != 0U)
#define DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM  -35.0f
#define DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM     -35.0f
#define DOG_GAIT_TURN_FOOT_Y_MM                 (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 160.0f)
#else
#define DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM  DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM
#define DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM     DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM
#define DOG_GAIT_TURN_FOOT_Y_MM                 DOG_GAIT_STAND_FOOT_Y_MM
#endif

#if (DOG_GAIT_SHIFT_FOOT_BASE_RIGHT_ENABLE != 0U)
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_NO_LOAD_MM  4.0f
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_LOAD_MM     4.0f
#define DOG_GAIT_SHIFTR_FOOT_Y_MM                 (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 100.0f)
#else
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_NO_LOAD_MM  DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_LOAD_MM     DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM
#define DOG_GAIT_SHIFT_FOOT_Y_MM                 DOG_GAIT_STAND_FOOT_Y_MM
#endif

#if (DOG_GAIT_SHIFT_FOOT_BASE_LEFT_ENABLE != 0U)
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_NO_LOAD_MM  4.0f
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_LOAD_MM     4.0f
#define DOG_GAIT_SHIFTL_FOOT_Y_MM                 (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 80.0f)
#else
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_NO_LOAD_MM  DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM
#define DOG_GAIT_SHIFT_FOOT_X_OFFSET_LOAD_MM     DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM
#define DOG_GAIT_SHIFT_FOOT_Y_MM                 DOG_GAIT_STAND_FOOT_Y_MM
#endif

#define DOG_GAIT_SHIFT_LOW_MM                    25.0f //平移步态下左右两边的高度差。

#define DOG_GAIT_WALK_PHASE_PER_LEG              0.5f // 表示单条腿完成一次抬起、前摆和落下所占用的相位长度。由于四条腿的步态是交替进行的，所以每条腿的步态相位为 0.5，整个步态周期为 2.0。
#define DOG_GAIT_WALK_TOTAL_PHASE                (DOG_GAIT_WALK_PHASE_PER_LEG * 4.0f)
#define DOG_GAIT_WALK_BODY_READY_MM              3.0f // 重心目标误差小于 3 mm 后，才允许摆腿相位继续推进。
#define DOG_GAIT_WALK_BODY_KP                    0.15f // 重心一阶平滑系数；每次只修正当前误差的 15%。
#define DOG_GAIT_WALK_BODY_MAX_STEP_MM           2.0f // 单次步态更新允许的最大重心移动量，防止目标变化时足端坐标突跳。
#define DOG_GAIT_WALK_BODY_LENGTH_MM             280.0f // 参考 Py-Apple 经验公式的机身前后支撑长度；应按本机前后髋关节间距实测调整。
#define DOG_GAIT_WALK_PHASE_CG_GAIN              0.7f // 前/后腿阶段重心切换增益；首轮调试关闭，避免足端整体产生正负 60 mm 偏移。
#define DOG_GAIT_WALK_PITCH_ANGLE_GAIN           1.5f // 参考公式 tan(pitch * 1.5) 中的倾角经验放大系数。
#define DOG_GAIT_WALK_BODY_TARGET_MAX_MM         1000.0f // 前后重心目标的安全限幅，首轮调试限制在正负 20 mm。
#define DOG_GAIT_WALK_CG_AXIS_SIGN              (-1.0f) // Py-Apple 与本工程 X 轴方向相反；负号保持本工程原有的前/后重心移动方向。
#define DOG_GAIT_WALK_ROLL_GAIN_MM               70.0f // roll 倾角到左右腿高度补偿的增益。
#define DOG_GAIT_WALK_ROLL_MAX_MM                20.0f // roll 左右腿高度补偿限幅。
#define DOG_GAIT_WALK_SIDE_PRELOAD_MM            0.0f // 抬某侧腿前给另一侧支撑腿的预加载补偿。
#define DOG_GAIT_WALK_SUPPORT_RETURN_MM          40.0f // 支撑腿相对机身向后移动的距离，与摆动步长独立。
#define DOG_GAIT_WALK_REAR_LIFT_END_PHASE        0.25f // 后腿摆动前段结束相位：先以抬高为主，X 基本保持。
#define DOG_GAIT_WALK_REAR_TRANSFER_END_PHASE    0.50f // 后腿摆动中段结束相位：在高位完成向前移动。
#define DOG_GAIT_WALK_REAR_TUCK_X_MM             15.0f // 后腿抬起初期主动后收量；先设为 0，仅观察逆运动学自然收腿效果。
#define DOG_GAIT_DEG_TO_RAD                      (DOG_GAIT_PI / 180.0f) // 角度 -> 弧度

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
    DOG_GAIT_SHIFT_LEFT = 0,
    DOG_GAIT_SHIFT_RIGHT,
} DogGaitShiftDirection_t;

typedef struct
{
    float x;
    float y;
} DogGaitFootBaseCoord_t;

static DogGaitInfo_t s_gait[DOG_GAIT_LEG_COUNT];
static float s_foot_y_offset[DOG_GAIT_LEG_COUNT];
static float s_trot_phase;
static float s_trot_speed_freq = DOG_GAIT_DEFAULT_SPEED_FREQ;
static float s_walk_phase; // walk 步态的当前相位，范围是 [0.0, DOG_GAIT_WALK_TOTAL_PHASE)，表示整个 walk 步态周期的进度。
static float s_walk_speed_freq = 0.03f; // 表示 walk 步态的速度频率。
static float s_walk_step_height_mm = 55.0f;
static float s_walk_step_length_mm = 50.0f;
static float s_walk_cg_base_x_mm; // 基础重心偏移
static float s_walk_imu_gain_mm = 60.0f; // 表示 walk 步态中，IMU 前后倾角对重心前后偏移的增益系数。也就是说，如果 IMU 检测到机器人前倾 1 度，那么重心会向前偏移 60.0mm，从而调整机器人的步态，使其保持平衡。
static float s_walk_body_x_goal_mm; // 目标重心偏移
static float s_walk_body_x_state_mm; // 当前重心偏移
static float s_walk_foot_x[DOG_GAIT_LEG_COUNT]; // walk 步态中各腿的足端 X 坐标
static float s_walk_foot_y[DOG_GAIT_LEG_COUNT]; // walk 步态中各腿的足端 Y 坐标
static uint8_t s_walk_cycle_done; // walk 步态周期是否完成
static float s_walk_touchdown_x[DOG_GAIT_LEG_COUNT];
static float s_walk_swing_start_x[DOG_GAIT_LEG_COUNT];
static uint8_t s_walk_leg_in_swing[DOG_GAIT_LEG_COUNT];
static uint8_t s_walk_support_phase;
static uint8_t s_walk_support_ready;
static float s_walk_support_start_x[DOG_GAIT_LEG_COUNT];
static float s_walk_support_start_y[DOG_GAIT_LEG_COUNT];
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

static void DogGait_UpdateLegAngles(void);
static void DogGait_FillServoAngles(float angles[DOG_SERVO_COUNT]);

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
    case DOG_GAIT_FOOT_BASE_SHIFT_LEFT:
        coord.x = (s_load_mode == DOG_GAIT_LOAD_WITH_PAYLOAD) ?
                  DOG_GAIT_SHIFT_FOOT_X_OFFSET_LOAD_MM :
                  DOG_GAIT_SHIFT_FOOT_X_OFFSET_NO_LOAD_MM;
        coord.y = DOG_GAIT_SHIFTL_FOOT_Y_MM;
        break;
    case DOG_GAIT_FOOT_BASE_SHIFT_RIGHT:
        coord.x = (s_load_mode == DOG_GAIT_LOAD_WITH_PAYLOAD) ?
                  DOG_GAIT_SHIFT_FOOT_X_OFFSET_LOAD_MM :
                  DOG_GAIT_SHIFT_FOOT_X_OFFSET_NO_LOAD_MM;
        coord.y = DOG_GAIT_SHIFTR_FOOT_Y_MM;
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
    //*hip_angle =DogGait_ClampFloat ((hip_base - hip_link) * 180.0f / DOG_GAIT_PI,-180,180);
    *knee_angle =(DOG_GAIT_PI - knee_link) * 180.0f / DOG_GAIT_PI;
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
 * 名称：DogGait_ClearFootYOffsets
 * 作用：清除四条腿的足端 y 方向附加偏移。
 * 输入：无。
 * 输出：无返回值，更新 s_foot_y_offset。
 */
static void DogGait_ClearFootYOffsets(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_foot_y_offset[i] = 0.0f;
    }
}

/*
 * 名称：DogGait_ClearWalkFootOffsets
 * 作用：清除 walk 步态中四条腿的足端坐标偏移。
 * 输入：无。
 * 输出：无返回值，更新 s_walk_foot_x 和 s_walk_foot_y。
 */
static void DogGait_ClearWalkFootOffsets(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_walk_foot_x[i] = 0.0f;
        s_walk_foot_y[i] = 0.0f;
    }
}

/*
 * 名称：DogGait_GetWalkBodyTarget
 * 作用：获取 walk 步态中重心的目标偏移量。
 * 输入：active_leg 当前活动的腿；pitch_deg IMU 检测到的俯仰角。
 * 输出：返回重心的目标偏移量。
 */
static float DogGait_SmoothStep(float phase)
{
    phase = DogGait_ClampFloat(phase, 0.0f, 1.0f);
    return phase * phase * (3.0f - 2.0f * phase);
}

static float DogGait_CalcLegReach(float x, float y, float l1, float l2)
{
    float dy = l1 + l2 - y;

    return sqrtf(x * x + dy * dy);
}

static float DogGait_GetWalkSupportReturnMm(void)
{
    return DogGait_ClampFloat(s_walk_step_length_mm,
                              -DOG_GAIT_WALK_SUPPORT_RETURN_MM,
                               DOG_GAIT_WALK_SUPPORT_RETURN_MM);
}

static float DogGait_WrapWalkLegPhase(float phase)
{
    while (phase < 0.0f)
    {
        phase += DOG_GAIT_WALK_TOTAL_PHASE;
    }
    while (phase >= DOG_GAIT_WALK_TOTAL_PHASE)
    {
        phase -= DOG_GAIT_WALK_TOTAL_PHASE;
    }
    return phase;
}

static void DogGait_ResetWalkFootStates(void)
{
    float support_return_mm = DogGait_GetWalkSupportReturnMm();

    /* Initialize a continuous stance arrangement at global phase 0. */
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        float leg_phase = DogGait_WrapWalkLegPhase(
            -((float)i * DOG_GAIT_WALK_PHASE_PER_LEG));
        float stance_phase =
            (leg_phase - DOG_GAIT_WALK_PHASE_PER_LEG) /
            (DOG_GAIT_WALK_TOTAL_PHASE - DOG_GAIT_WALK_PHASE_PER_LEG);

        if (stance_phase < 0.0f)
        {
            stance_phase = 0.0f;
        }

        s_walk_touchdown_x[i] = support_return_mm * DogGait_SmoothStep(stance_phase);
        if (i == DOG_GAIT_LEG_LF)
        {
            /* First LF swing starts at the neutral point. */
            s_walk_touchdown_x[i] = support_return_mm;
        }
        s_walk_swing_start_x[i] = 0.0f;
        s_walk_leg_in_swing[i] = 0U;
    }

    DogGait_ClearWalkFootOffsets();
    s_walk_support_phase = 0U;
    s_walk_support_ready = 0U;
}

static void DogGait_UpdateWalkFootTrajectories(void)
{
    float support_return_mm = DogGait_GetWalkSupportReturnMm();

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        float leg_phase = DogGait_WrapWalkLegPhase(
            s_walk_phase - ((float)i * DOG_GAIT_WALK_PHASE_PER_LEG));

        if (leg_phase < DOG_GAIT_WALK_PHASE_PER_LEG) // 摆动相
        {
            float swing_phase =
                leg_phase / DOG_GAIT_WALK_PHASE_PER_LEG;

            if (s_walk_leg_in_swing[i] == 0U)
            {
                s_walk_swing_start_x[i] =
                    s_walk_touchdown_x[i] - support_return_mm;
                s_walk_leg_in_swing[i] = 1U;
            }

            if ((i == DOG_GAIT_LEG_LB) ||
                (i == DOG_GAIT_LEG_RB))
            {
                if (swing_phase < DOG_GAIT_WALK_REAR_LIFT_END_PHASE)
                {
                    float lift_phase =
                        swing_phase / DOG_GAIT_WALK_REAR_LIFT_END_PHASE;
                    float lift_smooth = DogGait_SmoothStep(lift_phase);

                    s_walk_foot_x[i] =
                        s_walk_swing_start_x[i] -
                        DOG_GAIT_WALK_REAR_TUCK_X_MM * lift_smooth;
                    s_walk_foot_y[i] =
                        s_walk_step_height_mm * lift_smooth;
                }
                else if (swing_phase < DOG_GAIT_WALK_REAR_TRANSFER_END_PHASE)
                {
                    float transfer_phase =
                        (swing_phase - DOG_GAIT_WALK_REAR_LIFT_END_PHASE) /
                        (DOG_GAIT_WALK_REAR_TRANSFER_END_PHASE -
                         DOG_GAIT_WALK_REAR_LIFT_END_PHASE);
                    float transfer_smooth = DogGait_SmoothStep(transfer_phase);

                    s_walk_foot_x[i] =
                        s_walk_swing_start_x[i] -
                        DOG_GAIT_WALK_REAR_TUCK_X_MM +
                        (s_walk_step_length_mm + DOG_GAIT_WALK_REAR_TUCK_X_MM) *
                        transfer_smooth;
                    s_walk_foot_y[i] = s_walk_step_height_mm;
                }
                else
                {
                    float landing_phase =
                        (swing_phase - DOG_GAIT_WALK_REAR_TRANSFER_END_PHASE) /
                        (1.0f - DOG_GAIT_WALK_REAR_TRANSFER_END_PHASE);

                    s_walk_foot_x[i] =
                        s_walk_swing_start_x[i] + s_walk_step_length_mm;
                    s_walk_foot_y[i] =
                        s_walk_step_height_mm *
                        (1.0f - DogGait_SmoothStep(landing_phase));
                }
            }
            else
            {
                s_walk_foot_x[i] =
                    s_walk_swing_start_x[i] +
                    s_walk_step_length_mm * DogGait_SmoothStep(swing_phase);
                s_walk_foot_y[i] =
                    s_walk_step_height_mm * sinf(DOG_GAIT_PI * swing_phase);
            }
        }
        else //支撑相
        {
            float support_phase =
                (leg_phase - DOG_GAIT_WALK_PHASE_PER_LEG) /
                (DOG_GAIT_WALK_TOTAL_PHASE - DOG_GAIT_WALK_PHASE_PER_LEG);

            if (s_walk_leg_in_swing[i] != 0U)
            {
                s_walk_touchdown_x[i] =
                    s_walk_swing_start_x[i] + s_walk_step_length_mm;
                s_walk_leg_in_swing[i] = 0U;
            }

            s_walk_foot_x[i] =
                s_walk_touchdown_x[i] -
                support_return_mm * DogGait_SmoothStep(support_phase);
            s_walk_foot_y[i] = 0.0f;
        }
    }
}

static float DogGait_GetWalkBodyTarget(uint8_t active_leg, float pitch_deg)
{
    float pitch_angle_rad = pitch_deg * DOG_GAIT_WALK_PITCH_ANGLE_GAIN * DOG_GAIT_DEG_TO_RAD;
    float pitch_adjust;
    float reference_phase_adjust;
    float target;

    /* tan() 在正负 90 度附近发散，先将放大后的输入限制在正负 45 度。 */
    pitch_angle_rad = DogGait_ClampFloat(pitch_angle_rad,
                                         -0.25f * DOG_GAIT_PI,
                                          0.25f * DOG_GAIT_PI);
    pitch_adjust = s_walk_imu_gain_mm * tanf(pitch_angle_rad);

    if ((active_leg == DOG_GAIT_LEG_LF) ||
        (active_leg == DOG_GAIT_LEG_RF))
    {
        /* Py-Apple: period=-1, (l+xk)/4；两条前腿使用 -L/4。 */
        reference_phase_adjust = -DOG_GAIT_WALK_BODY_LENGTH_MM * 0.25f;
    }
    else
    {
        /* Py-Apple: period=+1；后腿阶段额外计入已经向前迈出的步长。 */
        reference_phase_adjust =
            (DOG_GAIT_WALK_BODY_LENGTH_MM + fabsf(s_walk_step_length_mm)) * 0.25f;
    }

    target = s_walk_cg_base_x_mm +
             DOG_GAIT_WALK_CG_AXIS_SIGN *
             (reference_phase_adjust * DOG_GAIT_WALK_PHASE_CG_GAIN +
              pitch_adjust);

    return DogGait_ClampFloat(target,
                              -DOG_GAIT_WALK_BODY_TARGET_MAX_MM,
                               DOG_GAIT_WALK_BODY_TARGET_MAX_MM);
}

/*
 * 名称：DogGait_OutputCurrentPose
 * 作用：输出当前步态姿态。
 * 输入：time_ms 更新时间。
 * 输出：无返回值。
 * 注解：和 DogGait_UpdateTrot() 本质相同。
 */
static void DogGait_OutputCurrentPose(uint16_t time_ms)
{
    float angles[DOG_SERVO_COUNT] = {0.0f};

    DogGait_UpdateLegAngles();
    DogGait_FillServoAngles(angles);
    g_dog_gait_lf_hip_angle = angles[DOG_SERVO_LF_HIP];
    g_dog_gait_lf_knee_angle = angles[DOG_SERVO_LF_KNEE];
    g_dog_gait_rf_hip_angle = angles[DOG_SERVO_RF_HIP];
    g_dog_gait_rf_knee_angle = angles[DOG_SERVO_RF_KNEE];
    DogServo_SetAngles(angles, time_ms);
}

/*
 * 名称：DogGait_ApplySideStepsApart
 * 作用：分开设置左右侧腿的步高、步长、速度和足端基准模式。
 * 输入：step_height_mm 步高；left_r 左侧步长；right_r 右侧步长；speed_freq 相位速度；base 足端基准模式。
 * 输出：无返回值，更新全局步态参数和四条腿状态。
 */
static void DogGait_ApplySideStepsApart(float step_height_mm,
                                   float *r,
                                   float speed_freq,
                                   DogGaitFootBase_t base)
{
    s_trot_speed_freq = speed_freq;
    s_foot_base = base;
    DogGait_ClearLegBiases();
    DogGait_ClearFootYOffsets();

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].h = step_height_mm;
    }

    s_gait[DOG_GAIT_LEG_LF].r = r[DOG_GAIT_LEG_LF];
    s_gait[DOG_GAIT_LEG_LB].r = r[DOG_GAIT_LEG_LB];
    s_gait[DOG_GAIT_LEG_RF].r = r[DOG_GAIT_LEG_RF];
    s_gait[DOG_GAIT_LEG_RB].r = r[DOG_GAIT_LEG_RB];

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].old_r = s_gait[i].r;
    }
}

/*
 * 名称：DogGait_ApplySideSteps
 * 作用：统一设置左右侧腿的步高、步长、速度和足端基准模式。
 * 输入：step_height_mm 步高；left_r 左侧步长；right_r 右侧步长；speed_freq 相位速度；base 足端基准模式。
 * 输出：无返回值，更新全局步态参数和四条腿状态。
 */
static void DogGait_ApplySideSteps(float step_height_mm,
                                   float Right_r,
                                   float Left_r,
                                   float speed_freq,
                                   DogGaitFootBase_t base)
{
    s_trot_speed_freq = speed_freq;
    s_foot_base = base;
    DogGait_ClearLegBiases();
    DogGait_ClearFootYOffsets();

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].h = step_height_mm;
    }

    s_gait[DOG_GAIT_LEG_LF].r = Left_r;
    s_gait[DOG_GAIT_LEG_LB].r = Left_r;
    s_gait[DOG_GAIT_LEG_RF].r = Right_r;
    s_gait[DOG_GAIT_LEG_RB].r = Right_r;

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
 * 作用：设置普通小跑步态参数，使用转向基准；转向基准未启用时自动复用站立基准。
 * 输入：step_height_mm 步高；step_length_mm 步长；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新小跑步态参数。
 */
void DogGait_SetTrotParams(float step_height_mm, float step_length_mm, float speed_freq)
{
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 100.0f);
    float clamped_step_length_mm = DogGait_ClampFloat(step_length_mm, -60.0f, 100.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.4f);

    DogGait_ApplySideSteps(clamped_step_height_mm,
                            clamped_step_length_mm,
                            clamped_step_length_mm,
                           clamped_speed_freq,
                           DOG_GAIT_FOOT_BASE_TURN);
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
 * 作用：设置循迹行走步态参数，使用转向基准；可通过左右步长和转向步长实现偏航调整。
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
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 100.0f);
    float clamped_left_forward_step_mm = DogGait_ClampFloat(left_forward_step_mm, -100.0f, 100.0f);
    float clamped_right_forward_step_mm = DogGait_ClampFloat(right_forward_step_mm, -100.0f, 100.0f);
    float clamped_steer_step_mm = DogGait_ClampFloat(steer_step_mm, -100.0f, 100.0f);
    float left_r = DogGait_ClampFloat(clamped_left_forward_step_mm + clamped_steer_step_mm, -80.0f, 100.0f);
    float right_r = DogGait_ClampFloat(clamped_right_forward_step_mm - clamped_steer_step_mm, -80.0f, 100.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(clamped_step_height_mm, right_r, left_r, clamped_speed_freq, DOG_GAIT_FOOT_BASE_TURN);
}

/*
 * 名称：DogGait_SetStepInPlaceParams
 * 作用：设置原地踏步参数，本质是步长为 0 的小跑，使用转向基准。
 * 输入：step_height_mm 步高；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新原地踏步参数。
 */
void DogGait_SetStepInPlaceParams(float step_height_mm, float speed_freq)
{
    float r[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 80.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideStepsApart(clamped_step_height_mm,
                           r,
                           clamped_speed_freq,
                           DOG_GAIT_FOOT_BASE_TURN);
}

/*
 * 名称：DogGait_SetShiftParams
 * 作用：设置左右平移参数，本质是步长为 0 且单侧足端降低的小跑。
 * 输入：direction 平移方向；step_height_mm 步高；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新左右平移参数。
 */
static void DogGait_SetShiftParams(DogGaitShiftDirection_t direction,
                                   float step_height_mm,
                                   float *step_length_mm,
                                   float speed_freq,
                                   DogGaitFootBase_t base)
{
    DogGaitFootBase_t foot_base = base;
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 80.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideStepsApart(clamped_step_height_mm,
                           step_length_mm,
                           clamped_speed_freq,
                           foot_base);

    if (direction == DOG_GAIT_SHIFT_LEFT)
    {
        s_foot_y_offset[DOG_GAIT_LEG_RF] = -DOG_GAIT_SHIFT_LOW_MM;
        s_foot_y_offset[DOG_GAIT_LEG_RB] = -DOG_GAIT_SHIFT_LOW_MM;
        s_foot_y_offset[DOG_GAIT_LEG_LF] = 0;
        s_foot_y_offset[DOG_GAIT_LEG_LB] = 0;
    }
    else
    {
        s_foot_y_offset[DOG_GAIT_LEG_LF] = -DOG_GAIT_SHIFT_LOW_MM;
        s_foot_y_offset[DOG_GAIT_LEG_LB] = -DOG_GAIT_SHIFT_LOW_MM;
        s_foot_y_offset[DOG_GAIT_LEG_RF] = 0;
        s_foot_y_offset[DOG_GAIT_LEG_RB] = 0;
    }
}

/*
 * 名称：DogGait_SetShiftLeftParams
 * 作用：设置左平移步态参数，右侧腿降低 DOG_GAIT_SHIFT_LOW_MM。
 * 输入：step_height_mm 步高；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新左平移步态参数。
 */
void DogGait_SetShiftLeftParams(float step_height_mm, float *step_length_mm, float speed_freq,DogGaitFootBase_t base)
{
    DogGait_SetShiftParams(DOG_GAIT_SHIFT_LEFT, step_height_mm, step_length_mm, speed_freq, base);
}

/*
 * 名称：DogGait_SetShiftRightParams
 * 作用：设置右平移步态参数，左侧腿降低 DOG_GAIT_SHIFT_LOW_MM。
 * 输入：step_height_mm 步高；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新右平移步态参数。
 */
void DogGait_SetShiftRightParams(float step_height_mm, float *step_length_mm, float speed_freq,DogGaitFootBase_t base)
{
    DogGait_SetShiftParams(DOG_GAIT_SHIFT_RIGHT, step_height_mm, step_length_mm, speed_freq, base);
}

/*
 * 名称：DogGait_SetShiftStepR
 * 作用：独立设置 shift 步态中四条腿的步长，允许每条腿有不同的步长。
 * 输入：r 四条腿的步长数组，顺序为 LF/RF/LB/RB。
 * 输出：无返回值，更新 s_gait 中四条腿的 r 和 old_r。
 */
void DogGait_SetShiftStepR(const float r[4])
{
    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].r = r[i];
        s_gait[i].old_r = r[i];
    }
}

/*
 * 名称：DogGait_SetTurnLeftParams
 * 作用：设置左转步态参数，左侧腿反向、右侧腿正向形成原地/小半径左转。
 * 输入：step_height_mm 步高；turn_step_mm 转向步长；speed_freq 每次更新的相位增量。
 * 输出：无返回值，更新左转步态参数。
 */
void DogGait_SetTurnLeftParams(float step_height_mm, float turn_step_mm, float speed_freq)
{
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 80.0f);
    float clamped_turn_step_mm = DogGait_ClampFloat(turn_step_mm, 0.0f, 80.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);


    DogGait_ApplySideSteps(clamped_step_height_mm,
                           clamped_turn_step_mm,
                           -clamped_turn_step_mm,
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
    float clamped_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 80.0f);
    float clamped_turn_step_mm = DogGait_ClampFloat(turn_step_mm, 0.0f, 80.0f);
    float clamped_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.5f);

    DogGait_ApplySideSteps(clamped_step_height_mm,
                           -clamped_turn_step_mm,
                           clamped_turn_step_mm,
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
    s_walk_phase = 0.0f;
    s_walk_body_x_goal_mm = 0.0f;
    s_walk_body_x_state_mm = 0.0f;
    s_walk_cycle_done = 0U;
    s_foot_base = DOG_GAIT_FOOT_BASE_STAND;
    s_trot_speed_freq = DOG_GAIT_DEFAULT_SPEED_FREQ;
    DogGait_ClearFootYOffsets();
    DogGait_ResetWalkFootStates();
    s_is_initialized = 1U;
}

/*
 * 名称：DogGait_ResetWalk
 * 作用：重置 walk 步态参数。
 * 输入：无。
 * 输出：无返回值，更新 walk 步态参数。
 */
void DogGait_ResetWalk(void)
{
    s_walk_phase = 0.0f;
    s_walk_body_x_goal_mm = s_walk_cg_base_x_mm;
    s_walk_body_x_state_mm = s_walk_cg_base_x_mm;
    s_walk_cycle_done = 0U;
    DogGait_ResetWalkFootStates();
}

void DogGait_StartWalkSupportPhase(void)
{
    if (s_is_initialized == 0U)
    {
        DogGait_Init();
    }

    for (uint8_t i = 0U; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_walk_support_start_x[i] = s_gait[i].x;
        s_walk_support_start_y[i] = s_gait[i].y;
    }

    /* Hold the exact pose at the end of WALK; do not force all legs to a common x/y. */
    s_walk_support_ready = 1U;
    s_walk_support_phase = 1U;
}

void DogGait_ResumeWalkFromSupportPhase(void)
{
    s_walk_support_phase = 0U;
    s_walk_support_ready = 0U;
}

void DogGait_UpdateWalkSupport(uint16_t time_ms)
{
    if ((s_is_initialized == 0U) || (s_walk_support_phase == 0U))
    {
        return;
    }

    for (uint8_t i = 0U; i < DOG_GAIT_LEG_COUNT; i++)
    {
        s_gait[i].x = s_walk_support_start_x[i];
        s_gait[i].y = s_walk_support_start_y[i];
    }

    DogGait_OutputCurrentPose(time_ms);
}

uint8_t DogGait_IsWalkSupportReady(void)
{
    return s_walk_support_ready;
}

uint8_t DogGait_GetWalkFrontRearAverageReach(float *front_reach_mm,
                                              float *rear_reach_mm)
{
    if ((front_reach_mm == 0) ||
        (rear_reach_mm == 0) ||
        (s_walk_support_ready == 0U))
    {
        return 0U;
    }

    *front_reach_mm =
        (DogGait_CalcLegReach(s_gait[DOG_GAIT_LEG_LF].x,
                              s_gait[DOG_GAIT_LEG_LF].y,
                              s_gait[DOG_GAIT_LEG_LF].l1,
                              s_gait[DOG_GAIT_LEG_LF].l2) +
         DogGait_CalcLegReach(s_gait[DOG_GAIT_LEG_RF].x,
                              s_gait[DOG_GAIT_LEG_RF].y,
                              s_gait[DOG_GAIT_LEG_RF].l1,
                              s_gait[DOG_GAIT_LEG_RF].l2)) * 0.5f;

    *rear_reach_mm =
        (DogGait_CalcLegReach(s_gait[DOG_GAIT_LEG_LB].x,
                              s_gait[DOG_GAIT_LEG_LB].y,
                              s_gait[DOG_GAIT_LEG_LB].l1,
                              s_gait[DOG_GAIT_LEG_LB].l2) +
         DogGait_CalcLegReach(s_gait[DOG_GAIT_LEG_RB].x,
                              s_gait[DOG_GAIT_LEG_RB].y,
                              s_gait[DOG_GAIT_LEG_RB].l1,
                              s_gait[DOG_GAIT_LEG_RB].l2)) * 0.5f;

    return 1U;
}

/*
 * 名称：DogGait_SetWalkParams
 * 作用：设置 walk 步态参数。
 * 输入：step_height_mm 步高；step_length_mm 步长；speed_freq 速度频率；cg_base_x_mm 重心基准 X 坐标；imu_gain_mm IMU 增益。
 * 输出：无返回值，更新 walk 步态参数。
 */
void DogGait_SetWalkParams(float step_height_mm,
                           float step_length_mm,
                           float speed_freq,
                           float cg_base_x_mm,
                           float imu_gain_mm)
{
    s_walk_step_height_mm = DogGait_ClampFloat(step_height_mm, 0.0f, 140.0f);
    s_walk_step_length_mm = DogGait_ClampFloat(step_length_mm, -80.0f, 80.0f);
    s_walk_speed_freq = DogGait_ClampFloat(speed_freq, 0.0f, 0.2f);
    s_walk_cg_base_x_mm = DogGait_ClampFloat(cg_base_x_mm, -50.0f, 50.0f);
    s_walk_imu_gain_mm = DogGait_ClampFloat(imu_gain_mm, -120.0f, 120.0f);
    s_foot_base = DOG_GAIT_FOOT_BASE_WALK;
    DogGait_ClearLegBiases();
    DogGait_ClearFootYOffsets();
}

/*
 * 名称：DogGait_UpdateWalk
 * 作用：更新 walk 步态。
 * 输入：time_ms 更新时间；pitch_deg IMU 检测到的俯仰角；roll_deg IMU 检测到的滚转角。
 * 输出：无返回值，更新 walk 步态参数。
 */
void DogGait_UpdateWalk(uint16_t time_ms, float pitch_deg, float roll_deg)
{
    static const DogGaitLeg_t leg_order[DOG_GAIT_LEG_COUNT] = {
        DOG_GAIT_LEG_LF,
        DOG_GAIT_LEG_RF,
        DOG_GAIT_LEG_LB,
        DOG_GAIT_LEG_RB,
    }; // 规定迈腿顺序：左前 -> 右前 -> 左后 -> 右后，循环。
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(DOG_GAIT_FOOT_BASE_WALK); // 获取当前足端基准坐标。
    uint8_t leg_phase = (uint8_t)(s_walk_phase / DOG_GAIT_WALK_PHASE_PER_LEG); // 根据总相位选择活动腿。
    DogGaitLeg_t active_leg;
    float local_phase;
    float dx = 0.0f;
    float lift = 0.0f;
    float roll_adjust = -DOG_GAIT_WALK_ROLL_GAIN_MM * tanf(roll_deg * DOG_GAIT_DEG_TO_RAD);
    //float roll_adjust = 0.0f;

    if (s_is_initialized == 0U)
    {
        DogGait_Init();
    }

    if (leg_phase >= DOG_GAIT_LEG_COUNT)
    {
        leg_phase = DOG_GAIT_LEG_COUNT - 1U;
    }

    active_leg = leg_order[leg_phase];
    s_walk_body_x_goal_mm = DogGait_GetWalkBodyTarget((uint8_t)active_leg, pitch_deg); // 使用活动腿、机身长度和 IMU 倾角计算经验重心目标。
    {
        float body_step_mm =
            (s_walk_body_x_goal_mm - s_walk_body_x_state_mm) * DOG_GAIT_WALK_BODY_KP;

        body_step_mm = DogGait_ClampFloat(body_step_mm,
                                          -DOG_GAIT_WALK_BODY_MAX_STEP_MM,
                                           DOG_GAIT_WALK_BODY_MAX_STEP_MM);
        s_walk_body_x_state_mm += body_step_mm; // 平滑移动重心，并限制每次更新的最大变化量。
    }

    local_phase = s_walk_phase - ((float)leg_phase * DOG_GAIT_WALK_PHASE_PER_LEG); // 计算当前活动腿的局部相位，范围为 [0, DOG_GAIT_WALK_PHASE_PER_LEG]。
    DogGait_ClearWalkFootOffsets();
    DogGait_GetPosByCycloidalEquation(s_gait[active_leg].bias_angle,
                                      local_phase,
                                      s_walk_step_height_mm,
                                      s_walk_step_length_mm,
                                      &dx,
                                      &lift); // 得到当前活动腿前后方向的位移 dx 和竖直方向上的抬腿量 lift。
    s_walk_foot_x[active_leg] = dx;
    s_walk_foot_y[active_leg] = lift;
    DogGait_UpdateWalkFootTrajectories();
    roll_adjust = DogGait_ClampFloat(roll_adjust,
                                     -DOG_GAIT_WALK_ROLL_MAX_MM,
                                     DOG_GAIT_WALK_ROLL_MAX_MM);

    for (uint8_t i = 0; i < DOG_GAIT_LEG_COUNT; i++)
    {
        float side_adjust = 0.0f;

        if ((i == DOG_GAIT_LEG_LF) ||
            (i == DOG_GAIT_LEG_LB))
        {
            side_adjust = roll_adjust;
        }
        else
        {
            side_adjust = -roll_adjust;
        }

        if ((active_leg == DOG_GAIT_LEG_LF) ||
            (active_leg == DOG_GAIT_LEG_LB))
        {
            if ((i == DOG_GAIT_LEG_RF) ||
                (i == DOG_GAIT_LEG_RB))
            {
                side_adjust -= DOG_GAIT_WALK_SIDE_PRELOAD_MM;
            }
        } // 给相较于活动腿的另一侧腿施加一个预加载的偏移量，使得在活动腿抬起时，另一侧腿能够更好地支撑身体。
        else
        {
            if ((i == DOG_GAIT_LEG_LF) ||
                (i == DOG_GAIT_LEG_LB))
            {
                side_adjust -= DOG_GAIT_WALK_SIDE_PRELOAD_MM;
            }
        }

        s_gait[i].x = base_coord.x + s_walk_body_x_state_mm + s_walk_foot_x[i]; // 足端 X = 基础 X + 机身前后重心偏移 + 当前腿前后迈步偏移
        s_gait[i].y = base_coord.y + side_adjust + s_walk_foot_y[i]; // 足端Y = 基础 Y + 机身左右重心偏移 + 当前腿抬腿偏移
    }

    DogGait_OutputCurrentPose(time_ms);

    /* 重心未到位时不增加 s_walk_phase，因此活动腿保持在当前轨迹点，相当于冻结摆腿。 */
    if (fabsf(s_walk_body_x_goal_mm - s_walk_body_x_state_mm) < DOG_GAIT_WALK_BODY_READY_MM)
    {
        s_walk_phase += s_walk_speed_freq;
        if (s_walk_phase >= DOG_GAIT_WALK_TOTAL_PHASE)
        {
            s_walk_phase -= DOG_GAIT_WALK_TOTAL_PHASE;
            s_walk_cycle_done = 1U;
        }
    }
}

/*
    * 名称：DogGait_IsWalkCycleDone
    * 作用：检查 walk 步态是否完成一个完整循环。
    * 输入：无。
    * 输出：返回 1 表示完成，0 表示未完成，并在返回后清除完成标志。
*/
uint8_t DogGait_IsWalkCycleDone(void)
{
    uint8_t result = s_walk_cycle_done;

    s_walk_cycle_done = 0U;
    return result;
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
    DogGait_ResetWalk();
    DogGait_ClearLegBiases();
    DogGait_ClearFootYOffsets();
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
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_RB].h, s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RB];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LB];
    }
    else
    {
        float phase = s_trot_phase - 0.5f;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RB];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, phase, s_gait[DOG_GAIT_LEG_RF].h, s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, phase, s_gait[DOG_GAIT_LEG_LB].h, s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LB];
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
 * 名称：DogGait_UpdateShift
 * 作用：推进一次 shift（平移）步态相位，更新四条腿足端轨迹、关节角和舵机目标角度。
 *       与 DogGait_UpdateTrot 类似，但使用 DOG_GAIT_FOOT_BASE_SHIFT 足端基准，
 *       允许四条腿有不同的步长，实现平移效果。
 * 输入：time_ms 舵机动作过渡时间。
 * 输出：无返回值，通过 DogServo_SetAngles() 输出舵机目标角度，并更新调试变量。
 */
void DogGait_UpdateShift(uint16_t time_ms,DogGaitFootBase_t base)
{
    float angles[DOG_SERVO_COUNT] = {0.0f};
    DogGaitFootBaseCoord_t base_coord = DogGait_GetFootBaseCoord(base);
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
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, s_trot_phase, s_gait[DOG_GAIT_LEG_RB].h, s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RB];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, s_trot_phase, 0.0f, -s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LB];
    }
    else
    {
        float phase = s_trot_phase - 0.5f;

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LF].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_LF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RB].bias_angle, phase, 0.0f, -s_gait[DOG_GAIT_LEG_RB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RB];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_RF].bias_angle, phase, s_gait[DOG_GAIT_LEG_RF].h, s_gait[DOG_GAIT_LEG_RF].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_RF].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_RF].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_RF];

        DogGait_GetPosByCycloidalEquation(s_gait[DOG_GAIT_LEG_LB].bias_angle, phase, s_gait[DOG_GAIT_LEG_LB].h, s_gait[DOG_GAIT_LEG_LB].r, &dx, &lift);
        s_gait[DOG_GAIT_LEG_LB].x = base_coord.x + dx;
        s_gait[DOG_GAIT_LEG_LB].y = base_coord.y + lift + s_foot_y_offset[DOG_GAIT_LEG_LB];
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
