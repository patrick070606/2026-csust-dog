#include "stair_walk.h"

#include "dog_gait.h"
#include "jy61p_imu.h"
#include "main.h"

#include <math.h>

#define STAIR_WALK_TEST_GAIT_PERIOD_MS        100U  // 爬楼梯步态的更新周期：每隔 100 ms 计算并下发一次新的足端目标。
#define STAIR_WALK_TEST_GAIT_MOVE_MS          80U   // 舵机完成每次步态目标的期望时间；小于更新周期，预留 20 ms 的稳定余量。

#define STAIR_WALK_TEST_STEP_H_MM             60.0f // 爬楼梯时的最大抬腿高度，单位 mm；较高的抬腿用于跨过台阶边缘。
#define STAIR_WALK_TEST_STEP_LEN_MM           50.0f // 每一步在前后方向上的步长参数，单位 mm；正值表示向前行走。
#define STAIR_WALK_TEST_SPEED_FREQ            0.03f // 每次步态更新增加的相位量；数值越大，一个完整步态周期完成得越快。
#define STAIR_WALK_TEST_CG_BASE_X_MM          0.0f // 行走时机身重心在 X（前后）方向的基础偏移，单位 mm，用于提高爬台阶稳定性。
#define STAIR_WALK_TEST_IMU_GAIN_MM           0.0f // 首轮调试关闭 IMU 前后纠偏，先单独观察后腿轨迹与髋关节运动。
// #define STAIR_WALK_TEST_STEP_H_MM             0.0f // 爬楼梯时的最大抬腿高度，单位 mm；较高的抬腿用于跨过台阶边缘。
// #define STAIR_WALK_TEST_STEP_LEN_MM           0.0f // 每一步在前后方向上的步长参数，单位 mm；正值表示向前行走。
// #define STAIR_WALK_TEST_SPEED_FREQ            0.0f // 每次步态更新增加的相位量；数值越大，一个完整步态周期完成得越快。
// #define STAIR_WALK_TEST_CG_BASE_X_MM          0.0f // 行走时机身重心在 X（前后）方向的基础偏移，单位 mm，用于提高爬台阶稳定性。
// #define STAIR_WALK_TEST_IMU_GAIN_MM           0.0f // IMU 姿态补偿增益：把俯仰/横滚角换算为足端或重心修正量，数值越大姿态修正越强。
#define STAIR_WALK_TEST_PITCH_FILTER_ALPHA    0.15f // IMU 一阶低通滤波中新测量值的权重；越小越平滑，但姿态响应越慢（俯仰和横滚共用）。
#define STAIR_WALK_MIN_CYCLES                 3U
#define STAIR_WALK_LEVEL_PITCH_DEG            4.0f  // “机身恢复水平”的俯仰角阈值：滤波后 |pitch| 不得超过 3°。
#define STAIR_WALK_LEVEL_ROLL_DEG             6.0f  // “机身恢复水平”的横滚角阈值：滤波后 |roll| 不得超过 6°。
#define STAIR_WALK_LEVEL_STABLE_MS            2000U // pitch 和 roll 同时满足水平阈值后，必须连续稳定 2 s 才判定上高台完成。

static uint32_t s_last_gait_ms;       // 上一次更新爬楼梯步态的系统时间，单位 ms，用于控制步态按设定周期更新。
static uint32_t s_level_start_ms;     // 检测到机身进入水平范围时的起始时间，单位 ms，用于判断是否已连续稳定 800 ms。
static float s_filtered_pitch_deg;    // 一阶低通滤波后的俯仰角 pitch，单位 °，用于姿态补偿和爬楼梯完成判定。
static float s_filtered_roll_deg;     // 一阶低通滤波后的横滚角 roll，单位 °，用于姿态补偿和爬楼梯完成判定。
static uint8_t s_has_pitch_filter;    // 俯仰角滤波器初始化标志：0 表示尚无有效初值，1 表示已经取得初值。
static uint8_t s_has_roll_filter;     // 横滚角滤波器初始化标志：0 表示尚无有效初值，1 表示已经取得初值。
static uint8_t s_is_running;          // 爬楼梯运行标志：1 表示正在执行，0 表示未启动或已经完成。
static uint8_t s_cycle_count;         // 已经完成的完整行走周期数；达到最少周期数后才允许判断爬楼梯结束。
static uint8_t s_support_phase;

static float StairWalkTest_GetPitchDeg(void)
{
    Jy61PImuStatus_t imu;

    if (Jy61PImu_GetStatus(&imu) == 0U)
    {
        return s_filtered_pitch_deg;
    }

    if (s_has_pitch_filter == 0U)
    {
        s_filtered_pitch_deg = imu.pitch_deg;
        s_has_pitch_filter = 1U;
    }
    else
    {
        s_filtered_pitch_deg =
            (s_filtered_pitch_deg * (1.0f - STAIR_WALK_TEST_PITCH_FILTER_ALPHA)) +
            (imu.pitch_deg * STAIR_WALK_TEST_PITCH_FILTER_ALPHA);
    }

    return s_filtered_pitch_deg;
}

static float StairWalkTest_GetRollDeg(void)
{
    Jy61PImuStatus_t imu;

    if (Jy61PImu_GetStatus(&imu) == 0U)
    {
        return s_filtered_roll_deg;
    }

    if (s_has_roll_filter == 0U)
    {
        s_filtered_roll_deg = -imu.roll_deg;
        s_has_roll_filter = 1U;
    }
    else
    {
        s_filtered_roll_deg =
            (s_filtered_roll_deg * (1.0f - STAIR_WALK_TEST_PITCH_FILTER_ALPHA)) +
            (-imu.roll_deg * STAIR_WALK_TEST_PITCH_FILTER_ALPHA);
    }

    return s_filtered_roll_deg;
}

void StairWalk_Init(void)
{
    s_last_gait_ms = HAL_GetTick();
    s_level_start_ms = 0U;
    s_filtered_pitch_deg = 0.0f;
    s_filtered_roll_deg = 0.0f;
    s_has_pitch_filter = 0U;
    s_has_roll_filter = 0U;
    s_is_running = 0U;
    s_cycle_count = 0U;
    s_support_phase = 0U;
}

void StairWalk_Start(void)
{
    DogGait_SetWalkParams(STAIR_WALK_TEST_STEP_H_MM,
                          STAIR_WALK_TEST_STEP_LEN_MM,
                          STAIR_WALK_TEST_SPEED_FREQ,
                          STAIR_WALK_TEST_CG_BASE_X_MM,
                          STAIR_WALK_TEST_IMU_GAIN_MM);
    /* 先装载本次楼梯参数，再按新的基础重心复位，避免沿用上一次 walk 的重心初值。 */
    DogGait_ResetWalk();

    s_last_gait_ms = HAL_GetTick();
    s_filtered_pitch_deg = 0.0f;
    s_filtered_roll_deg = 0.0f;
    s_has_pitch_filter = 0U;
    s_has_roll_filter = 0U;
    s_level_start_ms = 0U;
    s_cycle_count = 0U;
    s_support_phase = 0U;
    s_is_running = 1U;
}

void StairWalk_Update(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (s_is_running == 0U)
    {
        return;
    }

    if ((uint32_t)(now_ms - s_last_gait_ms) >= STAIR_WALK_TEST_GAIT_PERIOD_MS)
    {
        float pitch_deg = StairWalkTest_GetPitchDeg();
        float roll_deg = StairWalkTest_GetRollDeg();

        s_last_gait_ms = now_ms;

        if (s_support_phase != 0U)
        {
            DogGait_UpdateWalkSupport(STAIR_WALK_TEST_GAIT_MOVE_MS);
            return;
        }

        DogGait_UpdateWalk(STAIR_WALK_TEST_GAIT_MOVE_MS, pitch_deg, roll_deg);

        if (DogGait_IsWalkCycleDone() != 0U)
        {
            s_cycle_count++;

            if ((s_cycle_count >= STAIR_WALK_MIN_CYCLES) &&
                (s_support_phase == 0U))
            {
                DogGait_StartWalkSupportPhase();
                s_support_phase = 1U;
                s_level_start_ms = 0U;
            }
        }
    }
}

uint8_t StairWalk_IsFinished(void)
 {
    uint32_t now_ms = HAL_GetTick();
    Jy61PImuStatus_t imu;

    if (s_is_running == 0U)
    {
        s_level_start_ms = 0U;
        return 0U;
    }

    if ((s_cycle_count < STAIR_WALK_MIN_CYCLES) ||
        (s_support_phase == 0U))
    {
        s_level_start_ms = 0U;
        return 0U;
    }

    if ((DogGait_IsWalkSupportReady() == 0U) ||
        (Jy61PImu_GetStatus(&imu) == 0U) ||
        (s_has_pitch_filter == 0U) ||
        (s_has_roll_filter == 0U))
    {
        /* Support check failed: resume WALK and try again at the next cycle. */
        DogGait_ResumeWalkFromSupportPhase();
        s_support_phase = 0U;
        s_level_start_ms = 0U;
        return 0U;
    }

    if ((fabsf(s_filtered_pitch_deg) <= STAIR_WALK_LEVEL_PITCH_DEG) &&
        (fabsf(s_filtered_roll_deg) <= STAIR_WALK_LEVEL_ROLL_DEG))
    {
        if (s_level_start_ms == 0U)
        {
            s_level_start_ms = now_ms;
            return 0U;
        }

        if ((uint32_t)(now_ms - s_level_start_ms) >=
            STAIR_WALK_LEVEL_STABLE_MS)
        {
            s_is_running = 0U;
            return 1U;
        }
    }
    else
    {
        /* The robot is not level yet: continue with the next WALK cycle. */
        DogGait_ResumeWalkFromSupportPhase();
        s_support_phase = 0U;
        s_level_start_ms = 0U;
    }

    return 0U;
}

uint8_t StairWalk_IsRunning(void)
{
    return s_is_running;
}
