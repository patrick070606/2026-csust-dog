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
#define STAIR_WALK_PITCH_DIFF_0_MIN_DEG       (-1.0f)
#define STAIR_WALK_PITCH_DIFF_0_MAX_DEG        1.5f
#define STAIR_WALK_PITCH_DIFF_30_MIN_DEG      (-3.3f)
#define STAIR_WALK_PITCH_DIFF_30_MAX_DEG      (-1.3f)
#define STAIR_WALK_PITCH_DIFF_60_MIN_DEG      (-7.0f)
#define STAIR_WALK_PITCH_DIFF_60_MAX_DEG      (-4.0f)
#define STAIR_WALK_STAGE_CONFIRM_SAMPLES       3U
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
static uint8_t s_support_phase;

typedef enum
{
    /* Front/rear support heights: 0/0, 30/0, 60/0, 60/30, 90/30, 90/60, 90/90. */
    STAIR_WALK_STAGE_0_FRONT_0_REAR_0 = 0,
    STAIR_WALK_STAGE_1_FRONT_30_REAR_0,
    STAIR_WALK_STAGE_2_FRONT_60_REAR_0,
    STAIR_WALK_STAGE_3_FRONT_60_REAR_30,
    STAIR_WALK_STAGE_4_FRONT_90_REAR_30,
    STAIR_WALK_STAGE_5_FRONT_90_REAR_60,
    STAIR_WALK_STAGE_6_FRONT_90_REAR_90,
} StairWalkStage_t;

typedef enum
{
    STAIR_WALK_PITCH_BAND_UNKNOWN = 0U,
    STAIR_WALK_PITCH_BAND_DIFF_0_MM,
    STAIR_WALK_PITCH_BAND_DIFF_30_MM,
    STAIR_WALK_PITCH_BAND_DIFF_60_MM,
} StairWalkPitchBand_t;

volatile uint8_t g_stair_walk_stage;
volatile uint8_t g_stair_walk_pitch_band_mm;
volatile uint8_t g_stair_walk_pitch_stable_samples;

static StairWalkPitchBand_t StairWalk_GetPitchBand(float pitch_deg)
{
    if ((pitch_deg >= STAIR_WALK_PITCH_DIFF_0_MIN_DEG) &&
        (pitch_deg <= STAIR_WALK_PITCH_DIFF_0_MAX_DEG))
    {
        return STAIR_WALK_PITCH_BAND_DIFF_0_MM;
    }

    if ((pitch_deg >= STAIR_WALK_PITCH_DIFF_30_MIN_DEG) &&
        (pitch_deg <= STAIR_WALK_PITCH_DIFF_30_MAX_DEG))
    {
        return STAIR_WALK_PITCH_BAND_DIFF_30_MM;
    }

    if ((pitch_deg >= STAIR_WALK_PITCH_DIFF_60_MIN_DEG) &&
        (pitch_deg <= STAIR_WALK_PITCH_DIFF_60_MAX_DEG))
    {
        return STAIR_WALK_PITCH_BAND_DIFF_60_MM;
    }

    return STAIR_WALK_PITCH_BAND_UNKNOWN;
}

static StairWalkPitchBand_t StairWalk_GetExpectedNextPitchBand(void)
{
    switch ((StairWalkStage_t)g_stair_walk_stage)
    {
    case STAIR_WALK_STAGE_0_FRONT_0_REAR_0:
        return STAIR_WALK_PITCH_BAND_DIFF_30_MM;
    case STAIR_WALK_STAGE_1_FRONT_30_REAR_0:
        return STAIR_WALK_PITCH_BAND_DIFF_60_MM;
    case STAIR_WALK_STAGE_2_FRONT_60_REAR_0:
        return STAIR_WALK_PITCH_BAND_DIFF_30_MM;
    case STAIR_WALK_STAGE_3_FRONT_60_REAR_30:
        return STAIR_WALK_PITCH_BAND_DIFF_60_MM;
    case STAIR_WALK_STAGE_4_FRONT_90_REAR_30:
        return STAIR_WALK_PITCH_BAND_DIFF_30_MM;
    case STAIR_WALK_STAGE_5_FRONT_90_REAR_60:
        return STAIR_WALK_PITCH_BAND_DIFF_0_MM;
    default:
        return STAIR_WALK_PITCH_BAND_UNKNOWN;
    }
}

static void StairWalk_ApplyStageSupportHeights(void)
{
    float front_height_mm;
    float rear_height_mm;

    switch ((StairWalkStage_t)g_stair_walk_stage)
    {
    case STAIR_WALK_STAGE_1_FRONT_30_REAR_0:
        //front_height_mm = 30.0f;
        front_height_mm = 0.0f;
        rear_height_mm = 0.0f;
        break;
    case STAIR_WALK_STAGE_2_FRONT_60_REAR_0:
        //front_height_mm = 60.0f;
        front_height_mm = 0.0f;
        rear_height_mm = 0.0f;
        break;
    case STAIR_WALK_STAGE_3_FRONT_60_REAR_30:
        front_height_mm = 0.0f;
        rear_height_mm = 0.0f;
        break;
    case STAIR_WALK_STAGE_4_FRONT_90_REAR_30:
        //front_height_mm = 90.0f;
        front_height_mm = 0.0f;
        rear_height_mm = 30.0f;
        break;
    case STAIR_WALK_STAGE_5_FRONT_90_REAR_60:
        //front_height_mm = 90.0f;
        front_height_mm = 0.0f;
        //rear_height_mm = 60.0f;
        rear_height_mm = 0.0f;
        break;
    case STAIR_WALK_STAGE_6_FRONT_90_REAR_90:
        //front_height_mm = 90.0f;
        front_height_mm = 0.0f;
        //rear_height_mm = 90.0f;
        rear_height_mm = 0.0f;
        break;
    case STAIR_WALK_STAGE_0_FRONT_0_REAR_0:
    default:
        front_height_mm = 0.0f;
        rear_height_mm = 0.0f;
        break;
    }

    DogGait_SetWalkSupportHeights(front_height_mm, rear_height_mm);
}

static void StairWalk_UpdateStageAtFrontPreSwing(float pitch_deg)
{
    StairWalkPitchBand_t measured_band;
    StairWalkPitchBand_t expected_band;

    if (DogGait_IsWalkLeftFrontPreSwing() == 0U)
    {
        /* Do not combine samples from two different gait instants. */
        g_stair_walk_pitch_stable_samples = 0U;
        return;
    }

    measured_band = StairWalk_GetPitchBand(pitch_deg);
    expected_band = StairWalk_GetExpectedNextPitchBand();
    g_stair_walk_pitch_band_mm =
        (measured_band == STAIR_WALK_PITCH_BAND_DIFF_0_MM) ? 0U :
        (measured_band == STAIR_WALK_PITCH_BAND_DIFF_30_MM) ? 30U :
        (measured_band == STAIR_WALK_PITCH_BAND_DIFF_60_MM) ? 60U : 255U;

    if ((expected_band == STAIR_WALK_PITCH_BAND_UNKNOWN) ||
        (measured_band != expected_band))
    {
        g_stair_walk_pitch_stable_samples = 0U;
        return;
    }

    if (g_stair_walk_pitch_stable_samples < STAIR_WALK_STAGE_CONFIRM_SAMPLES)
    {
        g_stair_walk_pitch_stable_samples++;
    }

    if (g_stair_walk_pitch_stable_samples < STAIR_WALK_STAGE_CONFIRM_SAMPLES)
    {
        return;
    }

    g_stair_walk_pitch_stable_samples = 0U;
    g_stair_walk_stage++;
    StairWalk_ApplyStageSupportHeights();

    if (g_stair_walk_stage > STAIR_WALK_STAGE_6_FRONT_90_REAR_90)
    {
        g_stair_walk_stage = STAIR_WALK_STAGE_6_FRONT_90_REAR_90;
    }
}

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
    s_support_phase = 0U;
    g_stair_walk_stage = STAIR_WALK_STAGE_0_FRONT_0_REAR_0;
    g_stair_walk_pitch_band_mm = 255U;
    g_stair_walk_pitch_stable_samples = 0U;
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
    StairWalk_ApplyStageSupportHeights();

    s_last_gait_ms = HAL_GetTick();
    s_filtered_pitch_deg = 0.0f;
    s_filtered_roll_deg = 0.0f;
    s_has_pitch_filter = 0U;
    s_has_roll_filter = 0U;
    s_level_start_ms = 0U;
    s_support_phase = 0U;
    g_stair_walk_stage = STAIR_WALK_STAGE_0_FRONT_0_REAR_0;
    g_stair_walk_pitch_band_mm = 255U;
    g_stair_walk_pitch_stable_samples = 0U;
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
        Jy61PImuStatus_t imu;
        uint8_t pitch_valid =
            ((s_has_pitch_filter != 0U) &&
             (Jy61PImu_GetStatus(&imu) != 0U)) ? 1U : 0U;

        s_last_gait_ms = now_ms;

        if (s_support_phase != 0U)
        {
            DogGait_UpdateWalkSupport(STAIR_WALK_TEST_GAIT_MOVE_MS);
            return;
        }

        if (pitch_valid != 0U)
        {
            StairWalk_UpdateStageAtFrontPreSwing(pitch_deg);
        }
        else
        {
            /* Never advance a stair stage from a cached pitch value. */
            g_stair_walk_pitch_stable_samples = 0U;
        }

        if (s_support_phase == 0U)
        {
            DogGait_UpdateWalk(STAIR_WALK_TEST_GAIT_MOVE_MS, pitch_deg, roll_deg);

            if (g_stair_walk_stage == STAIR_WALK_STAGE_6_FRONT_90_REAR_90)
            {
                /* Apply the final 90/90 coordinates once before holding. */
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

    if (s_support_phase == 0U)
    {
        s_level_start_ms = 0U;
        return 0U;
    }

    if ((DogGait_IsWalkSupportReady() == 0U) ||
        (Jy61PImu_GetStatus(&imu) == 0U) ||
        (s_has_pitch_filter == 0U) ||
        (s_has_roll_filter == 0U))
    {
        /* The final stage has already been reached; hold rather than walking on. */
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
        /* Do not resume the generic walk after stage 6, otherwise it can overstep. */
        s_level_start_ms = 0U;
    }

    return 0U;
}

uint8_t StairWalk_IsRunning(void)
{
    return s_is_running;
}
