#include "stair_walk.h"

#include "dog_gait.h"
#include "jy61p_imu.h"
#include "main.h"

#include <math.h>

#define STAIR_WALK_TEST_GAIT_PERIOD_MS        100U
#define STAIR_WALK_TEST_GAIT_MOVE_MS          80U

#define STAIR_WALK_TEST_STEP_H_MM             60.0f
#define STAIR_WALK_TEST_STEP_LEN_MM           30.0f
#define STAIR_WALK_TEST_SPEED_FREQ            0.03f
#define STAIR_WALK_TEST_CG_BASE_X_MM          30.0f
#define STAIR_WALK_TEST_IMU_GAIN_MM           100.0f
#define STAIR_WALK_TEST_PITCH_FILTER_ALPHA    0.15f
#define STAIR_WALK_MIN_CYCLES                 2U
#define STAIR_WALK_LEVEL_PITCH_DEG            5.0f
#define STAIR_WALK_LEVEL_ROLL_DEG             6.0f
#define STAIR_WALK_LEVEL_STABLE_MS            800U

static uint32_t s_last_gait_ms;
static uint32_t s_level_start_ms;
static float s_filtered_pitch_deg;
static float s_filtered_roll_deg;
static uint8_t s_has_pitch_filter;
static uint8_t s_has_roll_filter;
static uint8_t s_is_running;
static uint8_t s_cycle_count;

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
        s_filtered_roll_deg = imu.roll_deg;
        s_has_roll_filter = 1U;
    }
    else
    {
        s_filtered_roll_deg =
            (s_filtered_roll_deg * (1.0f - STAIR_WALK_TEST_PITCH_FILTER_ALPHA)) +
            (imu.roll_deg * STAIR_WALK_TEST_PITCH_FILTER_ALPHA);
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
}

void StairWalk_Start(void)
{
    DogGait_ResetWalk();
    DogGait_SetWalkParams(STAIR_WALK_TEST_STEP_H_MM,
                          STAIR_WALK_TEST_STEP_LEN_MM,
                          STAIR_WALK_TEST_SPEED_FREQ,
                          STAIR_WALK_TEST_CG_BASE_X_MM,
                          STAIR_WALK_TEST_IMU_GAIN_MM);

    s_last_gait_ms = HAL_GetTick();
    s_filtered_pitch_deg = 0.0f;
    s_filtered_roll_deg = 0.0f;
    s_has_pitch_filter = 0U;
    s_has_roll_filter = 0U;
    s_level_start_ms = 0U;
    s_cycle_count = 0U;
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
        DogGait_UpdateWalk(STAIR_WALK_TEST_GAIT_MOVE_MS, pitch_deg, roll_deg);

        if (DogGait_IsWalkCycleDone() != 0U)
        {
            s_cycle_count++;
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

    if (s_cycle_count < STAIR_WALK_MIN_CYCLES)
    {
        s_level_start_ms = 0U;
        return 0U;
    }

    if (Jy61PImu_GetStatus(&imu) == 0U)
    {
        s_level_start_ms = 0U;
        return 0U;
    }

    if ((s_has_pitch_filter == 0U) || (s_has_roll_filter == 0U))
    {
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

        if ((uint32_t)(now_ms - s_level_start_ms) >= STAIR_WALK_LEVEL_STABLE_MS)
        {
            s_is_running = 0U;
            return 1U;
        }

        return 0U;
    }

    s_level_start_ms = 0U;
    return 0U;
}

uint8_t StairWalk_IsRunning(void)
{
    return s_is_running;
}
