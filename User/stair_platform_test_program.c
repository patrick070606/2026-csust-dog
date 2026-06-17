#include "stair_platform_test_program.h"

#include "dog_gait.h"
#include "dog_servo.h"
#include "main.h"

#define STAIR_TEST_GAIT_MOVE_MS                 100U
#define STAIR_TEST_SERVO_READY_MS               2000U
#define STAIR_TEST_CENTER_MOVE_MS               5000U
#define STAIR_TEST_CENTER_WAIT_MS               6500U
#define STAIR_TEST_STAND_MOVE_MS                2000U
#define STAIR_TEST_STAND_WAIT_MS                2500U

#define STAIR_TEST_PLATFORM_HEIGHT_MM           30.0f
#define STAIR_TEST_CLEARANCE_HEIGHT_MM          40.0f
#define STAIR_TEST_FRONT_CLEARANCE_HEIGHT_MM    55.0f
#define STAIR_TEST_DESCENT_CLEARANCE_MM         15.0f
#define STAIR_TEST_LIFT_FORWARD_MM              40.0f
#define STAIR_TEST_REAR_LIFT_FORWARD_MM         70.0f
#define STAIR_TEST_STEP_FORWARD_MM              35.0f
#define STAIR_TEST_BODY_ADVANCE_MM              25.0f
#define STAIR_TEST_REAR_PLACE_X_MM              45.0f
#define STAIR_TEST_PITCH_BIAS_DEG               0.0f
#define STAIR_TEST_POSE_MOVE_MS                 700U
#define STAIR_TEST_POSE_HOLD_MS                 900U
#define STAIR_TEST_SETTLE_MOVE_MS               1000U
#define STAIR_TEST_SETTLE_HOLD_MS               1300U

#define STAIR_TEST_PLATFORM_DISTANCE_MM         600U
#define STAIR_TEST_PLATFORM_EST_MM_PER_CYCLE    25U
#define STAIR_TEST_PLATFORM_FORWARD_CYCLES \
    ((STAIR_TEST_PLATFORM_DISTANCE_MM + STAIR_TEST_PLATFORM_EST_MM_PER_CYCLE - 1U) / \
     STAIR_TEST_PLATFORM_EST_MM_PER_CYCLE)
#define STAIR_TEST_PLATFORM_GAIT_PERIOD_MS      150U
#define STAIR_TEST_PLATFORM_STEP_H_MM           15.0f
#define STAIR_TEST_PLATFORM_STEP_R_MM           25.0f
#define STAIR_TEST_PLATFORM_SPEED_FREQ          0.125f
#define STAIR_TEST_PLATFORM_UPDATES_PER_CYCLE   8U
#define STAIR_TEST_FRONT_TROT_PREPARE_MS        2000U
#define STAIR_TEST_FRONT_WALK_UPDATES           32U

typedef enum
{
    STAIR_TEST_DONE = 0,              // 测试结束或未运行，保持最后姿态
    STAIR_TEST_ASCEND_SETTLE,         // 上台前复位足端偏移，让身体稳定
    STAIR_TEST_ASCEND_LF_LIFT,        // 左前腿向前抬高，准备跨上台面
    STAIR_TEST_ASCEND_LF_PLACE,       // 左前腿落到台面高度
    STAIR_TEST_ASCEND_RF_LIFT,        // 右前腿向前抬高，准备跨上台面
    STAIR_TEST_ASCEND_RF_PLACE,       // 右前腿落到台面高度
    STAIR_TEST_FRONT_TROT_PREPARE,    // 前腿上台后用2秒缓慢进入小跑起始姿态
    STAIR_TEST_FRONT_WALK_8_STEPS,    // 使用平台前进小跑逻辑约走8步
    STAIR_TEST_ASCEND_BODY_ADVANCE,   // 前腿站稳后身体向前推进，为后腿上台让位
    STAIR_TEST_ASCEND_LB_LIFT,        // 左后腿抬高，准备跨上台面
    STAIR_TEST_ASCEND_LB_PLACE,       // 左后腿落到台面高度
    STAIR_TEST_ASCEND_RB_LIFT,        // 右后腿抬高，准备跨上台面
    STAIR_TEST_ASCEND_RB_PLACE,       // 右后腿落到台面高度
    STAIR_TEST_PLATFORM_SETTLE,       // 四腿都在台面后复位偏移并稳定
    STAIR_TEST_PLATFORM_FORWARD,      // 在台面上持续小跑前进
    STAIR_TEST_DESCEND_SETTLE,        // 下台前复位足端偏移，让身体稳定
    STAIR_TEST_DESCEND_LF_LIFT,       // 左前腿向前下探，准备下台
    STAIR_TEST_DESCEND_LF_PLACE,      // 左前腿落到低一级地面
    STAIR_TEST_DESCEND_RF_LIFT,       // 右前腿向前下探，准备下台
    STAIR_TEST_DESCEND_RF_PLACE,      // 右前腿落到低一级地面
    STAIR_TEST_DESCEND_BODY_ADVANCE,  // 前腿下台后身体向前推进，为后腿下台让位
    STAIR_TEST_DESCEND_LB_LIFT,       // 左后腿抬起并向前下探
    STAIR_TEST_DESCEND_LB_PLACE,      // 左后腿落到低一级地面
    STAIR_TEST_DESCEND_RB_LIFT,       // 右后腿抬起并向前下探
    STAIR_TEST_DESCEND_RB_PLACE,      // 右后腿落到低一级地面
    STAIR_TEST_FINAL_SETTLE,          // 下台完成后复位足端偏移并稳定
} StairPlatformTestState_t;

static StairPlatformTestState_t s_state = STAIR_TEST_DONE;
static uint32_t s_state_start_ms;
static uint32_t s_last_gait_ms;
static uint16_t s_platform_forward_cycles;
static uint8_t s_platform_forward_updates;
static uint8_t s_front_walk_updates;
static DogGaitStairTarget_t s_stair_targets[DOG_GAIT_STAIR_LEG_COUNT];

static void StairPlatformTest_ResetTargets(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_STAIR_LEG_COUNT; i++)
    {
        s_stair_targets[i].x_offset_mm = 0.0f;
        s_stair_targets[i].y_offset_mm = 0.0f;
        s_stair_targets[i].hip_bias_deg = 0.0f;
    }
}

static void StairPlatformTest_SetPitchBias(float bias_deg)
{
    s_stair_targets[DOG_GAIT_STAIR_LEG_LF].hip_bias_deg = bias_deg;
    s_stair_targets[DOG_GAIT_STAIR_LEG_RF].hip_bias_deg = bias_deg;
    s_stair_targets[DOG_GAIT_STAIR_LEG_LB].hip_bias_deg = -bias_deg;
    s_stair_targets[DOG_GAIT_STAIR_LEG_RB].hip_bias_deg = -bias_deg;
}

static void StairPlatformTest_ApplyTargets(uint16_t move_ms)
{
    DogGait_SetStairPose(s_stair_targets, move_ms);
}

static void StairPlatformTest_ApplyTargetsWithBase(DogGaitStairBase_t base, uint16_t move_ms)
{
    DogGait_SetStairPoseWithBase(s_stair_targets, base, move_ms);
}

static void StairPlatformTest_AdvanceBody(void)
{
    for (uint8_t i = 0; i < DOG_GAIT_STAIR_LEG_COUNT; i++)
    {
        s_stair_targets[i].x_offset_mm -= STAIR_TEST_BODY_ADVANCE_MM;
    }
}

static void StairPlatformTest_SetState(StairPlatformTestState_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_start_ms = now_ms;

    if (state == STAIR_TEST_ASCEND_SETTLE)
    {
        StairPlatformTest_ResetTargets();
        StairPlatformTest_ApplyTargets(STAIR_TEST_SETTLE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_LF_LIFT)
    {
        StairPlatformTest_SetPitchBias(STAIR_TEST_PITCH_BIAS_DEG);
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm = STAIR_TEST_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm = STAIR_TEST_FRONT_CLEARANCE_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_LF_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm = STAIR_TEST_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm = STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_RF_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm = STAIR_TEST_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm = STAIR_TEST_FRONT_CLEARANCE_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_RF_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm = STAIR_TEST_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm = STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_FRONT_TROT_PREPARE)
    {
        DogGait_SetTrotParams(STAIR_TEST_PLATFORM_STEP_H_MM,
                              STAIR_TEST_PLATFORM_STEP_R_MM,
                              STAIR_TEST_PLATFORM_SPEED_FREQ);
        DogGait_UpdateTrot(STAIR_TEST_FRONT_TROT_PREPARE_MS);
        s_last_gait_ms = now_ms;
    }
    else if (state == STAIR_TEST_FRONT_WALK_8_STEPS)
    {
        s_front_walk_updates = 0U;
        DogGait_SetTrotParams(STAIR_TEST_PLATFORM_STEP_H_MM,
                              STAIR_TEST_PLATFORM_STEP_R_MM,
                              STAIR_TEST_PLATFORM_SPEED_FREQ);
        s_last_gait_ms = now_ms;
    }
    else if (state == STAIR_TEST_ASCEND_BODY_ADVANCE)
    {
        StairPlatformTest_SetPitchBias(-STAIR_TEST_PITCH_BIAS_DEG);
        StairPlatformTest_AdvanceBody();
        StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK,
                                               STAIR_TEST_SETTLE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_LB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm = STAIR_TEST_REAR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm = STAIR_TEST_CLEARANCE_HEIGHT_MM;
        StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK,
                                               STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_LB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm = STAIR_TEST_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm = STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK,
                                               STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_RB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm = STAIR_TEST_REAR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm = STAIR_TEST_CLEARANCE_HEIGHT_MM;
        StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK,
                                               STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_ASCEND_RB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm = STAIR_TEST_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm = STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK,
                                               STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_PLATFORM_SETTLE)
    {
        StairPlatformTest_ResetTargets();
        StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK,
                                               STAIR_TEST_SETTLE_MOVE_MS);
    }
    else if (state == STAIR_TEST_PLATFORM_FORWARD)
    {
        s_platform_forward_cycles = 0U;
        s_platform_forward_updates = 0U;
        DogGait_SetTrotParams(STAIR_TEST_PLATFORM_STEP_H_MM,
                              STAIR_TEST_PLATFORM_STEP_R_MM,
                              STAIR_TEST_PLATFORM_SPEED_FREQ);
        s_last_gait_ms = now_ms;
    }
    else if (state == STAIR_TEST_DESCEND_SETTLE)
    {
        StairPlatformTest_ResetTargets();
        StairPlatformTest_ApplyTargets(STAIR_TEST_SETTLE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_LF_LIFT)
    {
        StairPlatformTest_SetPitchBias(STAIR_TEST_PITCH_BIAS_DEG);
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm = STAIR_TEST_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm = STAIR_TEST_DESCENT_CLEARANCE_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_LF_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm = STAIR_TEST_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm = -STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_RF_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm = STAIR_TEST_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm = STAIR_TEST_DESCENT_CLEARANCE_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_RF_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm = STAIR_TEST_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm = -STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_BODY_ADVANCE)
    {
        StairPlatformTest_SetPitchBias(-STAIR_TEST_PITCH_BIAS_DEG);
        StairPlatformTest_AdvanceBody();
        StairPlatformTest_ApplyTargets(STAIR_TEST_SETTLE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_LB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm = STAIR_TEST_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm = STAIR_TEST_DESCENT_CLEARANCE_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_LB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm = STAIR_TEST_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm = -STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_RB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm = STAIR_TEST_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm = STAIR_TEST_DESCENT_CLEARANCE_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_DESCEND_RB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm = STAIR_TEST_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm = -STAIR_TEST_PLATFORM_HEIGHT_MM;
        StairPlatformTest_ApplyTargets(STAIR_TEST_POSE_MOVE_MS);
    }
    else if (state == STAIR_TEST_FINAL_SETTLE)
    {
        StairPlatformTest_ResetTargets();
        StairPlatformTest_ApplyTargets(STAIR_TEST_SETTLE_MOVE_MS);
    }
    else
    {
        DogGait_AllStand(STAIR_TEST_SETTLE_MOVE_MS);
    }
}

void StairPlatformTest_Init(void)
{
    uint32_t now_ms;

    HAL_Delay(STAIR_TEST_SERVO_READY_MS);

    DogServo_AllCenter(STAIR_TEST_CENTER_MOVE_MS);
    HAL_Delay(STAIR_TEST_CENTER_WAIT_MS);

    DogGait_SetLoadMode(DOG_GAIT_LOAD_WITH_PAYLOAD);
    DogGait_Init();
    DogGait_GotoStandPose(STAIR_TEST_STAND_MOVE_MS);
    HAL_Delay(STAIR_TEST_STAND_WAIT_MS);

    StairPlatformTest_ResetTargets();
    s_platform_forward_cycles = 0U;
    s_platform_forward_updates = 0U;
    s_front_walk_updates = 0U;
    now_ms = HAL_GetTick();
    s_last_gait_ms = now_ms;
    StairPlatformTest_SetState(STAIR_TEST_ASCEND_SETTLE, now_ms);
}

void StairPlatformTest_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = (uint32_t)(now_ms - s_state_start_ms);

    if (s_state == STAIR_TEST_FRONT_WALK_8_STEPS)
    {
        if (s_front_walk_updates >= STAIR_TEST_FRONT_WALK_UPDATES)
        {
            StairPlatformTest_SetState(STAIR_TEST_ASCEND_BODY_ADVANCE, now_ms);
            return;
        }

        if ((uint32_t)(now_ms - s_last_gait_ms) >= STAIR_TEST_PLATFORM_GAIT_PERIOD_MS)
        {
            s_last_gait_ms = now_ms;
            DogGait_UpdateTrot(STAIR_TEST_GAIT_MOVE_MS);
            s_front_walk_updates++;
        }

        return;
    }

    if (s_state == STAIR_TEST_PLATFORM_FORWARD)
    {
        if (s_platform_forward_cycles >= STAIR_TEST_PLATFORM_FORWARD_CYCLES)
        {
            StairPlatformTest_SetState(STAIR_TEST_DESCEND_SETTLE, now_ms);
            return;
        }

        if ((uint32_t)(now_ms - s_last_gait_ms) >= STAIR_TEST_PLATFORM_GAIT_PERIOD_MS)
        {
            s_last_gait_ms = now_ms;
            DogGait_UpdateTrot(STAIR_TEST_GAIT_MOVE_MS);
            s_platform_forward_updates++;
            if (s_platform_forward_updates >= STAIR_TEST_PLATFORM_UPDATES_PER_CYCLE)
            {
                s_platform_forward_updates = 0U;
                s_platform_forward_cycles++;
            }
        }

        return;
    }

    if ((s_state == STAIR_TEST_ASCEND_SETTLE) &&
        (elapsed_ms >= STAIR_TEST_SETTLE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_LF_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_LF_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_LF_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_LF_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_RF_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_RF_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_RF_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_RF_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_FRONT_TROT_PREPARE, now_ms);
    }
    else if ((s_state == STAIR_TEST_FRONT_TROT_PREPARE) &&
             (elapsed_ms >= STAIR_TEST_FRONT_TROT_PREPARE_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_FRONT_WALK_8_STEPS, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_BODY_ADVANCE) &&
             (elapsed_ms >= STAIR_TEST_SETTLE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_LB_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_LB_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_LB_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_LB_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_RB_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_RB_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_ASCEND_RB_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_ASCEND_RB_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_PLATFORM_SETTLE, now_ms);
    }
    else if ((s_state == STAIR_TEST_PLATFORM_SETTLE) &&
             (elapsed_ms >= STAIR_TEST_SETTLE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_PLATFORM_FORWARD, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_SETTLE) &&
             (elapsed_ms >= STAIR_TEST_SETTLE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_LF_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_LF_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_LF_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_LF_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_RF_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_RF_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_RF_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_RF_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_BODY_ADVANCE, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_BODY_ADVANCE) &&
             (elapsed_ms >= STAIR_TEST_SETTLE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_LB_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_LB_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_LB_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_LB_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_RB_LIFT, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_RB_LIFT) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DESCEND_RB_PLACE, now_ms);
    }
    else if ((s_state == STAIR_TEST_DESCEND_RB_PLACE) &&
             (elapsed_ms >= STAIR_TEST_POSE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_FINAL_SETTLE, now_ms);
    }
    else if ((s_state == STAIR_TEST_FINAL_SETTLE) &&
             (elapsed_ms >= STAIR_TEST_SETTLE_HOLD_MS))
    {
        StairPlatformTest_SetState(STAIR_TEST_DONE, now_ms);
    }
}
