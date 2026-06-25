#include "dog_task.h"

#include "dog_gait.h"
#include "dog_servo.h"
#include "image_command.h"
#include "main.h"
#include "throw_servo.h"
#include "usart.h"

#include <stdio.h>

#define DOG_TASK_GAIT_PERIOD_MS        150U
#define DOG_TASK_GAIT_MOVE_MS          100U
#define DOG_TASK_LED_ON_STATE          GPIO_PIN_SET
#define DOG_TASK_LED_OFF_STATE         GPIO_PIN_RESET
#define DOG_TASK_COLOR_PAUSE_MS        2000U
#define DOG_TASK_COLOR_PAUSE_HOLD_MS   150U
#define DOG_TASK_VISION_COLOR_STOP_TEST_ENABLE 0U
#define DOG_TASK_VISION_COLOR_STOP_MS          10000U
#define DOG_TASK_THROW_FORWARD_MS      0U
#define DOG_TASK_THROW_TRACK_DELAY_MS  3000U
#define DOG_TASK_VISION_ACK_TIMEOUT_MS 10U
#define DOG_TASK_STATUS_INTERVAL_MS    200U
#define DOG_TASK_STEP_H_MM             45.0f
#define DOG_TASK_FORWARD_R_MM          50.0f
#define DOG_TASK_TURN_R_MM             15.0f
#define DOG_TASK_SPEED_FREQ            0.20f

#define DOG_TASK_TRACK_DEADBAND        35
#define DOG_TASK_TRACK_RECOVER_MS      500U
#define DOG_TASK_TRACK_STEP_H_MM       45.0f
#define DOG_TASK_TRACK_LEFT_FORWARD_R_MM   60.0f
#define DOG_TASK_TRACK_RIGHT_FORWARD_R_MM  45.0f
#define DOG_TASK_TRACK_MAX_STEER_MM    18.0f
#define DOG_TASK_TRACK_STEER_GAIN      0.18f
#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          30.0f
#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  60.0f
#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 45.0f

#define DOG_TASK_STAIR_PLATFORM_HEIGHT_MM          30.0f // 表示上台阶后平台的高度，单位毫米，根据实际情况调整，过高可能导致步态不够稳定。  
#define DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM         40.0f // 上台阶时足端相对于平台的额外高度，单位毫米，根据实际情况调整，过高可能导致步态不够稳定。
#define DOG_TASK_STAIR_DESCENT_CLEARANCE_MM        15.0f // 下台阶时足端相对于平台的额外高度，单位毫米，根据实际情况调整，过高可能导致步态不够稳定。
#define DOG_TASK_STAIR_LIFT_FORWARD_MM             40.0f // 上台阶抬脚阶段身体前移的距离，单位毫米，根据实际情况调整，过高可能导致步态不够稳定。
#define DOG_TASK_STAIR_STEP_FORWARD_MM             35.0f // 上台阶落脚阶段身体前移的距离，单位毫米，根据实际情况调整，过高可能导致步态不够稳定。
#define DOG_TASK_STAIR_BODY_ADVANCE_MM             25.0f // 前腿已经放上台阶后，让机身相对四只脚向前移动约 25 mm。
#define DOG_TASK_STAIR_REAR_PLACE_X_MM             115.0f // 后腿上台或下台完成落脚时，后腿足端相对于机身站立基准位置的最终前向偏移量。
/* Keep at zero until the physical pitch direction is verified on the robot. */
#define DOG_TASK_STAIR_PITCH_BIAS_DEG               0.0f // 上台阶时身体前倾的角度，单位度，根据实际情况调整，过大可能导致步态不够稳定。
#define DOG_TASK_STAIR_POSE_MOVE_MS                 700U // 上台阶时每个阶段的移动时间，单位毫秒，根据实际情况调整，过短可能导致步态不够稳定。
#define DOG_TASK_STAIR_POSE_HOLD_MS                 900U // 上台阶时每个阶段的停顿时间，单位毫秒，根据实际情况调整，过短可能导致步态不够稳定。
#define DOG_TASK_STAIR_SETTLE_MOVE_MS               000U // 上台阶阶段，平台上站立等待稳定的移动时间，单位毫秒，根据实际情况调整，过短可能导致步态不够稳定。
#define DOG_TASK_STAIR_SETTLE_HOLD_MS               1300U // 上台阶阶段，平台上站立等待稳定的停顿时间，单位毫秒，根据实际情况调整，过短可能导致步态不够稳定。

#define DOG_TASK_PLATFORM_DISTANCE_MM               600U // 表示平台距离的常量，单位毫米，根据实际情况调整，过大可能导致步态不够稳定。
/* Open-loop distance estimate; replace this value with the measured travel per cycle. */
#define DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE    25U // 表示平台前进阶段每个周期的估算前进距离，单位毫米。
#define DOG_TASK_PLATFORM_FORWARD_CYCLES \
    ((DOG_TASK_PLATFORM_DISTANCE_MM + DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE - 1U) / \
     DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE) // 表示平台前进阶段需要执行的周期数，根据实际情况调整，过大可能导致步态不够稳定。
#define DOG_TASK_PLATFORM_GAIT_PERIOD_MS            150U
#define DOG_TASK_PLATFORM_STEP_H_MM                  15.0f
#define DOG_TASK_PLATFORM_STEP_R_MM                  25.0f
#define DOG_TASK_PLATFORM_SPEED_FREQ                 0.125f
#define DOG_TASK_PLATFORM_UPDATES_PER_CYCLE          8U

/* Left/right turn test entry is kept only for reference. */
#define DOG_TASK_TURN_TEST_DURATION_MS 900U

#if 0
#define DOG_TASK_AUTO_TEST_ENABLE      1U
#define DOG_TASK_AUTO_FORWARD_MS       3000U
#define DOG_TASK_AUTO_LEFT_MS          3000U
#define DOG_TASK_AUTO_RIGHT_MS         3000U
#endif

#define DOG_TASK_SERVO_READY_MS        2000U
#define DOG_TASK_CENTER_MOVE_MS        5000U
#define DOG_TASK_CENTER_WAIT_MS        6500U
#define DOG_TASK_STAND_MOVE_MS         2000U
#define DOG_TASK_STAND_WAIT_MS         2500U
#define DOG_TASK_USE_PAYLOAD_GAIT      1U

typedef enum
{
    DOG_TASK_MOTION_STOP = 0,
    DOG_TASK_MOTION_FORWARD,
    DOG_TASK_MOTION_BACKWARD,
    DOG_TASK_MOTION_TURN_LEFT,
    DOG_TASK_MOTION_TURN_RIGHT,
    DOG_TASK_MOTION_SHIFT_LEFT,   /* 左移 */
    DOG_TASK_MOTION_SHIFT_RIGHT,  /* 右移 */
} DogTaskMotion_t;

typedef enum
{
    DOG_TASK_EVENT_IDLE = 0,
    DOG_TASK_EVENT_COLOR_PAUSE,
    DOG_TASK_EVENT_FORK_TURN,
    DOG_TASK_EVENT_THROW_TRACK_DELAY,
    DOG_TASK_EVENT_THROW_FORWARD,
    DOG_TASK_EVENT_THROW_ROTATING,
    DOG_TASK_EVENT_STAIR_ASCEND_SETTLE, // 上台阶阶段的初始状态，保持站立姿态，等待稳定
    DOG_TASK_EVENT_STAIR_ASCEND_LF_LIFT, // 上台阶阶段，左前腿抬起
    DOG_TASK_EVENT_STAIR_ASCEND_LF_PLACE, // 上台阶阶段，左前腿放置
    DOG_TASK_EVENT_STAIR_ASCEND_RF_LIFT, // 上台阶阶段，右前腿抬起
    DOG_TASK_EVENT_STAIR_ASCEND_RF_PLACE, // 上台阶阶段，右前腿放置
    DOG_TASK_EVENT_STAIR_ASCEND_BODY_ADVANCE, // 上台阶阶段，身体前进
    DOG_TASK_EVENT_STAIR_ASCEND_LB_LIFT, // 上台阶阶段，左后腿抬起
    DOG_TASK_EVENT_STAIR_ASCEND_LB_PLACE, // 上台阶阶段，左后腿放置
    DOG_TASK_EVENT_STAIR_ASCEND_RB_LIFT,
    DOG_TASK_EVENT_STAIR_ASCEND_RB_PLACE,
    DOG_TASK_EVENT_STAIR_PLATFORM_SETTLE, // 上台阶阶段，平台上站立等待稳定
    DOG_TASK_EVENT_STAIR_PLATFORM_FORWARD, // 上台阶阶段，平台上前进。
    DOG_TASK_EVENT_STAIR_DESCEND_SETTLE,
    DOG_TASK_EVENT_STAIR_DESCEND_LF_LIFT,
    DOG_TASK_EVENT_STAIR_DESCEND_LF_PLACE,
    DOG_TASK_EVENT_STAIR_DESCEND_RF_LIFT,
    DOG_TASK_EVENT_STAIR_DESCEND_RF_PLACE,
    DOG_TASK_EVENT_STAIR_DESCEND_BODY_ADVANCE, // 上台阶，重心调整。
    DOG_TASK_EVENT_STAIR_DESCEND_LB_LIFT,
    DOG_TASK_EVENT_STAIR_DESCEND_LB_PLACE,
    DOG_TASK_EVENT_STAIR_DESCEND_RB_LIFT,
    DOG_TASK_EVENT_STAIR_DESCEND_RB_PLACE,
    DOG_TASK_EVENT_STAIR_FINAL_SETTLE,
} DogTaskEventState_t;

static DogTaskMotion_t s_motion = DOG_TASK_MOTION_STOP;
static DogTaskMotion_t s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
static DogTaskEventState_t s_event_state = DOG_TASK_EVENT_IDLE;
static uint32_t s_event_start_ms;
static uint32_t s_color_pause_ms = DOG_TASK_COLOR_PAUSE_MS;
static uint32_t s_color_pause_last_stand_ms;
static ImageCommand_t s_pending_event_command = IMAGE_COMMAND_NONE;
static uint32_t s_last_gait_ms;
static uint32_t s_last_track_ms;
static uint32_t s_last_status_ms;
static uint8_t s_has_seen_track;
static uint8_t s_is_track_correcting;
static uint8_t s_platform_track_boost;
static uint8_t s_purple_throw_delay_used;
static uint8_t s_brown_throw_delay_used;
static DogGaitStairTarget_t s_stair_targets[DOG_GAIT_STAIR_LEG_COUNT]; // 上台阶阶段每条腿的目标参数，单位毫米和度，根据实际情况调整，过大可能导致步态不够稳定。
static uint16_t s_platform_forward_cycles; // 表示平台前进阶段已经执行的周期数，用于估算前进距离，根据实际情况调整，过大可能导致步态不够稳定。
static uint8_t s_platform_forward_updates; // 表示平台前进阶段已经执行的更新次数，用于估算前进距离，根据实际情况调整，过大可能导致步态不够稳定。

#if 0
static uint8_t s_turn_test_active;
static uint32_t s_turn_test_start_ms;
static uint8_t s_auto_test_active;
static uint32_t s_auto_test_start_ms;
#endif

static void DogTask_SendVisionStatus(const char *tag);

static void DogTask_BeginPlatformTrackBoost(void)
{
    s_platform_track_boost = 1U;
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_pending_event_command = IMAGE_COMMAND_NONE;
}

static void DogTask_ApplyMotion(DogTaskMotion_t motion)
{
    if (motion == s_motion)
    {
        return;
    }

    if (motion == DOG_TASK_MOTION_SHIFT_LEFT)
    {
        DogGait_SetShiftLeftParams(DOG_TASK_SHIFT_STEP_H_MM,
                                   DOG_TASK_SHIFT_R_MM,
                                   DOG_TASK_SHIFT_SPEED_FREQ);
    }
    else if (motion == DOG_TASK_MOTION_SHIFT_RIGHT)
    {
        DogGait_SetShiftRightParams(DOG_TASK_SHIFT_STEP_H_MM,
                                    DOG_TASK_SHIFT_R_MM,
                                    DOG_TASK_SHIFT_SPEED_FREQ);
    }
#if 0  /* 注释掉其他步态，只测试平移步态 */
    else if (motion == DOG_TASK_MOTION_FORWARD)
    {
        DogGait_SetTrotParams(DOG_TASK_STEP_H_MM,
                              DOG_TASK_FORWARD_R_MM,
                              DOG_TASK_SPEED_FREQ);
    }
    else if (motion == DOG_TASK_MOTION_BACKWARD)
    {
        DogGait_SetTrotParams(DOG_TASK_STEP_H_MM,
                              -DOG_TASK_FORWARD_R_MM,
                              DOG_TASK_SPEED_FREQ);
    }
    else if (motion == DOG_TASK_MOTION_TURN_LEFT)
    {
        DogGait_SetTurnLeftParams(DOG_TASK_STEP_H_MM,
                                  DOG_TASK_TURN_R_MM,
                                  DOG_TASK_SPEED_FREQ);
    }
    else if (motion == DOG_TASK_MOTION_TURN_RIGHT)
    {
        DogGait_SetTurnRightParams(DOG_TASK_STEP_H_MM,
                                   DOG_TASK_TURN_R_MM,
                                   DOG_TASK_SPEED_FREQ);
    }
#endif
    else
    {
        DogGait_AllStand(DOG_TASK_GAIT_MOVE_MS);
    }

    s_motion = motion;
}

static void DogTask_SetCorrectionLed(uint8_t is_on)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,
                      LED_Pin,
                      (is_on != 0U) ? DOG_TASK_LED_ON_STATE : DOG_TASK_LED_OFF_STATE);
}

static const char *DogTask_MotionName(DogTaskMotion_t motion)
{
    if (motion == DOG_TASK_MOTION_FORWARD)
    {
        return "FORWARD";
    }
    if (motion == DOG_TASK_MOTION_BACKWARD)
    {
        return "BACKWARD";
    }
    if (motion == DOG_TASK_MOTION_TURN_LEFT)
    {
        return "TURN_LEFT";
    }
    if (motion == DOG_TASK_MOTION_TURN_RIGHT)
    {
        return "TURN_RIGHT";
    }
    if (motion == DOG_TASK_MOTION_SHIFT_LEFT)
    {
        return "SHIFT_LEFT";
    }
    if (motion == DOG_TASK_MOTION_SHIFT_RIGHT)
    {
        return "SHIFT_RIGHT";
    }
    return "STOP";
}

static const char *DogTask_EventName(DogTaskEventState_t state)
{
    static const char *names[] = {
        "IDLE",
        "COLOR_PAUSE",
        "FORK_TURN",
        "THROW_TRACK_DELAY",
        "THROW_FORWARD",
        "THROW_ROTATING",
        "STAIR_ASCEND_SETTLE",
        "STAIR_ASCEND_LF_LIFT",
        "STAIR_ASCEND_LF_PLACE",
        "STAIR_ASCEND_RF_LIFT",
        "STAIR_ASCEND_RF_PLACE",
        "STAIR_ASCEND_BODY_ADVANCE",
        "STAIR_ASCEND_LB_LIFT",
        "STAIR_ASCEND_LB_PLACE",
        "STAIR_ASCEND_RB_LIFT",
        "STAIR_ASCEND_RB_PLACE",
        "STAIR_PLATFORM_SETTLE",
        "STAIR_PLATFORM_FORWARD",
        "STAIR_DESCEND_SETTLE",
        "STAIR_DESCEND_LF_LIFT",
        "STAIR_DESCEND_LF_PLACE",
        "STAIR_DESCEND_RF_LIFT",
        "STAIR_DESCEND_RF_PLACE",
        "STAIR_DESCEND_BODY_ADVANCE",
        "STAIR_DESCEND_LB_LIFT",
        "STAIR_DESCEND_LB_PLACE",
        "STAIR_DESCEND_RB_LIFT",
        "STAIR_DESCEND_RB_PLACE",
        "STAIR_FINAL_SETTLE",
    };

    if ((uint8_t)state < (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return names[(uint8_t)state];
    }

    return "UNKNOWN";
}

static uint8_t DogTask_IsStairEventState(DogTaskEventState_t state) // 判断当前事件状态是否处于上台阶或下台阶阶段
{
    return (uint8_t)((state >= DOG_TASK_EVENT_STAIR_ASCEND_SETTLE) &&
                     (state <= DOG_TASK_EVENT_STAIR_FINAL_SETTLE)); // 根据事件状态的定义范围判断是否处于上台阶或下台阶阶段
}

static void DogTask_ResetStairTargets(void) // 将上台阶阶段每条腿的目标参数重置为默认值，单位毫米和度，根据实际情况调整，过大可能导致步态不够稳定。
{
    for (uint8_t i = 0; i < DOG_GAIT_STAIR_LEG_COUNT; i++)
    {
        s_stair_targets[i].x_offset_mm = 0.0f;
        s_stair_targets[i].y_offset_mm = 0.0f;
        s_stair_targets[i].hip_bias_deg = 0.0f;
    }
}

static void DogTask_SetStairPitchBias(float bias_deg) // 设置上台阶阶段每条腿的关节角度偏移，单位度，根据实际情况调整，过大可能导致步态不够稳定。
{
    s_stair_targets[DOG_GAIT_STAIR_LEG_LF].hip_bias_deg = bias_deg;
    s_stair_targets[DOG_GAIT_STAIR_LEG_RF].hip_bias_deg = bias_deg;
    s_stair_targets[DOG_GAIT_STAIR_LEG_LB].hip_bias_deg = -bias_deg;
    s_stair_targets[DOG_GAIT_STAIR_LEG_RB].hip_bias_deg = -bias_deg;
}

static void DogTask_ApplyStairTargets(uint16_t move_ms) // 应用上台阶阶段的目标参数，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
{
    s_motion = DOG_TASK_MOTION_STOP;
    DogGait_SetStairPose(s_stair_targets, move_ms);
}

static void DogTask_AdvanceStairBody(void) // 在上台阶阶段前腿已经放上台阶后，让机身相对四只脚向前移动一定距离，单位毫米，根据实际情况调整，过大可能导致步态不够稳定。
{
    for (uint8_t i = 0; i < DOG_GAIT_STAIR_LEG_COUNT; i++)
    {
        s_stair_targets[i].x_offset_mm -= DOG_TASK_STAIR_BODY_ADVANCE_MM; // 通过减少每条腿的前向偏移量来实现身体相对于四只脚向前移动，单位毫米，根据实际情况调整，过大可能导致步态不够稳定。
    }
}

static void DogTask_SetStairState(DogTaskEventState_t state, uint32_t now_ms) // 设置当前的上台阶事件状态，并根据状态执行相应的动作，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
{
    s_event_state = state;
    s_event_start_ms = now_ms;
    s_is_track_correcting = 0U;

    if (state == DOG_TASK_EVENT_STAIR_ASCEND_SETTLE) // 上台阶阶段的初始状态，保持站立姿态，等待稳定
    {
        DogTask_ResetStairTargets();
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_SETTLE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_LF_LIFT) // 上台阶阶段，左前腿抬起
    {
        DogTask_SetStairPitchBias(DOG_TASK_STAIR_PITCH_BIAS_DEG); // 上台阶时身体前倾一定角度，单位度，根据实际情况调整，过大可能导致步态不够稳定。
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm =
            DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_LF_PLACE) // 上台阶阶段，左前腿放置
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm =
            DOG_TASK_STAIR_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm =
            DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_RF_LIFT) // 上台阶阶段，右前腿抬起
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm =
            DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_RF_PLACE) // 上台阶阶段，右前腿放置
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm =
            DOG_TASK_STAIR_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm =
            DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_BODY_ADVANCE) // 上台阶阶段，身体前进
    {
        DogTask_SetStairPitchBias(-DOG_TASK_STAIR_PITCH_BIAS_DEG);
        DogTask_AdvanceStairBody();
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_SETTLE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_LB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm =
            DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_LB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm =
            DOG_TASK_STAIR_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm =
            DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_RB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm =
            DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_ASCEND_RB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm =
            DOG_TASK_STAIR_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm =
            DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_PLATFORM_SETTLE)
    {
        DogTask_ResetStairTargets();
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_SETTLE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_PLATFORM_FORWARD)
    {
        s_platform_forward_cycles = 0U;
        s_platform_forward_updates = 0U;
        DogGait_SetTrotParams(DOG_TASK_PLATFORM_STEP_H_MM,
                              DOG_TASK_PLATFORM_STEP_R_MM,
                              DOG_TASK_PLATFORM_SPEED_FREQ);
        s_motion = DOG_TASK_MOTION_FORWARD;
        s_last_gait_ms = now_ms;
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_SETTLE) // 下台阶阶段的初始状态，保持站立姿态，等待稳定
    {
        DogTask_ResetStairTargets();
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_SETTLE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_LF_LIFT) // 下台阶阶段，左前腿抬起
    {
        DogTask_SetStairPitchBias(DOG_TASK_STAIR_PITCH_BIAS_DEG);
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm =
            DOG_TASK_STAIR_DESCENT_CLEARANCE_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_LF_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].x_offset_mm =
            DOG_TASK_STAIR_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LF].y_offset_mm =
            -DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_RF_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm =
            DOG_TASK_STAIR_DESCENT_CLEARANCE_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_RF_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].x_offset_mm =
            DOG_TASK_STAIR_STEP_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RF].y_offset_mm =
            -DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_BODY_ADVANCE) // 下台阶阶段，重心调整，让身体相对于四只脚向前移动一定距离，单位毫米，根据实际情况调整，过大可能导致步态不够稳定。
    {
        DogTask_SetStairPitchBias(-DOG_TASK_STAIR_PITCH_BIAS_DEG);
        DogTask_AdvanceStairBody();
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_SETTLE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_LB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm =
            DOG_TASK_STAIR_DESCENT_CLEARANCE_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_LB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].x_offset_mm =
            DOG_TASK_STAIR_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_LB].y_offset_mm =
            -DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_RB_LIFT)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm =
            DOG_TASK_STAIR_LIFT_FORWARD_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm =
            DOG_TASK_STAIR_DESCENT_CLEARANCE_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_DESCEND_RB_PLACE)
    {
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].x_offset_mm =
            DOG_TASK_STAIR_REAR_PLACE_X_MM;
        s_stair_targets[DOG_GAIT_STAIR_LEG_RB].y_offset_mm =
            -DOG_TASK_STAIR_PLATFORM_HEIGHT_MM;
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_POSE_MOVE_MS);
    }
    else if (state == DOG_TASK_EVENT_STAIR_FINAL_SETTLE)
    {
        DogTask_ResetStairTargets();
        DogTask_ApplyStairTargets(DOG_TASK_STAIR_SETTLE_MOVE_MS);
    }
}

static uint8_t DogTask_IsEventCommand(ImageCommand_t command)
{
    /* Keep lost-line 9999 non-latching so tracking can recover on the next valid frame. */
    return (uint8_t)((command == IMAGE_COMMAND_TURN_LEFT) ||
                     (command == IMAGE_COMMAND_TURN_RIGHT) ||
                     (command == IMAGE_COMMAND_PLATFORM) ||
                     (command == IMAGE_COMMAND_PURPLE) ||
                     (command == IMAGE_COMMAND_BROWN));
}

static void DogTask_ResumeTracking(uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
}

static void DogTask_BeginThrowForward(ImageCommand_t command, uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_THROW_FORWARD;
    s_event_start_ms = now_ms;
    s_pending_event_command = command;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}

static void DogTask_BeginThrowTrackDelay(ImageCommand_t command, uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_THROW_TRACK_DELAY;
    s_event_start_ms = now_ms;
    s_pending_event_command = command;
}

static void DogTask_BeginForkTurn(DogTaskMotion_t motion, uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_FORK_TURN;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = motion;
    DogTask_ApplyMotion(motion);
}

static void DogTask_BeginThrowRotation(ImageCommand_t command, uint32_t now_ms)
{
    ThrowServoDirection_t direction = THROW_SERVO_DIRECTION_CW;

    if (command == IMAGE_COMMAND_PURPLE)
    {
        direction = THROW_SERVO_DIRECTION_CCW;
    }

    s_event_state = DOG_TASK_EVENT_THROW_ROTATING;
    s_event_start_ms = now_ms;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    ThrowServo_Start(direction);
}

#if 0
static void DogTask_BeginStairSequence(uint32_t now_ms) // 开始上台阶或下台阶的序列，具体是上台阶还是下台阶可以在后续的状态中根据需要进行区分，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
{
    s_pending_event_command = IMAGE_COMMAND_PLATFORM;
    s_has_seen_track = 0U;
    s_last_track_ms = now_ms;
    DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_SETTLE, now_ms);
}
#endif

static void __attribute__((unused)) DogTask_BeginColorPause(uint32_t now_ms, uint32_t pause_ms)
{
    s_event_state = DOG_TASK_EVENT_COLOR_PAUSE; // 颜色事件的处理状态，机器人在这个状态下会暂停移动，等待一段时间后恢复跟踪，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
    s_event_start_ms = now_ms;
    s_color_pause_ms = pause_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE; // 颜色事件的命令，当前没有具体的命令需要处理，所以设置为 NONE。
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    s_color_pause_last_stand_ms = now_ms;
    DogGait_AllStand(DOG_TASK_GAIT_MOVE_MS); // 在颜色事件发生时让机器人立即停止移动，进入站立姿态，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
    s_motion = DOG_TASK_MOTION_STOP;
    DogTask_SetCorrectionLed(1U);
}

static void DogTask_SendVisionAck(void)
{
    DogTask_SendVisionStatus("OK");
}

static void DogTask_SendVisionStatus(const char *tag)
{
    char message[64];
    int len = snprintf(message,
                       sizeof(message),
                       "%s E:%s M:%s\n",
                       tag,
                       DogTask_EventName(s_event_state),
                       DogTask_MotionName(s_motion));

    if (len <= 0)
    {
        return;
    }

    if (len >= (int)sizeof(message))
    {
        len = (int)sizeof(message) - 1;
    }

    (void)HAL_UART_Transmit(&huart2,
                            (uint8_t *)message,
                            (uint16_t)len,
                            DOG_TASK_VISION_ACK_TIMEOUT_MS);
}

static void DogTask_ExecuteEventCommand(ImageCommand_t command, uint32_t now_ms)
{
    if (command == IMAGE_COMMAND_TURN_LEFT)
    {
        DogTask_SendVisionAck();
        DogTask_BeginForkTurn(DOG_TASK_MOTION_TURN_LEFT, now_ms);
    }
    else if (command == IMAGE_COMMAND_TURN_RIGHT)
    {
        DogTask_SendVisionAck();
        DogTask_BeginForkTurn(DOG_TASK_MOTION_TURN_RIGHT, now_ms);
    }
    else if ((command == IMAGE_COMMAND_PURPLE) ||
             (command == IMAGE_COMMAND_BROWN))
    {
        DogTask_SendVisionAck();
        if (((command == IMAGE_COMMAND_PURPLE) && (s_purple_throw_delay_used == 0U)) ||
            ((command == IMAGE_COMMAND_BROWN) && (s_brown_throw_delay_used == 0U)))
        {
            if (command == IMAGE_COMMAND_PURPLE)
            {
                s_purple_throw_delay_used = 1U;
            }
            else
            {
                s_brown_throw_delay_used = 1U;
            }

            DogTask_BeginThrowTrackDelay(command, now_ms);
        }
        else
        {
            DogTask_BeginThrowForward(command, now_ms);
        }
    }
    else if (command == IMAGE_COMMAND_STOP)
    {
        s_is_track_correcting = 0U;
        s_event_state = DOG_TASK_EVENT_IDLE;
        s_pending_event_command = IMAGE_COMMAND_NONE;
        s_has_seen_track = 0U;
        s_last_track_ms = now_ms;
        s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    }
    else if (command == IMAGE_COMMAND_PLATFORM)
    {
        DogTask_SendVisionAck();
        /* Main firmware no longer enters the old stair sequence on blue. */
        /* DogTask_BeginStairSequence(now_ms); */
        DogTask_BeginPlatformTrackBoost();
    }
    else
    {
        DogTask_ResumeTracking(now_ms);
    }
}

static void DogTask_UpdateEventState(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (s_event_state == DOG_TASK_EVENT_IDLE)
    {
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - s_event_start_ms);

    if (s_event_state == DOG_TASK_EVENT_THROW_TRACK_DELAY)
    {
        if (elapsed_ms >= DOG_TASK_THROW_TRACK_DELAY_MS)
        {
            DogTask_BeginThrowForward(s_pending_event_command, now_ms);
        }

        return;
    }

    s_is_track_correcting = 0U;

    if (s_event_state == DOG_TASK_EVENT_FORK_TURN)
    {
        if (elapsed_ms >= DOG_TASK_TURN_TEST_DURATION_MS)
        {
            DogTask_ResumeTracking(now_ms);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_COLOR_PAUSE)
    {
        if (elapsed_ms >= s_color_pause_ms)
        {
            DogTask_ResumeTracking(now_ms);
        }
        else
        {
            if ((uint32_t)(now_ms - s_color_pause_last_stand_ms) >= DOG_TASK_COLOR_PAUSE_HOLD_MS)
            {
                s_color_pause_last_stand_ms = now_ms;
                DogGait_AllStand(DOG_TASK_GAIT_MOVE_MS);
                s_motion = DOG_TASK_MOTION_STOP;
            }
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_THROW_FORWARD)
    {
        if ((uint32_t)(now_ms - s_event_start_ms) >= DOG_TASK_THROW_FORWARD_MS)
        {
            DogTask_BeginThrowRotation(s_pending_event_command, now_ms);
        }
        else
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_THROW_ROTATING)
    {
        if (ThrowServo_IsBusy() == 0U)
        {
            ThrowServo_Stop();
            DogGait_SetLoadMode(DOG_GAIT_LOAD_NONE);
            DogTask_ResumeTracking(now_ms);
        }
        else
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
    }
    else if (DogTask_IsStairEventState(s_event_state) != 0U)
    {
        if (s_event_state == DOG_TASK_EVENT_STAIR_PLATFORM_FORWARD) // 如果当前状态是上台阶阶段的平台前进状态，则根据已经执行的周期数来判断是否完成了预定的前进距离，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
        {
            if (s_platform_forward_cycles >= DOG_TASK_PLATFORM_FORWARD_CYCLES) // 如果已经执行的周期数达到了预定的周期数，则认为平台前进阶段完成，进入下一个状态，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
            {
                DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_SETTLE, now_ms);
            }
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_SETTLE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_SETTLE_HOLD_MS)) // 如果当前状态是上台阶阶段的初始状态，并且已经等待了足够的时间来稳定站立，则进入下一个状态，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_LF_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_LF_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_LF_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_LF_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_RF_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_RF_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_RF_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_RF_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_BODY_ADVANCE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_BODY_ADVANCE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_SETTLE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_LB_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_LB_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_LB_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_LB_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_RB_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_RB_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_ASCEND_RB_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_ASCEND_RB_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_PLATFORM_SETTLE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_PLATFORM_SETTLE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_SETTLE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_PLATFORM_FORWARD, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_SETTLE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_SETTLE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_LF_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_LF_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_LF_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_LF_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_RF_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_RF_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_RF_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_RF_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_BODY_ADVANCE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_BODY_ADVANCE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_SETTLE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_LB_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_LB_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_LB_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_LB_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_RB_LIFT, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_RB_LIFT) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_DESCEND_RB_PLACE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_DESCEND_RB_PLACE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_POSE_HOLD_MS))
        {
            DogTask_SetStairState(DOG_TASK_EVENT_STAIR_FINAL_SETTLE, now_ms);
        }
        else if ((s_event_state == DOG_TASK_EVENT_STAIR_FINAL_SETTLE) &&
                 (elapsed_ms >= DOG_TASK_STAIR_SETTLE_HOLD_MS))
        {
            DogTask_ResumeTracking(now_ms);
        }
    }
}

#if 0
static void DogTask_BeginTurnTest(DogTaskMotion_t motion, uint32_t now_ms)
{
    DogTask_ApplyMotion(motion);
    s_turn_test_active = 1U;
    s_turn_test_start_ms = now_ms;
}

static void DogTask_StopTurnTest(void)
{
    s_turn_test_active = 0U;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}
#endif

static void DogTask_ApplyCommand(ImageCommand_t command, uint32_t now_ms)
{
    (void)now_ms;

    if (command == IMAGE_COMMAND_FORWARD)
    {
        s_is_track_correcting = 0U;
        /* 测试用：FORWARD命令改为左移 */
        DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_LEFT);
    }
#if 0
    else if (command == IMAGE_COMMAND_TURN_LEFT)
    {
        DogTask_BeginTurnTest(DOG_TASK_MOTION_TURN_LEFT, now_ms);
    }
    else if (command == IMAGE_COMMAND_TURN_RIGHT)
    {
        DogTask_BeginTurnTest(DOG_TASK_MOTION_TURN_RIGHT, now_ms);
    }
#endif
    else if (command == IMAGE_COMMAND_STOP)
    {
        s_is_track_correcting = 0U;
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    }
}

static void DogTask_ApplyTrackError(int16_t error)
{
    float steer = 0.0f;
    float track_step_h = DOG_TASK_TRACK_STEP_H_MM;
    float track_left_forward = DOG_TASK_TRACK_LEFT_FORWARD_R_MM;
    float track_right_forward = DOG_TASK_TRACK_RIGHT_FORWARD_R_MM;

    if (s_platform_track_boost != 0U)
    {
        track_step_h = DOG_TASK_PLATFORM_TRACK_STEP_H_MM;
        track_left_forward = DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM;
        track_right_forward = DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM;
    }

    if (error > DOG_TASK_TRACK_DEADBAND)
    {
        s_is_track_correcting = 1U;
        steer = (float)error * DOG_TASK_TRACK_STEER_GAIN;
        if (steer > DOG_TASK_TRACK_MAX_STEER_MM)
        {
            steer = DOG_TASK_TRACK_MAX_STEER_MM;
        }

        /* Positive camera error means the line is to the right; steer right. */
        s_last_track_recover_motion = DOG_TASK_MOTION_TURN_RIGHT;
        DogGait_SetTrackParams(track_step_h,
                               track_left_forward,
                               track_right_forward,
                               steer,
                               DOG_TASK_SPEED_FREQ);
        s_motion = DOG_TASK_MOTION_TURN_RIGHT;
    }
    else if (error < -DOG_TASK_TRACK_DEADBAND)
    {
        s_is_track_correcting = 1U;
        steer = (float)(-error) * DOG_TASK_TRACK_STEER_GAIN;
        if (steer > DOG_TASK_TRACK_MAX_STEER_MM)
        {
            steer = DOG_TASK_TRACK_MAX_STEER_MM;
        }

        /* Negative camera error means the line is to the left; steer left. */
        s_last_track_recover_motion = DOG_TASK_MOTION_TURN_LEFT;
        DogGait_SetTrackParams(track_step_h,
                               track_left_forward,
                               track_right_forward,
                               -steer,
                               DOG_TASK_SPEED_FREQ);
        s_motion = DOG_TASK_MOTION_TURN_LEFT;
    }
    else
    {
        s_is_track_correcting = 0U;
        s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
        DogGait_SetTrackParams(track_step_h,
                               track_left_forward,
                               track_right_forward,
                               0.0f,
                               DOG_TASK_SPEED_FREQ);
        s_motion = DOG_TASK_MOTION_FORWARD;
    }
}

#if 0
static void DogTask_RunMotionBlocking(DogTaskMotion_t motion, uint32_t duration_ms)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_gait_ms = start_ms;

    DogTask_ApplyMotion(motion);

    while ((uint32_t)(HAL_GetTick() - start_ms) < duration_ms)
    {
        uint32_t now_ms = HAL_GetTick();

        if ((uint32_t)(now_ms - last_gait_ms) >= DOG_TASK_GAIT_PERIOD_MS)
        {
            last_gait_ms = now_ms;
            DogGait_UpdateTrot(DOG_TASK_GAIT_MOVE_MS);
        }

        HAL_Delay(1U);
    }
}

static void DogTask_RunAutoTestBlocking(void)
{
    DogTask_RunMotionBlocking(DOG_TASK_MOTION_FORWARD, DOG_TASK_AUTO_FORWARD_MS);
    DogTask_RunMotionBlocking(DOG_TASK_MOTION_TURN_LEFT, DOG_TASK_AUTO_LEFT_MS);
    DogTask_RunMotionBlocking(DOG_TASK_MOTION_TURN_RIGHT, DOG_TASK_AUTO_RIGHT_MS);
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}
#endif

void DogTask_Init(void)
{
    ThrowServo_Init();
    HAL_Delay(DOG_TASK_SERVO_READY_MS);

    DogServo_AllCenter(DOG_TASK_CENTER_MOVE_MS);
    HAL_Delay(DOG_TASK_CENTER_WAIT_MS);

    DogGait_SetLoadMode((DOG_TASK_USE_PAYLOAD_GAIT != 0U) ? DOG_GAIT_LOAD_WITH_PAYLOAD : DOG_GAIT_LOAD_NONE);
    DogGait_Init();
    DogGait_GotoStandPose(DOG_TASK_STAND_MOVE_MS);
    HAL_Delay(DOG_TASK_STAND_WAIT_MS);

    ImageCommand_Init();
    DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_LEFT);  /* 启动后直接执行左移 */
    DogTask_SetCorrectionLed(0U);

    s_last_gait_ms = HAL_GetTick();
    s_last_track_ms = s_last_gait_ms;
    s_last_status_ms = s_last_gait_ms;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_platform_track_boost = 0U;
    s_purple_throw_delay_used = 0U;
    s_brown_throw_delay_used = 0U;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_event_start_ms = s_last_gait_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    DogTask_ResetStairTargets(); // 初始化上台阶阶段的目标参数，单位毫米和度，根据实际情况调整，过大可能导致步态不够稳定。
    s_platform_forward_cycles = 0U;
    s_platform_forward_updates = 0U;
#if 0
    s_turn_test_active = 0U;
    s_turn_test_start_ms = s_last_gait_ms;
    s_auto_test_active = 0U;
    s_auto_test_start_ms = s_last_gait_ms;

    if (DOG_TASK_AUTO_TEST_ENABLE != 0U)
    {
        DogTask_RunAutoTestBlocking();
        s_last_gait_ms = HAL_GetTick();
        s_last_track_ms = s_last_gait_ms;
    }
#endif
}

void DogTask_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    ImageCommand_t command = ImageCommand_TakeLatest();
    ImageTrack_t track = ImageCommand_TakeLatestTrack();
    uint32_t track_lost_ms = (uint32_t)(now_ms - s_last_track_ms);

    ThrowServo_Update();

    if (s_event_state == DOG_TASK_EVENT_THROW_TRACK_DELAY)
    {
        DogTask_UpdateEventState(now_ms);

        if (s_event_state == DOG_TASK_EVENT_THROW_TRACK_DELAY)
        {
            if (track.valid != 0U)
            {
                s_has_seen_track = 1U;
                s_last_track_ms = now_ms;
                DogTask_ApplyTrackError(track.error);
            }
            else if ((s_has_seen_track != 0U) &&
                     (track_lost_ms < DOG_TASK_TRACK_RECOVER_MS))
            {
                /* Keep the last track gait for short frame gaps so differential steering is not overwritten. */
                s_is_track_correcting = (uint8_t)(s_last_track_recover_motion != DOG_TASK_MOTION_FORWARD);
            }
            else if ((s_has_seen_track != 0U) &&
                     (track_lost_ms >= DOG_TASK_TRACK_RECOVER_MS))
            {
                s_is_track_correcting = 0U;
                DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
            }
        }
    }
    else if (s_event_state != DOG_TASK_EVENT_IDLE)
    {
        DogTask_UpdateEventState(now_ms);
    }
    else if (DogTask_IsEventCommand(command) != 0U)
    {
        s_is_track_correcting = 0U;
        DogTask_ExecuteEventCommand(command, now_ms);
    }
    else if (track.valid != 0U)
    {
        s_has_seen_track = 1U;
        s_last_track_ms = now_ms;
        DogTask_ApplyTrackError(track.error);
    }
    else if ((s_has_seen_track != 0U) &&
             (track_lost_ms < DOG_TASK_TRACK_RECOVER_MS))
    {
        /* Keep the last track gait for short frame gaps so differential steering is not overwritten. */
        s_is_track_correcting = (uint8_t)(s_last_track_recover_motion != DOG_TASK_MOTION_FORWARD);
    }
    else if ((s_has_seen_track != 0U) &&
             (track_lost_ms >= DOG_TASK_TRACK_RECOVER_MS))
    {
        s_is_track_correcting = 0U;
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    }
    else
    {
        DogTask_ApplyCommand(command, now_ms);
    }

    if (s_event_state == DOG_TASK_EVENT_COLOR_PAUSE)
    {
        DogTask_SetCorrectionLed(1U);
    }
    else
    {
        DogTask_SetCorrectionLed(0U);
    }

    if ((s_event_state == DOG_TASK_EVENT_STAIR_PLATFORM_FORWARD) &&
        ((uint32_t)(now_ms - s_last_gait_ms) >= DOG_TASK_PLATFORM_GAIT_PERIOD_MS)) // 如果当前状态是上台阶阶段的平台前进状态，并且已经到了更新步态的时间，则更新步态，并根据已经执行的更新次数来判断是否完成了预定的前进距离，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
    {
        s_last_gait_ms = now_ms;
        DogGait_UpdateTrot(DOG_TASK_GAIT_MOVE_MS);

        s_platform_forward_updates++;
        if (s_platform_forward_updates >= DOG_TASK_PLATFORM_UPDATES_PER_CYCLE) // 如果已经执行的更新次数达到了每个周期的预定更新次数，则增加已执行的周期数，并重置更新次数，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
        {
            s_platform_forward_updates = 0U;
            s_platform_forward_cycles++;
        }
    }
    else if ((s_motion != DOG_TASK_MOTION_STOP) &&
             ((uint32_t)(now_ms - s_last_gait_ms) >= DOG_TASK_GAIT_PERIOD_MS))
    {
        s_last_gait_ms = now_ms;

        /* 根据运动状态调用对应的步态更新函数 */
        if ((s_motion == DOG_TASK_MOTION_SHIFT_LEFT) ||
            (s_motion == DOG_TASK_MOTION_SHIFT_RIGHT))
        {
            DogGait_UpdateShift(DOG_TASK_GAIT_MOVE_MS);
        }
#if 0  /* 注释掉其他步态更新，只测试平移步态 */
        else if ((s_motion == DOG_TASK_MOTION_FORWARD) ||
                 (s_motion == DOG_TASK_MOTION_BACKWARD) ||
                 (s_motion == DOG_TASK_MOTION_TURN_LEFT) ||
                 (s_motion == DOG_TASK_MOTION_TURN_RIGHT))
        {
            DogGait_UpdateTrot(DOG_TASK_GAIT_MOVE_MS);
        }
#endif
    }

    if ((uint32_t)(now_ms - s_last_status_ms) >= DOG_TASK_STATUS_INTERVAL_MS)
    {
        s_last_status_ms = now_ms;
        DogTask_SendVisionStatus("ST");
    }
}

void Dog_Task_MoveLR(void)
{
    
}
