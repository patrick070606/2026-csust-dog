#include "dog_task.h"

#include "dog_gait.h"
#include "dog_servo.h"
#include "image_command.h"
#include "main.h"
#include "throw_servo.h"
#include "usart.h"

#include <stdio.h>

#define DOG_TASK_GAIT_PERIOD_MS        150U // 表示机器人步态更新的时间间隔，单位毫秒。
#define DOG_TASK_GAIT_MOVE_MS          100U // 表示每次下发步态更新时，舵机从当前角度移动到新目标所用的时间。
/*DOG_TASK_GAIT_PERIOD_MS 和 DOG_TASK_GAIT_MOVE_MS 表示：任务每隔 150 ms 计算一次新的腿部目标姿态，但要求舵机用 100 ms 完成这次移动。*/
#define DOG_TASK_LED_ON_STATE          GPIO_PIN_SET // 表示 LED 灯亮的状态，GPIO_PIN_SET 表示将 GPIO 引脚设置为高电平，通常用于点亮 LED。
#define DOG_TASK_LED_OFF_STATE         GPIO_PIN_RESET // 表示 LED 灯灭的状态，GPIO_PIN_RESET 表示将 GPIO 引脚设置为低电平，通常用于熄灭 LED。
#define DOG_TASK_COLOR_PAUSE_MS        2000U // 表示颜色暂停的时间，单位毫秒。
#define DOG_TASK_COLOR_PAUSE_HOLD_MS   150U // 表示颜色暂停保持的时间，单位毫秒。
#define DOG_TASK_VISION_COLOR_STOP_TEST_ENABLE 0U // 表示是否启用视觉颜色停止测试，0表示禁用，1表示启用。
#define DOG_TASK_VISION_COLOR_STOP_MS          10000U // 表示「视觉颜色停止测试模式」的停止保持时间，单位毫秒。
#define DOG_TASK_THROW_FORWARD_MS      0U // 表示投掷前前进阶段的持续时间，单位毫秒。这个现在是 0，我其实不知道这个变量是什么时候加上去的，可能是中间调试的时候为了让机器人在投掷前稍微前进一点点，避免投掷时机器人离目标太远。但是暂时可以不用管。
#define DOG_TASK_THROW_TRACK_DELAY_MS  3000U // 表示首次识别到紫色 / 棕色投掷事件后，先继续循迹的延迟时间。体现在实际中，就是识别到紫色 / 棕色后，往前走 DOG_TASK_THROW_TRACK_DELAY_MS 这么多 ms，然后再开始旋转投掷。
#define DOG_TASK_VISION_ACK_TIMEOUT_MS 10U // 表示 stm32 给 k230 回传状态字符串时，发送超时的时间。也就是说如果串口发送在 10ms 内没有完成，就返回超时。
#define DOG_TASK_STATUS_INTERVAL_MS    200U // 表示 stm32 向 k230 周期性发送状态反馈的时间间隔，单位毫秒。
#define DOG_TASK_STEP_H_MM             45.0f // 表示机器人步态的步高，单位毫米。
#define DOG_TASK_FORWARD_R_MM          50.0f // 表示机器人步态的前进半径，单位毫米。
#define DOG_TASK_TURN_R_MM             15.0f // 表示机器人步态的转弯半径，单位毫米。
#define DOG_TASK_SPEED_FREQ            0.20f // 表示机器人步态的速度频率，单位为每毫秒的步长。

#define DOG_TASK_TRACK_DEADBAND        35U // 表示循迹误差的死区范围，单位毫米。也就是说，如果摄像头识别到的线条偏离机器人中心线的距离在 ±35mm 以内，就认为机器人不需要调整方向，继续前进即可。 
#define DOG_TASK_TRACK_RECOVER_MS      500U // 表示循迹丢失后，机器人保持上一次循迹动作的时间，单位毫秒。
#define DOG_TASK_TRACK_STEP_H_MM       45.0f // 表示循迹时的步高，单位毫米。
#define DOG_TASK_TRACK_LEFT_FORWARD_R_MM   60.0f // 表示循迹时向左前进的半径，单位毫米。    
#define DOG_TASK_TRACK_RIGHT_FORWARD_R_MM  45.0f // 表示循迹时向右前进的半径，单位毫米。
#define DOG_TASK_TRACK_MAX_STEER_MM    18.0f // 表示循迹时的最大转向量，单位毫米。也就是说，如果摄像头识别到的线条偏离机器人中心线的距离超过 ±35mm，就会根据偏离的距离计算出一个转向量 steer，然后将 steer 限制在 ±18mm 以内，防止机器人转向过度。
#define DOG_TASK_TRACK_STEER_GAIN      0.18f // 表示循迹时的转向增益系数。这个增益系数就是用来计算转向量 steer 的。steer = error * DOG_TASK_TRACK_STEER_GAIN。
#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          30.0f // 表示平台循迹时的步高，单位毫米。
#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  60.0f // 表示平台循迹时向左前进的半径，单位毫米。    
#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 45.0f // 表示平台循迹时向右前进的半径，单位毫米。

/* Left/right turn test entry is kept only for reference. */
#define DOG_TASK_TURN_TEST_DURATION_MS 900U // 表示左/右转测试的持续时间，单位毫秒。这个测试是用来验证机器人在转弯时的步态和转向是否正常的。    

#if 0
/* 自动测试入口：早期用于不依赖视觉模块，按固定时长依次测试前进、左转、右转；当前关闭，仅保留参考。 */
#define DOG_TASK_AUTO_TEST_ENABLE      1U
/* 自动测试中，前进动作保持的时间，单位毫秒。 */
#define DOG_TASK_AUTO_FORWARD_MS       3000U
/* 自动测试中，左转动作保持的时间，单位毫秒。 */
#define DOG_TASK_AUTO_LEFT_MS          3000U
/* 自动测试中，右转动作保持的时间，单位毫秒。 */
#define DOG_TASK_AUTO_RIGHT_MS         3000U
#endif

#define DOG_TASK_SERVO_READY_MS        2000U // 表示舵机准备就绪的时间，单位毫秒。这个时间是用来等待舵机上电后稳定的时间，确保舵机可以正常工作。    
#define DOG_TASK_CENTER_MOVE_MS        5000U // 表示机器人舵机回到中心位置的移动时间，单位毫秒。
#define DOG_TASK_CENTER_WAIT_MS        6500U // 表示机器人舵机回到中心位置后等待的时间，单位毫秒。
#define DOG_TASK_STAND_MOVE_MS         2000U // 表示机器人站立动作的移动时间，单位毫秒。
#define DOG_TASK_STAND_WAIT_MS         2500U // 表示机器人站立动作后等待的时间，单位毫秒。
#define DOG_TASK_USE_PAYLOAD_GAIT      1U // 表示机器人是否使用负载步态，1 表示使用，0 表示不使用。

typedef enum
{
    DOG_TASK_MOTION_STOP = 0, // 表示机器人停止运动的状态。
    DOG_TASK_MOTION_FORWARD, // 表示机器人向前运动的状态。
    DOG_TASK_MOTION_BACKWARD, // 表示机器人向后运动的状态。
    DOG_TASK_MOTION_TURN_LEFT, // 表示机器人向左转的状态。
    DOG_TASK_MOTION_TURN_RIGHT, // 表示机器人向右转的状态。
} DogTaskMotion_t; // 机器人步态的枚举类型。

typedef enum
{
    DOG_TASK_EVENT_IDLE = 0, // 表示机器人处于空闲状态。
    DOG_TASK_EVENT_COLOR_PAUSE, // 表示机器人因颜色识别而暂停的状态。
    DOG_TASK_EVENT_FORK_TURN, // 表示机器人遇到分叉路口转弯的状态。
    DOG_TASK_EVENT_THROW_TRACK_DELAY, // 表示机器人在投掷前的循迹状态。
    DOG_TASK_EVENT_THROW_FORWARD, // 表示机器人在投掷前向前移动的状态。
    DOG_TASK_EVENT_THROW_ROTATING, // 表示机器人在投掷时进行旋转的状态。
} DogTaskEventState_t; // 机器人事件处理状态的枚举类型。

static DogTaskMotion_t s_motion = DOG_TASK_MOTION_STOP; // 当前正在执行的运动模式，例如停止、前进、左转或右转。
static DogTaskMotion_t s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD; // 短时间丢线时，用来记住上一帧循迹修正方向。
static DogTaskEventState_t s_event_state = DOG_TASK_EVENT_IDLE; // 当前事件状态机所在状态，例如普通循迹、分岔转向、投掷流程等。
static uint32_t s_event_start_ms; // 当前事件状态开始的系统时间，单位毫秒，用于计算事件已经执行多久。
static uint32_t s_color_pause_ms = DOG_TASK_COLOR_PAUSE_MS; // 颜色暂停事件本次需要保持的时间，单位毫秒。
static uint32_t s_color_pause_last_stand_ms; // 颜色暂停期间，上一次重新下发站立姿态的时间。
static ImageCommand_t s_pending_event_command = IMAGE_COMMAND_NONE; // 延迟执行的事件命令，例如紫色/棕色投掷命令。
static uint32_t s_last_gait_ms; // 上一次更新步态的时间。
static uint32_t s_last_track_ms; // 上一次收到有效循迹误差的时间。
static uint32_t s_last_status_ms; // 上一次向视觉模块回传 ST 状态的时间。
static uint8_t s_has_seen_track; // 是否已经收到过至少一帧有效循迹数据。
static uint8_t s_is_track_correcting; // 当前是否处于左/右纠偏状态。
static uint8_t s_platform_track_boost; // 蓝色平台事件触发后，是否启用平台循迹增强参数。
static uint8_t s_purple_throw_delay_used; // 紫色投掷事件是否已经使用过首次循迹延迟。
static uint8_t s_brown_throw_delay_used; // 棕色投掷事件是否已经使用过首次循迹延迟。

volatile uint32_t g_dog_task_run_count; // DogTask_Run() 被调用的次数，方便调试器观察主循环是否正常运行。
volatile uint32_t g_dog_task_gait_update_count; // 步态更新次数，方便判断是否持续下发步态。
volatile uint32_t g_dog_task_motion_stop_count; // 切换到停止运动的次数，方便排查异常停止。
volatile uint32_t g_dog_task_last_now_ms; // 最近一次 DogTask_Run() 中读取到的系统时间。
volatile uint32_t g_dog_task_last_gait_elapsed_ms; // 距离上一次步态更新已经过去的时间。
volatile uint32_t g_dog_task_last_track_lost_ms; // 距离上一次有效循迹数据已经过去的时间。
volatile int32_t g_dog_task_last_command; // 最近一次读取到的视觉事件命令。
volatile int32_t g_dog_task_last_motion; // 当前运动模式的调试镜像。
volatile int32_t g_dog_task_last_track_valid; // 最近一次循迹数据是否有效。
volatile int32_t g_dog_task_last_track_error; // 最近一次视觉循迹误差。

#if 0
/* 早期转向测试状态变量：当前关闭，仅保留历史测试入口。 */
static uint8_t s_turn_test_active;
static uint32_t s_turn_test_start_ms;
/* 早期自动测试状态变量：当前关闭，仅保留历史测试入口。 */
static uint8_t s_auto_test_active;
static uint32_t s_auto_test_start_ms;
#endif

/* 提前声明状态回传函数，供 OK 应答函数和周期 ST 回传复用。 */
static void DogTask_SendVisionStatus(const char *tag);

/* 收到蓝色平台命令后，开启平台循迹增强参数，并回到普通事件空闲状态继续循迹。 */
static void DogTask_BeginPlatformTrackBoost(void)
{
    s_platform_track_boost = 1U;
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_pending_event_command = IMAGE_COMMAND_NONE;
}

/* 根据目标运动模式设置步态参数，并记录当前运动状态；重复设置同一状态时直接返回。 */
static void DogTask_ApplyMotion(DogTaskMotion_t motion)
{
    if (motion == s_motion)
    {
        return;
    }

    if (motion == DOG_TASK_MOTION_STOP)
    {
        g_dog_task_motion_stop_count++;
    }

    if (motion == DOG_TASK_MOTION_FORWARD)
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
    else
    {
        DogGait_AllStand(DOG_TASK_GAIT_MOVE_MS);
    }

    s_motion = motion;
}

/* 控制 PC13 指示灯，用于显示颜色暂停或纠偏相关状态。 */
static void DogTask_SetCorrectionLed(uint8_t is_on)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,
                      LED_Pin,
                      (is_on != 0U) ? DOG_TASK_LED_ON_STATE : DOG_TASK_LED_OFF_STATE);
}

/* 将运动枚举转换成字符串，供 USART2 状态回传使用。 */
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
    return "STOP";
}

/* 将事件状态枚举转换成字符串，供 USART2 状态回传使用。 */
static const char *DogTask_EventName(DogTaskEventState_t state)
{
    static const char *names[] = {
        "IDLE",
        "COLOR_PAUSE",
        "FORK_TURN",
        "THROW_TRACK_DELAY",
        "THROW_FORWARD",
        "THROW_ROTATING",
    };

    if ((uint8_t)state < (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return names[(uint8_t)state];
    }

    return "UNKNOWN";
}

/* 判断视觉命令是否属于需要进入事件状态机处理的命令。 */
static uint8_t DogTask_IsEventCommand(ImageCommand_t command)
{
    /* Keep lost-line 9999 non-latching so tracking can recover on the next valid frame. */
    return (uint8_t)((command == IMAGE_COMMAND_TURN_LEFT) ||
                     (command == IMAGE_COMMAND_TURN_RIGHT) ||
                     (command == IMAGE_COMMAND_PLATFORM) ||
                     (command == IMAGE_COMMAND_PURPLE) ||
                     (command == IMAGE_COMMAND_BROWN));
}

/* 结束当前事件流程，清理事件和循迹标志，并恢复普通前进循迹。 */
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

/* 进入投掷前前进阶段；该阶段结束后会开始驱动投掷舵机旋转。 */
static void DogTask_BeginThrowForward(ImageCommand_t command, uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_THROW_FORWARD;
    s_event_start_ms = now_ms;
    s_pending_event_command = command;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}

/* 进入投掷前循迹延迟阶段；首次识别紫色/棕色时先继续循迹一段时间再投掷。 */
static void DogTask_BeginThrowTrackDelay(ImageCommand_t command, uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_THROW_TRACK_DELAY;
    s_event_start_ms = now_ms;
    s_pending_event_command = command;
}

/* 进入分岔转向阶段，按传入的左转或右转运动模式保持一小段时间后恢复循迹。 */
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

/* 进入投掷旋转阶段，停止机器狗步态，并按紫色/棕色选择投掷舵机旋转方向。 */
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

/* 进入颜色暂停阶段，持续下发站立姿态压住步态；当前主流程基本不使用。 */
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

/* 向视觉模块发送一次 OK 应答，表示 STM32 已经收到并处理事件命令。 */
static void DogTask_SendVisionAck(void)
{
    DogTask_SendVisionStatus("OK");
}

/* 通过 USART2 向视觉模块发送当前事件状态和运动状态，tag 可为 OK 或 ST。 */
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

/* 执行视觉事件命令：分岔转向、平台循迹增强、紫/棕投掷或停止。 */
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
        DogTask_BeginPlatformTrackBoost();
    }
    else
    {
        DogTask_ResumeTracking(now_ms);
    }
}

/* 推进当前事件状态机，根据已经经过的时间决定是否切换到下一阶段或恢复循迹。 */
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
}

#if 0
/* 早期转向测试入口：启动固定方向转向测试；当前关闭，仅保留参考。 */
static void DogTask_BeginTurnTest(DogTaskMotion_t motion, uint32_t now_ms)
{
    DogTask_ApplyMotion(motion);
    s_turn_test_active = 1U;
    s_turn_test_start_ms = now_ms;
}

/* 早期转向测试停止入口：结束测试并停止运动；当前关闭，仅保留参考。 */
static void DogTask_StopTurnTest(void)
{
    s_turn_test_active = 0U;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}
#endif

/* 处理普通单字节运动命令，目前主要处理前进和停止；事件命令由事件状态机处理。 */
static void DogTask_ApplyCommand(ImageCommand_t command, uint32_t now_ms)
{
    (void)now_ms;

    if (command == IMAGE_COMMAND_FORWARD)
    {
        s_is_track_correcting = 0U;
        DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
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

/* 根据视觉循迹误差计算左右腿差速步长，并设置前进、左纠偏或右纠偏步态。 */
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
/* 阻塞式运行某个运动一段时间的早期测试函数；当前关闭，仅保留参考。 */
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

/* 阻塞式自动测试前进、左转、右转的早期测试函数；当前关闭，仅保留参考。 */
static void DogTask_RunAutoTestBlocking(void)
{
    DogTask_RunMotionBlocking(DOG_TASK_MOTION_FORWARD, DOG_TASK_AUTO_FORWARD_MS);
    DogTask_RunMotionBlocking(DOG_TASK_MOTION_TURN_LEFT, DOG_TASK_AUTO_LEFT_MS);
    DogTask_RunMotionBlocking(DOG_TASK_MOTION_TURN_RIGHT, DOG_TASK_AUTO_RIGHT_MS);
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}
#endif

/* 初始化机器狗任务：初始化投掷舵机、总线舵机回中、进入站立、启动视觉接收并默认前进。 */
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
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
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

/* 机器狗主循环任务：读取视觉数据、处理事件状态机、更新步态、控制 LED 并周期回传状态。 */
void DogTask_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    ImageCommand_t command = ImageCommand_TakeLatest();
    ImageTrack_t track = ImageCommand_TakeLatestTrack();
    uint32_t track_lost_ms = (uint32_t)(now_ms - s_last_track_ms);

    g_dog_task_run_count++;
    g_dog_task_last_now_ms = now_ms;
    g_dog_task_last_command = (int32_t)command;
    g_dog_task_last_motion = (int32_t)s_motion;
    g_dog_task_last_track_valid = (int32_t)track.valid;
    g_dog_task_last_track_error = (int32_t)track.error;
    g_dog_task_last_track_lost_ms = track_lost_ms;
    g_dog_task_last_gait_elapsed_ms = (uint32_t)(now_ms - s_last_gait_ms);

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

    if ((s_motion != DOG_TASK_MOTION_STOP) &&
        ((uint32_t)(now_ms - s_last_gait_ms) >= DOG_TASK_GAIT_PERIOD_MS))
    {
        s_last_gait_ms = now_ms;
        g_dog_task_gait_update_count++;
        DogGait_UpdateTrot(DOG_TASK_GAIT_MOVE_MS);

    }

    if ((uint32_t)(now_ms - s_last_status_ms) >= DOG_TASK_STATUS_INTERVAL_MS)
    {
        s_last_status_ms = now_ms;
        DogTask_SendVisionStatus("ST");
    }
}
