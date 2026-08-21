#include "dog_task.h"

#include "dog_gait.h"
#include "dog_servo.h"
#include "image_command.h"
#include "jy61p_imu.h"
#include "main.h"
#include "stair_walk.h"
#include "throw_servo.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>

#define DOG_TASK_GAIT_NORMAL_PERIOD_MS      120U // 正常运行时的步态更新周期，单位毫秒。
#define DOG_TASK_GAIT_NORMAL_MOVE_MS        120U // 正常运行时的舵机目标过渡时间，单位毫秒。
#define DOG_TASK_GAIT_SHIFT_PERIOD_MS       120U // 左/右平移时的步态更新周期，单位毫秒。
#define DOG_TASK_GAIT_SHIFT_MOVE_MS         120U // 左/右平移时的舵机目标过渡时间，单位毫秒。
#define DOG_TASK_GAIT_SPEED_BUMP_PERIOD_MS  130U // 减速带阶段的步态更新周期，单位毫秒。
#define DOG_TASK_GAIT_SPEED_BUMP_MOVE_MS    130U // 减速带阶段的舵机目标过渡时间，单位毫秒。
#define DOG_TASK_SPEED_BUMP_WALK_ENABLE          1U    // 1: 减速带使用专用 walk；0: 保持原有 trot 方案。
#define DOG_TASK_SPEED_BUMP_WALK_PERIOD_MS       100U  // 减速带 walk 的目标更新周期。
#define DOG_TASK_SPEED_BUMP_WALK_MOVE_MS         80U   // 减速带 walk 的普通舵机过渡时间。
#define DOG_TASK_SPEED_BUMP_WALK_STEP_H_MM       60.0f // 与当前上楼梯 walk 一致的抬腿高度。
#define DOG_TASK_SPEED_BUMP_WALK_STEP_LEN_MM     60.0f // 与当前上楼梯 walk 一致的步长。
#define DOG_TASK_SPEED_BUMP_WALK_SPEED_FREQ      0.1f // 相位增量。
#define DOG_TASK_SPEED_BUMP_WALK_CG_BASE_X_MM    26.0f  // 减速带 walk 重心 X 基准。
#define DOG_TASK_SPEED_BUMP_WALK_IMU_GAIN_MM     60.0f // 与当前上楼梯 walk 一致的 pitch 重心补偿增益。
#define DOG_TASK_SPEED_BUMP_WALK_PITCH_ANGLE_GAIN 0.0f // 减速带 pitch 重心补偿的角度倍率。
#define DOG_TASK_SPEED_BUMP_WALK_PHASE_CG_GAIN   0.6f // 减速带 walk 前后腿阶段动态重心缩放；与上楼梯独立可调。
#define DOG_TASK_SPEED_BUMP_WALK_BODY_KP_FRONT_TO_REAR 0.6f // 减速带前腿切后腿时的重心收敛系数。
#define DOG_TASK_SPEED_BUMP_WALK_BODY_KP_REAR_TO_FRONT 0.3f // 减速带后腿切前腿时的重心收敛系数。
#define DOG_TASK_SPEED_BUMP_WALK_FRONT_REAR_UNIFIED 1U // 1: 减速带前腿与后腿同轨迹；0: 前腿保持正弦。
#define DOG_TASK_SPEED_BUMP_WALK_RB_PRELOAD_STABLE_UPDATES 0U // 0: 减速带关闭 RB 起摆前预加载稳定停顿。
#define DOG_TASK_SPEED_BUMP_WALK_SECOND_FRONT_TO_REAR_HOLD_UPDATES 1U // 1: 第二前腿落地后保持 100 ms。
#define DOG_TASK_SPEED_BUMP_WALK_RB_FINAL_PRELOAD_STABLE_UPDATES 1U // 1: RB 起摆前额外预加载稳定 100 ms。
#define DOG_TASK_SPEED_BUMP_WALK_REAR_PRELOAD_RELEASE_HOLD_UPDATES 0U // 0: 关闭释放冻结。
#define DOG_TASK_SPEED_BUMP_WALK_ORDER_TRANSITION_UPDATES 2U // 2: 周期切换过渡 200 ms。
#define DOG_TASK_SPEED_BUMP_WALK_FRONT_HEIGHT_MM 0.0f  // 前腿对支撑面高度；减速带首轮保持平地基准。
#define DOG_TASK_SPEED_BUMP_WALK_REAR_HEIGHT_MM  0.0f  // 后腿对支撑面高度；减速带首轮保持平地基准。
/* 减速带 walk 仅保留 LB 与 EXTRA LB 的标定值；其余预加载侧向补偿项为 0。 */
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_LF_Y_MM           0.0f
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_RF_Y_MM           0.0f
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_LB_Y_MM           -2.0f
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_EXTRA_LF_Y_MM     0.0f
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_EXTRA_RF_Y_MM     0.0f
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_EXTRA_LB_Y_MM     -10.0f
#define DOG_TASK_SPEED_BUMP_WALK_PRELOAD_LB_RIGHT_RF_Y_MM  0.0f
#define DOG_TASK_SPEED_BUMP_WALK_CYCLE_COUNT     10U   // 完成该数量的完整 walk 周期后退出减速带。
#define DOG_TASK_SPEED_BUMP_TEST_DURATION_MS 50000U // 独立过减速带测试的超时兜底；walk 正常按周期数结束，约 20 s。
#define DOG_TASK_SPEED_BUMP_ENTRY_TEST_DURATION_MS 150000U // 减速带前循迹阶段的独立测试持续时间，单位毫秒；到点后回到站立姿态。
#define DOG_TASK_LED_ON_STATE          GPIO_PIN_SET // 表示 LED 灯亮的状态，GPIO_PIN_SET 表示将 GPIO 引脚设置为高电平，通常用于点亮 LED。
#define DOG_TASK_LED_OFF_STATE         GPIO_PIN_RESET // 表示 LED 灯灭的状态，GPIO_PIN_RESET 表示将 GPIO 引脚设置为低电平，通常用于熄灭 LED。
#define DOG_TASK_COLOR_PAUSE_MS        2000U // 表示颜色暂停的时间，单位毫秒。
#define DOG_TASK_COLOR_PAUSE_HOLD_MS   150U // 表示颜色暂停保持的时间，单位毫秒。
#define DOG_TASK_VISION_COLOR_STOP_TEST_ENABLE 0U // 表示是否启用视觉颜色停止测试，0表示禁用，1表示启用。
#define DOG_TASK_VISION_COLOR_STOP_MS          10000U // 表示「视觉颜色停止测试模式」的停止保持时间，单位毫秒。
#define DOG_TASK_THROW_FORWARD_MS      0U // 表示投掷前前进阶段的持续时间，单位毫秒。这个现在是 0，我其实不知道这个变量是什么时候加上去的，可能是中间调试的时候为了让机器人在投掷前稍微前进一点点，避免投掷时机器人离目标太远。但是暂时可以不用管。
#define DOG_TASK_THROW_TRACK_DELAY_MS  1000U // 表示首次识别到紫色 / 棕色投掷事件后，先继续循迹的延迟时间。体现在实际中，就是识别到紫色 / 棕色后，往前走 DOG_TASK_THROW_TRACK_DELAY_MS 这么多 ms，然后再开始旋转投掷。
#define DOG_TASK_VISION_ACK_TIMEOUT_MS 10U // 表示 stm32 给 k230 回传状态字符串时，发送超时的时间。也就是说如果串口发送在 10ms 内没有完成，就返回超时。
#define DOG_TASK_PLATFORM_PAUSE_TEST_ENABLE 1U
#define DOG_TASK_PLATFORM_PAUSE_TEST_MS 5000U
#define DOG_TASK_PLATFORM_YES_SEND_MS 5000U
#define DOG_TASK_PLATFORM_YES_INTERVAL_MS 200U
#define DOG_TASK_PLATFORM_FAKE_IMU_TEST_MS 5000U
#define DOG_TASK_STATUS_INTERVAL_MS    200U // 表示 stm32 向 k230 周期性发送状态反馈的时间间隔，单位毫秒。
#define DOG_TASK_STEP_H_MM             45.0f // 表示机器人步态的步高，单位毫米。
#define DOG_TASK_FORWARD_R_MM          70.0f // 表示机器人步态的前进半径，单位毫米。
#define DOG_TASK_SHIFT_H_MM             45.0f // 表示机器人步态的平移步高，单位毫米。

//0 LF 1 RF 2 LB 3 RB
float DOG_TASK_SHIFT_R_MML[4] = {20.0f, 80.0f, 20.0f, 80.0f}; // 表示机器人步态的前进半径，单位毫米。四条腿的前进半径可以不同.
float DOG_TASK_SHIFT_R_MMR[4] = {80.0f, 20.0f, 80.0f, 20.0f}; // 表示机器人步态的前进半径，单位毫米。四条腿的前进半径可以不同.
#define DOG_TASK_TURN_R_MM             15.0f // 表示机器人步态的转弯半径，单位毫米。
#define DOG_TASK_SPEED_FREQ            0.25f // 表示机器人步态的速度频率，单位为每毫秒的步长。

#define DOG_TASK_TRACK_DEADBAND        35U // 表示循迹误差的死区范围，单位毫米。也就是说，如果摄像头识别到的线条偏离机器人中心线的距离在 ±35mm 以内，就认为机器人不需要调整方向，继续前进即可。 
#define DOG_TASK_TRACK_RECOVER_MS      500U // 表示循迹丢失后，机器人保持上一次循迹动作的时间，单位毫秒。
#define DOG_TASK_TRACK_STEP_H_MM       40.0f // 表示循迹时的步高，单位毫米。
#define DOG_TASK_TRACK_LEFT_FORWARD_R_MM   55.0f // 表示循迹时向左前进的半径，单位毫米。    
#define DOG_TASK_TRACK_RIGHT_FORWARD_R_MM  55.0f // 表示循迹时向右前进的半径，单位毫米。
#define DOG_TASK_SPEED_BUMP_TRACK_STEP_H_MM       45.0f // 过减速带循迹步高，单位毫米。
#define DOG_TASK_SPEED_BUMP_TRACK_LEFT_FORWARD_R_MM  40.0f // 过减速带循迹左侧步长，单位毫米。
#define DOG_TASK_SPEED_BUMP_TRACK_RIGHT_FORWARD_R_MM 40.0f // 过减速带循迹右侧步长，单位毫米。
#define DOG_TASK_TRACK_MAX_STEER_MM    30.0f // 表示循迹时的最大转向量，单位毫米。将 steer 限制在 ±18mm 以内，防止机器人转向过度。
#define DOG_TASK_TRACK_STEER_GAIN      0.4f // 表示循迹时的转向增益系数。这个增益系数就是用来计算转向量 steer 的。steer = error * DOG_TASK_TRACK_STEER_GAIN。
#define DOG_TASK_TRACK_ROLL_GAIN       0.15f // 循迹差速时机身横滚补偿增益，roll_mm = steer * ROLL_GAIN。
#define DOG_TASK_TRACK_MAX_ROLL_MM     99.0f // 循迹差速时机身横滚补偿上限，单位毫米。（横滚补偿与差速值steer呈线性关系，受steer上下限约束，此处事实上不构成限制，仅为预留）
#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          45.0f // 表示平台循迹时的步高，单位毫米。
#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  50.0f // 表示平台循迹时向左前进的半径，单位毫米。    
#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 50.0f // 表示平台循迹时向右前进的半径，单位毫米。
#define DOG_TASK_START_SHIFT_LEFT_DURATION_MS 2000U // 启动后的左平移阶段持续时间，单位毫秒。
#define DOG_TASK_SPEED_BUMP_ENTRY_DELAY_MS 8000U // 左平移结束后、进入减速带前的普通循迹时间，单位毫秒。
#define DOG_TASK_SPEED_BUMP_EXIT_DELAY_MS  15000U // 进入减速带状态后，退出到普通循迹前的保持时间，单位毫秒。
#define DOG_TASK_BLACK_CENTER_STABLE_MS    500U // 上楼梯阶段，黑框识别到机器狗已经到中心后，需要稳定保持的时间。
#define DOG_TASK_DOWNHILL_MIN_MS           1500U // 进入下坡循迹后，最少要跑的时间。
#define DOG_TASK_LEVEL_PITCH_DEG           5.0f // 判断机身前后方向接近水平的 pitch 阈值。
#define DOG_TASK_LEVEL_ROLL_DEG            6.0f // 判断机身左右方向接近水平的 roll 阈值。
#define DOG_TASK_LEVEL_STABLE_MS           800U // 判断机身接近水平后，需要保持的时间，单位毫秒。   
#define DOG_TASK_ORANGE_TRACK_DELAY_MS     4000U // 橙色循迹延迟时间，单位毫秒。
#define DOG_TASK_SHIFT_RIGHT_MS            12000U // 右平移时间，单位毫秒。
#define DOG_TASK_LAP_PAUSE_MS              8000U // 完成一圈后的暂停时间，单位毫秒。

#define DOG_TASK_BLACK_TRACK_DELAY_MS      3000U // 识别黑框后，按视觉偏差循迹的保持时间。
#define DOG_TASK_STAIR_WALK_ENABLE         1U // 置 0 跳过上楼梯，直接进入绿色分岔测试阶段。
#define DOG_TASK_FORK_TEST_ENABLE           0U // 置 1 上电后跳过分岔前全部任务阶段。
#define DOG_TASK_FORK_TEST_LAP_COUNT        0U // 分岔测试圈数：1 表示直接测试第二圈绿色后的左转流程。

/* Left/right turn test entry is kept only for reference. */
#define DOG_TASK_TURN_TEST_DURATION_MS 3000U // 表示左/右转测试的持续时间，单位毫秒。这个测试是用来验证机器人在转弯时的步态和转向是否正常的。
#define DOG_TASK_GREEN_TURN_DURATION_MS 2000U // 表示绿色岔路第一圈右转的持续时间，单位毫秒。
#define DOG_TASK_GREEN_TRACK_DELAY_MS 3000U //识别绿色后，保持视觉循迹的时间，单位毫秒。
#define DOG_TASK_GREEN_SECOND_LAP_LEFT_TURN_DURATION_MS 2000U // 第二圈绿色岔路左转的持续时间，单位毫秒。
#define DOG_TASK_GREEN_LEFT_STEER_MM     25.0f // 表示绿色岔路差速转向量，单位毫米；正/负号分别对应右/左转。

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
    DOG_TASK_MOTION_SHIFT_LEFT, // 表示机器人向左平移的状态。
    DOG_TASK_MOTION_SHIFT_RIGHT, // 表示机器人向右平移的状态。
} DogTaskMotion_t; // 机器人步态的枚举类型。

typedef enum
{
    DOG_TASK_EVENT_IDLE = 0, // 表示机器人处于空闲状态。
    DOG_TASK_EVENT_COLOR_PAUSE, // 表示机器人因颜色识别而暂停的状态。
    DOG_TASK_EVENT_FORK_TURN, // 表示机器人遇到分叉路口转弯的状态。
    DOG_TASK_EVENT_THROW_TRACK_DELAY, // 表示机器人在投掷前的循迹状态。
    DOG_TASK_EVENT_THROW_FORWARD, // 表示机器人在投掷前向前移动的状态。
    DOG_TASK_EVENT_THROW_ROTATING, // 表示机器人在投掷时进行旋转的状态。
    DOG_TASK_EVENT_PLATFORM_PAUSE, // 表示蓝色平台事件触发后，测试用的停止等待状态。
    DOG_TASK_EVENT_PLATFORM_YES_SEND, // 表示蓝色平台暂停结束后，测试用的连续发送 YES 状态。
    DOG_TASK_EVENT_STAIR_WALK, // 表示机器人爬楼梯的状态。
    DOG_TASK_EVENT_START_SHIFT_LEFT, // 表示机器人启动后，开始阶段向左平移的状态。  
    DOG_TASK_EVENT_SPEED_BUMP_ENTRY_TRACK, // 表示进入减速带前的普通循迹状态。
    DOG_TASK_EVENT_SPEED_BUMP, // 表示机器人通过减速带的状态。
    DOG_TASK_EVENT_SHIFT_TO_CENTER, // 表示机器人在上楼梯阶段，识别到黑色线条后，向中心位置平移的状态。
    DOG_TASK_EVENT_ORANGE_TRACK_DELAY, // 表示机器人在橙色循迹阶段的延迟状态。
    DOG_TASK_EVENT_SHIFT_RIGHT, // 表示机器人向右平移的状态。
    DOG_TASK_EVENT_LAP_PAUSE, // 表示机器人完成一圈后的暂停状态。
    DOG_TASK_EVENT_BLACK_TRACK_DELAY,
    DOG_TASK_EVENT_GREEN_TRACK_DELAY,
} DogTaskEventState_t; // 机器人事件处理状态的枚举类型。


typedef enum
{
    DOG_TASK_STAGE_START_SHIFT_LEFT = 0, // 表示机器人启动后，开始阶段向左平移的任务阶段。
    DOG_TASK_STAGE_SPEED_BUMP_ENTRY_TRACK, // 表示进入减速带前的普通循迹任务阶段。
    DOG_TASK_STAGE_SPEED_BUMP, // 表示机器人通过减速带的任务阶段。
    DOG_TASK_STAGE_TRACK_TO_BLUE, // 表示机器人循迹到蓝色平台的任务阶段。   
    DOG_TASK_STAGE_STAIR_WALK, // 表示机器人爬楼梯的任务阶段。  
    DOG_TASK_STAGE_WAIT_BLACK, // 表示机器人在上楼梯阶段，等待识别到黑色线条的任务阶段。
    DOG_TASK_STAGE_SHIFT_TO_CENTER, // 表示机器人在上楼梯阶段，识别到黑色线条后，向中心位置平移的任务阶段。
    DOG_TASK_STAGE_DOWNHILL_TRACK, // 表示机器人在下坡循迹阶段的任务阶段。
    DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL, // 表示机器人在下坡后循迹阶段的任务阶段。
    DOG_TASK_STAGE_GREEN_TURN, // 表示机器人在绿色转弯阶段的任务阶段。
    DOG_TASK_STAGE_TRACK_TO_THROW, // 表示机器人在投掷前循迹阶段的任务阶段。
    DOG_TASK_STAGE_THROW_TARGET, // 表示机器人在投掷目标阶段的任务阶段。
    DOG_TASK_STAGE_TRACK_TO_ORANGE, // 表示机器人在橙色循迹阶段的任务阶段。
    DOG_TASK_STAGE_ORANGE_TRACK_DELAY, // 表示机器人在橙色循迹阶段的延迟状态。
    DOG_TASK_STAGE_SHIFT_RIGHT, // 表示机器人向右平移的任务阶段。
    DOG_TASK_STAGE_LAP_PAUSE, // 表示机器人完成一圈后的暂停任务阶段。
    DOG_TASK_STAGE_FINISHED, // 表示机器人任务完成的任务阶段。
} DogTaskStage_t;
static DogTaskMotion_t s_motion = DOG_TASK_MOTION_STOP; // 当前正在执行的运动模式，例如停止、前进、左转或右转。
static DogTaskMotion_t s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD; // 短时间丢线时，用来记住上一帧循迹修正方向。
static DogTaskEventState_t s_event_state = DOG_TASK_EVENT_IDLE; // 当前事件状态机所在状态，例如普通循迹、分岔转向、投掷流程等。
static DogTaskStage_t s_task_stage = DOG_TASK_STAGE_START_SHIFT_LEFT; // 当前任务阶段，例如循迹到蓝色平台、爬楼梯或等待黑色线条。
static uint32_t s_event_start_ms; // 当前事件状态开始的系统时间，单位毫秒，用于计算事件已经执行多久。
static uint32_t s_color_pause_ms = DOG_TASK_COLOR_PAUSE_MS; // 颜色暂停事件本次需要保持的时间，单位毫秒。
static uint32_t s_color_pause_last_stand_ms; // 颜色暂停期间，上一次重新下发站立姿态的时间。
static ImageCommand_t s_pending_event_command = IMAGE_COMMAND_NONE; // 延迟执行的事件命令，例如紫色/棕色投掷命令。
static uint32_t s_last_gait_ms; // 上一次更新步态的时间。
static uint32_t s_last_track_ms; // 上一次收到有效循迹误差的时间。
static uint32_t s_last_status_ms; // 上一次向视觉模块回传 ST 状态的时间。
static uint32_t s_platform_yes_last_ms; // 蓝色平台暂停测试结束后，上一次向 K230 发送 YES 的时间。
static uint32_t s_black_center_start_ms; // 上楼梯阶段，黑框识别到机器狗已经到中心后，开始计时的时间。  
static uint32_t s_level_start_ms; // 上楼梯阶段，机器狗机身接近水平后，开始计时的时间。
static uint8_t s_has_seen_track; // 是否已经收到过至少一帧有效循迹数据。
static uint8_t s_is_track_correcting; // 当前是否处于左/右纠偏状态。
static uint8_t s_platform_track_boost; // 蓝色平台事件触发后，是否启用平台循迹增强参数。
static uint8_t s_wait_platform_imu; // 蓝色平台事件触发后，等待 IMU 确认机器狗已经上完高台。
static uint8_t s_purple_throw_delay_used; // 紫色投掷事件是否已经使用过首次循迹延迟。
static uint8_t s_brown_throw_delay_used; // 棕色投掷事件是否已经使用过首次循迹延迟。
static uint8_t s_lap_count; // 圈数计数。
static uint8_t s_speed_bump_test_active;
static uint32_t s_speed_bump_test_start_ms;
static uint8_t s_speed_bump_entry_test_active;
static uint32_t s_speed_bump_entry_test_start_ms;
static uint32_t s_speed_bump_entry_test_last_gait_ms;
static uint8_t s_speed_bump_walk_cycle_count;
static uint8_t s_color_reaction_test_lap2_ready;

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
volatile uint8_t g_dog_task_speed_bump_walk_cycle_count; // 减速带 walk 已完成的完整周期数。
volatile HAL_StatusTypeDef g_k230_tx_status;

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
static void DogTask_ApplyTrackError(int16_t error);

/* 循迹差速时机身横滚补偿量与 steer 线性相关，并在安全范围内限幅。 */
static float DogTask_TrackRollFromSteer(float steer_mm)
{
    float roll_mm = steer_mm * DOG_TASK_TRACK_ROLL_GAIN;

    if (roll_mm > DOG_TASK_TRACK_MAX_ROLL_MM)
    {
        roll_mm = DOG_TASK_TRACK_MAX_ROLL_MM;
    }
    else if (roll_mm < -DOG_TASK_TRACK_MAX_ROLL_MM)
    {
        roll_mm = -DOG_TASK_TRACK_MAX_ROLL_MM;
    }

    return roll_mm;
}

#if !DOG_TASK_PLATFORM_PAUSE_TEST_ENABLE
static uint8_t DogTask_IsPlatformFinishedByImu(void)
{
    static uint32_t start_ms = 0U;

    if (s_wait_platform_imu == 0U)
    {
        start_ms = 0U;
        return 0U;
    }

    if (start_ms == 0U)
    {
        start_ms = HAL_GetTick();
        return 0U;
    }

    return (uint8_t)((uint32_t)(HAL_GetTick() - start_ms) >= DOG_TASK_PLATFORM_FAKE_IMU_TEST_MS);
}
#endif

/* 向 K230 发送控制消息，包含消息内容和长度。 */
static void DogTask_SendK230Control(uint8_t *message, uint16_t len)
{
    if ((message == 0) || (len == 0U))
    {
        return;
    }

    g_k230_tx_status = HAL_UART_Transmit(&huart1,
                                         message,
                                         len,
                                         DOG_TASK_VISION_ACK_TIMEOUT_MS);
}

static void DogTask_SendK230Yes(void)
{
    uint8_t message[] = "YES\n";

    DogTask_SendK230Control(message, (uint16_t)(sizeof(message) - 1U));
}

/* 收到蓝色平台命令后，开启平台循迹增强参数，并回到普通事件空闲状态继续循迹。 */
static void DogTask_BeginPlatformTrackBoost(void)
{
    s_platform_track_boost = 1U;
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_pending_event_command = IMAGE_COMMAND_NONE;
}

static uint16_t DogTask_GetGaitPeriodMs(void)
{
    if (s_task_stage == DOG_TASK_STAGE_SPEED_BUMP)
    {
        return DOG_TASK_GAIT_SPEED_BUMP_PERIOD_MS;
    }

    if ((s_motion == DOG_TASK_MOTION_SHIFT_LEFT) ||
        (s_motion == DOG_TASK_MOTION_SHIFT_RIGHT))
    {
        return DOG_TASK_GAIT_SHIFT_PERIOD_MS;
    }

    return DOG_TASK_GAIT_NORMAL_PERIOD_MS;
}

static uint16_t DogTask_GetGaitMoveMs(void)
{
    if (s_task_stage == DOG_TASK_STAGE_SPEED_BUMP)
    {
        return DOG_TASK_GAIT_SPEED_BUMP_MOVE_MS;
    }

    if ((s_motion == DOG_TASK_MOTION_SHIFT_LEFT) ||
        (s_motion == DOG_TASK_MOTION_SHIFT_RIGHT))
    {
        return DOG_TASK_GAIT_SHIFT_MOVE_MS;
    }

    return DOG_TASK_GAIT_NORMAL_MOVE_MS;
}

static DogGaitFootBase_t DogTask_GetTrackFootBase(void)
{
    return (s_task_stage == DOG_TASK_STAGE_SPEED_BUMP) ?
               DOG_GAIT_FOOT_BASE_SPEED_BUMP :
               DOG_GAIT_FOOT_BASE_TURN;
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
    else if (motion == DOG_TASK_MOTION_SHIFT_LEFT)
    {
        DogGait_SetShiftLeftParams(DOG_TASK_STEP_H_MM,
                                   DOG_TASK_SHIFT_R_MML,
                                   DOG_TASK_SPEED_FREQ,
                                   DOG_GAIT_FOOT_BASE_SHIFT_LEFT);

        DogGait_SetShiftStepR(DOG_TASK_SHIFT_R_MML);

    }
    else if (motion == DOG_TASK_MOTION_SHIFT_RIGHT)
    {
        DogGait_SetShiftRightParams(DOG_TASK_STEP_H_MM,
                                    DOG_TASK_SHIFT_R_MMR,
                                    DOG_TASK_SPEED_FREQ,
                                    DOG_GAIT_FOOT_BASE_SHIFT_RIGHT
                                   );

        DogGait_SetShiftStepR(DOG_TASK_SHIFT_R_MMR);
    }
    else
    {
        DogGait_AllStand(DogTask_GetGaitMoveMs());
    }

    s_motion = motion;
}

/* 控制 PC13 指示灯，用于显示颜色暂停或、纠偏相关状态。 */
static void DogTask_SetCorrectionLed(uint8_t is_on)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,
                      LED_Pin,
                      (is_on != 0U) ? DOG_TASK_LED_ON_STATE : DOG_TASK_LED_OFF_STATE);
}

/* 将运动枚举转换成字符串，供 USART1 状态回传使用。 */
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

/* 将事件状态枚举转换成字符串，供 USART1 状态回传使用。 */
static const char *DogTask_EventName(DogTaskEventState_t state)
{
    static const char *names[] = {
        "IDLE",
        "COLOR_PAUSE",
        "FORK_TURN",
        "THROW_TRACK_DELAY",
        "THROW_FORWARD",
        "THROW_ROTATING",
        "PLATFORM_PAUSE",
        "PLATFORM_YES_SEND",
        "STAIR_WALK",
        "START_SHIFT_LEFT",
        "SPEED_BUMP_ENTRY_TRACK",
        "SPEED_BUMP",
        "SHIFT_TO_CENTER",
        "ORANGE_TRACK_DELAY",
        "SHIFT_RIGHT",
        "LAP_PAUSE",
        "BLACK_TRACK_DELAY",
        "GREEN_TRACK_DELAY",
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
                     (command == IMAGE_COMMAND_BROWN) ||
                     (command == IMAGE_COMMAND_GREEN) ||
                     (command == IMAGE_COMMAND_BLACK) ||
                     (command == IMAGE_COMMAND_ORANGE));
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

/* 进入启动后向左平移阶段，等待一段时间后再开始循迹。 */
static void DogTask_BeginStartShiftLeft(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_START_SHIFT_LEFT;
    s_event_state = DOG_TASK_EVENT_START_SHIFT_LEFT;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_platform_track_boost = 0U;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_SHIFT_LEFT;
    DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_LEFT);
}

/* 左平移结束后，先按普通循迹运行一段时间，再进入减速带。 */
static void DogTask_BeginSpeedBumpEntryTrack(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_SPEED_BUMP_ENTRY_TRACK;
    s_event_state = DOG_TASK_EVENT_SPEED_BUMP_ENTRY_TRACK;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
}

/* 进入减速带阶段：改用减速带专用循迹参数与步态时序。 */
static void DogTask_BeginSpeedBump(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_SPEED_BUMP;
    s_event_state = DOG_TASK_EVENT_SPEED_BUMP;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    s_speed_bump_walk_cycle_count = 0U;
    g_dog_task_speed_bump_walk_cycle_count = 0U;

#if (DOG_TASK_SPEED_BUMP_WALK_ENABLE != 0U)
    /* This mode intentionally does not reuse stair_walk.c.  Its parameters
     * and completion criterion are dedicated to the low speed-bump course. */
    DogGait_SetWalkParams(DOG_TASK_SPEED_BUMP_WALK_STEP_H_MM,
                          DOG_TASK_SPEED_BUMP_WALK_STEP_LEN_MM,
                          DOG_TASK_SPEED_BUMP_WALK_SPEED_FREQ,
                          DOG_TASK_SPEED_BUMP_WALK_CG_BASE_X_MM,
                          DOG_TASK_SPEED_BUMP_WALK_IMU_GAIN_MM);
    DogGait_SetWalkPreloadSideOffsets(DOG_TASK_SPEED_BUMP_WALK_PRELOAD_LF_Y_MM,
                                      DOG_TASK_SPEED_BUMP_WALK_PRELOAD_RF_Y_MM,
                                      DOG_TASK_SPEED_BUMP_WALK_PRELOAD_LB_Y_MM,
                                      DOG_TASK_SPEED_BUMP_WALK_PRELOAD_EXTRA_LF_Y_MM,
                                      DOG_TASK_SPEED_BUMP_WALK_PRELOAD_EXTRA_RF_Y_MM,
                                      DOG_TASK_SPEED_BUMP_WALK_PRELOAD_EXTRA_LB_Y_MM,
                                      DOG_TASK_SPEED_BUMP_WALK_PRELOAD_LB_RIGHT_RF_Y_MM);
    DogGait_SetWalkPitchAngleGain(
        DOG_TASK_SPEED_BUMP_WALK_PITCH_ANGLE_GAIN);
    DogGait_SetWalkBodyKpFrontToRear(DOG_TASK_SPEED_BUMP_WALK_BODY_KP_FRONT_TO_REAR);
    DogGait_SetWalkBodyKpRearToFront(DOG_TASK_SPEED_BUMP_WALK_BODY_KP_REAR_TO_FRONT);
    DogGait_SetWalkFrontRearUnified(DOG_TASK_SPEED_BUMP_WALK_FRONT_REAR_UNIFIED);
    DogGait_SetWalkRbPreloadStableUpdates(DOG_TASK_SPEED_BUMP_WALK_RB_PRELOAD_STABLE_UPDATES);
    DogGait_SetWalkSecondFrontToRearHoldUpdates(DOG_TASK_SPEED_BUMP_WALK_SECOND_FRONT_TO_REAR_HOLD_UPDATES);
    DogGait_SetWalkRbFinalPreloadStableUpdates(DOG_TASK_SPEED_BUMP_WALK_RB_FINAL_PRELOAD_STABLE_UPDATES);
    DogGait_SetWalkRearPreloadReleaseHoldUpdates(DOG_TASK_SPEED_BUMP_WALK_REAR_PRELOAD_RELEASE_HOLD_UPDATES);
    DogGait_SetWalkOrderTransitionUpdates(DOG_TASK_SPEED_BUMP_WALK_ORDER_TRANSITION_UPDATES);
    DogGait_SetWalkPhaseCgGain(DOG_TASK_SPEED_BUMP_WALK_PHASE_CG_GAIN);
    DogGait_ResetWalk();
    DogGait_SetWalkSupportHeights(DOG_TASK_SPEED_BUMP_WALK_FRONT_HEIGHT_MM,
                                  DOG_TASK_SPEED_BUMP_WALK_REAR_HEIGHT_MM);
    s_motion = DOG_TASK_MOTION_FORWARD;
#else
    /* 减速带阶段统一按视觉循迹参数运行；初始帧使用零误差直行。 */
    DogTask_ApplyTrackError(0);
#endif
}

/* 主流程和独立减速带测试共用的 walk 推进；walk 未启用或条件不满足时返回 0。 */
static uint8_t DogTask_UpdateSpeedBumpWalk(uint32_t now_ms)
{
#if (DOG_TASK_SPEED_BUMP_WALK_ENABLE != 0U)
    if ((s_task_stage == DOG_TASK_STAGE_SPEED_BUMP) &&
        (s_event_state == DOG_TASK_EVENT_SPEED_BUMP) &&
        ((uint32_t)(now_ms - s_last_gait_ms) >=
         DOG_TASK_SPEED_BUMP_WALK_PERIOD_MS))
    {
        Jy61PImuStatus_t imu;
        float pitch_deg = 0.0f;
        float roll_deg = 0.0f;

        s_last_gait_ms = now_ms;
        g_dog_task_gait_update_count++;

        if (Jy61PImu_GetStatus(&imu) != 0U)
        {
            pitch_deg = imu.pitch_deg;
        }

        /* Reuse the stair-walk low-pass filter and continuous roll deadband
         * so speed-bump vibration is not sent directly to the left/right
         * foot-height compensation. */
        roll_deg = StairWalk_GetFilteredRollDeg();

        DogGait_UpdateWalk(DOG_TASK_SPEED_BUMP_WALK_MOVE_MS,
                           pitch_deg,
                           roll_deg);

        if (DogGait_IsWalkCycleDone() != 0U)
        {
            s_speed_bump_walk_cycle_count++;
            g_dog_task_speed_bump_walk_cycle_count =
                s_speed_bump_walk_cycle_count;
        }

        return 1U;
    }
#endif
    return 0U;
}

/* 进入循迹到蓝色平台阶段。 */
static void DogTask_BeginTrackToBlue(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_TRACK_TO_BLUE;
    s_platform_track_boost = 0U;
    s_level_start_ms = 0U;

    /* 若减速带最后一帧为直行，强制重新下发参数以退出减速带专用基准。 */
    s_motion = DOG_TASK_MOTION_STOP;
    DogTask_ResumeTracking(now_ms);
}

/* 将状态机切到准备根据黑框偏差居中的状态。 */
static void DogTask_BeginShiftToCenter(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_SHIFT_TO_CENTER;
    s_event_state = DOG_TASK_EVENT_SHIFT_TO_CENTER;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_black_center_start_ms = 0U;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}

/* 进入下坡循迹阶段。 */
static void DogTask_BeginDownhillTrack(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_DOWNHILL_TRACK;
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_platform_track_boost = 0U;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_level_start_ms = 0U;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
}

/* 跳过楼梯测试时，直接进入绿色分岔前的普通循迹阶段。 */
static void DogTask_BeginTrackAfterDownhill(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL;
    s_level_start_ms = 0U;
    DogTask_ResumeTracking(now_ms);
}

/* 进入到判断机身是否平的状态。*/
/* Hold normal visual tracking for a fixed interval after black is detected.
 * The platform tracking parameters remain active until downhill begins. */
static void DogTask_BeginBlackTrackDelay(uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_BLACK_TRACK_DELAY;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
}

static uint8_t DogTask_IsBodyLevelStable(uint32_t now_ms)
{
    Jy61PImuStatus_t imu;

    if (Jy61PImu_GetStatus(&imu) == 0U)
    {
        s_level_start_ms = 0U;
        return 0U;
    }

    if ((fabsf(imu.pitch_deg) <= DOG_TASK_LEVEL_PITCH_DEG) &&
        (fabsf(imu.roll_deg) <= DOG_TASK_LEVEL_ROLL_DEG))
    {
        if (s_level_start_ms == 0U)
        {
            s_level_start_ms = now_ms;
            return 0U;
        }

        return (uint8_t)((uint32_t)(now_ms - s_level_start_ms) >= DOG_TASK_LEVEL_STABLE_MS);
    }

    s_level_start_ms = 0U;
    return 0U;
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

/* 进入绿色岔路左转阶段，使用循迹差速方式（左腿慢、右腿快），保持向前行进的同时左转。 */
static void DogTask_BeginGreenLeftTurn(uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_FORK_TURN;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_TURN_LEFT;

    /* 用循迹差速参数实现左转: steer 为负表示左侧步长减小、右侧步长增大 */
    DogGait_SetTrackParams(DOG_TASK_TRACK_STEP_H_MM,
                           DOG_TASK_TRACK_LEFT_FORWARD_R_MM,
                           DOG_TASK_TRACK_RIGHT_FORWARD_R_MM,
                           -DOG_TASK_GREEN_LEFT_STEER_MM,
                           DogTask_TrackRollFromSteer(-DOG_TASK_GREEN_LEFT_STEER_MM),
                           DOG_TASK_SPEED_FREQ);
    s_motion = DOG_TASK_MOTION_TURN_LEFT;
}

/* 第二圈识别绿色后，先按视觉误差循迹，再进入固定左转。 */
static void DogTask_BeginGreenTrackDelay(uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_GREEN_TRACK_DELAY;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyTrackError(0);
}

/* 进入绿色岔路右转阶段，使用循迹差速方式（右腿慢、左腿快），保持向前行进的同时右转。 */
static void DogTask_BeginGreenRightTurn(uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_FORK_TURN;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_TURN_RIGHT;

    /* 用循迹差速参数实现右转: steer 为正表示右侧步长减小、左侧步长增大 */
    DogGait_SetTrackParams(DOG_TASK_TRACK_STEP_H_MM,
                           DOG_TASK_TRACK_LEFT_FORWARD_R_MM,
                           DOG_TASK_TRACK_RIGHT_FORWARD_R_MM,
                           DOG_TASK_GREEN_LEFT_STEER_MM,
                           DogTask_TrackRollFromSteer(DOG_TASK_GREEN_LEFT_STEER_MM),
                           DOG_TASK_SPEED_FREQ);
    s_motion = DOG_TASK_MOTION_TURN_RIGHT;
}

/* 进入上楼梯行走阶段。 */
static void DogTask_BeginStairWalk(uint32_t now_ms)
{
    s_event_state = DOG_TASK_EVENT_STAIR_WALK;
    s_task_stage = DOG_TASK_STAGE_STAIR_WALK;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    StairWalk_Start();
}

/* 进入橙色循迹延迟阶段。*/
static void DogTask_BeginOrangeTrackDelay(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_ORANGE_TRACK_DELAY;
    s_event_state = DOG_TASK_EVENT_ORANGE_TRACK_DELAY;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
}

/* 进入向右平移阶段。 */
static void DogTask_BeginShiftRight(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_SHIFT_RIGHT;
    s_event_state = DOG_TASK_EVENT_SHIFT_RIGHT;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_SHIFT_RIGHT;
    DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_RIGHT);
}

/* 进入完成一圈后的暂停阶段。 */
static void DogTask_BeginLapPause(uint32_t now_ms)
{
    s_task_stage = DOG_TASK_STAGE_LAP_PAUSE;
    s_event_state = DOG_TASK_EVENT_LAP_PAUSE;
    s_event_start_ms = now_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_last_track_ms = now_ms;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
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
    DogGait_AllStand(DogTask_GetGaitMoveMs()); // 在颜色事件发生时让机器人立即停止移动，进入站立姿态，单位毫秒，根据实际情况调整，过大可能导致步态不够稳定。
    s_motion = DOG_TASK_MOTION_STOP;
    DogTask_SetCorrectionLed(1U);
}

/* 向视觉模块发送一次 OK 应答，表示 STM32 已经收到并处理事件命令。 */
static void DogTask_SendVisionAck(void)
{
    DogTask_SendVisionStatus("OK");
}

/* 通过 USART1 向视觉模块发送当前事件状态和运动状态，tag 可为 OK 或 ST。 */
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

    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)message,
                            (uint16_t)len,
                            DOG_TASK_VISION_ACK_TIMEOUT_MS);
}

/* 执行视觉事件命令：分岔转向、平台循迹增强、紫/棕投掷或停止。 */
static void DogTask_ExecuteEventCommand(ImageCommand_t command, uint32_t now_ms)
{
    if ((command == IMAGE_COMMAND_BLACK) &&
        (s_task_stage == DOG_TASK_STAGE_WAIT_BLACK))
    {
        DogTask_SendVisionAck();
        /* Follow the detected black-frame track for a short interval before
         * switching to the normal downhill forward/track trot. */
        DogTask_BeginBlackTrackDelay(now_ms);
    }
    else if ((command == IMAGE_COMMAND_GREEN) &&
             (s_task_stage == DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL))
    {
        DogTask_SendVisionAck();
        s_task_stage = DOG_TASK_STAGE_GREEN_TURN;
        /* 识别绿色后先按视觉误差循迹，再按圈数差速转向（第一圈右转、第二圈左转）。 */
        DogTask_BeginGreenTrackDelay(now_ms);
    }
    else if ((command == IMAGE_COMMAND_ORANGE) &&
             (s_task_stage == DOG_TASK_STAGE_TRACK_TO_ORANGE))
    {
        DogTask_SendVisionAck();
        DogTask_BeginOrangeTrackDelay(now_ms);
    }
    else if (command == IMAGE_COMMAND_TURN_LEFT)
    {
        DogTask_SendVisionAck();
        DogTask_BeginForkTurn(DOG_TASK_MOTION_TURN_LEFT, now_ms);
    }
    else if (command == IMAGE_COMMAND_TURN_RIGHT)
    {
        DogTask_SendVisionAck();
        DogTask_BeginForkTurn(DOG_TASK_MOTION_TURN_RIGHT, now_ms);
    }
    else if (((command == IMAGE_COMMAND_PURPLE) && (s_task_stage == DOG_TASK_STAGE_TRACK_TO_THROW) && (s_lap_count == 0U)) ||
             ((command == IMAGE_COMMAND_BROWN) && (s_task_stage == DOG_TASK_STAGE_TRACK_TO_THROW) && (s_lap_count != 0U)))
    {
        DogTask_SendVisionAck();
        s_task_stage = DOG_TASK_STAGE_THROW_TARGET;
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
        if (s_task_stage == DOG_TASK_STAGE_TRACK_TO_BLUE)
        {
            DogTask_SendVisionAck();
#if (DOG_TASK_STAIR_WALK_ENABLE != 0U)
            DogTask_BeginStairWalk(now_ms);
#else
            DogTask_BeginTrackAfterDownhill(now_ms);
#endif
        }
    }
}

/* 推进当前事件状态机，根据已经经过的时间决定是否切换到下一阶段或恢复循迹。 */
static void DogTask_UpdateEventState(uint32_t now_ms, ImageTrack_t track)
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

    if (s_event_state == DOG_TASK_EVENT_START_SHIFT_LEFT)
    {
        if (elapsed_ms >= DOG_TASK_START_SHIFT_LEFT_DURATION_MS)
        {
            DogTask_BeginSpeedBumpEntryTrack(now_ms);
        }
        else
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_LEFT);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_SPEED_BUMP_ENTRY_TRACK)
    {
        if (elapsed_ms >= DOG_TASK_SPEED_BUMP_ENTRY_DELAY_MS)
        {
            DogTask_BeginSpeedBump(now_ms);
        }
        else if (track.valid != 0U)
        {
            s_has_seen_track = 1U;
            s_last_track_ms = now_ms;
            DogTask_ApplyTrackError(track.error);
        }
        else if ((s_has_seen_track != 0U) &&
                 ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_SPEED_BUMP)
    {
#if (DOG_TASK_SPEED_BUMP_WALK_ENABLE != 0U)
        /* A speed-bump walk ends by completed gait cycles, not elapsed wall
         * time.  Line-based trot steering is deliberately disabled here
         * because the walk implementation has one shared forward step. */
        if (s_speed_bump_walk_cycle_count >=
            DOG_TASK_SPEED_BUMP_WALK_CYCLE_COUNT)
        {
            DogTask_BeginTrackToBlue(now_ms);
        }
        else
        {
            s_is_track_correcting = 0U;
        }
#else
        if (elapsed_ms >= DOG_TASK_SPEED_BUMP_EXIT_DELAY_MS)
        {
            DogTask_BeginTrackToBlue(now_ms);
        }
        else if (track.valid != 0U)
        {
            s_has_seen_track = 1U;
            s_last_track_ms = now_ms;
            DogTask_ApplyTrackError(track.error);
        }
        else if ((s_has_seen_track != 0U) &&
                 ((uint32_t)(now_ms - s_last_track_ms) < DOG_TASK_TRACK_RECOVER_MS))
        {
            /* 帧间空档保持上一帧纠偏，与普通循迹逻辑一致。 */
            s_is_track_correcting =
                (uint8_t)(s_last_track_recover_motion != DOG_TASK_MOTION_FORWARD);
        }
        else if ((s_has_seen_track != 0U) &&
                 ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
        {
            s_is_track_correcting = 0U;
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
#endif
    }
    else if (s_event_state == DOG_TASK_EVENT_BLACK_TRACK_DELAY)
    {
        if (elapsed_ms >= DOG_TASK_BLACK_TRACK_DELAY_MS)
        {
            DogTask_BeginDownhillTrack(now_ms);
        }
        else if (track.valid != 0U)
        {
            s_has_seen_track = 1U;
            s_last_track_ms = now_ms;
            DogTask_ApplyTrackError(track.error);
        }
        else if ((s_has_seen_track != 0U) &&
                 ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_GREEN_TRACK_DELAY)
    {
        if (elapsed_ms >= DOG_TASK_GREEN_TRACK_DELAY_MS)
        {
            if (s_lap_count == 0U)
            {
                /* 第一圈：循迹结束后差速右转 */
                DogTask_BeginGreenRightTurn(now_ms);
            }
            else
            {
                /* 第二圈：循迹结束后差速左转 */
                DogTask_BeginGreenLeftTurn(now_ms);
            }
        }
        else if (track.valid != 0U)
        {
            s_has_seen_track = 1U;
            s_last_track_ms = now_ms;
            DogTask_ApplyTrackError(track.error);
        }
        else if ((s_has_seen_track != 0U) &&
                 ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_SHIFT_TO_CENTER)
    {
        if (track.valid != 0U)
        {
            s_has_seen_track = 1U;
            s_last_track_ms = now_ms;

            if ((track.error >= -(int16_t)DOG_TASK_TRACK_DEADBAND) &&
                (track.error <= (int16_t)DOG_TASK_TRACK_DEADBAND))
            {
                DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);

                if (s_black_center_start_ms == 0U)
                {
                    s_black_center_start_ms = now_ms;
                }
                else if ((uint32_t)(now_ms - s_black_center_start_ms) >= DOG_TASK_BLACK_CENTER_STABLE_MS)
                {
                    DogTask_BeginDownhillTrack(now_ms);
                }
            }
            else
            {
                s_black_center_start_ms = 0U;

                if (track.error > 0)
                {
                    DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_RIGHT);
                }
                else
                {
                    DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_LEFT);
                }
            }
        }
        else if ((s_has_seen_track == 0U) ||
                 ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
        {
            s_black_center_start_ms = 0U;
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_FORK_TURN)
    {
        uint32_t turn_duration_ms = DOG_TASK_TURN_TEST_DURATION_MS;

        if (s_task_stage == DOG_TASK_STAGE_GREEN_TURN)
        {
            turn_duration_ms = (s_lap_count == 0U) ?
                               DOG_TASK_GREEN_TURN_DURATION_MS :
                               DOG_TASK_GREEN_SECOND_LAP_LEFT_TURN_DURATION_MS;
        }

        if (elapsed_ms >= turn_duration_ms)
        {
            if (s_task_stage == DOG_TASK_STAGE_GREEN_TURN)
            {
                s_task_stage = DOG_TASK_STAGE_TRACK_TO_THROW;
            }

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
                DogGait_AllStand(DogTask_GetGaitMoveMs());
                s_motion = DOG_TASK_MOTION_STOP;
            }
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_PLATFORM_PAUSE)
    {
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);

        if (elapsed_ms >= DOG_TASK_PLATFORM_PAUSE_TEST_MS)
        {
            DogTask_SendK230Yes();
            s_event_state = DOG_TASK_EVENT_PLATFORM_YES_SEND;
            s_event_start_ms = now_ms;
            s_platform_yes_last_ms = now_ms;
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_PLATFORM_YES_SEND)
    {
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);

        if (elapsed_ms >= DOG_TASK_PLATFORM_YES_SEND_MS)
        {
            DogTask_BeginPlatformTrackBoost();
            DogTask_ResumeTracking(now_ms);
        }
        else if ((uint32_t)(now_ms - s_platform_yes_last_ms) >= DOG_TASK_PLATFORM_YES_INTERVAL_MS)
        {
            s_platform_yes_last_ms = now_ms;
            DogTask_SendK230Yes();
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_STAIR_WALK)
    {
        StairWalk_Update();

        if (StairWalk_IsFinished() != 0U)
        {
            DogTask_SendK230Yes();
            s_task_stage = DOG_TASK_STAGE_WAIT_BLACK;
            DogTask_BeginPlatformTrackBoost();
            DogTask_ResumeTracking(now_ms);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_ORANGE_TRACK_DELAY)
    {
        if (elapsed_ms >= DOG_TASK_ORANGE_TRACK_DELAY_MS)
        {
            DogTask_BeginShiftRight(now_ms);
        }
        else if (track.valid != 0U)
        {
            s_has_seen_track = 1U;
            s_last_track_ms = now_ms;
            DogTask_ApplyTrackError(track.error);
        }
        else if ((s_has_seen_track != 0U) &&
                 ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_SHIFT_RIGHT)
    {
        if (elapsed_ms >= DOG_TASK_SHIFT_RIGHT_MS)
        {
            DogTask_BeginLapPause(now_ms);
        }
        else
        {
            DogTask_ApplyMotion(DOG_TASK_MOTION_SHIFT_RIGHT);
        }
    }
    else if (s_event_state == DOG_TASK_EVENT_LAP_PAUSE)
    {
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);

        if (elapsed_ms >= DOG_TASK_LAP_PAUSE_MS)
        {
            if (s_lap_count == 0U)
            {
                s_lap_count++;
                s_purple_throw_delay_used = 0U;
                s_brown_throw_delay_used = 0U;
                DogTask_BeginStartShiftLeft(now_ms);
            }
            else
            {
                s_task_stage = DOG_TASK_STAGE_FINISHED;
                s_event_state = DOG_TASK_EVENT_IDLE;
                DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
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
            s_task_stage = DOG_TASK_STAGE_TRACK_TO_ORANGE;
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

    if (s_task_stage == DOG_TASK_STAGE_SPEED_BUMP)
    {
        track_step_h = DOG_TASK_SPEED_BUMP_TRACK_STEP_H_MM;
        track_left_forward = DOG_TASK_SPEED_BUMP_TRACK_LEFT_FORWARD_R_MM;
        track_right_forward = DOG_TASK_SPEED_BUMP_TRACK_RIGHT_FORWARD_R_MM;
    }
    else if (s_platform_track_boost != 0U)
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
        DogGait_SetTrackParamsWithFootBase(track_step_h,
                                           track_left_forward,
                                           track_right_forward,
                                           steer,
                                           DogTask_TrackRollFromSteer(steer),
                                           DOG_TASK_SPEED_FREQ,
                                           DogTask_GetTrackFootBase());
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
        DogGait_SetTrackParamsWithFootBase(track_step_h,
                                           track_left_forward,
                                           track_right_forward,
                                           -steer,
                                           DogTask_TrackRollFromSteer(-steer),
                                           DOG_TASK_SPEED_FREQ,
                                           DogTask_GetTrackFootBase());
        s_motion = DOG_TASK_MOTION_TURN_LEFT;
    }
    else
    {
        s_is_track_correcting = 0U;
        s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
        DogGait_SetTrackParamsWithFootBase(track_step_h,
                                           track_left_forward,
                                           track_right_forward,
                                           0.0f,
                                           0.0f,
                                           DOG_TASK_SPEED_FREQ,
                                           DogTask_GetTrackFootBase());
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

        if ((uint32_t)(now_ms - last_gait_ms) >= DOG_TASK_GAIT_NORMAL_PERIOD_MS)
        {
            last_gait_ms = now_ms;
            DogGait_UpdateTrot(DOG_TASK_GAIT_NORMAL_MOVE_MS);
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
     Jy61PImu_Init();
     StairWalk_Init();
    DogTask_SetCorrectionLed(0U);

    s_last_gait_ms = HAL_GetTick();
    s_last_track_ms = s_last_gait_ms;
    s_last_status_ms = s_last_gait_ms;
    s_has_seen_track = 0U;
    s_is_track_correcting = 0U;
    s_platform_track_boost = 0U;
    s_wait_platform_imu = 0U;
    s_purple_throw_delay_used = 0U;
    s_brown_throw_delay_used = 0U;
    s_lap_count = 0U;
    s_black_center_start_ms = 0U;
    s_level_start_ms = 0U;
    s_last_track_recover_motion = DOG_TASK_MOTION_FORWARD;
    s_platform_yes_last_ms = 0U;
    s_event_state = DOG_TASK_EVENT_IDLE;
    s_task_stage = DOG_TASK_STAGE_START_SHIFT_LEFT;
    s_event_start_ms = s_last_gait_ms;
    s_pending_event_command = IMAGE_COMMAND_NONE;
#if (DOG_TASK_FORK_TEST_ENABLE != 0U)
    s_lap_count = DOG_TASK_FORK_TEST_LAP_COUNT;
    DogTask_BeginTrackAfterDownhill(s_last_gait_ms);
#else
    DogTask_BeginStartShiftLeft(s_last_gait_ms);
#endif
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

/* 上电测试：完成回中和站立后，按主流程的减速带方案行走（walk 优先）。 */
void DogTask_SpeedBumpTest_Init(void)
{
    uint32_t now_ms;

    ThrowServo_Init();
    HAL_Delay(DOG_TASK_SERVO_READY_MS);

    DogServo_AllCenter(DOG_TASK_CENTER_MOVE_MS);
    HAL_Delay(DOG_TASK_CENTER_WAIT_MS);

    DogGait_SetLoadMode((DOG_TASK_USE_PAYLOAD_GAIT != 0U) ?
                            DOG_GAIT_LOAD_WITH_PAYLOAD :
                            DOG_GAIT_LOAD_NONE);
    DogGait_Init();
    DogGait_GotoStandPose(DOG_TASK_STAND_MOVE_MS);
    HAL_Delay(DOG_TASK_STAND_WAIT_MS);

    ImageCommand_Init();
    Jy61PImu_Init();
    StairWalk_Init();
    DogTask_SetCorrectionLed(0U);
    now_ms = HAL_GetTick();
    s_speed_bump_test_start_ms = now_ms;
    s_speed_bump_test_active = 1U;
    s_platform_track_boost = 0U;
    s_last_gait_ms = now_ms;

    /* 与主流程进入减速带阶段共用同一套初始化。 */
    DogTask_BeginSpeedBump(now_ms);
}

/* 非阻塞过减速带测试：与主流程一致使用 walk，按周期数结束并保留超时兜底。 */
void DogTask_SpeedBumpTest_Run(void)
{
    uint32_t now_ms;

    if (s_speed_bump_test_active == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    Jy61PImu_Update(now_ms);

    if ((uint32_t)(now_ms - s_speed_bump_test_start_ms) >=
        DOG_TASK_SPEED_BUMP_TEST_DURATION_MS)
    {
        s_speed_bump_test_active = 0U;
        DogGait_AllStand(DOG_TASK_GAIT_SPEED_BUMP_MOVE_MS);
        return;
    }

#if (DOG_TASK_SPEED_BUMP_WALK_ENABLE != 0U)
    if (s_speed_bump_walk_cycle_count >= DOG_TASK_SPEED_BUMP_WALK_CYCLE_COUNT)
    {
        s_speed_bump_test_active = 0U;
        DogGait_AllStand(DOG_TASK_SPEED_BUMP_WALK_MOVE_MS);
        return;
    }

    DogTask_UpdateSpeedBumpWalk(now_ms);
#else
    ImageTrack_t track;

    /* 使用视觉误差设置左右腿差速；SPEED_BUMP 阶段会保留减速带专用基准。 */
    track = ImageCommand_TakeLatestTrack();
    if (track.valid != 0U)
    {
        DogTask_ApplyTrackError(track.error);
    }

    if ((uint32_t)(now_ms - s_last_gait_ms) >=
        DOG_TASK_GAIT_SPEED_BUMP_PERIOD_MS)
    {
        s_last_gait_ms = now_ms;
        DogGait_UpdateTrot(DOG_TASK_GAIT_SPEED_BUMP_MOVE_MS);
    }
#endif
}

/* 减速带前循迹阶段的独立测试入口：完成回中和站立后，固定进入 SPEED_BUMP_ENTRY_TRACK 阶段。 */
void DogTask_SpeedBumpEntryTest_Init(void)
{
    uint32_t now_ms;

    ThrowServo_Init();
    HAL_Delay(DOG_TASK_SERVO_READY_MS);

    DogServo_AllCenter(DOG_TASK_CENTER_MOVE_MS);
    HAL_Delay(DOG_TASK_CENTER_WAIT_MS);

    DogGait_SetLoadMode((DOG_TASK_USE_PAYLOAD_GAIT != 0U) ?
                            DOG_GAIT_LOAD_WITH_PAYLOAD :
                            DOG_GAIT_LOAD_NONE);
    DogGait_Init();
    DogGait_GotoStandPose(DOG_TASK_STAND_MOVE_MS);
    HAL_Delay(DOG_TASK_STAND_WAIT_MS);

    ImageCommand_Init();
    DogTask_SetCorrectionLed(0U);
    now_ms = HAL_GetTick();
    s_speed_bump_entry_test_start_ms = now_ms;
    s_speed_bump_entry_test_last_gait_ms = now_ms;
    s_speed_bump_entry_test_active = 1U;

    /* 只启用减速带前循迹阶段，不接后续状态机。 */
    DogTask_BeginSpeedBumpEntryTrack(now_ms);
}

/* 非阻塞减速带前循迹测试：只推进 SPEED_BUMP_ENTRY_TRACK 的循迹逻辑，满 15 秒后回到站立姿态。 */
void DogTask_SpeedBumpEntryTest_Run(void)
{
    uint32_t now_ms;
    ImageTrack_t track;

    if (s_speed_bump_entry_test_active == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_speed_bump_entry_test_start_ms) >=
        DOG_TASK_SPEED_BUMP_ENTRY_TEST_DURATION_MS)
    {
        s_speed_bump_entry_test_active = 0U;
        DogGait_AllStand(DogTask_GetGaitMoveMs());
        return;
    }

    /* 与事件状态机中 SPEED_BUMP_ENTRY_TRACK 相同的循迹/停线逻辑，但不做超时跳转。 */
    track = ImageCommand_TakeLatestTrack();
    if (track.valid != 0U)
    {
        s_has_seen_track = 1U;
        s_last_track_ms = now_ms;
        DogTask_ApplyTrackError(track.error);
    }
    else if ((s_has_seen_track != 0U) &&
             ((uint32_t)(now_ms - s_last_track_ms) >= DOG_TASK_TRACK_RECOVER_MS))
    {
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    }

    if ((s_motion != DOG_TASK_MOTION_STOP) &&
        ((uint32_t)(now_ms - s_speed_bump_entry_test_last_gait_ms) >=
         DogTask_GetGaitPeriodMs()))
    {
        s_speed_bump_entry_test_last_gait_ms = now_ms;
        DogGait_UpdateTrot(DogTask_GetGaitMoveMs());
    }
}

/* 上楼梯测试入口：完成正常回中、站立和视觉/IMU 初始化后，直接开始
 * 循迹等待蓝色平台事件。蓝色事件和上楼完成后的后续处理均复用主状态机。 */
void DogTask_StairWalkTest_Init(void)
{
    DogTask_Init();

    /* 跳过启动平移、减速带等前置任务，直接进入“循迹到蓝色”阶段。 */
    DogTask_BeginTrackToBlue(HAL_GetTick());
}

/* 上楼梯测试运行入口：保留原有蓝色触发、上楼完成和后续循迹逻辑。 */
void DogTask_StairWalkTest_Run(void)
{
    DogTask_Run();
}

/* 颜色反应测试入口：复用主任务初始化，直接进入绿色分岔前的循迹阶段。
 * 不注入命令、不屏蔽串口，由真实 K230 串口驱动后续颜色事件。 */
void DogTask_ColorReactionTest_Init(void)
{
    uint32_t now_ms;

    DogTask_Init();
    now_ms = HAL_GetTick();
    DogTask_BeginTrackAfterDownhill(now_ms);
    s_lap_count = 0U;
    s_purple_throw_delay_used = 0U;
    s_brown_throw_delay_used = 0U;
    s_color_reaction_test_lap2_ready = 1U;
}

/* 颜色反应测试运行入口：完全走正常主任务状态机，读取真实 K230 串口。
 * 第二圈开始时跳回绿色分岔前，方便继续联调绿色/棕色/橙色。 */
void DogTask_ColorReactionTest_Run(void)
{
    DogTask_Run();

    if ((s_color_reaction_test_lap2_ready != 0U) &&
        (s_lap_count == 1U) &&
        (s_event_state == DOG_TASK_EVENT_START_SHIFT_LEFT))
    {
        s_color_reaction_test_lap2_ready = 0U;
        DogTask_BeginTrackAfterDownhill(HAL_GetTick());
    }
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
    
    #if 0
    DogGait_SetShiftLeftParams(DOG_TASK_SHIFT_H_MM, DOG_TASK_SHIFT_R_MML, DOG_TASK_SPEED_FREQ,DOG_GAIT_FOOT_BASE_SHIFT_LEFT);
    //DogGait_SetShiftRightParams(DOG_TASK_SHIFT_H_MM, DOG_TASK_SHIFT_R_MMR, DOG_TASK_SPEED_FREQ,DOG_GAIT_FOOT_BASE_SHIFT_RIGHT);
    DogGait_UpdateShift(90U,DOG_GAIT_FOOT_BASE_SHIFT_RIGHT);
    HAL_Delay(90U);
    #endif
    #if 1

    Jy61PImu_Update(now_ms);
    ThrowServo_Update();

#if !DOG_TASK_PLATFORM_PAUSE_TEST_ENABLE
    if ((s_wait_platform_imu != 0U) &&
        (DogTask_IsPlatformFinishedByImu() != 0U))
    {
        DogTask_SendK230Yes();
        s_wait_platform_imu = 0U;
    }
#endif

    if (s_task_stage == DOG_TASK_STAGE_FINISHED)
    {
        DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
    }
    else if (s_event_state == DOG_TASK_EVENT_THROW_TRACK_DELAY)
    {
        DogTask_UpdateEventState(now_ms, track);

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
        DogTask_UpdateEventState(now_ms, track);
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

    if ((s_task_stage == DOG_TASK_STAGE_DOWNHILL_TRACK) &&
        ((uint32_t)(now_ms - s_event_start_ms) >= DOG_TASK_DOWNHILL_MIN_MS) &&
        (DogTask_IsBodyLevelStable(now_ms) != 0U))
    {
        s_task_stage = DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL;
        s_level_start_ms = 0U;
    }

    if (s_event_state == DOG_TASK_EVENT_COLOR_PAUSE)
    {
        DogTask_SetCorrectionLed(1U);
    }
    else
    {
        DogTask_SetCorrectionLed(0U);
    }

    if (DogTask_UpdateSpeedBumpWalk(now_ms) != 0U)
    {
        /* 减速带 walk 本帧已推进。 */
    }
    else if ((s_motion != DOG_TASK_MOTION_STOP) &&
        ((uint32_t)(now_ms - s_last_gait_ms) >= DogTask_GetGaitPeriodMs()))
    {
        s_last_gait_ms = now_ms;
        g_dog_task_gait_update_count++;

        // 根据运动模式选择步态更新函数
        if (s_motion == DOG_TASK_MOTION_SHIFT_LEFT)
        {
            DogGait_UpdateShift(DogTask_GetGaitMoveMs(), DOG_GAIT_FOOT_BASE_SHIFT_LEFT);
        }
        else if (s_motion == DOG_TASK_MOTION_SHIFT_RIGHT)
        {
            DogGait_UpdateShift(DogTask_GetGaitMoveMs(), DOG_GAIT_FOOT_BASE_SHIFT_RIGHT);
        }
        else
        {
            DogGait_UpdateTrot(DogTask_GetGaitMoveMs());
        }
    }

    if ((uint32_t)(now_ms - s_last_status_ms) >= DOG_TASK_STATUS_INTERVAL_MS)
    {
        s_last_status_ms = now_ms;
        DogTask_SendVisionStatus("ST");
    }
        #endif
}
