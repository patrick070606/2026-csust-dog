# DogRobot 工程说明

本工程是基于 STM32F103C8T6 的四足机器狗控制程序。当前主线已经从早期单项测试整理为比赛主流程版本，`DogRobot` 目标已编译通过。

当前核心功能：

- 通过 USART1 控制 8 个 HTS-35H 总线舵机。
- 通过 USART2 接收 K230 视觉模块发送的循迹误差和颜色事件。
- 通过 USART3 接收 JY61P IMU 姿态数据。
- 根据视觉误差进行差速小跑循迹。
- 支持左右平移、左右转向、投球舵机控制。
- 已接入正式 `stair_walk` 上楼梯模块。
- 已接通比赛主流程：左平移、循迹到蓝色、上楼梯、黑框居中、下坡、绿色转弯、投球、橙色、右平移、圈数切换。

注意：当前代码已经编译通过，但完整比赛流程尚未实机全流程验证。上楼梯和平移步态此前单独测过，但更换机器狗后参数仍需要重新校准。

## 目录结构

```text
Core/
  Inc/                     STM32 HAL 头文件和外设声明
  Src/
    main.c                 主入口，只负责 HAL/外设初始化和 DogTask 调度
    usart.c                USART1/USART2/USART3 配置
    tim.c                  TIM1 PWM 配置
    gpio.c                 PC13 LED 配置

User/
  bus_servo.c/.h           总线舵机控制器协议发送
  dog_servo_config.c/.h    逻辑关节到真实舵机 ID、方向、偏置、限位的配置
  dog_servo.c/.h           角度到总线舵机 position 的转换与 8 舵机同步输出
  dog_gait.c/.h            站立、小跑、原地踏步、侧移、转向、视觉循迹、WALK 步态
  stair_walk.c/.h          正式上楼梯 WALK 模块
  image_command.c/.h       USART2 视觉数据接收和协议解析
  jy61p_imu.c/.h           USART3 JY61P IMU 数据帧解析和调试镜像变量
  dog_task.c/.h            机器狗主任务、比赛流程和事件状态机
  throw_servo.c/.h         PB13 PWM 投掷舵机控制

vision/
  main.py                  当前推荐 K230 视觉侧脚本，文本颜色协议
  main(12)(1).py           较旧 K230 视觉侧脚本

information/
  README.md                当前工程说明
  README_DogRobot_BusServo_Status.md
                           历史调试记录

tools/
  dog_gait_angle_tool.html 步态角度/几何辅助工具
```

## 硬件连接

### 主控

```text
MCU: STM32F103C8T6
Clock: HSE + PLL, SYSCLK 72 MHz
```

### 总线舵机控制器

```text
USART1_TX  PA9   -> 总线舵机控制器 RX
USART1_RX  PA10  <- 总线舵机控制器 TX，可选
GND              -> 总线舵机控制器 GND

Baud: 9600
Format: 8N1
```

总线舵机发送端固定使用：

```c
#define BUS_SERVO_UART huart1
```

### K230 视觉模块

```text
STM32 USART2_TX  PA2  -> K230 RX
STM32 USART2_RX  PA3  <- K230 TX
STM32 GND             -> K230 GND

Baud: 115200
Format: 8N1
Interrupt: USART2_IRQn
```

如果 K230 使用当前脚本中的 UART3 引脚映射，常见连接是：

```text
K230 GPIO32 / UART3_TXD -> STM32 PA3 / USART2_RX
K230 GPIO33 / UART3_RXD <- STM32 PA2 / USART2_TX
GND                     -> GND
```

### 投掷 PWM 舵机

```text
PB13 -> TIM1_CH1N
PWM  -> 20 ms 周期，us 计数分辨率
```

PB13 是 TIM1 的互补输出 `CH1N`，启动 PWM 时使用：

```c
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1)
```

### JY61P IMU

```text
STM32 USART3_TX  PB10 -> JY61P RX，可选
STM32 USART3_RX  PB11 <- JY61P TX
STM32 GND             -> JY61P GND

Baud: 9600
Format: 8N1
Interrupt: USART3_IRQn
```

调试器可直接观察：

```c
g_jy61p_imu_roll_deg
g_jy61p_imu_pitch_deg
g_jy61p_imu_yaw_deg
g_jy61p_imu_gyro_z_dps
g_jy61p_imu_is_valid
g_jy61p_imu_frame_count
g_jy61p_imu_checksum_error_count
g_jy61p_imu_uart_error_count
g_jy61p_imu_rx_restart_count
```

## 主程序流程

入口在 `Core/Src/main.c`：

```c
MX_GPIO_Init();
MX_TIM1_Init();
MX_USART1_UART_Init();
MX_USART2_UART_Init();
MX_USART3_UART_Init();
DogTask_Init();

while (1)
{
    DogTask_Run();
}
```

`DogTask_Init()` 当前启动流程：

```text
初始化投掷 PWM 舵机
-> 等待总线舵机控制器稳定
-> 8 个总线舵机回中
-> 设置负载步态模式
-> 初始化步态并进入站立姿态
-> 启动 USART2 视觉接收
-> 启动 USART3 JY61P IMU 接收
-> 初始化 StairWalk 模块
-> 进入启动左平移阶段
```

`DogTask_Run()` 是非阻塞主循环任务，持续处理：

- K230 最新视觉命令和循迹误差。
- JY61P IMU 数据有效性更新。
- 投掷舵机状态更新。
- 上楼梯、黑框居中、下坡、投球、橙色延迟、圈数等事件状态机。
- 每 `150 ms` 更新一次步态。
- 每 `200 ms` 向 K230 回传一次状态字符串。

## 当前比赛主流程

当前 `DogTaskStage_t` 表示比赛顶层阶段：

```text
DOG_TASK_STAGE_START_SHIFT_LEFT
DOG_TASK_STAGE_TRACK_TO_BLUE
DOG_TASK_STAGE_STAIR_WALK
DOG_TASK_STAGE_WAIT_BLACK
DOG_TASK_STAGE_SHIFT_TO_CENTER
DOG_TASK_STAGE_DOWNHILL_TRACK
DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL
DOG_TASK_STAGE_GREEN_TURN
DOG_TASK_STAGE_TRACK_TO_THROW
DOG_TASK_STAGE_THROW_TARGET
DOG_TASK_STAGE_TRACK_TO_ORANGE
DOG_TASK_STAGE_ORANGE_TRACK_DELAY
DOG_TASK_STAGE_SHIFT_RIGHT
DOG_TASK_STAGE_LAP_PAUSE
DOG_TASK_STAGE_FINISHED
```

完整流程：

```text
启动、回中、站立
-> 左平移 2s
-> 循迹到蓝色
-> 蓝色触发上楼梯
-> 上楼梯完成后发送 YES
-> K230 进入黑框识别
-> 黑色触发左右平移居中
-> 居中后进入下坡循迹
-> IMU 判断下坡结束
-> 绿色触发转弯
-> 第 1 圈右转，第 2 圈左转
-> 第 1 圈紫色投球，第 2 圈棕色投球
-> 投球后等待橙色
-> 橙色后继续循迹 5s
-> 右平移 2s
-> 停止 5s
-> 第 1 圈重新开始，第 2 圈保持停止
```

## 运动状态、事件状态和流程状态

`DogTaskMotion_t` 表示底层基础运动：

```text
STOP
FORWARD
BACKWARD
TURN_LEFT
TURN_RIGHT
SHIFT_LEFT
SHIFT_RIGHT
```

`DogTaskEventState_t` 表示当前正在执行的事件：

```text
IDLE
COLOR_PAUSE
FORK_TURN
THROW_TRACK_DELAY
THROW_FORWARD
THROW_ROTATING
PLATFORM_PAUSE
PLATFORM_YES_SEND
STAIR_WALK
START_SHIFT_LEFT
SHIFT_TO_CENTER
ORANGE_TRACK_DELAY
SHIFT_RIGHT
LAP_PAUSE
```

`DogTaskStage_t` 表示比赛进度。三者关系：

```text
DogTaskStage_t      顶层比赛流程
DogTaskEventState_t 当前正在执行的事件
DogTaskMotion_t     底层具体运动方式
```

上楼梯不放在 `DogTaskMotion_t` 中，因为它不是普通小跑方向，而是独立 `stair_walk` 模块控制。

## K230 到 STM32 协议

当前主协议：

```text
E:<误差数字>,C:<颜色>\n
```

颜色字段当前支持：

```text
C:none
C:blue
C:purple
C:brown
C:green
C:black
C:orange
```

映射关系：

```text
C:blue    -> IMAGE_COMMAND_PLATFORM
C:purple  -> IMAGE_COMMAND_PURPLE
C:brown   -> IMAGE_COMMAND_BROWN
C:green   -> IMAGE_COMMAND_GREEN
C:black   -> IMAGE_COMMAND_BLACK
C:orange  -> IMAGE_COMMAND_ORANGE
C:none    -> IMAGE_COMMAND_NONE
```

普通循迹时，`E` 字段是 K230 计算出的横向误差：

```text
error > 0      目标线在画面右侧，STM32 右纠偏
error < 0      目标线在画面左侧，STM32 左纠偏
|error| <= 35  视为居中直行
```

`C:black` 比较特殊：STM32 会把它作为事件命令，同时保留同一帧的 `E:error`，供黑框平移居中使用。

STM32 仍兼容旧的数字事件值，来自 `E` 字段：

```text
 1000  -> 右转分岔       IMAGE_COMMAND_TURN_RIGHT
-1000  -> 左转分岔       IMAGE_COMMAND_TURN_LEFT
 2000  -> 蓝色平台       IMAGE_COMMAND_PLATFORM
 3000  -> 紫色住户       IMAGE_COMMAND_PURPLE
 4000  -> 棕色住户       IMAGE_COMMAND_BROWN
 9999  -> 丢线/停止      IMAGE_COMMAND_STOP
```

## STM32 到 K230 协议

### 状态回传

STM32 会通过 USART2 回传：

```text
OK E:<event_state> M:<motion>\n
ST E:<event_state> M:<motion>\n
```

- `OK` 是收到事件后的确认。
- `ST` 是周期性状态反馈，每 `200 ms` 发送一次。
- `event_state` 来自 `DogTask_EventName()`。
- `motion` 来自 `DogTask_MotionName()`。

### YES 控制命令

上楼梯完成后，STM32 会发送：

```text
YES\n
```

K230 收到 `YES` 后应进入黑框识别阶段。

## 上楼梯模块

正式上楼梯模块位于：

```text
User/stair_walk.c
User/stair_walk.h
```

主流程收到 `C:blue` 后：

```text
DOG_TASK_STAGE_TRACK_TO_BLUE
-> DOG_TASK_STAGE_STAIR_WALK
-> DOG_TASK_EVENT_STAIR_WALK
-> StairWalk_Start()
-> 周期调用 StairWalk_Update()
-> StairWalk_IsFinished() 判断结束
-> 发送 YES
-> DOG_TASK_STAGE_WAIT_BLACK
```

当前上楼梯结束判定：

```text
至少执行一定 WALK 轮数
-> IMU pitch/roll 有效
-> pitch/roll 进入水平阈值
-> 稳定保持一段时间
-> 结束
```

当前尚未迁移灯哥项目中 `cal_ges()` 的剩余滤波融合部分。

## 黑框平移居中

上楼梯结束并发送 `YES` 后，K230 应进入黑框识别阶段。

STM32 收到 `C:black` 且当前处于 `DOG_TASK_STAGE_WAIT_BLACK` 时：

```text
进入 DOG_TASK_STAGE_SHIFT_TO_CENTER
进入 DOG_TASK_EVENT_SHIFT_TO_CENTER
根据 E:error 左右平移
误差进入死区并稳定 DOG_TASK_BLACK_CENTER_STABLE_MS
进入下坡循迹
```

当前方向假设：

```text
error > 0 -> 右平移
error < 0 -> 左平移
```

如果实机发现越调越偏，需要对调左右平移方向。

## 下坡与后半段流程

当前下坡阶段使用普通循迹，结束条件为 IMU 水平稳定：

```text
DOG_TASK_STAGE_DOWNHILL_TRACK
-> 至少运行 DOG_TASK_DOWNHILL_MIN_MS
-> pitch/roll 进入阈值
-> 稳定 DOG_TASK_LEVEL_STABLE_MS
-> DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL
```

下坡 IMU 判定尚未单独实机验证。如果该判定不准，绿色、投球、橙色和圈数逻辑会触发不到。

后半段逻辑：

```text
TRACK_AFTER_DOWNHILL 收到绿色
-> 第 1 圈右转，第 2 圈左转
-> TRACK_TO_THROW
-> 第 1 圈响应紫色，第 2 圈响应棕色
-> 投球
-> TRACK_TO_ORANGE
-> 收到橙色后继续循迹 5s
-> 右平移 2s
-> 停止 5s
-> 第 1 圈重新开始，第 2 圈 FINISHED
```

## 关键任务参数

```c
#define DOG_TASK_GAIT_PERIOD_MS        150U
#define DOG_TASK_GAIT_MOVE_MS          100U
#define DOG_TASK_STEP_H_MM             45.0f
#define DOG_TASK_FORWARD_R_MM          50.0f
#define DOG_TASK_TURN_R_MM             15.0f
#define DOG_TASK_SPEED_FREQ            0.20f

#define DOG_TASK_TRACK_DEADBAND        35U
#define DOG_TASK_TRACK_RECOVER_MS      500U
#define DOG_TASK_TRACK_STEP_H_MM       45.0f
#define DOG_TASK_TRACK_LEFT_FORWARD_R_MM   60.0f
#define DOG_TASK_TRACK_RIGHT_FORWARD_R_MM  45.0f
#define DOG_TASK_TRACK_MAX_STEER_MM    18.0f
#define DOG_TASK_TRACK_STEER_GAIN      0.18f

#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          30.0f
#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  60.0f
#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 45.0f

#define DOG_TASK_START_SHIFT_LEFT_MS       2000U
#define DOG_TASK_BLACK_CENTER_STABLE_MS    500U
#define DOG_TASK_DOWNHILL_MIN_MS           1500U
#define DOG_TASK_LEVEL_PITCH_DEG           5.0f
#define DOG_TASK_LEVEL_ROLL_DEG            6.0f
#define DOG_TASK_LEVEL_STABLE_MS           800U
#define DOG_TASK_ORANGE_TRACK_DELAY_MS     5000U
#define DOG_TASK_SHIFT_RIGHT_MS            2000U
#define DOG_TASK_LAP_PAUSE_MS              5000U
```

## 构建

Debug 构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug --target DogRobot
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release --target DogRobot
```

当前主目标：

```text
DogRobot
```

早期 `DogRobotStairWalkTest` 和 `DogRobotTrackBlueBoostTest` 测试目标已经清理，当前不要再使用旧测试任务作为主线依据。

## 调试建议

### 判断 STM32 是否收到 K230 数据

在 `User/image_command.c` 的 `ImageCommand_FinishFrame()` 打断点，观察：

```c
s_frame
s_frame_len
s_latest_command
s_latest_track
```

### 判断蓝色是否进入上楼梯

在 `User/dog_task.c` 中观察：

```c
s_task_stage
s_event_state
```

收到 `C:blue` 后应进入：

```text
DOG_TASK_STAGE_STAIR_WALK
DOG_TASK_EVENT_STAIR_WALK
```

### 判断 STM32 是否发送 YES

在 `DogTask_SendK230Yes()` 打断点。执行后如果：

```c
g_k230_tx_status == HAL_OK
```

说明 HAL 认为 USART2 发送完成。若 K230 仍收不到，用 USB-TTL 或逻辑分析仪检查 PA2。

### 判断黑框居中是否有效

观察：

```c
s_task_stage
s_event_state
g_dog_task_last_track_valid
g_dog_task_last_track_error
```

进入黑框居中后应处于：

```text
DOG_TASK_STAGE_SHIFT_TO_CENTER
DOG_TASK_EVENT_SHIFT_TO_CENTER
```

如果偏差越调越大，优先对调黑框居中的左右平移方向。

### 判断下坡 IMU 是否有效

观察：

```c
g_jy61p_imu_is_valid
g_jy61p_imu_pitch_deg
g_jy61p_imu_roll_deg
s_task_stage
```

下坡结束后应从：

```text
DOG_TASK_STAGE_DOWNHILL_TRACK
```

切换到：

```text
DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL
```

## 当前注意事项

- 当前完整流程已编译通过，但未完成实机全流程验证。
- 下坡 IMU 判定尚未单独测试，可能导致后半段绿色/投球/橙色/圈数流程触发不到。
- 黑框平移方向需要实机确认。
- `C:black` 居中依赖 K230 持续发送黑色和误差。
- 更换机器狗后，需要重新确认舵机偏置、步态高度、步长、速度、IMU 阈值和投球舵机方向。
- 当前仍未迁移灯哥项目中 `cal_ges()` 的剩余滤波融合。
- 旧文档和旧测试脚本可能与当前主线不一致，以 `DogRobot` 主目标和当前 `README.md` 为准。
