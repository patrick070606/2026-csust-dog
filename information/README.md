# DogRobot 工程说明

本工程是基于 STM32F103C8T6 的四足机器狗控制程序。当前主线功能：

- 通过 USART1 控制 8 个 HTS-35H 总线舵机。
- 通过 USART2 接收 K230 视觉模块发送的循迹误差和颜色事件。
- 根据视觉误差进行差速小跑循迹。
- 根据绿色分岔、蓝色平台、紫色/棕色住户事件执行对应任务。
- 通过 TIM1_CH1N / PB13 控制投掷 PWM 舵机。

旧的 `README_DogRobot_BusServo_Status.md` 主要是历史调试记录，不一定代表当前代码状态。本文档以当前源码为准。

## 目录结构

```text
Core/
  Inc/                     STM32 HAL 头文件和外设声明
  Src/
    main.c                 主入口，只负责 HAL/外设初始化和 DogTask 调度
    usart.c                USART1/USART2 配置
    tim.c                  TIM1 PWM 配置
    gpio.c                 PC13 LED 配置

User/
  bus_servo.c/.h           总线舵机控制器协议发送
  dog_servo_config.c/.h    逻辑关节到真实舵机 ID、方向、偏置、限位的配置
  dog_servo.c/.h           角度到总线舵机 position 的转换与 8 舵机同步输出
  dog_gait.c/.h            站立、小跑、转向、视觉循迹步态
  image_command.c/.h       USART2 视觉数据接收和协议解析
  dog_task.c/.h            机器狗主任务和事件状态机
  throw_servo.c/.h         PB13 PWM 投掷舵机控制

vision/
  main.py
  main(12)(1).py           K230 视觉侧脚本，注意不同版本协议可能不同

tests/
  check_*.py               面向源码片段的轻量一致性检查脚本

information/
  README.md                当前工程说明
  README_DogRobot_BusServo_Status.md
                           历史调试记录
```

## 硬件连接

### 主控

```text
MCU: STM32F103C8T6
Clock: HSE + PLL, SYSCLK 72 MHz
```

### 总线舵机控制器

源码位置：`Core/Src/usart.c`、`User/bus_servo.c`

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

源码位置：`Core/Src/usart.c`、`User/image_command.c`、`User/dog_task.c`

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

源码位置：`Core/Src/tim.c`、`User/throw_servo.c`

```text
PB13 -> TIM1_CH1N
PWM  -> 20 ms 周期，us 计数分辨率
```

PB13 是 TIM1 的互补输出 `CH1N`，启动 PWM 时使用：

```c
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1)
```

### IMU

当前工程没有真实 IMU 驱动或硬件接口代码：

- 没有 `imu.c/.h`、`mpu6050`、`icm`、`bmi` 等驱动。
- `HAL_I2C_MODULE_ENABLED` 当前是注释状态。
- `HAL_SPI_MODULE_ENABLED` 当前也是注释状态。

`User/dog_task.c` 中的 `DogTask_IsPlatformFinishedByImu()` 只是占位函数，并不读取真实 IMU。它在 `DOG_TASK_PLATFORM_PAUSE_TEST_ENABLE == 0U` 时才参与编译，当前默认测试宏为 `1U`，所以现在走的是蓝色平台 5s 暂停测试逻辑。

## 主程序流程

入口在 `Core/Src/main.c`：

```c
MX_GPIO_Init();
MX_TIM1_Init();
MX_USART1_UART_Init();
MX_USART2_UART_Init();
DogTask_Init();

while (1)
{
    DogTask_Run();
}
```

`DogTask_Init()` 当前启动流程：

```text
1. 初始化投掷 PWM 舵机，并输出停止脉宽。
2. 等待总线舵机控制器稳定。
3. 8 个总线舵机回中。
4. 设置负载步态模式。
5. 初始化步态并进入站立姿态。
6. 启动 USART2 视觉接收。
7. 默认进入前进/循迹状态。
```

`DogTask_Run()` 是非阻塞主循环任务，持续处理：

- 投掷舵机状态更新。
- USART2 最新视觉命令和循迹误差。
- 分岔转向、投掷、蓝色平台暂停/YES 发送等事件状态机。
- 每 `150 ms` 更新一次步态。
- 每 `200 ms` 向 K230 回传一次状态字符串。

## K230 到 STM32 协议

当前 STM32 端主协议是：

```text
E:<误差数字>,C:<颜色>\n
```

颜色字段当前支持：

```text
C:green   绿色，不触发 STM32 事件
C:blue    蓝色平台，映射为 IMAGE_COMMAND_PLATFORM
C:purple  紫色住户，映射为 IMAGE_COMMAND_PURPLE
C:brown   棕色住户，映射为 IMAGE_COMMAND_BROWN
C:none    未识别到颜色，不触发 STM32 事件
```

普通循迹时，`E` 字段是 K230 计算出的横向误差：

```text
error > 0      目标线在画面右侧，STM32 右纠偏
error < 0      目标线在画面左侧，STM32 左纠偏
|error| <= 35  视为居中直行
```

颜色事件优先级高于普通误差。也就是说，如果同一帧里有 `C:blue`，STM32 会优先触发蓝色平台事件，而不是把 `E` 当作循迹误差使用。

STM32 仍兼容旧的数字事件值，来自 `E` 字段：

```text
 1000  -> 右转分岔       IMAGE_COMMAND_TURN_RIGHT
-1000  -> 左转分岔       IMAGE_COMMAND_TURN_LEFT
 2000  -> 蓝色平台       IMAGE_COMMAND_PLATFORM
 3000  -> 紫色住户       IMAGE_COMMAND_PURPLE
 4000  -> 棕色住户       IMAGE_COMMAND_BROWN
 9999  -> 丢线/停止      IMAGE_COMMAND_STOP
```

注意：如果 K230 发的是 `E:<error>,C:3\n` 这种数字颜色，当前 STM32 不会把 `C:3` 解析成蓝色。要么 K230 改成发 `C:blue`，要么 STM32 解析器增加数字颜色兼容。

## STM32 到 K230 协议

### 状态回传

STM32 收到事件命令后会通过 USART2 回传状态：

```text
OK E:<event_state> M:<motion>\n
ST E:<event_state> M:<motion>\n
```

其中：

- `OK` 是收到事件后的确认。
- `ST` 是周期性状态反馈，每 `200 ms` 发送一次。
- `event_state` 来自 `DogTask_EventStateName()`。
- `motion` 来自 `DogTask_MotionName()`。

当前事件状态名包括：

```text
IDLE
COLOR_PAUSE
FORK_TURN
THROW_TRACK_DELAY
THROW_FORWARD
THROW_ROTATING
PLATFORM_PAUSE
PLATFORM_YES_SEND
```

### YES 控制命令

当前 STM32 会通过 USART2 给 K230 发送：

```text
YES\n
```

发送函数在 `User/dog_task.c`：

```c
static void DogTask_SendK230Yes(void)
```

发送底层使用：

```c
HAL_UART_Transmit(&huart2, ...)
```

所以 `YES` 从 STM32 `PA2 / USART2_TX` 发出。

当前已经删除了 `NO` 发送相关代码。代码中没有 `DogTask_SendK230No()`，也没有 `g_k230_no_send_count`。

## 蓝色平台逻辑

当前测试宏：

```c
#define DOG_TASK_PLATFORM_PAUSE_TEST_ENABLE 1U
#define DOG_TASK_PLATFORM_PAUSE_TEST_MS 5000U
#define DOG_TASK_PLATFORM_YES_SEND_MS 5000U
#define DOG_TASK_PLATFORM_YES_INTERVAL_MS 200U
#define DOG_TASK_PLATFORM_FAKE_IMU_TEST_MS 5000U
```

当 STM32 解析到 `IMAGE_COMMAND_PLATFORM`，也就是收到 `C:blue` 或旧协议 `E:2000` 后：

```text
1. STM32 回传一次 OK 状态。
2. 进入 DOG_TASK_EVENT_PLATFORM_PAUSE。
3. 机器狗停止 5s。
4. 5s 到后立即发送一次 YES。
5. 进入 DOG_TASK_EVENT_PLATFORM_YES_SEND。
6. 接下来 5s 内每 200ms 发送一次 YES。
7. YES 连续发送结束后，开启平台循迹增强参数。
8. 恢复普通事件空闲状态，继续循迹。
```

当前不会发送 `NO`。

平台循迹增强参数：

```c
#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          30.0f
#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  60.0f
#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 45.0f
```

如果后续接入真实 IMU，建议把 `DOG_TASK_PLATFORM_PAUSE_TEST_ENABLE` 改为 `0U`，再把 `DogTask_IsPlatformFinishedByImu()` 改成读取真实 IMU 状态。当前它只是 5s 假反馈占位。

## 事件行为

事件状态机在 `User/dog_task.c`。

```text
DOG_TASK_EVENT_IDLE
  普通视觉循迹。

DOG_TASK_EVENT_FORK_TURN
  收到 -1000/1000 后执行约 900 ms 左/右转，然后恢复循迹。

DOG_TASK_EVENT_THROW_TRACK_DELAY
  紫色/棕色首次触发后，先继续循迹一段延时，再进入投掷流程。

DOG_TASK_EVENT_THROW_FORWARD
  投掷前前进阶段；当前 DOG_TASK_THROW_FORWARD_MS 为 0，实际会立即进入旋转。

DOG_TASK_EVENT_THROW_ROTATING
  停止步态并驱动 PB13 PWM 舵机旋转约 1900 ms。

DOG_TASK_EVENT_PLATFORM_PAUSE
  蓝色平台触发后的 5s 停止等待。

DOG_TASK_EVENT_PLATFORM_YES_SEND
  蓝色平台暂停结束后的 5s 连续发送 YES。

DOG_TASK_EVENT_COLOR_PAUSE
  保留的颜色暂停状态，当前主事件路径基本不使用。
```

紫色和棕色事件会触发投掷舵机：

```text
紫色 IMAGE_COMMAND_PURPLE -> THROW_SERVO_DIRECTION_CCW
棕色 IMAGE_COMMAND_BROWN  -> THROW_SERVO_DIRECTION_CW
```

投掷 PWM 参数在 `User/throw_servo.c`：

```c
#define THROW_SERVO_STOP_PULSE_US 1500U
#define THROW_SERVO_CW_PULSE_US   1000U
#define THROW_SERVO_CCW_PULSE_US  1750U
#define THROW_SERVO_ROTATE_MS     1900U
```

## 步态与任务参数

核心步态在 `User/dog_gait.c`：

- `DogGait_GotoStandPose()`：进入站立姿态。
- `DogGait_SetTrotParams()`：普通前进/后退小跑。
- `DogGait_SetTurnLeftParams()` / `DogGait_SetTurnRightParams()`：近似原地转向。
- `DogGait_SetTrackParams()`：左右腿不同步长的视觉差速循迹。
- `DogGait_UpdateTrot()`：按当前参数推进小跑相位并输出 8 个舵机角度。

当前 `User/dog_task.c` 中主要任务参数：

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
```

8 个总线舵机配置集中在 `User/dog_servo_config.c`：

```text
逻辑关节       ID  reverse  center  offset  min  max
LF_KNEE        1     -1      500      -5    200  800
LF_HIP         2     -1      500      40    200  800
RF_KNEE        3      1      500       5    200  800
RF_HIP         4      1      500      20    200  800
LB_KNEE        5     -1      500      35    200  800
LB_HIP         6     -1      500      10    200  800
RB_KNEE        7      1      500       5    200  800
RB_HIP         8      1      500      40    200  800
```

角度到总线舵机位置的转换在 `User/dog_servo.c`：

```text
position = center_pos + offset_pos + reverse * angle_deg * (1000 / 240)
```

随后会限制到 `min_pos ~ max_pos`。

## K230 视觉侧注意事项

仓库里存在多个 K230 脚本版本。当前 STM32 端要求 K230 给 STM32 发：

```text
E:<误差数字>,C:<green|blue|purple|brown|none>\n
```

但 `vision/main(12)(1).py` 中目前仍可看到：

```text
E:<error>,C:<0|1|2|3>
```

并且蓝色对应 `C:3`。这个和当前 STM32 文本颜色协议不一致。实机测试前必须统一协议，否则 STM32 不会把蓝色解析为 `IMAGE_COMMAND_PLATFORM`，也就不会进入 5s 暂停和连续发送 `YES` 的逻辑。

如果保留当前 STM32 解析器，K230 侧应发送：

```text
绿色: E:<error>,C:green\n
蓝色: E:<error>,C:blue\n
紫色: E:<error>,C:purple\n
棕色: E:<error>,C:brown\n
无色: E:<error>,C:none\n
```

K230 接收 STM32 的 `YES\n` 后，可以恢复/继续颜色识别并在屏幕上打印接收状态。若 K230 屏幕没有打印，先用断点确认 STM32 是否进入 `DogTask_SendK230Yes()`，再用 USB-TTL 或逻辑分析仪确认 STM32 PA2 是否真的输出 `YES\n`。

## 构建

本工程使用 CMake Presets 和 `gcc-arm-none-eabi` 工具链。

Debug 构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

主目标：

```text
DogRobot
```

额外测试目标：

```text
DogRobotTrackBlueBoostTest
```

构建产物位于：

```text
build/Debug/
build/Release/
```

## 调试建议

### 判断 STM32 是否收到 K230 数据

在 `User/image_command.c` 的 `HAL_UART_RxCpltCallback()` 或 `ImageCommand_FinishFrame()` 打断点。

观察：

```c
s_rx_data
s_frame
s_frame_len
```

如果能停在 `ImageCommand_FinishFrame()`，说明至少收到了一整帧。

### 判断蓝色是否被 STM32 解析到

在 `User/dog_task.c` 中这里打断点：

```c
else if (command == IMAGE_COMMAND_PLATFORM)
```

如果识别到蓝色后没有停，优先检查 K230 发的是 `C:blue` 还是 `C:3`。

### 判断 STM32 是否发送 YES

在 `User/dog_task.c` 中：

```c
static void DogTask_SendK230Yes(void)
```

和：

```c
g_k230_tx_status = HAL_UART_Transmit(&huart2, ...)
```

处打断点。执行完 `HAL_UART_Transmit()` 后，如果：

```c
g_k230_tx_status == HAL_OK
```

说明 HAL 认为 USART2 发送完成。若 K230 仍收不到，再用 USB-TTL 接 STM32 PA2 验证物理 TX 是否有 `YES\n`。

## 维护建议

- `main.c` 尽量只保留外设初始化和 `DogTask` 调度。
- 总线舵机协议只改 `User/bus_servo.c`。
- 舵机 ID、方向、偏置、限位只改 `User/dog_servo_config.c`。
- 步态几何、足端轨迹、IK 和差速循迹优先改 `User/dog_gait.c`。
- 视觉串口协议优先同步修改 `User/image_command.c` 和 K230 脚本。
- 任务流程、事件状态和动作参数优先改 `User/dog_task.c`。
- 每次实机调试尽量只改一个参数，先架空或托举验证，再落地测试。
- 不要关闭 SWD，不要把 PA13/PA14 改成普通 GPIO。

## 当前注意事项

- 当前 STM32 端不发送 `NO`，只在蓝色平台测试流程中发送 `YES`。
- 当前没有真实 IMU 接口代码，只有假 IMU/延时占位逻辑。
- 当前 STM32 颜色协议要求文本颜色；K230 数字颜色脚本需要同步修改后才能触发蓝色平台逻辑。
- 源码中仍有一部分旧中文注释乱码，不影响编译，但后续维护时建议逐步清理。
- `tests/` 中部分检查脚本是历史一致性检查，不应视为完整自动化测试体系。
- 实机测试前务必确认供电能力、舵机限位、机械干涉和总线控制器状态。
