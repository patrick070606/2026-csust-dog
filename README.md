# DogRobot 工程说明

本工程是基于 STM32F103C8T6 的四足机器狗控制程序，当前主线功能是：

- 通过 USART1 控制 8 个 HTS-35H 总线舵机。
- 通过 USART2 接收 K230 视觉模块的循迹误差和事件命令。
- 根据视觉误差进行差速小跑循迹。
- 根据绿色分岔、蓝色平台、紫色/棕色住户事件执行对应动作。
- 通过 TIM1_CH1N/PB13 控制投掷/附加 PWM 舵机。

旧的 `README_DogRobot_BusServo_Status.md` 主要是调试过程记录，内容较长且包含历史状态。本文档以当前源码为准，作为后续阅读、接线、构建和维护入口。

## 目录结构

```text
Core/
  Inc/                     STM32 HAL 头文件、外设声明
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
  image_command.c/.h       USART2 视觉数据接收、数字协议解析
  dog_task.c/.h            当前机器狗主任务和事件状态机
  throw_servo.c/.h         PB13 PWM 投掷/附加舵机控制

vision/
  main.py                  K230 视觉循迹与颜色识别脚本

tests/
  check_*.py               面向源码片段的轻量一致性检查脚本

information/
  README.md                当前工程说明
  README_DogRobot_BusServo_Status.md
                           历史调试记录
  *.pdf                    舵机、控制器、STM32 示例资料
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

当前总线舵机发送端固定使用：

```c
#define BUS_SERVO_UART huart1
```

### K230 视觉模块

源码位置：`Core/Src/usart.c`、`User/image_command.c`、`vision/main.py`

```text
STM32 USART2_TX  PA2  -> K230 RX
STM32 USART2_RX  PA3  <- K230 TX
STM32 GND             -> K230 GND

Baud: 115200
Format: 8N1
Interrupt: USART2_IRQn
```

K230 侧当前使用 UART3：

```text
GPIO32 -> UART3_TXD
GPIO33 -> UART3_RXD
Baud   -> 115200
```

### 投掷/附加 PWM 舵机

源码位置：`Core/Src/tim.c`、`User/throw_servo.c`

```text
PB13 -> TIM1_CH1N
PWM  -> 20 ms 周期，1 us 计数分辨率
```

注意：PB13 是 TIM1 的互补输出 `CH1N`，启动 PWM 时使用：

```c
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1)
```

### LED

源码位置：`Core/Src/gpio.c`、`User/dog_task.c`

```text
PC13 -> 事件/修正状态指示
```

当前逻辑中，`DOG_TASK_EVENT_COLOR_PAUSE` 状态会点亮 PC13，其余状态关闭。不同 STM32F103C8T6 板载 LED 的有效电平可能不同，如亮灭相反，优先检查硬件有效电平。

## 舵机配置

8 个总线舵机配置集中在 `User/dog_servo_config.c`。

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

随后会被限制到 `min_pos ~ max_pos`。当前限位为 `200~800`，实机增大步态幅度前应先架空或托举，确认没有卡死、打限位、异常发热或供电压降。

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
4. 设置带负载步态模式。
5. 初始化步态并进入站立姿态。
6. 启动 USART2 视觉接收。
7. 默认进入前进/循迹状态。
```

`DogTask_Run()` 是非阻塞循环任务，持续处理：

- 投掷舵机状态更新。
- USART2 最新视觉命令和循迹误差。
- 分岔转向、投掷、平台增益等事件状态机。
- 每 `150 ms` 更新一次步态。
- 每 `200 ms` 向 K230 回传一次状态字符串。

## K230 与 STM32 通信协议

当前主协议是纯数字 ASCII 帧：

```text
<value>\n
```

STM32 也兼容历史格式：

```text
E:<value>\n
```

普通循迹时，`value` 是 K230 计算出的横向误差：

```text
error > 0   目标线在画面右侧，STM32 右纠偏
error < 0   目标线在画面左侧，STM32 左纠偏
|error| <= 35  视为居中直行
```

特殊事件映射：

```text
 1000  -> 右转分岔       IMAGE_COMMAND_TURN_RIGHT
-1000  -> 左转分岔       IMAGE_COMMAND_TURN_LEFT
 2000  -> 蓝色平台       IMAGE_COMMAND_PLATFORM
 3000  -> 紫色住户       IMAGE_COMMAND_PURPLE
 4000  -> 棕色住户       IMAGE_COMMAND_BROWN
 9999  -> 丢线/停止      IMAGE_COMMAND_STOP
```

STM32 收到事件命令后会通过 USART2 回传状态：

```text
OK E:<event_state> M:<motion>
ST E:<event_state> M:<motion>
```

其中 `OK` 是事件确认，`ST` 是周期状态反馈。K230 脚本收到以 `OK` 开头的行后，会在屏幕上显示 `STM32 OK`。

## 视觉任务逻辑

K230 脚本位置：`vision/main.py`

主要功能：

- 白色赛道线识别，计算 `error = line_center_x - image_center_x`。
- LAB 阈值识别绿色、蓝色、紫色、棕色区域。
- 按任务阶段限制颜色触发顺序。
- 通过 UART3 向 STM32 发送数字帧。
- 接收 STM32 `OK/ST` 状态反馈并在屏幕显示。

当前阶段顺序：

```text
STAGE_WAIT_FORK_GREEN
  识别绿色分岔，决定左/右转，并进入住户颜色阶段

STAGE_WAIT_HOUSE
  奇数圈等待紫色，偶数圈等待棕色

STAGE_WAIT_BLUE
  等待蓝色平台事件，随后进入下一圈
```

关键视觉参数集中在 `vision/main.py` 顶部：

```python
UART_BAUD = 115200
COLOR_CONFIRM_FRAMES = 2
ERROR_SMOOTH_ALPHA = 0.65
ERROR_DEADBAND = 3
ERROR_SEND_GAIN = 1.0

WHITE_TRACK_THRESHOLD = (50, 100, -30, 30, -30, 30)
GREEN_BAR_THRESHOLD = (28, 42, -38, -11, -9, 22)
BLUE_BAR_THRESHOLD = (7, 26, 5, 46, -49, -19)
PURPLE_BAR_THRESHOLD = (12, 50, 6, 24, -28, -2)
BROWN_BAR_THRESHOLD = (25, 46, 11, 45, -3, 40)
```

现场调色时可以临时开启：

```python
SAMPLE_MODE = True
DEBUG_LAB = True
```

正式运行前建议改回 `False`，避免打印过多影响稳定性。

## 步态与任务参数

核心步态在 `User/dog_gait.c`：

- `DogGait_GotoStandPose()`：进入站立姿态。
- `DogGait_SetTrotParams()`：普通前进/后退小跑。
- `DogGait_SetTurnLeftParams()` / `DogGait_SetTurnRightParams()`：原地/近似原地转向测试。
- `DogGait_SetTrackParams()`：左右腿不同步长的视觉差速循迹。
- `DogGait_UpdateTrot()`：按当前参数推进小跑相位并输出 8 个舵机角度。

当前站立/行走基准：

```text
连杆长度: L1 = 100 mm, L2 = 100 mm
站立足端 X 偏移: -50 mm
站立足端 Y: 60 mm
默认步态相位增量: 0.20
```

`User/dog_task.c` 中的当前主任务参数：

```c
#define DOG_TASK_GAIT_PERIOD_MS        150U
#define DOG_TASK_GAIT_MOVE_MS          100U
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
```

蓝色平台事件当前不会执行旧的上/下台阶状态机，而是启用平台循迹增强参数：

```c
#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          30.0f
#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  60.0f
#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 45.0f
```

## 事件行为

事件状态机在 `User/dog_task.c`：

```text
DOG_TASK_EVENT_IDLE
  普通视觉循迹

DOG_TASK_EVENT_FORK_TURN
  收到 -1000/1000 后执行约 900 ms 左/右转，然后恢复循迹

DOG_TASK_EVENT_THROW_TRACK_DELAY
  紫色/棕色首次触发后，先继续循迹一段延时，再进入投掷流程

DOG_TASK_EVENT_THROW_FORWARD
  投掷前前进阶段；当前 DOG_TASK_THROW_FORWARD_MS 为 0，实际会立即进入旋转

DOG_TASK_EVENT_THROW_ROTATING
  停止步态并驱动 PB13 PWM 舵机旋转约 1900 ms

DOG_TASK_EVENT_COLOR_PAUSE
  保留的颜色暂停状态，当前主事件路径基本不使用
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

## 维护建议

- `main.c` 尽量只保留外设初始化和 `DogTask` 调度。
- 总线舵机协议只改 `User/bus_servo.c`。
- 舵机 ID、方向、偏置、限位只改 `User/dog_servo_config.c`。
- 步态几何、足端轨迹、IK 和差速循迹只改 `User/dog_gait.c`。
- 视觉串口协议只改 `User/image_command.c` 和 `vision/main.py`。
- 任务流程、事件状态和动作参数优先改 `User/dog_task.c`。
- 每次实机调试尽量只改一个参数，先架空或托举验证，再落地测试。
- 不要关闭 SWD，不要把 PA13/PA14 改成普通 GPIO。

## 常用调参入口

### 机器狗走得太猛

优先降低：

```c
DOG_TASK_TRACK_STEP_H_MM
DOG_TASK_TRACK_LEFT_FORWARD_R_MM
DOG_TASK_TRACK_RIGHT_FORWARD_R_MM
DOG_TASK_TRACK_MAX_STEER_MM
DOG_TASK_TRACK_STEER_GAIN
```

### 循迹左右修正过度

优先降低：

```c
DOG_TASK_TRACK_STEER_GAIN
DOG_TASK_TRACK_MAX_STEER_MM
```

如果小误差也频繁摆动，可以适当增大：

```c
DOG_TASK_TRACK_DEADBAND
```

### 识别颜色误触发

优先检查 `vision/main.py`：

```python
COLOR_CONFIRM_FRAMES
*_BAR_THRESHOLD
COLOR_ROI
COLOR_COOLDOWN_MS
HOUSE_COOLDOWN_MS
```

### 投掷方向或时间不对

优先检查 `User/throw_servo.c`：

```c
THROW_SERVO_CW_PULSE_US
THROW_SERVO_CCW_PULSE_US
THROW_SERVO_ROTATE_MS
```

### 某个关节方向反了或站姿偏

优先检查 `User/dog_servo_config.c`：

```text
reverse
offset_pos
min_pos
max_pos
```

## 当前注意事项

- 源码中仍有一部分旧中文注释乱码，不影响编译，但后续维护时建议逐步清理。
- `README_DogRobot_BusServo_Status.md` 是历史调试记录，不一定代表当前代码状态。
- 蓝色平台事件当前是“平台循迹参数增强”，不是旧版完整上/下台阶动作。
- `tests/` 中部分检查脚本可能引用开发者本机的临时路径，作为历史一致性检查参考，不应视为完整自动化测试体系。
- 实机测试时务必先确认供电能力、舵机限位、机械干涉和总线控制器状态。
