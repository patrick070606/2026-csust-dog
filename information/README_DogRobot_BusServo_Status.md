# DogRobot 总线舵机项目当前进度

更新时间：2026-05-25

本文用于同步 `DogRobot` 当前代码结构、硬件接口、已完成测试、各测试入口、临时修改办法和下一步任务清单。后续继续调试时，优先以本文和当前源码为准。

## 1. 当前总览

当前主工程位置：

```text
E:\STM32CubeIDE\workspace\DogRobot
```

当前目标板和硬件链路：

```text
STM32F103C8T6
  -> USART1 9600 8N1
  -> Hiwonder/幻尔总线舵机控制器
  -> 8 个 HTS-35H 总线舵机
```

当前额外接入了图像/OpenMV 命令串口：

```text
STM32 USART2 115200 8N1
PA2 = USART2_TX
PA3 = USART2_RX
```

当前主任务流程已经改为自动测试：

```text
上电
-> 等待舵机控制器稳定 2 秒
-> 8 舵机回中，动作 5 秒
-> 等待 6.5 秒
-> 初始化步态
-> 进入站立姿态，动作 2 秒
-> 等待 2.5 秒
-> 启动 USART2 图像命令接收
-> 正方向小跑 3 秒
-> 负方向小跑 3 秒
-> 循环
```

注意：当前虽然保留了 `image_command` 图像命令模块，但 `dog_task.c` 主流程暂时不根据图像命令切换动作，只丢弃已收到命令并执行固定正反方向循环。

## 2. 当前代码结构

核心文件：

```text
Core/Src/main.c              主入口，只做 HAL、时钟、GPIO、USART 初始化和 DogTask 调度
Core/Src/usart.c             USART1 总线舵机，USART2 图像命令输入
Core/Src/stm32f1xx_it.c      USART2 中断入口

User/bus_servo.c/.h          幻尔总线舵机协议发送层
User/dog_servo_config.c/.h   逻辑关节到真实舵机 ID、方向、偏置、限位的配置表
User/dog_servo.c/.h          角度到舵机 position 的转换和 8 舵机同步输出
User/dog_gait.c/.h           四足步态、足端轨迹、IK 和转向参数
User/image_command.c/.h      USART2 图像命令接收和单字节命令解析
User/dog_task.c/.h           当前主任务流程
```

CMake 已包含用户模块：

```cmake
target_sources(${BUILD_UNIT_0_NAME} PRIVATE
  User/bus_servo.c
  User/dog_servo_config.c
  User/dog_servo.c
  User/dog_gait.c
  User/image_command.c
  User/dog_task.c
)
```

## 3. 硬件接口

### 3.1 USART1：总线舵机控制器

配置位置：`Core/Src/usart.c`

```text
USART1_TX = PA9
USART1_RX = PA10
Baud      = 9600
Format    = 8N1
```

接线：

```text
STM32 PA9  / USART1_TX -> 总线舵机控制器 RX
STM32 PA10 / USART1_RX <- 总线舵机控制器 TX，可选
STM32 GND              -> 总线舵机控制器 GND
```

当前总线舵机发送使用：

```c
#define BUS_SERVO_UART huart1
```

位置：`User/bus_servo.c`

### 3.2 USART2：图像/OpenMV 命令

配置位置：`Core/Src/usart.c`

```text
USART2_TX = PA2
USART2_RX = PA3
Baud      = 115200
Format    = 8N1
Interrupt = USART2_IRQn
```

接线：

```text
OpenMV TX -> STM32 PA3 / USART2_RX
OpenMV RX -> STM32 PA2 / USART2_TX，可选
OpenMV GND -> STM32 GND
```

接收入口：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
```

位置：`User/image_command.c`

### 3.3 LED

PC13 用作运行指示。当前在 `DogTask_Run()` 中每 500 ms 翻转一次。

## 4. 舵机配置

当前真实 ID 映射在 `User/dog_servo_config.c`：

```text
逻辑关节        ID  reverse  center_pos  offset_pos  min_pos  max_pos
LF_KNEE         1   -1       500         -5          200      800
LF_HIP          2   -1       500          25         200      800
RF_KNEE         3    1       500           5         200      800
RF_HIP          4    1       500          20         200      800
LB_KNEE         5   -1       500          35         200      800
LB_HIP          6   -1       500           0         200      800
RB_KNEE         7    1       500           5         200      800
RB_HIP          8    1       500          40         200      800
```

角度到位置转换在 `User/dog_servo.c`：

```text
position = center_pos + offset_pos + reverse * angle_deg * (1000 / 240)
position 再被限制到 min_pos ~ max_pos
```

当前限位是 `200~800`，已经比早期 `300~700` 放宽。每次增大步态幅度前仍要观察机械限位、堵转、电流、温升和供电压降。

## 5. 当前主任务流程

入口在 `Core/Src/main.c`：

```c
MX_GPIO_Init();
MX_USART1_UART_Init();
MX_USART2_UART_Init();
DogTask_Init();

while (1)
{
    DogTask_Run();
}
```

主流程在 `User/dog_task.c`。

当前关键参数：

```c
#define DOG_TASK_GAIT_PERIOD_MS       150U
#define DOG_TASK_GAIT_MOVE_MS         100U
#define DOG_TASK_LED_PERIOD_MS        500U
#define DOG_TASK_DIRECTION_HOLD_MS    3000U

#define DOG_TASK_STEP_H_MM            30.0f
#define DOG_TASK_FORWARD_R_MM         40.0f
#define DOG_TASK_TURN_R_MM            15.0f
#define DOG_TASK_SPEED_FREQ           0.25f

#define DOG_TASK_SERVO_READY_MS       2000U
#define DOG_TASK_CENTER_MOVE_MS       5000U
#define DOG_TASK_CENTER_WAIT_MS       6500U
#define DOG_TASK_STAND_MOVE_MS        2000U
#define DOG_TASK_STAND_WAIT_MS        2500U
```

当前自动正反向循环的实现：

```c
DogGait_SetTrotParams(DOG_TASK_STEP_H_MM,
                      DOG_TASK_FORWARD_R_MM,
                      DOG_TASK_SPEED_FREQ);

DogGait_SetTrotParams(DOG_TASK_STEP_H_MM,
                      -DOG_TASK_FORWARD_R_MM,
                      DOG_TASK_SPEED_FREQ);
```

含义：

```text
正方向：step_length = +40 mm
负方向：step_length = -40 mm
每个方向保持 3000 ms
每 150 ms 调用一次 DogGait_UpdateTrot(100)
```

## 6. 步态接口说明

### 6.1 站立

接口：

```c
void DogGait_GotoStandPose(uint16_t time_ms);
void DogGait_AllStand(uint16_t time_ms);
```

调用位置：

```c
DogGait_GotoStandPose(DOG_TASK_STAND_MOVE_MS);
```

站立足端坐标在 `User/dog_gait.c`：

```c
#define DOG_GAIT_STAND_FOOT_X_OFFSET_MM -70.0f
#define DOG_GAIT_STAND_FOOT_X_MM       (DOG_GAIT_STAND_FOOT_X_OFFSET_MM)
#define DOG_GAIT_STAND_FOOT_Y_MM       (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)
```

当前 `L1=100, L2=100`，所以站立坐标为：

```text
x = -70 mm
y = 60 mm
```

### 6.2 正向/负向小跑

接口：

```c
void DogGait_SetTrotParams(float step_height_mm,
                           float step_length_mm,
                           float speed_freq);
void DogGait_UpdateTrot(uint16_t time_ms);
```

测试正方向：

```c
DogGait_SetTrotParams(30.0f, 40.0f, 0.25f);
```

测试负方向：

```c
DogGait_SetTrotParams(30.0f, -40.0f, 0.25f);
```

`speed_freq = 0.25f` 表示相位序列为：

```text
0.00 -> 0.25 -> 0.50 -> 0.75 -> 0.00
```

这个值可以整除一个周期，比 `0.24f` 更不容易在周期尾部产生跳变。

### 6.3 左转/右转

接口：e

```c
void DogGait_SetTurnLeftParams(float step_height_mm,
                               float turn_step_mm,
                               float speed_freq);
void DogGait_SetTurnRightParams(float step_height_mm,
                                float turn_step_mm,
                                float speed_freq);
```

当前左转参数逻辑：

```c
s_gait[DOG_GAIT_LEG_LF].r = -safe_r;
s_gait[DOG_GAIT_LEG_LB].r = -safe_r;
s_gait[DOG_GAIT_LEG_RF].r = safe_r;
s_gait[DOG_GAIT_LEG_RB].r = safe_r;
```

当前右转参数逻辑：

```c
s_gait[DOG_GAIT_LEG_LF].r = safe_r;
s_gait[DOG_GAIT_LEG_LB].r = safe_r;
s_gait[DOG_GAIT_LEG_RF].r = -safe_r;
s_gait[DOG_GAIT_LEG_RB].r = -safe_r;
```

注意：这只是一个差速/原地扭转式的初步转向方案。之前实机观察左转效果不够理想，更像后半身偏出，不建议把当前左右侧反向 `r` 方案当最终转向方案。

## 7. 图像命令接口

图像命令模块位置：

```text
User/image_command.c
User/image_command.h
```

解码接口：

```c
uint8_t ImageCommand_DecodeByte(uint8_t data, ImageCommand_t *command);
ImageCommand_t ImageCommand_TakeLatest(void);
```

当前支持命令：

```text
F / f / T / t / B / b / 1  -> IMAGE_COMMAND_FORWARD
L / l / 2                  -> IMAGE_COMMAND_TURN_LEFT
R / r / C / c / 3           -> IMAGE_COMMAND_TURN_RIGHT
S / s / 0                  -> IMAGE_COMMAND_STOP
```

当前接收逻辑已经启用：

```c
ImageCommand_Init();
```

但当前主任务里暂时不执行命令：

```c
(void)ImageCommand_TakeLatest();
```

这样做是为了保留 USART2 接收链路，同时让当前测试固定为“正方向 3 秒 / 负方向 3 秒”。

如果要恢复图像命令控制，请在 `DogTask_Run()` 中把丢弃命令改为处理命令。例如：

```c
ImageCommand_t command = ImageCommand_TakeLatest();

if (command == IMAGE_COMMAND_FORWARD)
{
    DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
}
else if (command == IMAGE_COMMAND_TURN_LEFT)
{
    DogTask_ApplyMotion(DOG_TASK_MOTION_TURN_LEFT);
}
else if (command == IMAGE_COMMAND_TURN_RIGHT)
{
    DogTask_ApplyMotion(DOG_TASK_MOTION_TURN_RIGHT);
}
else if (command == IMAGE_COMMAND_STOP)
{
    DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
}
```

同时应删除或关闭自动正反向 3 秒切换逻辑，避免两套流程互相抢状态。

## 8. 每个测试的接口和修改办法

### 8.1 只测试总线舵机单个 ID

接口：

```c
void BusServo_SetPosition(uint8_t id, uint16_t position, uint16_t time_ms);
```

位置：`User/bus_servo.c`

临时修改 `DogTask_Init()` 或 `main.c` 的 `USER CODE BEGIN 2`：

```c
BusServo_SetPosition(7, 500, 1000);
HAL_Delay(2000);
BusServo_SetPosition(7, 600, 1000);
HAL_Delay(2000);
BusServo_SetPosition(7, 400, 1000);
HAL_Delay(2000);
```

需要包含：

```c
#include "bus_servo.h"
```

### 8.2 测试所有舵机回中

接口：

```c
void DogServo_AllCenter(uint16_t time_ms);
```

位置：`User/dog_servo.c`

临时改法：

```c
DogServo_AllCenter(5000);
HAL_Delay(8000);

while (1)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
}
```

用途：确认 8 个舵机都能慢速回到当前软件零点，且没有机械干涉。

### 8.3 测试所有舵机同步小幅动作

接口：

```c
void DogServo_TestAllCenterAndSmallMove(void);
```

临时改法：

```c
DogServo_TestAllCenterAndSmallMove();
```

用途：确认 8 舵机同步输出链路正常。

### 8.4 逐关节小幅测试

接口：

```c
void DogServo_TestOneByOneSmallMove(void);
```

临时改法：

```c
DogServo_TestOneByOneSmallMove();
```

用途：逐个确认逻辑关节、真实 ID、方向 `reverse`、偏置 `offset_pos` 是否正确。

测试顺序：

```text
LF_KNEE
LF_HIP
RF_KNEE
RF_HIP
LB_KNEE
LB_HIP
RB_KNEE
RB_HIP
```

### 8.5 只测试站立

接口：

```c
DogGait_Init();
DogGait_GotoStandPose(2000);
```

临时改法：

```c
DogServo_AllCenter(5000);
HAL_Delay(6500);
DogGait_Init();
DogGait_GotoStandPose(2000);
HAL_Delay(3000);

while (1)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
}
```

用途：确认从回中到站立姿态的路径安全。

### 8.6 固定正方向小跑

接口：

```c
DogGait_SetTrotParams(30.0f, 40.0f, 0.25f);
DogGait_UpdateTrot(100);
```

临时改法：在 `DogTask_Init()` 末尾保留：

```c
DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
```

在 `DogTask_Run()` 中删除或注释 3 秒切换逻辑：

```c
if ((uint32_t)(now_ms - s_last_direction_switch_ms) >= DOG_TASK_DIRECTION_HOLD_MS)
{
    ...
}
```

### 8.7 固定负方向小跑

临时改法：在 `DogTask_Init()` 末尾改成：

```c
DogTask_ApplyMotion(DOG_TASK_MOTION_BACKWARD);
```

同样需要关闭 3 秒切换逻辑。

### 8.8 当前自动正反方向循环测试

当前代码默认就是这个测试，不需要额外修改。

关键代码在 `User/dog_task.c`：

```c
#define DOG_TASK_DIRECTION_HOLD_MS    3000U
```

切换规则：

```text
FORWARD  -> 3 秒后 BACKWARD
BACKWARD -> 3 秒后 FORWARD
```

### 8.9 左转/右转测试

当前 `DogTask_ApplyMotion()` 已有左转/右转分支，但默认主流程不会进入。

临时测试左转：

```c
DogTask_ApplyMotion(DOG_TASK_MOTION_TURN_LEFT);
```

临时测试右转：

```c
DogTask_ApplyMotion(DOG_TASK_MOTION_TURN_RIGHT);
```

并关闭正反向 3 秒切换逻辑。

注意：左转/右转目前不是最终方案，测试时必须架空或托举。

### 8.10 USART2 图像命令测试

接口：

```c
ImageCommand_Init();
ImageCommand_TakeLatest();
```

如果只想验证 OpenMV 发来的字节是否能被解析，需要在 `DogTask_Run()` 中恢复命令处理，并用 LED 或动作观察结果。

建议先用动作较小的模式，例如收到任意有效命令只闪 LED，不直接走路。确认串口链路稳定后再接步态动作。

## 9. 修改代码的原则

1. 主入口 `Core/Src/main.c` 尽量只保留初始化和调度，不再塞具体测试动作。
2. 所有任务流程优先放到 `User/dog_task.c`。
3. 舵机协议发送只走 `User/bus_servo.c`。
4. 逻辑关节到真实 ID、方向、偏置、限位只改 `User/dog_servo_config.c`。
5. 步态轨迹、站立点、IK 参数只改 `User/dog_gait.c`。
6. OpenMV/图像协议只改 `User/image_command.c`。
7. 每次只改一个变量或一个测试流程，实机观察稳定后再继续扩大动作幅度。
8. 不要把 `User/*.c` 加到 `target_compile_options`，要加到 `target_sources`。
9. 不要关闭 SWD，不要把 PA13/PA14 改成普通 GPIO。

## 10. 已完成事项

- [x] DogRobot CMake 工程可编译生成 `DogRobot.elf`。
- [x] USART1 通过 `huart1` 控制总线舵机控制器。
- [x] `bus_servo` 协议发送层已建立。
- [x] `dog_servo_config` 逻辑关节配置表已建立。
- [x] `dog_servo` 角度到 position 输出层已建立。
- [x] 8 个舵机 ID、方向、偏置已完成一轮实机校准。
- [x] `DogServo_AllCenter()` 可让 8 舵机回中。
- [x] `DogServo_TestOneByOneSmallMove()` 可逐关节小幅测试。
- [x] `dog_gait` 已迁入小跑、站立、左右转参数接口。
- [x] `dog_task` 已从 `main.c` 拆出主任务流程。
- [x] USART2 图像命令模块已合入并开启中断接收。
- [x] 当前主流程已改为正方向 3 秒 / 负方向 3 秒循环。
- [x] 2026-05-24 编译验证通过：`cmake --build build\Debug`。

## 11. 已知问题和风险

1. `User/dog_gait.c`、`User/dog_servo.c`、`User/bus_servo.c` 里仍有部分旧中文注释乱码，不影响编译，但影响维护。
2. 当前 `DogGait_SetTurnLeftParams()` / `DogGait_SetTurnRightParams()` 是左右侧腿步长反向的初步方案，不是最终稳定转向方案。
3. 当前站立点为 `x=-70, y=60`，需要继续实机确认是否适合负载运动。
4. 当前步态幅度 `step_height=30mm, step_length=40mm` 已经不算很小，实机测试应架空或托举开始。
5. 当前 `ImageCommand_TakeLatest()` 被调用但结果被丢弃，图像控制链路还没有重新接回主任务。
6. `DogGait_GotoStandPose()` 附近有一行乱码注释和代码挤在一起的风险，需要后续清理确认；如站立动作异常，优先检查该函数是否实际调用了 `DogGait_UpdateLegAngles()`。

## 12. 下一步清单

- [ ] 先架空或托举，验证当前“正方向 3 秒 / 负方向 3 秒”动作是否安全。
- [ ] 观察 8 个舵机是否有堵转、异响、过热、供电压降或控制器报警。
- [ ] 如果动作过大，优先把 `DOG_TASK_FORWARD_R_MM` 从 `40.0f` 降到 `15.0f~25.0f`。
- [ ] 如果抬腿过高或干涉，优先把 `DOG_TASK_STEP_H_MM` 从 `30.0f` 降到 `10.0f~20.0f`。
- [ ] 清理 `dog_gait.c` 中乱码注释，并确认 `DogGait_GotoStandPose()` 明确调用 `DogGait_UpdateLegAngles()`。
- [ ] 把正反向自动测试跑稳后，再恢复 USART2 图像命令控制。
- [ ] 给 `DogTask` 增加明确模式：自动测试模式、图像命令模式、只站立模式。
- [ ] 重新设计左转/右转，不再直接把当前左右侧反向 `r` 方案作为最终方案。
- [ ] 如需长期调试，增加串口调试输出或 LED 状态码，区分回中、站立、正向、负向、停止。
- [ ] 每次实机通过后，同步更新本文档中的参数和观察结果。


---

## 13. 2026-05-27 当前进度记录

本节记录本轮调试后的 `DogRobot_v2` 当前状态。后续继续调试时，优先以本节和当前源码为准。

### 13.1 当前总体目标

当前工程已经从“固定动作测试”推进到“视觉辅助行走”的阶段。目标链路为：

```text
OpenMV 识别赛道中心线/颜色/岔道
  -> 通过 UART3 发送 ASCII 数据到 STM32 USART2
  -> STM32 解析视觉偏差或事件命令
  -> DogTask 调整步态
  -> 总线舵机执行直行、纠偏、转弯、停止、后续任务动作
```

当前重点不是继续迁移旧工程大段代码，而是逐步把视觉、电控和步态调成稳定闭环。

### 13.2 OpenMV 当前发送协议

OpenMV 脚本位置：

```text
DogRobot_v2/openmv/main1.0(1)(1).py
```

旧协议曾经是：

```text
$error,state,cmd\n
```

当前已改为 STM32 端更容易解析的简单 ASCII 协议：

```text
正常循迹：E:<error>\n
丢线/异常：S\n
特殊事件：R\n / L\n / U\n / P\n / B\n
```

示例：

```text
E:-35\n
E:12\n
E:0\n
S\n
R\n
L\n
```

视觉偏差定义已经统一为：

```text
error = target_x - 160
```

即：

```text
error > 0：目标/中心线在画面右侧
error < 0：目标/中心线在画面左侧
error = 0：基本居中
```

当前 OpenMV 中对应代码为：

```python
error = center_x - IMG_CENTER_X
```

### 13.3 STM32 当前接收与任务逻辑

相关文件：

```text
User/image_command.c/.h
User/dog_task.c
```

`image_command` 当前支持两类数据：

```text
1. 视觉偏差 track：
   E:-35\n
   E=12\n
   -20\n
   +8\n

2. 单字符命令：
   F/T/B -> forward
   L     -> turn left
   R/C   -> turn right
   S     -> stop
```

`DogTask_Run()` 当前优先级：

```text
1. 如果收到有效 track：
   - 记录已经收到视觉数据
   - 更新时间戳
   - 触发 PC13 快闪
   - 调用 DogTask_ApplyTrackError(track.error)

2. 如果之前收到过 track，但超过 800 ms 没有新 track：
   - 停止运动

3. 如果没有 track 且未超时：
   - 才响应 F/L/R/S 字母命令，作为备用调试入口

4. 如果当前不是 STOP：
   - 按 DOG_TASK_GAIT_PERIOD_MS 周期调用 DogGait_UpdateTrot()

5. LED：
   - 收到有效 track 时快闪
   - 平时 500 ms 慢闪作为心跳
```

旧的“3 秒正向/反向自动切换”测试逻辑已经移除，避免覆盖视觉循迹控制。

### 13.4 LED 验证方式

当前使用 PC13 验证 STM32 是否解析到有效 track：

```c
#define DOG_TASK_TRACK_FLASH_PERIOD_MS 80U
#define DOG_TASK_TRACK_FLASH_COUNT     6U
```

现象：

```text
OpenMV 持续发送 E:<error>\n 且 STM32 成功解析：
  PC13 会明显快闪或高频闪动

没有收到有效 track：
  PC13 只保持 500 ms 慢闪心跳
```

如果 PC13 不快闪，优先检查：

```text
1. OpenMV 是否使用 uart.write(...)，而不是 print(...)
2. 是否发送了换行符 \n
3. OpenMV TX 是否接到 STM32 PA3 / USART2_RX
4. GND 是否共地
5. OpenMV UART 波特率是否为 115200
6. STM32 是否已经烧录当前工程
```

### 13.5 当前视觉循迹差速参数

视觉循迹不再直接调用强转左/右转，而是使用差速小跑：

```text
DogTask_ApplyTrackError(error)
  -> 计算 steer_mm
  -> DogGait_SetTrackParams(step_h, forward_r, steer_mm, speed_freq)
```

当前参数位于 `User/dog_task.c`：

```c
#define DOG_TASK_SPEED_FREQ           0.5f

#define DOG_TASK_TRACK_DEADBAND       15
#define DOG_TASK_TRACK_TIMEOUT_MS     800U
#define DOG_TASK_TRACK_STEP_H_MM      20.0f
#define DOG_TASK_TRACK_FORWARD_R_MM   25.0f
#define DOG_TASK_TRACK_MAX_STEER_MM   12.0f
#define DOG_TASK_TRACK_STEER_GAIN     0.2f
#define DOG_TASK_TRACK_RIGHT_BIAS_MM  3.0f
```

含义：

```text
DOG_TASK_TRACK_DEADBAND：
  视觉死区。abs(error) <= 15 时，不使用视觉比例纠偏，只保留机械右偏置。

DOG_TASK_TRACK_STEP_H_MM：
  循迹小跑抬腿高度，目前为 20 mm。

DOG_TASK_TRACK_FORWARD_R_MM：
  循迹基础前进步长，目前为 25 mm。

DOG_TASK_TRACK_MAX_STEER_MM：
  最大左右差速修正量，目前为 12 mm。

DOG_TASK_TRACK_STEER_GAIN：
  视觉偏差到差速步长的比例系数，目前为 0.2。

DOG_TASK_TRACK_RIGHT_BIAS_MM：
  固定右修正偏置，目前为 3 mm，用于抵消机器狗默认向左偏的机械趋势。
```

已观察到的现象：

```text
1. right_bias = 3 mm 时，机器狗仍可能有向左偏趋势。
2. 将 deadband=10、max_steer=16、gain=0.3、right_bias=6 后，机器狗明显向右偏。
3. 当前已回退到较保守参数：deadband=15、max_steer=12、gain=0.2、right_bias=3。
4. 后续建议一次只改一个参数，并记录现象。
```

### 13.6 负载/非负载足端 X 初始坐标

本轮发现：

```text
负载与非负载状态下，DOG_GAIT_STAND_FOOT_X_OFFSET_MM 的合适值不同。
该值会明显改变机器狗站立姿态和行走状态。
```

当前已经拆成两套参数，位置在 `User/dog_gait.c`：

```c
#define DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM -60.0f
#define DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM    -50.0f
```

当前模式开关在 `User/dog_task.c`：

```c
#define DOG_TASK_USE_PAYLOAD_GAIT     1U
```

含义：

```text
DOG_TASK_USE_PAYLOAD_GAIT = 0U：
  使用非负载足端 X 初始坐标 -60.0f

DOG_TASK_USE_PAYLOAD_GAIT = 1U：
  使用负载足端 X 初始坐标 -50.0f
```

当前默认是负载模式：

```text
DOG_TASK_USE_PAYLOAD_GAIT = 1U
```

站立姿态和小跑轨迹的基准 X 都会跟随该模式切换。

### 13.7 当前转向接口状态

当前保留两类转向逻辑：

```text
1. 强转/测试接口：
   DogGait_SetTurnLeftParams(...)
   DogGait_SetTurnRightParams(...)

2. 循迹差速接口：
   DogGait_SetTrackParams(...)
```

当前用于视觉直道循迹的是 `DogGait_SetTrackParams()`。

`DogGait_SetTurnLeftParams()` / `DogGait_SetTurnRightParams()` 仍然是较粗糙的左右侧腿步长反向方案，更适合接下来专门调试“精准左转/右转”，目前不建议直接作为直线循迹主逻辑。

### 13.8 当前风险与注意事项

```text
1. 本轮最后没有执行完整编译验证；烧录前建议执行 cmake --build build\Debug。
2. dog_gait.c、dog_task.c 中仍有部分中文注释编码显示异常，建议后续逐步清理。
3. U/P/B 任务命令 STM32 端尚未实现。
4. 强转左/右转尚未完成实机精确标定。
5. 循迹参数仍需在真实赛道上细调。
6. 当前只拆分了负载/非负载足端 X，Y 值和其他姿态参数尚未拆分。
```

---

## 14. 下一阶段计划清单

### 14.1 第一阶段：左转/右转精准标定

目标：

```text
让机器狗能够稳定、可重复地完成固定角度左转/右转，为后续岔道和颜色事件做准备。
```

任务：

- [ ] 架空或托举机器狗，分别测试 `DogGait_SetTurnLeftParams()` 和 `DogGait_SetTurnRightParams()` 的实际方向是否正确。
- [ ] 确认左转命令是否真左转、右转命令是否真右转；如方向相反，优先修正腿侧 `r` 分配逻辑。
- [ ] 在地面低速测试左转/右转，记录单次转向持续时间、转向角度和滑移情况。
- [ ] 为左转/右转建立独立参数，例如 `TURN_STEP_H_MM`、`TURN_SPEED_FREQ`、`TURN_DURATION_MS_90_DEG`。
- [ ] 判断当前“左右侧反向步长”方案是否足够精准；如果不稳定，改为“边前进边转向”的差速转向方案。
- [ ] 建立明确的转向任务接口，避免把转向逻辑散落在视觉处理里。
- [ ] 实机记录推荐参数，并同步更新本文档。

### 14.2 第二阶段：视觉 + 电控联动完成直线行走和转弯

目标：

```text
OpenMV 在普通直道上发送 E:<error>\n；
STM32 使用差速小跑保持直行；
遇到转弯/岔道命令时，STM32 切换到已标定的转向流程。
```

任务：

- [ ] 确认 OpenMV 持续发送 `E:<error>\n` 时，PC13 会快闪。
- [ ] 在短直道上测试视觉循迹，记录是否偏左/偏右、是否左右振荡。
- [ ] 微调 `DOG_TASK_TRACK_DEADBAND`、`DOG_TASK_TRACK_MAX_STEER_MM`、`DOG_TASK_TRACK_STEER_GAIN`、`DOG_TASK_TRACK_RIGHT_BIAS_MM`。
- [ ] 建立“直行循迹模式”和“事件转向模式”的状态机，避免转弯时仍被 `E:<error>` 持续覆盖。
- [ ] 转弯完成后重新进入视觉循迹模式。

调参建议：

```text
偏得慢但拉不回来：增大 gain 或 max_steer
左右摇摆：减小 gain 或增大 deadband
整体左偏：增大 right_bias
整体右偏：减小 right_bias
步态不稳：减小 forward_r 或 step_h
```

### 14.3 第三阶段：颜色识别与任务命令接入

目标：

```text
OpenMV 识别绿色岔道、蓝色高台、紫色/棕色住户色带；
STM32 根据字母命令执行对应动作或切换状态。
```

任务：

- [ ] 在 STM32 的 `ImageCommand_DecodeByte()` 中明确增加 `U/P/B` 命令枚举。
- [ ] 在 `dog_task.c` 中建立任务状态机，例如 `TRACK`、`TURNING`、`THROWING`、`FORK`、`STAIRS`、`STOP`。
- [ ] 规定颜色事件优先级：green -> 岔道，purple/brown -> 投掷或住户任务，blue -> 高台/楼梯/下一阶段动作。
- [ ] 对颜色事件增加去抖和冷却，避免 OpenMV 连续多帧重复触发同一个动作。
- [ ] 明确任务动作执行期间是否暂停视觉循迹。
- [ ] 动作完成后恢复循迹或进入下一任务状态。

### 14.4 第四阶段：岔道行走与投掷停止

目标：

```text
机器狗识别岔道后，能够按目标方向稳定进入分支；
到达住户/投掷区域后停止或执行投掷动作。
```

任务：

- [ ] 标定岔道前的减速距离和转向时机。
- [ ] 决定岔道策略：固定 L/R 转向，或视觉辅助转弯。
- [ ] 为岔道转向建立独立参数，不与普通左/右转混用。
- [ ] 设计投掷/停止动作接口；若后续有额外机构，单独建模块。
- [ ] 测试颜色误识别时的恢复策略，例如超时回到循迹、丢线停止等。

### 14.5 第五阶段：上楼梯步态

目标：

```text
在直线循迹和转弯稳定后，再接入上楼梯步态，避免把基础行走问题和楼梯问题混在一起。
```

任务：

- [ ] 先确认普通平地小跑稳定。
- [ ] 再确认负载/非负载足端 X 初始坐标稳定。
- [ ] 单独恢复或重写 `getLadderGaitInfo()` 思路，不要直接大幅套用旧工程参数。
- [ ] 先架空测试楼梯步态的腿序和关节范围。
- [ ] 再低速、低台阶、有人托举测试。
- [ ] 为楼梯模式建立独立参数，例如 `STAIR_STEP_H_MM`、`STAIR_FORWARD_R_MM`、`STAIR_SPEED_FREQ`、`STAIR_BODY_BIAS`、`STAIR_FOOT_X/Y`。
- [ ] 楼梯模式开始和结束都要有明确状态切换，避免与视觉循迹同时抢控制。

### 14.6 近期最小闭环目标

建议下一个最小闭环目标为：

```text
1. 调准左转/右转方向和幅度
2. 确认 OpenMV E:<error>\n 能稳定触发 PC13 快闪
3. 在 1 条短直道上完成视觉辅助直行
4. 在直道末端用 R/L 命令触发一次固定角度转弯
5. 转弯后恢复视觉循迹
```

只有这个闭环稳定后，再继续加入颜色事件、岔道投掷和楼梯步态。
# 2026-05-29 当前进度同步

本节记录本轮转向、足端基准点和下一步视觉循迹联调的当前状态。后续继续调试时，优先以本节和当前源码为准。

## A. 当前任务入口状态

入口仍然是：

```text
Core/Src/main.c
  -> DogTask_Init()
  -> while (1) { DogTask_Run(); }
```

当前 `DogTask_Run()` 已在 `main.c` 主循环中启用。

当前 `DogTask_Init()` 中：

```c
// ImageCommand_Init();
// DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
```

也就是说，当前默认不启动 USART2 图像命令接收，也不会在站立后直接前进。之前用于无视觉模块测试的阻塞式自动流程仍保留在代码里，但当前也处于注释状态：

```c
// if (DOG_TASK_AUTO_TEST_ENABLE != 0U)
// {
//     DogTask_RunAutoTestBlocking();
//     ...
// }
```

如果需要恢复“站立后前进 3s、左转 3s、右转 3s”的无视觉自测，需要重新打开这段阻塞式自测代码。

## B. 当前 DogTask 关键参数

当前 `User/dog_task.c` 中的主要运动参数为：

```c
#define DOG_TASK_GAIT_PERIOD_MS        150U
#define DOG_TASK_GAIT_MOVE_MS          100U

#define DOG_TASK_STEP_H_MM             30.0f
#define DOG_TASK_FORWARD_R_MM          40.0f
#define DOG_TASK_TURN_R_MM             15.0f
#define DOG_TASK_SPEED_FREQ            0.25f

#define DOG_TASK_TRACK_DEADBAND        15
#define DOG_TASK_TRACK_TIMEOUT_MS      800U
#define DOG_TASK_TRACK_STEP_H_MM       20.0f
#define DOG_TASK_TRACK_FORWARD_R_MM    25.0f
#define DOG_TASK_TRACK_MAX_STEER_MM    12.0f
#define DOG_TASK_TRACK_STEER_GAIN      0.2f
#define DOG_TASK_TRACK_RIGHT_BIAS_MM   3.0f

#define DOG_TASK_TURN_TEST_DURATION_MS 900U
```

## C. 足端初始坐标已经拆成站立、直行、转向三套

当前 `User/dog_gait.c` 中已经不再只使用一套站立足端 X/Y。现在拆成三套基准点：

```c
// 站立
#define DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM -60.0f
#define DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM    -50.0f
#define DOG_GAIT_STAND_FOOT_Y_MM       (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)

// 直行 / 循迹
#define DOG_GAIT_WALK_FOOT_X_OFFSET_NO_LOAD_MM  -60.0f
#define DOG_GAIT_WALK_FOOT_X_OFFSET_LOAD_MM     -50.0f
#define DOG_GAIT_WALK_FOOT_Y_MM        (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)

// 左转 / 右转
#define DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM  -45.0f
#define DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM     -35.0f
#define DOG_GAIT_TURN_FOOT_Y_MM        (DOG_GAIT_DEFAULT_L1_MM + DOG_GAIT_DEFAULT_L2_MM - 140.0f)
```

当前 `L1=100, L2=100`，所以三套 Y 初值均为：

```text
y = 60 mm
```

当前逻辑：

```text
DogGait_GotoStandPose()
  -> 使用 DOG_GAIT_FOOT_BASE_STAND

DogGait_SetTrotParams()
DogGait_SetTrackParams()
  -> 使用 DOG_GAIT_FOOT_BASE_WALK

DogGait_SetTurnLeftParams()
DogGait_SetTurnRightParams()
  -> 使用 DOG_GAIT_FOOT_BASE_TURN

DogGait_UpdateTrot()
  -> 根据当前 s_foot_base 选择 base_x/base_y
  -> 再叠加摆线 dx/lift
```

这次拆分的原因：实测左转/右转时机器狗重心会向前偏，转向不应继续完全共用直行足端初始坐标。后续如果转向仍然前倾，优先继续微调：

```c
DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM
DOG_GAIT_TURN_FOOT_X_OFFSET_NO_LOAD_MM
```

如果调转向时影响了直行稳定性，不要改 `WALK` 参数，优先只改 `TURN` 参数。

## D. 已修复的构建问题

拆分足端基准点时，旧函数：

```c
DogGait_GetStandFootX()
```

已经被替换为：

```c
DogGait_GetFootBaseX(s_foot_base)
DogGait_GetFootBaseY(s_foot_base)
```

之前 `DogGait_UpdateTrot()` 中残留了一处旧调用，导致 build 报：

```text
implicit declaration of function 'DogGait_GetStandFootX'
```

当前已经修正，`DogGait_UpdateTrot()` 使用当前模式下的 `base_x/base_y`。

同时清理了 `DogGait_GotoStandPose()` 中中文注释和代码挤在同一行的风险，确保站姿会明确执行：

```c
s_foot_base = DOG_GAIT_FOOT_BASE_STAND;
DogGait_SetStandFootPos();
DogGait_UpdateLegAngles();
DogGait_FillServoAngles(angles);
DogServo_SetAngles(angles, time_ms);
```

## E. 下一步：视觉 + 电控联调精准循迹

下一阶段目标：

```text
OpenMV 持续发送 E:<error>\n
STM32 接收并解析 error
DogTask_ApplyTrackError(error)
DogGait_SetTrackParams(step_h, forward_r, steer_mm, speed_freq)
机器狗通过左右步长差动态修正偏差，实现稳定直线循迹
```

视觉联调前建议先切换到视觉模式：

```c
ImageCommand_Init();
```

并确认无视觉自测代码保持关闭，避免上电后自动前进/转向覆盖视觉调试流程：

```c
// DogTask_RunAutoTestBlocking();
```

联调步骤建议：

1. 先只接 OpenMV，不让机器狗行走，确认 OpenMV 发送格式为：

```text
E:<error>\n
```

2. 确认 `error` 定义仍然是：

```text
error = target_x - 160
error > 0：目标线在画面右侧
error < 0：目标线在画面左侧
```

3. 打开 `ImageCommand_Init()` 后，观察 PC13：

```text
收到有效 E:<error>\n -> PC13 快闪
没有有效视觉数据      -> PC13 慢闪
```

4. 上地短直道低速测试，只调视觉循迹参数，不同时改足端基准点：

```c
DOG_TASK_TRACK_DEADBAND
DOG_TASK_TRACK_MAX_STEER_MM
DOG_TASK_TRACK_STEER_GAIN
DOG_TASK_TRACK_RIGHT_BIAS_MM
```

推荐一次只改一个参数。

5. 调参方向：

```text
偏得慢但拉不回来：增大 gain 或 max_steer
左右摆动明显：减小 gain 或增大 deadband
整体向左偏：增大 right_bias
整体向右偏：减小 right_bias
步态不稳：优先减小 forward_r 或 step_h
```

6. 如果直行循迹时重心或姿态不理想，优先调：

```c
DOG_GAIT_WALK_FOOT_X_OFFSET_LOAD_MM
DOG_GAIT_WALK_FOOT_Y_MM
```

如果只有左转/右转时前倾或后倾，优先调：

```c
DOG_GAIT_TURN_FOOT_X_OFFSET_LOAD_MM
DOG_GAIT_TURN_FOOT_Y_MM
```

不要把直行和转向的问题混在同一组参数里调。

## F. 近期最小闭环目标更新

下一步最小闭环目标更新为：

```text
1. 确认当前代码可以 build 通过
2. 打开 ImageCommand_Init()
3. OpenMV 发送 E:<error>\n，确认 PC13 快闪
4. 在短直道上让机器狗只做视觉动态循迹
5. 记录是否整体左偏/右偏、是否左右摆动、是否步态不稳
6. 只调 DOG_TASK_TRACK_* 参数完成直线精准循迹
7. 直线稳定后，再接回 L/R 固定转向或岔道事件
8. 转向阶段只调 DOG_GAIT_TURN_* 足端基准点，不污染直行参数
```

---

# 2026-05-29 最新调试进度同步

本节记录 2026-05-29 后续联调中的最新状态。若本节与前文旧记录不一致，优先以本节和当前源码为准。

## 1. 当前上电后的任务入口状态

当前 `DogTask_Init()` 中仍然启用了视觉命令接收，并且初始化后默认进入前进运动：

```c
ImageCommand_Init();
DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
```

也就是说，当前代码更适合接入 OpenMV 后做视觉循迹联调；如果完全不接视觉，机器狗初始化站立后会进入前进参数，但实际是否持续行走还取决于主循环是否持续调用 `DogTask_Run()` 以及舵机供电、总线通信是否正常。

## 2. 当前视觉纠偏逻辑

当前 `DogTask_ApplyTrackError(error)` 使用的是简单阈值转向逻辑：

```text
error > DOG_TASK_TRACK_DEADBAND   -> TURN_RIGHT
error < -DOG_TASK_TRACK_DEADBAND  -> TURN_LEFT
abs(error) <= deadband            -> FORWARD
```

也就是说，目前视觉偏差不是连续比例差速纠偏，而是在前进、左转、右转三个运动模式之间切换。这个逻辑便于快速验证方向是否正确，但在精准循迹阶段可能导致动作跳变、步态相位被打断、左右摆动或某条腿抬腿高度不稳定。

后续建议改回更适合循迹的连续差速方案：

```c
DogGait_SetTrackParams(step_h, forward_r, steer_mm, speed_freq);
```

让机器狗保持 `WALK` 足端基准点，只通过左右腿步长差修正偏差，而不是频繁切换到 `TURN` 足端基准点。

## 3. 丢线后的运动策略已修改

之前逻辑是：曾经收到过有效 `E:<error>` 后，如果超过：

```c
DOG_TASK_TRACK_RECOVER_MS = 500U
```

没有再次收到有效轨迹，就执行：

```c
DogTask_ApplyMotion(DOG_TASK_MOTION_STOP);
```

这会导致 OpenMV 短暂丢线、画面抖动或串口帧不连续时，机器狗莫名停止。

当前已经改为：

```c
DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
```

因此当前丢线策略为：

```text
丢线 500 ms 内：继续执行上一帧恢复动作
丢线超过 500 ms：不再自动停止，而是继续前进
```

这个改动的目标是避免循迹过程中因为短暂丢线导致机器狗直接停住。

## 4. 当前仍然会停止的情况

虽然已经删除“丢线 500ms 后自动停止”，但以下情况仍会让机器狗停止：

```text
1. 收到事件命令 L/R/U/P/B 后，先进入 2 秒事件暂停
2. 事件暂停期间持续保持 STOP
3. 收到明确的 S/s 停止命令
4. 代码中主动调用 DogTask_ApplyMotion(DOG_TASK_MOTION_STOP)
```

因此如果联调时仍出现停止，需要优先检查 OpenMV 是否仍在丢线或异常时发送 `S\n`，以及颜色事件是否被误识别为 `L/R/U/P/B`。

## 5. 右前腿抬腿高度突然变低的当前判断

当前观察到“走固定步数后右前腿抬腿高度突然变低”。根据现有代码，优先怀疑不是右前腿单独的轨迹公式错误，而是以下因素叠加：

```text
1. RF/RB 对角步态中，右前腿 RF 只在步态周期后半段抬腿
2. DOG_TASK_SPEED_FREQ = 0.25f，步态相位采样较粗
3. 视觉纠偏会在 FORWARD/TURN_LEFT/TURN_RIGHT 之间切换
4. FORWARD 使用 WALK 足端基准点，TURN 使用 TURN 足端基准点
5. STOP 或事件暂停会让步态相位回到 0
```

如果右前腿变低只在接入视觉后明显出现，优先检查任务层运动模式切换，而不是先加右前腿高度补偿。

建议排查顺序：

```text
1. 先断开视觉，只跑纯 FORWARD，确认 RF 抬腿是否稳定
2. 如果纯前进稳定，说明问题主要来自视觉纠偏或事件命令切换
3. 将视觉纠偏从直接 TURN_LEFT/RIGHT 改为 DogGait_SetTrackParams() 差速纠偏
4. 适当降低 DOG_TASK_SPEED_FREQ，例如先从 0.25 降到 0.125
5. 如果纯前进时 RF 仍然偏低，再检查右前髋/膝舵机零位、限位和机械负载
```

## 6. 下一步建议

下一步视觉 + 电控联调建议按这个顺序推进：

```text
1. 确认 OpenMV 不在普通丢线时发送 S\n
2. 确认 STM32 能持续收到 E:<error>\n
3. 在短直道上测试当前阈值转向逻辑，记录是否左右摆动、是否会误停
4. 若出现摆动或腿部高度突变，优先改为 DogGait_SetTrackParams() 连续差速循迹
5. 稳定直线循迹后，再单独恢复颜色事件和固定角度转向
```

本轮未执行 build。

---

# 2026-06-16 当前进度同步

本节记录本轮新增的高台/楼梯动作和 PWM 舵机引脚调整。若本节与前文旧记录不一致，优先以本节和当前源码为准。

## 1. 蓝色高台命令已接入楼梯状态机

OpenMV 当前蓝色高台事件仍发送：

```text
U\n
```

STM32 侧解析链路为：

```text
User/image_command.c
  U/u -> IMAGE_COMMAND_PLATFORM

User/dog_task.c
  IMAGE_COMMAND_PLATFORM -> DogTask_BeginStairSequence()
```

收到蓝色命令后，`DogTask` 会进入独立楼梯状态机。楼梯动作执行期间会暂时接管运动控制，不再使用普通 `E:<error>` 循迹数据修正步态，也不会因为重复收到蓝色事件而重新进入流程。

当前楼梯流程为：

```text
停稳
-> 左前腿抬起并放到 30 mm 高台
-> 右前腿抬起并放到 30 mm 高台
-> 机身向前推进，转移重心
-> 左后腿抬起并放到高台
-> 右后腿抬起并放到高台
-> 高台上恢复站稳
-> 高台上前进约 600 mm
-> 左前腿下台
-> 右前腿下台
-> 机身向前推进
-> 左后腿下台
-> 右后腿下台
-> 恢复普通循迹
```

主要入口和状态在：

```text
User/dog_task.c
  DOG_TASK_EVENT_STAIR_*
  DogTask_BeginStairSequence()
  DogTask_SetStairState()
  DogTask_UpdateEventState()
```

## 2. 楼梯动作参数说明

当前楼梯参数集中在 `User/dog_task.c` 顶部，均为宏定义：

```c
#define DOG_TASK_STAIR_PLATFORM_HEIGHT_MM          30.0f
#define DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM         40.0f
#define DOG_TASK_STAIR_DESCENT_CLEARANCE_MM        15.0f
#define DOG_TASK_STAIR_LIFT_FORWARD_MM             40.0f
#define DOG_TASK_STAIR_STEP_FORWARD_MM             35.0f
#define DOG_TASK_STAIR_BODY_ADVANCE_MM             25.0f
#define DOG_TASK_STAIR_REAR_PLACE_X_MM             15.0f
#define DOG_TASK_STAIR_PITCH_BIAS_DEG               0.0f
#define DOG_TASK_STAIR_POSE_MOVE_MS                 700U
#define DOG_TASK_STAIR_POSE_HOLD_MS                 900U
#define DOG_TASK_STAIR_SETTLE_MOVE_MS               1000U
#define DOG_TASK_STAIR_SETTLE_HOLD_MS               1300U
```

含义：

```text
DOG_TASK_STAIR_PLATFORM_HEIGHT_MM
  高台目标高度，目前按 30 mm。

DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM
  上台时摆动腿抬到的高度，目前为 40 mm，相对 30 mm 高台有约 10 mm 越障余量。

DOG_TASK_STAIR_DESCENT_CLEARANCE_MM
  下台前先抬脚离开台面的高度，用于避免脚尖刮平台边缘。

DOG_TASK_STAIR_LIFT_FORWARD_MM
  摆动腿抬起时相对站立基准向前伸出的距离。

DOG_TASK_STAIR_STEP_FORWARD_MM
  前腿落到台面时的最终前向落脚偏移。

DOG_TASK_STAIR_BODY_ADVANCE_MM
  前腿已经上台后，让机身相对支撑足向前推进的目标距离。
  代码表现为四条腿的 x_offset 同时减小。

DOG_TASK_STAIR_REAR_PLACE_X_MM
  后腿完成上台或下台时，相对站立基准的最终前向落脚偏移。

DOG_TASK_STAIR_PITCH_BIAS_DEG
  髋关节俯仰补偿预留项，目前为 0，暂不参与动作。

DOG_TASK_STAIR_POSE_MOVE_MS / DOG_TASK_STAIR_POSE_HOLD_MS
  单个抬腿或落脚阶段的舵机动作时间和状态保持时间。

DOG_TASK_STAIR_SETTLE_MOVE_MS / DOG_TASK_STAIR_SETTLE_HOLD_MS
  停稳或恢复站姿阶段的动作时间和保持时间。
```

注意：`x_offset_mm` 与站立姿态的 `DOG_GAIT_STAND_FOOT_X_OFFSET_*` 本质上都是足端相对机身的 X 方向偏移。当前实现中：

```c
s_gait[i].x = base_x + targets[i].x_offset_mm;
```

因此：

```text
base_x 决定平时站在哪里；
x_offset_mm 决定楼梯动作当前阶段相对站姿偏到哪里。
```

## 3. 高台上前进 600 mm 的开环估算

高台上前进距离当前没有编码器或边缘传感器参与判断，采用步态周期数开环估算：

```c
#define DOG_TASK_PLATFORM_DISTANCE_MM               600U
#define DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE    25U
#define DOG_TASK_PLATFORM_FORWARD_CYCLES \
    ((DOG_TASK_PLATFORM_DISTANCE_MM + DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE - 1U) / \
     DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE)

#define DOG_TASK_PLATFORM_GAIT_PERIOD_MS            150U
#define DOG_TASK_PLATFORM_STEP_H_MM                  15.0f
#define DOG_TASK_PLATFORM_STEP_R_MM                  25.0f
#define DOG_TASK_PLATFORM_SPEED_FREQ                 0.125f
#define DOG_TASK_PLATFORM_UPDATES_PER_CYCLE          8U
```

当前计算：

```text
目标距离 = 600 mm
估计每周期前进 = 25 mm
需要周期数 = ceil(600 / 25) = 24
每周期更新 = 1.0 / 0.125 = 8 次
每次更新间隔 = 150 ms
平台前进名义时间 = 24 * 8 * 150 ms = 28.8 s
```

`DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE` 必须实机标定。建议先在高台或平整地面上执行固定周期数，测量实际位移后再修改该宏。

更可靠的后续方案是让 OpenMV 识别平台末端或下台标志，由视觉命令触发下台；当前版本是“走够估计距离后自动下台”。

## 4. 步态层新增楼梯足端目标接口

`User/dog_gait.h/.c` 新增：

```c
typedef struct
{
    float x_offset_mm;
    float y_offset_mm;
    float hip_bias_deg;
} DogGaitStairTarget_t;

void DogGait_SetStairPose(const DogGaitStairTarget_t targets[DOG_GAIT_STAIR_LEG_COUNT],
                          uint16_t time_ms);
```

该接口不直接处理视觉命令，只接收四条腿的足端偏移目标：

```text
DogTask_SetStairState()
  -> 修改 s_stair_targets[]
  -> DogTask_ApplyStairTargets()
  -> DogGait_SetStairPose()
  -> DogServo_SetAngles()
```

当前楼梯模式使用显式足端目标 `x_offset/y_offset` 作为主动作，`hip_bias_deg` 只作为小幅姿态补偿预留。旧工程 `dog_robot(stm32)` 的楼梯逻辑主要是通过摆线 `r/h` 生成迈腿轨迹，并用 `bias_big` 做经验式重心调整；当前版本没有直接照搬旧工程的数值。

## 5. PA0 PWM 舵机已改到 PB13

原先投掷/附加舵机 PWM 使用：

```text
TIM2_CH1 -> PA0
```

当前已改为：

```text
TIM1_CH1N -> PB13
```

修改位置：

```text
Core/Src/tim.c
  MX_TIM1_Init()
  PB13 配置为 GPIO_MODE_AF_PP

Core/Inc/tim.h
  extern TIM_HandleTypeDef htim1
  void MX_TIM1_Init(void)

Core/Src/main.c
  MX_TIM1_Init()

User/throw_servo.c
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse)
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1)
```

注意：PB13 对应 STM32F103 的 `TIM1_CH1N` 互补输出，不是普通 `TIMx_CHx` 输出，所以启动 PWM 时使用 `HAL_TIMEx_PWMN_Start()`。

## 6. 当前调试建议

楼梯步态第一次实机测试建议按以下顺序：

```text
1. 架空或托举，只触发 U，观察四条腿顺序是否正确。
2. 检查每个阶段是否有舵机打限位、卡顿、异响或供电压降。
3. 先只测试上台，不急着让它自动下台；必要时临时把平台前进结束改成停止。
4. 标定 DOG_TASK_PLATFORM_ESTIMATED_MM_PER_CYCLE。
5. 如果前腿上台后后腿一抬就后仰，再小幅增加 DOG_TASK_STAIR_PITCH_BIAS_DEG，例如 3~5 度。
6. 若台阶边缘刮脚，优先调 DOG_TASK_STAIR_CLEARANCE_HEIGHT_MM 或 DOG_TASK_STAIR_LIFT_FORWARD_MM。
7. 若后腿上台时身体拉扯明显，优先调 DOG_TASK_STAIR_BODY_ADVANCE_MM 和 DOG_TASK_STAIR_REAR_PLACE_X_MM。
```

本轮未执行 build。

---

# 2026-06-16 K230 联调当前进度同步

本节记录 K230 替换 OpenMV 后的最新电控/视觉联调状态。若本节与前文 OpenMV 或早期楼梯触发记录不一致，优先以本节和当前源码为准。

## 1. 当前视觉模块

当前视觉代码位置：

```text
DogRobot_v2/vision/main.py
```

当前使用 CanMV/K230 豪华版，主要功能为：

```text
1. 识别循迹线，计算偏差 error
2. 识别色带/特殊事件
3. 通过 UART 向 STM32 发送纯数字 ASCII 帧
4. 接收 STM32 返回的 OK，并在屏幕显示 STM32 OK 约 10 秒
```

## 2. K230 当前发送协议

当前 K230 不再发送 `E:<error>,C:<color>`，也不再发送颜色字符串。当前发送格式为：

```text
<error_or_command_value>\n
```

正常循迹：

```text
-320 到 +320 左右的偏差数字
0 表示基本居中
```

特殊事件映射：

```text
R 右转分岔  ->  1000
L 左转分岔  -> -1000
U 蓝色动作  ->  2000
P 紫色动作  ->  3000
B 棕色动作  ->  4000
丢线/异常   ->  9999
```

示例：

```text
0\n
35\n
-28\n
1000\n
3000\n
9999\n
```

当前 K230 中对应代码：

```python
CMD_ERROR_VALUES = {
    "R": 1000,
    "L": -1000,
    "U": 2000,
    "P": 3000,
    "B": 4000,
    "X": 9999,
}

send_error = cmd_to_send_error(cmd, send_error)
data = "%d\n" % send_error
uart.write(data)
```

### K230 颜色停车测试开关

代码位置：

```text
DogRobot_v2/vision/main.py
```

当前测试开关：

```python
COLOR_STOP_TEST_ENABLE = True
COLOR_STOP_TEST_MS = 10000
COLOR_STOP_TEST_VALUE = 9999
```

当 `COLOR_STOP_TEST_ENABLE = True` 时，K230 会放开颜色阶段限制，绿、蓝、紫、棕任意颜色连续识别确认后，持续发送：

```text
9999\n
```

持续时间为 10 秒，并在屏幕上显示：

```text
COLOR STOP: <color_name>
```

正式跑图前需要把 `COLOR_STOP_TEST_ENABLE` 改成 `False`，恢复原来的分岔、住户颜色、高台颜色阶段逻辑。

## 3. STM32 当前解析逻辑

解析代码位置：

```text
DogRobot_v2/User/image_command.c
DogRobot_v2/User/image_command.h
```

当前 `ImageCommand_ParseErrorField()` 仍兼容两类格式：

```text
E:<number>\n
<number>\n
```

但当前 K230 实际使用的是纯数字格式。

当前 STM32 特殊数字映射：

```text
1000   -> IMAGE_COMMAND_TURN_RIGHT
-1000  -> IMAGE_COMMAND_TURN_LEFT
2000   -> IMAGE_COMMAND_PLATFORM
3000   -> IMAGE_COMMAND_PURPLE
4000   -> IMAGE_COMMAND_BROWN
9999   -> IMAGE_COMMAND_STOP
其他数字 -> ImageTrack_t.error，用于循迹
```

注意：`9999` 当前被当作明确 STOP 命令处理，不会进入 2 秒颜色暂停，也不会向 K230 返回 OK。

## 4. 当前循迹逻辑

任务代码位置：

```text
DogRobot_v2/User/dog_task.c
```

当前上电初始化后：

```c
ImageCommand_Init();
DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);
```

也就是说，当前默认启用 K230/USART2 接收，并进入前进参数，后续由 `DogTask_Run()` 根据视觉数据修正。

当前循迹参数：

```c
#define DOG_TASK_TRACK_DEADBAND        35
#define DOG_TASK_TRACK_RECOVER_MS      500U
#define DOG_TASK_TRACK_STEP_H_MM       20.0f
#define DOG_TASK_TRACK_FORWARD_R_MM    20.0f
#define DOG_TASK_TRACK_MAX_STEER_MM    4.0f
#define DOG_TASK_TRACK_STEER_GAIN      0.04f
#define DOG_TASK_TRACK_RIGHT_BIAS_MM   0.0f
```

当前循迹不是直接调用原地 `TURN_LEFT/TURN_RIGHT`，而是使用连续差速接口：

```c
DogGait_SetTrackParams(step_h, forward_r, steer_mm, speed_freq);
```

当前方向约定：

```text
error > 0：目标/线在画面右侧，STM32 按右纠偏处理
error < 0：目标/线在画面左侧，STM32 按左纠偏处理
abs(error) <= 35：按直行处理
```

对应代码逻辑：

```c
error > DOG_TASK_TRACK_DEADBAND
  -> steer = error * DOG_TASK_TRACK_STEER_GAIN
  -> DogGait_SetTrackParams(..., -steer + DOG_TASK_TRACK_RIGHT_BIAS_MM, ...)

error < -DOG_TASK_TRACK_DEADBAND
  -> steer = -error * DOG_TASK_TRACK_STEER_GAIN
  -> DogGait_SetTrackParams(..., steer + DOG_TASK_TRACK_RIGHT_BIAS_MM, ...)
```

## 5. 颜色/事件 ACK 与暂停逻辑

当前以下事件会让 STM32 给 K230 回传：

```text
OK\n
```

并进入 2 秒颜色暂停：

```text
1000   IMAGE_COMMAND_TURN_RIGHT
-1000  IMAGE_COMMAND_TURN_LEFT
2000   IMAGE_COMMAND_PLATFORM
3000   IMAGE_COMMAND_PURPLE
4000   IMAGE_COMMAND_BROWN
```

K230 收到 `OK` 后，屏幕显示：

```text
STM32 OK
```

显示时间约 10 秒。因此只要 K230 屏幕出现 `STM32 OK`，就说明 STM32 已经收到了对应事件并执行了 ACK 分支。

当前颜色/事件暂停参数：

```c
#define DOG_TASK_COLOR_PAUSE_MS        2000U
#define DOG_TASK_COLOR_PAUSE_HOLD_MS   150U
```

之前问题：颜色事件进入后只调用一次 `DogGait_AllStand()`，后续 2 秒内因为 `DogTask_ApplyMotion(STOP)` 的早退逻辑，不会重复下发站立命令。

当前已修改为：进入颜色暂停后，2 秒内每 150 ms 直接重发一次站立姿态：

```c
if ((uint32_t)(now_ms - s_color_pause_last_stand_ms) >= DOG_TASK_COLOR_PAUSE_HOLD_MS)
{
    s_color_pause_last_stand_ms = now_ms;
    DogGait_AllStand(DOG_TASK_GAIT_MOVE_MS);
    s_motion = DOG_TASK_MOTION_STOP;
}
```

这样颜色事件期间不会只依赖一次站立命令，而是持续压住步态。

## 6. PC13 当前用途

当前 PC13 不再作为普通心跳闪烁使用。当前逻辑为：

```text
处于 DOG_TASK_EVENT_COLOR_PAUSE 状态：PC13 亮
其他任何状态：PC13 灭
```

也就是说，PC13 当前用于判断 STM32 是否进入了颜色/事件暂停状态。

注意：若使用常见 STM32F103C8T6 Blue Pill 板载 PC13 LED，板载 LED 可能是低电平点亮。当前源码中：

```c
#define DOG_TASK_LED_ON_STATE          GPIO_PIN_SET
#define DOG_TASK_LED_OFF_STATE         GPIO_PIN_RESET
```

如果实测灯的亮灭与预期相反，应优先检查板载 LED 的有效电平。

## 7. 当前 U/蓝色事件与楼梯状态机关系

源码中仍保留楼梯状态机：

```text
DogTask_BeginStairSequence()
DOG_TASK_EVENT_STAIR_*
```

但是当前 `IMAGE_COMMAND_PLATFORM` 即 `2000/U/蓝色动作` 在 `DogTask_ExecuteEventCommand()` 中被处理为：

```c
DogTask_SendVisionAck();
DogTask_BeginColorPause(now_ms);
```

因此当前版本中，`2000` 不会直接触发楼梯状态机，而是和其他颜色/事件一样 ACK 后暂停 2 秒。

如果后续要恢复蓝色高台触发楼梯，需要把 `IMAGE_COMMAND_PLATFORM` 分支改回：

```c
DogTask_SendVisionAck();
DogTask_BeginStairSequence(now_ms);
```

并重新实机验证楼梯动作。

## 8. 当前已知现象和判断

当前已经确认：

```text
1. K230 与 STM32 串口通信已恢复。
2. K230 屏幕可以显示 STM32 OK，说明 STM32 确实收到事件帧。
3. STM32 当前解析的是纯数字协议，颜色字符串不参与电控逻辑。
4. 循迹已经能够正常工作，error > 0 按右纠偏处理。
5. 颜色/事件收到后，STM32 会 ACK，并进入 2 秒暂停。
6. 2 秒暂停内已经改成每 150 ms 重复下发站立姿态。
```

如果后续仍出现“屏幕显示 STM32 OK，但机器狗没有明显停住”，优先排查：

```text
1. PC13 是否进入颜色暂停状态。
2. 总线舵机是否真的收到重复 DogGait_AllStand() 指令。
3. DOG_TASK_GAIT_MOVE_MS = 100U 是否过短，可临时增大到 150~250 ms。
4. 机械惯性或上一条步态动作是否导致短时间内看起来仍在动。
5. K230 是否在颜色事件后持续发送普通 error，导致 2 秒后马上恢复行走。
```

本轮未执行 build。





































