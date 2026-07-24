# 上楼梯 WALK 步态与比赛流程进度

更新时间：2026-07-05

## 当前状态

当前上楼梯 WALK 步态已经从独立测试入口整理为正式模块，并接入 `DogTask` 主比赛流程。今天已经完成编译验证：`DogRobot` 目标编译通过。

注意：今天新增的完整流程尚未进行实机全流程测试。底层上楼梯和平移步态此前已经单独测试过，但由于后续可能更换机器狗，参数仍需要重新校准。

## 已完成内容

### 1. 清理测试入口

已经移除早期测试入口和对应构建目标：

```text
User/stair_walk_test_program.c
User/stair_walk_test_program.h
Core/Src/main_stair_walk_test.c
User/track_blue_boost_test_program.c
User/track_blue_boost_test_program.h
Core/Src/main_track_blue_boost_test.c
```

当前主目标以 `DogRobot` 为准，不再保留独立上楼梯测试目标和蓝色平台测试目标。

### 2. 新增正式 stair_walk 模块

新增正式上楼梯模块：

```text
User/stair_walk.c
User/stair_walk.h
```

对外接口：

```c
void StairWalk_Init(void);
void StairWalk_Start(void);
void StairWalk_Update(void);
uint8_t StairWalk_IsFinished(void);
uint8_t StairWalk_IsRunning(void);
```

模块内部调用 `DogGait_ResetWalk()`、`DogGait_SetWalkParams()`、`DogGait_UpdateWalk()` 和 `DogGait_IsWalkCycleDone()`，并使用 JY61P IMU 的 pitch/roll 作为上楼梯 WALK 输入。

### 3. WALK 步态接口已经接入 dog_gait

`User/dog_gait.h` 中已经有 WALK 相关接口：

```c
void DogGait_ResetWalk(void);
void DogGait_SetWalkParams(float step_height_mm,
                           float step_length_mm,
                           float speed_freq,
                           float cg_base_x_mm,
                           float imu_gain_mm);
void DogGait_UpdateWalk(uint16_t time_ms, float pitch_deg, float roll_deg);
uint8_t DogGait_IsWalkCycleDone(void);
```

当前 WALK 是单腿顺序步态，相序为：

```text
LF -> RF -> LB -> RB
```

当前仍是简化版姿态模型，尚未完整迁移灯哥项目中的 `cal_ges()` 滤波融合和完整四足姿态基础坐标计算。

### 4. 上楼梯完成判定已经改为 IMU 水平稳定判定

`StairWalk_IsFinished()` 当前不再用固定轮数直接结束，而是：

```text
已启动上楼梯
-> WALK 至少执行 STAIR_WALK_MIN_CYCLES 轮
-> IMU 有效
-> pitch/roll 滤波已经初始化
-> pitch 和 roll 都进入水平阈值
-> 连续稳定 STAIR_WALK_LEVEL_STABLE_MS
-> 判定上楼梯结束
```

当前参数：

```c
#define STAIR_WALK_MIN_CYCLES                 2U
#define STAIR_WALK_LEVEL_PITCH_DEG            5.0f
#define STAIR_WALK_LEVEL_ROLL_DEG             6.0f
#define STAIR_WALK_LEVEL_STABLE_MS            800U
```

这个判定还需要实机验证，尤其是更换机器狗后 IMU 安装角度和机械姿态可能变化。

### 5. K230 图像命令类型已经扩展

`ImageCommand_t` 已扩展：

```c
IMAGE_COMMAND_GREEN,
IMAGE_COMMAND_BLACK,
IMAGE_COMMAND_ORANGE,
```

`image_command.c` 当前支持解析：

```text
C:green
C:black
C:orange
```

同时对 `C:black` 做了特殊处理：如果同一帧里有 `E:<error>`，会保留该误差，供黑框平移居中使用。

### 6. 顶层比赛流程已经接入

`DogTaskStage_t` 当前用于描述完整比赛大阶段：

```text
START_SHIFT_LEFT
TRACK_TO_BLUE
STAIR_WALK
WAIT_BLACK
SHIFT_TO_CENTER
DOWNHILL_TRACK
TRACK_AFTER_DOWNHILL
GREEN_TURN
TRACK_TO_THROW
THROW_TARGET
TRACK_TO_ORANGE
ORANGE_TRACK_DELAY
SHIFT_RIGHT
LAP_PAUSE
FINISHED
```

当前主流程：

```text
启动
-> 回中
-> 站立
-> 左平移 2s
-> 循迹到蓝色
-> 蓝色触发上楼梯
-> 上楼梯完成后发送 YES
-> 等待黑色围栏/黑框
-> 黑色触发左右平移居中
-> 进入下坡循迹
-> IMU 判定下坡结束
-> 等绿色
-> 第 1 圈绿色右转，第 2 圈绿色左转
-> 第 1 圈等紫色投球，第 2 圈等棕色投球
-> 投球完成后等橙色
-> 橙色后继续循迹 5s
-> 右平移 2s
-> 停止 5s
-> 第 1 圈重新开始，第 2 圈 FINISHED
```

### 7. 左右平移复用原有步态

当前新增的运动状态：

```c
DOG_TASK_MOTION_SHIFT_LEFT
DOG_TASK_MOTION_SHIFT_RIGHT
```

只是把已有平移步态接入 `DogTask_ApplyMotion()`：

```c
DogGait_SetShiftLeftParams(...)
DogGait_SetShiftRightParams(...)
```

没有修改 `dog_gait.c` 中原有平移步态算法。

### 8. 下坡 IMU 判定已接入但未实测

当前下坡阶段为普通循迹，结束条件为：

```text
进入 DOWNHILL_TRACK 至少 DOG_TASK_DOWNHILL_MIN_MS
-> pitch/roll 进入水平阈值
-> 连续稳定 DOG_TASK_LEVEL_STABLE_MS
-> 切换到 TRACK_AFTER_DOWNHILL
```

当前参数：

```c
#define DOG_TASK_DOWNHILL_MIN_MS           1500U
#define DOG_TASK_LEVEL_PITCH_DEG           5.0f
#define DOG_TASK_LEVEL_ROLL_DEG            6.0f
#define DOG_TASK_LEVEL_STABLE_MS           800U
```

该部分尚未单独实机测试。若下坡 IMU 判定不准，后续绿色/投球/橙色/圈数流程会触发不到。

## 当前风险

1. 当前代码已经编译通过，但没有实机全流程测试。
2. 黑框居中方向可能需要实机确认；如果越调越偏，需要对调左右平移方向。
3. `C:black` 居中依赖 K230 持续发送 `C:black` 和 `E:error`。
4. 下坡 IMU 判定未测，可能卡在 `DOG_TASK_STAGE_DOWNHILL_TRACK`。
5. 更换机器狗后，上楼梯、平移、小跑循迹、投球相关参数都可能需要重新校准。
6. `cal_ges()` 中剩余滤波融合和完整姿态基础坐标仍未迁移。

## 下一步建议

### 1. 明天优先验证下坡 IMU

- [ ] 观察 `g_jy61p_imu_pitch_deg` 和 `g_jy61p_imu_roll_deg` 在平地、上坡、下坡结束时的变化。
- [ ] 确认 `DOG_TASK_LEVEL_PITCH_DEG` 和 `DOG_TASK_LEVEL_ROLL_DEG` 是否合适。
- [ ] 确认 `DOG_TASK_LEVEL_STABLE_MS` 是否过短或过长。
- [ ] 必要时增加下坡 IMU 判定旁路宏，用固定时间先测试后半段流程。

### 2. 分段实机测试主流程

建议按顺序测试：

```text
 左平移 2s
-> 循迹到蓝色
-> 上楼梯
-> YES
-> 黑框识别
-> 平移居中 
```

再测试：

```text
下坡结束
-> 绿色转弯
-> 紫色/棕色投球
-> 橙色后循迹 5s
-> 右平移 2s
-> 停 5s
-> 圈数切换
```

### 3. 继续保留 cal_ges() 迁移任务

当前先暂停 `cal_ges()` 中未迁移的滤波融合。若上楼梯实机仍出现重心不稳、后腿摆动、左右晃动明显，再继续迁移灯哥项目中完整姿态基础坐标和滤波融合部分。
