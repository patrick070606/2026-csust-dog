# dog_task.c 状态机表（带时间戳）

> 来源：`User/dog_task.c`（枚举 102–154 行；转换逻辑 `DogTask_ExecuteEventCommand` 793 行、`DogTask_UpdateEventState` 884 行）。
> 时间单位均为毫秒（ms）。T0 指**主状态机启动时刻**，即 `DogTask_Init()` 完成舵机回中与站立等待之后，约为上电后 11000ms。

## 1. 运动状态机 `DogTaskMotion_t`（`s_motion`）

| 编号 | 枚举 | 含义 |
|---|---|---|
| 0 | `DOG_TASK_MOTION_STOP` | 停止运动 |
| 1 | `DOG_TASK_MOTION_FORWARD` | 向前运动 |
| 2 | `DOG_TASK_MOTION_BACKWARD` | 向后运动 |
| 3 | `DOG_TASK_MOTION_TURN_LEFT` | 向左转 |
| 4 | `DOG_TASK_MOTION_TURN_RIGHT` | 向右转 |
| 5 | `DOG_TASK_MOTION_SHIFT_LEFT` | 向左平移 |
| 6 | `DOG_TASK_MOTION_SHIFT_RIGHT` | 向右平移 |

## 2. 事件状态机 `DogTaskEventState_t`（`s_event_state`）

| 编号 | 枚举 | 含义 | 持续时长/退出条件 | 退出 → 下一状态 |
|---|---|---|---|---|
| 0 | `DOG_TASK_EVENT_IDLE` | 空闲，普通循迹 | 无固定时长，等待视觉命令 | 收到事件命令 → 对应事件状态 |
| 1 | `DOG_TASK_EVENT_COLOR_PAUSE` | 颜色识别暂停（主流程基本不使用） | `s_color_pause_ms`=2000 | 到时 → IDLE |
| 2 | `DOG_TASK_EVENT_FORK_TURN` | 分岔/绿色转弯 | 普通分岔 3000；绿色第一圈右转 5000；第二圈左转 2000 | 到时 → IDLE；若在 `GREEN_TURN` 阶段先切到 `TRACK_TO_THROW` |
| 3 | `DOG_TASK_EVENT_THROW_TRACK_DELAY` | 首次识别紫/棕后的投掷前循迹延迟 | 1000 | → THROW_FORWARD |
| 4 | `DOG_TASK_EVENT_THROW_FORWARD` | 投掷前前进 | `DOG_TASK_THROW_FORWARD_MS`=0 | → THROW_ROTATING |
| 5 | `DOG_TASK_EVENT_THROW_ROTATING` | 投掷旋转（紫=逆时针，棕=顺时针） | 投掷舵机忙标志结束 | → IDLE，阶段切 `TRACK_TO_ORANGE` |
| 6 | `DOG_TASK_EVENT_PLATFORM_PAUSE` | 蓝色平台停止等待（测试用） | 5000 | → PLATFORM_YES_SEND |
| 7 | `DOG_TASK_EVENT_PLATFORM_YES_SEND` | 连续发送 `YES` | 5000（每 200ms 发一次） | → IDLE（开启平台增强循迹） |
| 8 | `DOG_TASK_EVENT_STAIR_WALK` | 爬楼梯 | `StairWalk` 内部时长 | 爬完发 `YES`，阶段切 `WAIT_BLACK` → IDLE |
| 9 | `DOG_TASK_EVENT_START_SHIFT_LEFT` | 启动/第二圈开始左平移 | 5000 | → IDLE，阶段切 `TRACK_TO_BLUE` |
| 10 | `DOG_TASK_EVENT_SPEED_BUMP_ENTRY_TRACK` | 减速带前循迹（已从主流程移除，仅独立测试保留） | — | 不进入主流程 |
| 11 | `DOG_TASK_EVENT_SPEED_BUMP` | 过减速带（已从主流程移除，仅独立测试保留） | — | 不进入主流程 |
| 12 | `DOG_TASK_EVENT_SHIFT_TO_CENTER` | 黑框居中平移 | 中心稳定 500 | → IDLE，阶段切 `DOWNHILL_TRACK` |
| 13 | `DOG_TASK_EVENT_ORANGE_TRACK_DELAY` | 橙色循迹延迟 | 4000 | → SHIFT_RIGHT |
| 14 | `DOG_TASK_EVENT_SHIFT_RIGHT` | 向右平移 | 8000 | → LAP_PAUSE |
| 15 | `DOG_TASK_EVENT_LAP_PAUSE` | 一圈结束暂停 | 8000 | 第一圈 → START_SHIFT_LEFT；第二圈 → 阶段 `FINISHED`、事件回 IDLE |
| 16 | `DOG_TASK_EVENT_BLACK_TRACK_DELAY` | 黑框循迹延迟 | 3000 | 阶段切 `DOWNHILL_TRACK` → IDLE |
| 17 | `DOG_TASK_EVENT_GREEN_TRACK_DELAY` | 绿色循迹延迟（第二圈） | 5000 | → FORK_TURN 左转 |

## 3. 任务阶段 `DogTaskStage_t`（`s_task_stage`）

| 编号 | 枚举 | 含义 | 进入时刻（相对 T0）/触发 | 持续时长 |
|---|---|---|---|---|
| 0 | `DOG_TASK_STAGE_START_SHIFT_LEFT` | 启动左平移 | 0 / 上电 `Init`、第二圈开始 | 5000 |
| 1 | `DOG_TASK_STAGE_SPEED_BUMP_ENTRY_TRACK` | 减速带前循迹（已从主流程移除，仅独立测试保留） | — | — |
| 2 | `DOG_TASK_STAGE_SPEED_BUMP` | 过减速带（已从主流程移除，仅独立测试保留） | — | — |
| 3 | `DOG_TASK_STAGE_TRACK_TO_BLUE` | 循迹到蓝色平台 | 左平移结束（5000） | 视觉触发 `PLATFORM`（时刻不定） |
| 4 | `DOG_TASK_STAGE_STAIR_WALK` | 爬楼梯 | `PLATFORM` 时刻 | `StairWalk` 内部时长 |
| 5 | `DOG_TASK_STAGE_WAIT_BLACK` | 等待识别黑框 | 楼梯完成时刻 | 视觉触发 `BLACK`（时刻不定） |
| 6 | `DOG_TASK_STAGE_SHIFT_TO_CENTER` | 黑框居中（预留，未接入主流程） | — | — |
| 7 | `DOG_TASK_STAGE_DOWNHILL_TRACK` | 下坡循迹 | 黑框延迟 3000ms 结束 | ≥1500 + 水平稳定 800 |
| 8 | `DOG_TASK_STAGE_TRACK_AFTER_DOWNHILL` | 下坡后循迹 | 机身水平稳定 | 视觉触发 `GREEN`（时刻不定） |
| 9 | `DOG_TASK_STAGE_GREEN_TURN` | 绿色转弯 | `GREEN` 时刻 | 第一圈右转 5000；第二圈循迹 5000 + 左转 2000 |
| 10 | `DOG_TASK_STAGE_TRACK_TO_THROW` | 投掷前循迹 | 绿色转弯结束 | 视觉触发 紫/棕（时刻不定） |
| 11 | `DOG_TASK_STAGE_THROW_TARGET` | 投掷目标 | 紫（第一圈）/棕（第二圈）时刻 | 首次延迟 1000 + 前进 0 + 旋转（舵机时长） |
| 12 | `DOG_TASK_STAGE_TRACK_TO_ORANGE` | 投掷后循迹 | 投掷完成 | 视觉触发 `ORANGE`（时刻不定） |
| 13 | `DOG_TASK_STAGE_ORANGE_TRACK_DELAY` | 橙色循迹延迟 | `ORANGE` 时刻 | 4000 |
| 14 | `DOG_TASK_STAGE_SHIFT_RIGHT` | 向右平移 | 橙色延迟结束 | 8000 |
| 15 | `DOG_TASK_STAGE_LAP_PAUSE` | 一圈暂停 | 右平移结束 | 8000 |
| 16 | `DOG_TASK_STAGE_FINISHED` | 任务完成 | 第二圈暂停结束 | — |

## 4. 上电初始化时间轴（`DogTask_Init`，固定）

| 累计时刻 | 动作 | 说明 |
|---|---|---|
| 0 | `ThrowServo_Init()` 开始 | 上电 |
| 2000 | 舵机回中 | 下发 0°（过渡 5000ms），`HAL_Delay` 6500ms |
| 8500 | 站立姿态 | 下发站立（过渡 2000ms），`HAL_Delay` 2500ms |
| 11000 | 主状态机启动 | 进入阶段 0 左平移（T0） |

## 5. 任务主流程时间轴（第一圈）

> 视觉触发行的时间取决于 K230 识别，标注为“视觉触发，时刻不定”；其余为固定时长累加。

| 阶段 | 相对 T0 时刻 | 持续 | 说明 |
|---|---:|---:|---|
| 0 左平移 | 0 | 5000 | 事件 9 `START_SHIFT_LEFT` |
| 1 循迹到蓝平台 | 5000 | 视觉触发 | 事件 0 `IDLE`，等 `PLATFORM` 命令 |
| 2 爬楼梯 | 视觉触发 | `StairWalk` 时长 | 事件 8 `STAIR_WALK` |
| 3 等黑框 | 楼梯完成 | 视觉触发 | 事件 0 `IDLE`，等 `BLACK` 命令 |
| 4 黑框延迟 | `BLACK` 时刻 | 3000 | 事件 16 `BLACK_TRACK_DELAY` |
| 5 下坡循迹 | 黑框延迟结束 | ≥1500+800 | 事件 0；机身水平稳定后进阶段 8 |
| 6 下坡后循迹 | 水平稳定 | 视觉触发 | 事件 0，等 `GREEN` 命令 |
| 7 绿色转弯 | `GREEN` 时刻 | 5000（第一圈右转） | 事件 2 `FORK_TURN` |
| 8 投掷前循迹 | 转弯结束 | 视觉触发 | 事件 0，等紫/棕命令 |
| 9 投掷目标 | 紫/棕时刻 | 1000+0+旋转 | 事件 3→4→5 |
| 10 投掷后循迹 | 投掷完成 | 视觉触发 | 事件 0，等 `ORANGE` 命令 |
| 11 橙色延迟 | `ORANGE` 时刻 | 4000 | 事件 13 `ORANGE_TRACK_DELAY` |
| 12 右平移 | 橙色延迟结束 | 8000 | 事件 14 `SHIFT_RIGHT` |
| 13 一圈暂停 | 右平移结束 | 8000 | 事件 15 `LAP_PAUSE` |

## 6. 圈间流程

- **第一圈结束**（阶段 15 暂停 8000ms 后）：`s_lap_count` 0→1，重置投掷延迟标志，回到阶段 0 开始第二圈。
- **第二圈绿色转弯**：先事件 17 `GREEN_TRACK_DELAY`（循迹 5000ms），再事件 2 左转 2000ms。
- **第二圈投掷**：识别棕色，投掷延迟标志重置后可再次触发。
- **第二圈结束**（阶段 15 暂停 8000ms 后）：阶段切 16 `FINISHED`，事件回 0 `IDLE`，停车。

## 7. 备注

- 状态 12 `SHIFT_TO_CENTER`（事件/阶段）虽已定义且有 `BeginShiftToCenter()`，但当前主流程**未调用**：`BLACK` 命令直接走 `BLACK_TRACK_DELAY` → 下坡。
- `DOG_TASK_STAGE_DOWNHILL_TRACK` → `TRACK_AFTER_DOWNHILL` 由 `DogTask_Run()` 判定：下坡 ≥1500ms 且机身 pitch/roll 接近水平（5°/6°）稳定 800ms。
- `DogTask_SpeedBumpTest_*` / `DogTask_SpeedBumpEntryTest_*` 是独立的减速带测试入口；这两个事件/阶段已从主流程移除，只保留在独立测试中使用。
