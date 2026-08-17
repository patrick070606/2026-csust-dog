# 小跑轨迹优化设计：关节空间列表选点与 Gurobi 求解

> 状态：设计中，参数已初步确认，待实测验证
> 日期：2026-08-17
> 范围：`tools/dog_gait_angle_tool.html` 对应的轨迹建模、目标定义与后续 Gurobi 模型

## 1. 背景

当前小跑轨迹由“摆线足端轨迹 + 相位推进”生成，目标点不是优化出来的。该方法优点是简单、平滑，但不能保证在关节速度/加速度、舵机限位、支撑相时间等约束下最优。

本设计改为：

```text
候选关节目标点列表 -> 候选段预计算 -> Gurobi 选择最优轨迹 -> 导出目标点列表
```

目标函数衡量的对象不是原始足端目标点，而是真实舵机实际会走出的关节曲线：

```text
q_c(t)：舵机收到的线性插值指令
q(t)  ：经过角速度/角加速度限制后的实际关节曲线
P(t)  ：实际足端轨迹，P(t) = FK(q(t))
```

## 2. 运动学模型

### 2.1 连杆参数

| 参数 | 含义 | 当前值 |
|---|---|---:|
| `l1` | 代码小腿长 | 125 mm |
| `l2` | 代码大腿长 | 100 mm |

当前代码中 `DogGait_CalcAbsoluteAngleByPos()` 使用：

```text
dy = l1 + l2 - y
L  = sqrt(x^2 + dy^2)

qH = atan2(x, dy) - acos((L^2 + l1^2 - l2^2) / (2 * l1 * L))
qK = pi - acos((l1^2 + l2^2 - L^2) / (2 * l1 * l2))
```

正运动学：

```text
x = l1 * sin(qH) + l2 * sin(qH + qK)
y = l1 + l2 - l1 * cos(qH) - l2 * cos(qH + qK)
```

### 2.2 当前任务参数

| 参数 | 当前值 |
|---|---:|
| 足端基准 `base_x` | -20 mm |
| 足端基准 `base_y` | 54 mm |
| 步高 `h` | 45 mm |
| 步长 `r` | 70 mm |
| 更新周期 `T` | 60 ms |
| 相位增量 `speed_freq` | 0.125 |
| 单周期更新点数 | 8 |
| 单周期时间 | 480 ms |

## 3. 指令曲线与实际曲线

### 3.1 线性插值指令

舵机收到目标角后按时间线性插值，指令曲线为：

```text
q_c(t) = q_i + omega_i * (t - t_i)

omega_i = (q_{i+1} - q_i) / T_i
```

### 3.2 实际关节曲线

真实舵机存在角加速度上限，实际曲线 `q(t)` 与 `q_c(t)` 不完全相同，会在指令拐角处拉弯并滞后。

当前不引入 `Kp/Kd` 模拟量，直接使用工具现有的“目标速度 + 加速度 clamp + 积分”算法作为 `q_c -> q` 跟随模型：

```text
desired_omega = clamp((q_target - q) / time_left, -V_max, V_max)
q_ddot        = clamp((desired_omega - q_dot) / dt, -A_max, A_max)
q_dot        += q_ddot * dt
q            += q_dot * dt + 0.5 * q_ddot * dt^2
```

该模型作为当前默认模型；若后续实测发现实际舵机跟随偏差明显，再考虑补充标定跟踪模型。

当前取值：`V_max = 300 deg/s`、`A_max = 6000 deg/s²`。

## 4. 决策变量与候选段

### 4.1 决策变量

优化变量是关节空间目标点，而不是足端目标点：

```text
候选点 j：
  q_Hj, q_Kj
  x_j, y_j       由 FK 预计算
  h_j            实际离地高度相关量
  L_j            可达距离
```

若工具输入仍为足端列表，则先通过 IK 转为关节目标点，再进入本模型。

### 4.2 候选段

相邻两个候选点构成一段：

```text
段 (a -> b)：
  q_c(t)：a 到 b 的关节线性插值
  q(t)  ：加速度限制后的实际曲线
  P(t)  ：FK(q(t)) 加密采样
  T_ab  ：该段时长
  T_sup ：该段实际支撑时长
  sup_ab：该段是否包含支撑时间（0/1）
  stride_ab：该段支撑时间内脚相对机身水平位移取负，无滑动时等于机身前进位移
  d_leg_ab：该段单腿实际足端水平位移，默认按加密采样后的 |dx| 累加
  d_sup_ab：该段支撑时间内水平位移，无滑动时取 stride_ab
  d_high_ab：该段足端高度 >= h_high 时的单腿位移，并计入该段支撑相位移 d_sup_ab
  软目标成本：E_ab, C_ab, I_ab
```

所有非线性物理量在候选段预计算阶段离线算好，Gurobi 只负责选择段。

## 5. 硬约束

### 5.1 候选段选择

```text
y_i,a,b = 1  表示第 i 段选择候选段 a -> b

每个时刻只选一段：
  sum_{a,b} y_i,a,b = 1

前后段连续：
  sum_b y_i,a,b = sum_b y_{i-1},b,a
```

### 5.2 运动学硬约束

```text
可达性：
  L_min <= sqrt(x_i^2 + (l1 + l2 - y_i)^2) <= l1 + l2 - eps

关节角范围：
  q_H_min <= q_Hi <= q_H_max
  q_K_min <= q_Ki <= q_K_max

角速度：
  |omega_Hi| <= V_max
  |omega_Ki| <= V_max

角加速度：
  |alpha_Hi| <= A_max
  |alpha_Ki| <= A_max
```

### 5.3 周期闭合

```text
最后一个候选段结束后，关节状态回到第一个候选点：
  q(cycle_end) = q(cycle_start)
```

## 6. 支撑相定义与强约束

### 6.1 支撑相定义

支撑相不能由预设相位判断，必须从实际足端曲线观察：

```text
对实际 q(t) 做 FK，得到 y(t)

支撑条件：
  y(t) <= y_stance + eps

为避免反复触地，使用滞回：
  进入支撑：y(t) <= y_stance + eps_down
  离开支撑：y(t) >= y_stance + eps_up
```

支撑相为连续时间区间：

```text
[t_TD, t_LO]
```

其中 `t_TD` 为落地时刻，`t_LO` 为离地时刻。

当前取值：`eps_down = eps_up = 3 mm`。

### 6.2 支撑/摆动时间强约束

真实小跑中，每条腿一个周期内一半支撑、一半摆动：

```text
t_LO - t_TD = T_cycle / 2
```

等价的离散形式：

```text
支撑总时长 = T_cycle / 2
摆动总时长 = T_cycle / 2
```

该约束是硬约束，不作为软目标。

在候选段模型中，每段预计算支撑时长：

```text
T_sup,ab = 段 (a -> b) 中 y(t) <= y_stance + eps 的时长
```

强约束为：

```text
sum_i sum_{a,b} T_sup,ab * y_i,a,b = T_cycle / 2
```

支撑区间连续性为硬约束，不允许碎片化支撑：

```text
s_i = sum_{a,b} sup_ab * y_i,a,b
```

`sup_ab` 在候选段预计算阶段得到：该段内存在支撑时间取 1，否则取 0。

整个周期只允许一个连续支撑块。使用循环下标 `i-1`，即 `i=1` 时的前一段为最后一段：

```text
u_i >= s_i - s_{i-1}
u_i <= s_i
u_i <= 1 - s_{i-1}

d_i >= s_{i-1} - s_i
d_i <= s_{i-1}
d_i <= 1 - s_i

sum_i u_i = 1
sum_i d_i = 1
```

其中 `u_i` 表示支撑块的进入边，`d_i` 表示支撑块的离开边，`u_i/d_i in {0, 1}`。

## 7. 软目标

软目标只衡量“满足硬约束之后的质量”，不重复承担安全上限。

### 7.1 E：关节平滑/出力

```text
E = sum_i ( omega_Hi^2 + omega_Ki^2 + alpha_Hi^2 + alpha_Ki^2 )
```

### 7.2 I：落地/离地冲击

```text
I = v_y,TD^2 + v_y,LO^2 + a_y,TD^2
```

其中 `v_y,TD`、`v_y,LO` 为实际足端曲线在落地、离地时刻的竖直速度。

### 7.3 F：任务保真

```text
候选段预计算：
  stride_ab = 段 (a -> b) 支撑时间内脚相对机身水平位移取负

Gurobi 全局定义：
  r_actual = sum_i sum_{a,b} stride_ab * y_i,a,b

F = (r_actual - r_des)^2
```

其中 `r_actual` 为单一支撑块对应的实际步长。

当前取值：`r_des = 55 mm`。

### 7.4 C：离地高度

C 暂保留为单向惩罚：

```text
C = sum_{swing} max(0, h_des - h_i)^2
```

低于期望离地高度才罚，高于期望不罚。

当前 `h_des = 30 mm`。

### 7.5 H：高位位移占比

鼓励足端高度高于 `h_high` 时的位移占总位移的比例达到目标值：

当前 `h_high = 25 mm`、`ratio_high_target = 0.6`。

```text
候选段预计算：
  d_leg_ab ：该段单腿实际足端水平位移
  d_sup_ab ：该段支撑时间内水平位移，无滑动时取 stride_ab
  d_high_ab：该段足端高度 >= h_high 时的单腿位移，并计入该段支撑相位移 d_sup_ab

全局位移：
  D_leg  = sum_{i,a,b} d_leg_ab * y_i,a,b
  D_sup  = sum_{i,a,b} d_sup_ab * y_i,a,b
  D_high = sum_{i,a,b} d_high_ab * y_i,a,b
  D_total = D_leg + D_sup

软目标：
  H = max(0, ratio_high_target * D_total - D_high)
```

`D_high` 同时计入高位单腿位移和对应支撑相位移，因此 `D_high` 与 `D_total` 的统计口径一致。`H` 表示高位位移占比不足目标时的位移缺口，达到目标后不再额外奖励。

### 7.6 总目标

```text
min J = lambda_E * E + lambda_I * I + lambda_F * F + lambda_C * C + lambda_H * H
```

E、I、F、H 不使用目标区间和容忍尺度；R 项已移除。

### 7.7 当前目标参数

| 参数 | 值 | 说明 |
|---|---:|---|
| 位移口径 | 水平位移 | 单腿位移与支撑相位移均按水平位移累加 |
| `h_high` | 25 mm | 高位位移统计阈值 |
| `ratio_high_target` | 0.6 | 高位位移占总位移的目标比例 |
| `h_des` | 30 mm | 离地高度目标 |
| `r_des` | 55 mm | 目标步长 |
| `V_max` | 300 deg/s | 舵机角速度上限 |
| `A_max` | 6000 deg/s² | 舵机角加速度上限 |
| `eps_down` | 3 mm | 进入支撑的触地检测阈值 |
| `eps_up` | 3 mm | 离开支撑的触地检测阈值 |
| `lambda_E` | 2 | 关节平滑/出力权重 |
| `lambda_I` | 3 | 落地/离地冲击权重 |
| `lambda_F` | 2 | 任务保真权重 |
| `lambda_C` | 1 | 离地高度权重 |
| `lambda_H` | 2 | 高位位移占比权重 |

## 8. Gurobi 模型

Gurobi 不能直接处理 `atan2/acos/sin/cos`，因此非线性全部放在候选段预计算阶段。

### 8.1 主模型

```text
变量：
  y_i,a,b in {0, 1}
  s_i, u_i, d_i in {0, 1}
  r_actual >= 0
  D_total >= 0
  D_high >= 0
  H_short >= 0

目标：
  min sum_{i,a,b} (lambda_E*E_ab + lambda_I*I_ab + lambda_C*C_ab) * y_i,a,b
      + lambda_F * ((r_actual - r_des)^2)
      + lambda_H * H_short

约束：
  sum_{a,b} y_i,a,b = 1
  前后段连续性
  r_actual = sum_{i,a,b} stride_ab * y_i,a,b
  s_i = sum_{a,b} sup_ab * y_i,a,b
  D_total = sum_{i,a,b} (d_leg_ab + d_sup_ab) * y_i,a,b
  D_high = sum_{i,a,b} d_high_ab * y_i,a,b
  H_short >= ratio_high_target * D_total - D_high
  支撑区间连续性（单一连续支撑块）
  支撑时长 = T_cycle / 2
  周期闭合
  候选段本身已通过运动学硬约束过滤
```

由于 F 保留全局二次目标，主模型为 0-1 混合整数二次规划（MIQP）；二次项只有 `r_actual` 一个，规模仍可控。

### 8.2 MILP 退化方案

若希望退回纯 MILP，可把 F 写成关于 `r_actual` 的分段线性目标，例如用一个辅助变量表示与 `r_des` 的偏差并加入线性目标。本设计默认保留 F 的二次形式，优先保证任务保真；MIQP 的二次项很少，求解压力可控。

## 9. 工具改造要求

`tools/dog_gait_angle_tool.html` 需要按以下链路调整：

```text
关节目标点列表
  -> 生成线性指令 q_c(t)
  -> 加速度限制仿真 q(t)
  -> FK 得到实际足端轨迹 P(t)
  -> 检测支撑/摆动区间
  -> 统计 E/C/I、stride_ab/sup_ab/T_sup、d_leg_ab/d_sup_ab/d_high_ab
```

需要增加：

- 列表输入从“足端目标点”改为“关节目标点”，或由足端点自动转换。
- 实际曲线仿真不再直接跟踪目标端点，而是跟踪线性指令 `q_c(t)`。
- 显示实际支撑时长、摆动时长及 50/50 偏差。
- 显示实际 `r_actual` 与目标 `r_des` 的偏差。
- 显示高位位移占比 `D_high / D_total` 与目标 `ratio_high_target` 的偏差。
- 显示支撑区间连续性检查，支撑区间必须是单一连续块。
- 支撑时长不满足强约束时给出错误状态。
- CSV 增加实际 `q(t)`、`y(t)`、支撑标志、支撑块编号等字段。

## 10. 参数状态

目标参数、舵机速度/加速度上限与触地阈值均已确认，见 [7.7 当前目标参数](#77-当前目标参数)。`V_max`、`A_max`、`eps_down/eps_up` 后续可结合实测结果修正。

## 11. 当前状态

- [x] 确定衡量对象为实际关节曲线 `q(t)`
- [x] 确定支撑相从实际足端曲线观察
- [x] 确定支撑/摆动时间 50/50 为强约束
- [x] 确定支撑区间连续性为硬约束，支撑区间必须为单一连续块
- [x] 确定 `r_actual` 为 Gurobi 全局变量，F 只保留步长项并保留二次目标（MIQP）
- [x] 确定高位位移占比为软目标 H，采用线性缺口惩罚
- [x] 确认目标参数：位移口径、`h_high`、`ratio_high_target`、`h_des` 与目标权重
- [x] 确认 `V_max`、`A_max`、`eps_down/eps_up`、`r_des`，后续可结合实测修正
- [x] 确定 `q_c -> q` 使用现有简化模型，不引入 `Kp/Kd` 模拟量
- [x] 确定 E/I/F 为直接二次目标，R 移除
- [ ] 生成候选点库
- [ ] 实现候选段预计算
- [ ] 接入 Gurobi 求解
