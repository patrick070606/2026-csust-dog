# 候选段轨迹优化逐步检查方案

> 配套实现：`tools/gait_optimizer/`
> 基础参数见：`docs/gait_trajectory_optimization_design.md`

## 1. 语法与导入

```powershell
python -m compileall tools/gait_optimizer
python -m unittest tools.gait_optimizer.tests.test_gait_optimizer
```

预期：无语法错误，5 个测试全部通过。

## 2. IK/FK 一致性

检查摆线 8 个足端点经过 IK 后，再 FK 是否能回到原位置。

```powershell
python -c "from tools.gait_optimizer.kinematics import build_trot_waypoints, inverse_kinematics, forward_kinematics; from tools.gait_optimizer.candidate import GaitParams; p=GaitParams(); wps=build_trot_waypoints(base_x=p.base_x,base_y=p.base_y,bias_angle_deg=0,h=p.step_height,r=p.step_length,l1=p.l1,l2=p.l2,speed_freq=p.speed_freq); [print(round(abs(forward_kinematics(*inverse_kinematics(w.x,w.y,p.l1,p.l2),p.l1,p.l2)[0]-w.x),9)) for w in wps]"
```

预期：所有误差接近 0。

## 3. 基础摆线点

确认基础周期正好 8 个点，相位为 `0, 0.125, ..., 0.875`。

```powershell
python -c "from tools.gait_optimizer.kinematics import build_trot_waypoints; from tools.gait_optimizer.candidate import GaitParams; p=GaitParams(); print([w.phase for w in build_trot_waypoints(base_x=p.base_x,base_y=p.base_y,bias_angle_deg=0,h=p.step_height,r=p.step_length,l1=p.l1,l2=p.l2,speed_freq=p.speed_freq)])"
```

预期：输出 8 个相位值。

## 4. 单段舵机仿真

检查单段 60 ms 仿真结果有限、不越限。

```powershell
python -c "from tools.gait_optimizer.servo_sim import simulate_interval; s=simulate_interval(-41.65,80.76,-46.88,97.79,duration_s=0.06,v_max=300,a_max=6000,dt=0.001); print(len(s), min(x.hip_deg for x in s), max(x.knee_deg for x in s))"
```

预期：约 61 个采样点，角度值有限且平滑。

## 5. 候选点库

确认每个时间槽都有候选点。

```powershell
python -c "from tools.gait_optimizer.candidate import GaitParams,generate_candidate_library; from tools.gait_optimizer.kinematics import build_trot_waypoints; p=GaitParams(); b=build_trot_waypoints(base_x=p.base_x,base_y=p.base_y,bias_angle_deg=0,h=p.step_height,r=p.step_length,l1=p.l1,l2=p.l2,speed_freq=p.speed_freq); c=generate_candidate_library(b,x_steps=(-6,0,6),y_steps=(-3,0,3),l1=p.l1,l2=p.l2,hip_min_deg=p.hip_min_deg,hip_max_deg=p.hip_max_deg,knee_min_deg=p.knee_min_deg,knee_max_deg=p.knee_max_deg); print([len(s) for s in c])"
```

预期：8 个槽都有非空候选点。

## 6. 候选段预计算

确认每个槽都有可行候选段。

```powershell
python -c "from collections import Counter; from tools.gait_optimizer.candidate import GaitParams,generate_candidate_library,precompute_segments; from tools.gait_optimizer.kinematics import build_trot_waypoints; p=GaitParams(); b=build_trot_waypoints(base_x=p.base_x,base_y=p.base_y,bias_angle_deg=0,h=p.step_height,r=p.step_length,l1=p.l1,l2=p.l2,speed_freq=p.speed_freq); c=generate_candidate_library(b,x_steps=(-6,0,6),y_steps=(-3,0,3),l1=p.l1,l2=p.l2,hip_min_deg=p.hip_min_deg,hip_max_deg=p.hip_max_deg,knee_min_deg=p.knee_min_deg,knee_max_deg=p.knee_max_deg); print(Counter(s.slot for s in precompute_segments(c,p)))"
```

预期：8 个槽的候选段数量都大于 0。

## 7. Gurobi 求解

运行端到端优化。

```powershell
python -m tools.gait_optimizer.cli --quiet --time-limit 30
```

预期：返回最优解，并生成：

- `tools/gait_optimizer/output/target_waypoints.csv`
- `tools/gait_optimizer/output/simulated_cycle.csv`

## 8. 连续仿真校验

当前默认实现使用离散速度状态图，因此把选中状态边的轨迹按时间拼接后，以下指标应与 Gurobi 模型一致：

- 支撑总时长应接近 `T_cycle / 2`
- 支撑区间应只有一个连续块
- `r_actual` 应接近 `r_des`
- `D_high / D_total` 是软目标，不保证一定达到 `ratio_high_target`
- 关节速度和加速度应在 `V_max/A_max` 内

CLI 还会输出 `velocity state jump`。这是速度档离散化造成的边界速度跳变，当前默认 11 档；若数值过大，应增加速度档数量，或改用连续的入口速度迭代方案。

## 9. CSV 结果核对

打开 `target_waypoints.csv`，检查：

- 8 行目标点，相位覆盖一个完整周期
- 目标点之间在髋/膝关节空间连续
- 最后一个周期目标点与第一个目标点一致

打开 `simulated_cycle.csv`，检查：

- 时间连续覆盖约 480 ms
- 实际 `q(t)`、足端 `x/y`、支撑标志均有限
- 支撑标志与实际 `y` 高度一致

## 10. 实机验证

在固件中替换当前小跑目标点后：

1. 空载慢速运行一个周期，确认髋/膝关节角度连续、无跳变。
2. 检查四足相位关系是否仍符合小跑对角步态。
3. 用高速录像核对足端离地高度和支撑/摆动时间。
4. 实测 `V_max/A_max/eps`，修正参数后重新预计算。
