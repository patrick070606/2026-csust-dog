"""Post-solve metrics for a continuously simulated selected trajectory."""

from __future__ import annotations

from dataclasses import dataclass

from .candidate import GaitParams, _central_derivative, _support_intervals
from .servo_sim import SimState


@dataclass
class CycleMetrics:
    support_blocks: int
    t_sup_s: float
    t_cycle_s: float
    r_actual_mm: float
    d_total_mm: float
    d_high_mm: float
    high_ratio: float
    e_cost: float
    i_cost: float
    c_cost: float
    max_hip_vel: float
    max_knee_vel: float
    max_hip_accel: float
    max_knee_accel: float


def compute_cycle_metrics(states: list[SimState], params: GaitParams) -> CycleMetrics:
    times = [s.time_s for s in states]
    x = [s.x for s in states]
    y = [s.y for s in states]
    lift = [v - params.base_y for v in y]
    vy = _central_derivative(y, times)
    ay = _central_derivative(vy, times)
    intervals = _support_intervals(lift, x, times, params.eps_down, params.eps_up)
    support_mask = [False] * len(times)
    t_sup = 0.0
    r_actual = 0.0
    d_sup = 0.0
    i_cost = 0.0
    for start, end in intervals:
        support_mask[start : end + 1] = [True] * (end - start + 1)
        t_sup += times[end] - times[start]
        r_actual += abs(x[end] - x[start])
        d_sup += abs(x[end] - x[start])
        i_cost += vy[start] ** 2 + vy[end] ** 2 + ay[start] ** 2

    dt_steps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    dx_steps = [x[i + 1] - x[i] for i in range(len(x) - 1)]
    d_leg = sum(abs(dx) for dx in dx_steps)
    d_high = sum(
        abs(dx_steps[i])
        for i in range(len(dx_steps))
        if lift[i] >= params.h_high or lift[i + 1] >= params.h_high
    )
    d_high += d_sup

    e_cost = 0.0
    c_cost = 0.0
    for i in range(len(times) - 1):
        dt = dt_steps[i]
        e_cost += (
            states[i].hip_vel**2
            + states[i].knee_vel**2
            + states[i].hip_accel**2
            + states[i].knee_accel**2
        ) * dt
        if not support_mask[i]:
            c_cost += max(0.0, params.h_des - lift[i]) ** 2 * dt

    d_total = d_leg + d_sup
    support_blocks = len(intervals)
    if support_blocks >= 2 and intervals[0][0] == 0 and intervals[-1][1] == len(times) - 1:
        # The first and last intervals join into one block across the cycle wrap.
        support_blocks -= 1
    return CycleMetrics(
        support_blocks=support_blocks,
        t_sup_s=t_sup,
        t_cycle_s=times[-1] - times[0] if times else 0.0,
        r_actual_mm=r_actual,
        d_total_mm=d_total,
        d_high_mm=d_high,
        high_ratio=d_high / d_total if d_total > 1e-9 else 0.0,
        e_cost=e_cost,
        i_cost=i_cost,
        c_cost=c_cost,
        max_hip_vel=max(abs(s.hip_vel) for s in states),
        max_knee_vel=max(abs(s.knee_vel) for s in states),
        max_hip_accel=max(abs(s.hip_accel) for s in states),
        max_knee_accel=max(abs(s.knee_accel) for s in states),
    )
