"""Candidate generation, candidate-segment precompute, and cost statistics."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .kinematics import Waypoint, forward_kinematics, inverse_kinematics
from .servo_sim import simulate_interval


@dataclass(frozen=True)
class CandidatePoint:
    slot: int
    index: int
    phase: float
    x: float
    y: float
    hip_deg: float
    knee_deg: float


@dataclass(frozen=True)
class Segment:
    slot: int
    a: CandidatePoint
    b: CandidatePoint
    feasible: bool
    t_sup: float
    sup: int
    stride: float
    d_leg: float
    d_sup: float
    d_high: float
    e_cost: float
    c_cost: float
    i_cost: float


@dataclass(frozen=True)
class GaitParams:
    l1: float = 125.0
    l2: float = 100.0
    base_x: float = -20.0
    base_y: float = 54.0
    bias_angle_deg: float = 0.0
    step_height: float = 45.0
    step_length: float = 70.0
    speed_freq: float = 0.125
    update_period_s: float = 0.06
    v_max: float = 300.0
    a_max: float = 6000.0
    eps_down: float = 3.0
    eps_up: float = 3.0
    h_high: float = 25.0
    h_des: float = 30.0
    ratio_high_target: float = 0.6
    r_des: float = 55.0
    lambda_e: float = 2.0
    lambda_i: float = 3.0
    lambda_f: float = 2.0
    lambda_c: float = 1.0
    lambda_h: float = 2.0
    hip_min_deg: float = -80.0
    hip_max_deg: float = 80.0
    knee_min_deg: float = -130.0
    knee_max_deg: float = 130.0
    sim_dt_s: float = 0.001


def generate_candidate_library(
    base_waypoints: list[Waypoint],
    *,
    x_steps: tuple[float, ...] = (0.0,),
    y_steps: tuple[float, ...] = (0.0,),
    l1: float,
    l2: float,
    hip_min_deg: float,
    hip_max_deg: float,
    knee_min_deg: float,
    knee_max_deg: float,
) -> list[list[CandidatePoint]]:
    """Generate per-slot candidate points around the cycloid waypoints."""
    library: list[list[CandidatePoint]] = []
    for slot, base in enumerate(base_waypoints):
        slot_candidates: list[CandidatePoint] = []
        seen: set[tuple[float, float]] = set()
        for dx in x_steps:
            for dy in y_steps:
                x = base.x + dx
                y = base.y + dy
                key = (round(x, 6), round(y, 6))
                if key in seen:
                    continue
                hip, knee = inverse_kinematics(x, y, l1, l2)
                if not all(math.isfinite(v) for v in (hip, knee)):
                    continue
                if not (
                    hip_min_deg <= hip <= hip_max_deg
                    and knee_min_deg <= knee <= knee_max_deg
                ):
                    continue
                seen.add(key)
                slot_candidates.append(
                    CandidatePoint(
                        slot=slot,
                        index=len(slot_candidates),
                        phase=base.phase,
                        x=x,
                        y=y,
                        hip_deg=hip,
                        knee_deg=knee,
                    )
                )
        if not slot_candidates:
            raise ValueError(f"slot {slot} has no feasible candidate point")
        library.append(slot_candidates)
    return library


def _support_intervals(
    lift: list[float], x: list[float], times: list[float], eps_down: float, eps_up: float
) -> list[tuple[int, int]]:
    intervals: list[tuple[int, int]] = []
    start = None
    for i in range(len(lift)):
        if start is None and lift[i] <= eps_down:
            start = i
        elif start is not None and lift[i] > eps_up:
            intervals.append((start, i))
            start = None
    if start is not None:
        intervals.append((start, len(lift) - 1))
    return intervals


def _central_derivative(values: list[float], times: list[float]) -> list[float]:
    out = [0.0] * len(values)
    for i in range(len(values)):
        a = max(0, i - 1)
        b = min(len(values) - 1, i + 1)
        dt = times[b] - times[a]
        if dt > 0:
            out[i] = (values[b] - values[a]) / dt
    return out


def _segment_from_states(
    a: CandidatePoint,
    b: CandidatePoint,
    params: GaitParams,
    states,
) -> Segment:
    times = [s.time_s for s in states]
    x = [s.x for s in states]
    y = [s.y for s in states]
    lift = [v - params.base_y for v in y]
    hip = [s.hip_deg for s in states]
    knee = [s.knee_deg for s in states]

    if not (
        all(params.hip_min_deg <= v <= params.hip_max_deg for v in hip)
        and all(params.knee_min_deg <= v <= params.knee_max_deg for v in knee)
    ):
        return Segment(a.slot, a, b, False, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    vy = _central_derivative(y, times)
    ay = _central_derivative(vy, times)
    dt_steps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    dx_steps = [x[i + 1] - x[i] for i in range(len(x) - 1)]

    intervals = _support_intervals(lift, x, times, params.eps_down, params.eps_up)
    t_sup = 0.0
    stride = 0.0
    d_sup = 0.0
    i_cost = 0.0
    support_mask = [False] * len(times)
    for start, end in intervals:
        t_sup += times[end] - times[start]
        support_mask[start : end + 1] = [True] * (end - start + 1)
        net_dx = x[end] - x[start]
        stride += abs(net_dx)
        d_sup += abs(net_dx)
        i_cost += vy[start] ** 2 + vy[end] ** 2 + ay[start] ** 2

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

    return Segment(
        slot=a.slot,
        a=a,
        b=b,
        feasible=True,
        t_sup=t_sup,
        sup=1 if t_sup > params.sim_dt_s else 0,
        stride=stride,
        d_leg=d_leg,
        d_sup=d_sup,
        d_high=d_high,
        e_cost=e_cost,
        c_cost=c_cost,
        i_cost=i_cost,
    )


def _compute_segment(
    a: CandidatePoint,
    b: CandidatePoint,
    params: GaitParams,
    start_hip_vel: float = 0.0,
    start_knee_vel: float = 0.0,
) -> Segment:
    states = simulate_interval(
        a.hip_deg,
        a.knee_deg,
        b.hip_deg,
        b.knee_deg,
        duration_s=params.update_period_s,
        v_max=params.v_max,
        a_max=params.a_max,
        dt=params.sim_dt_s,
        l1=params.l1,
        l2=params.l2,
        start_hip_vel=start_hip_vel,
        start_knee_vel=start_knee_vel,
    )
    return _segment_from_states(a, b, params, states)


def precompute_segments(
    candidate_library: list[list[CandidatePoint]],
    params: GaitParams,
    entry_velocities: dict[int, tuple[float, float]] | None = None,
) -> list[Segment]:
    m = len(candidate_library)
    segments: list[Segment] = []
    for i in range(m):
        start_hip_vel, start_knee_vel = (
            entry_velocities.get(i, (0.0, 0.0))
            if entry_velocities is not None
            else (0.0, 0.0)
        )
        for a in candidate_library[i]:
            for b in candidate_library[(i + 1) % m]:
                seg = _compute_segment(
                    a, b, params, start_hip_vel=start_hip_vel, start_knee_vel=start_knee_vel
                )
                if seg.feasible:
                    segments.append(seg)
    return segments
