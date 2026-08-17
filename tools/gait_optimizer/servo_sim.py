"""Servo command and actual joint simulation reused from the HTML tool."""

from __future__ import annotations

from dataclasses import dataclass

from .kinematics import Waypoint, forward_kinematics


@dataclass
class SimState:
    time_s: float
    hip_deg: float
    knee_deg: float
    hip_vel: float
    knee_vel: float
    hip_accel: float
    knee_accel: float
    x: float
    y: float
    target_hip: float
    target_knee: float


def normalize_angles(values: list[float]) -> list[float]:
    """Unwrap absolute joint angles so command interpolation stays continuous."""
    if not values:
        return []
    continuous = [values[0]]
    offset = 0.0
    prev = values[0]
    for raw in values[1:]:
        delta = raw + offset - prev
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        next_value = raw + offset
        continuous.append(next_value)
        prev = next_value
    return continuous


def simulate_interval(
    start_hip: float,
    start_knee: float,
    target_hip: float,
    target_knee: float,
    *,
    duration_s: float,
    v_max: float,
    a_max: float,
    dt: float = 0.001,
    l1: float = 125.0,
    l2: float = 100.0,
    start_hip_vel: float = 0.0,
    start_knee_vel: float = 0.0,
) -> list[SimState]:
    """Simulate one 60 ms command interval with the existing clamp model.

    The baseline implementation starts each candidate segment from rest. This
    is a simplification and is checked explicitly by the step-by-step plan.
    """
    states: list[SimState] = []
    hip = start_hip
    knee = start_knee
    hip_vel = start_hip_vel
    knee_vel = start_knee_vel
    t = 0.0
    n = max(1, round(duration_s / dt))
    step = duration_s / n

    for _ in range(n + 1):
        time_left = max(duration_s - t, step)
        desired_hip = max(-v_max, min(v_max, (target_hip - hip) / time_left))
        desired_knee = max(-v_max, min(v_max, (target_knee - knee) / time_left))
        hip_accel = max(-a_max, min(a_max, (desired_hip - hip_vel) / step))
        knee_accel = max(-a_max, min(a_max, (desired_knee - knee_vel) / step))
        x, y = forward_kinematics(hip, knee, l1, l2)
        states.append(
            SimState(
                time_s=t,
                hip_deg=hip,
                knee_deg=knee,
                hip_vel=hip_vel,
                knee_vel=knee_vel,
                hip_accel=hip_accel,
                knee_accel=knee_accel,
                x=x,
                y=y,
                target_hip=target_hip,
                target_knee=target_knee,
            )
        )

        new_hip_vel = hip_vel + hip_accel * step
        new_knee_vel = knee_vel + knee_accel * step
        hip += hip_vel * step + 0.5 * hip_accel * step * step
        knee += knee_vel * step + 0.5 * knee_accel * step * step
        hip_vel = new_hip_vel
        knee_vel = new_knee_vel
        t += step
    return states


def simulate_continuous(
    waypoints: list[Waypoint],
    *,
    l1: float,
    l2: float,
    v_max: float,
    a_max: float,
    dt: float = 0.001,
    start_hip_vel: float = 0.0,
    start_knee_vel: float = 0.0,
) -> list[SimState]:
    """Simulate the full command list continuously, like the HTML tool."""
    hips = normalize_angles([wp.hip_deg for wp in waypoints])
    knees = normalize_angles([wp.knee_deg for wp in waypoints])
    intervals = []
    for i in range(len(waypoints) - 1):
        intervals.append(
            {
                "start": waypoints[i],
                "end": waypoints[i + 1],
                "target_hip": hips[i + 1],
                "target_knee": knees[i + 1],
                "duration": (waypoints[i + 1].phase - waypoints[i].phase)
                * 0.06
                / 0.125,
            }
        )

    states: list[SimState] = []
    hip = hips[0]
    knee = knees[0]
    hip_vel = start_hip_vel
    knee_vel = start_knee_vel
    elapsed = 0.0
    for iv in intervals:
        n = max(1, round(iv["duration"] / dt))
        step = iv["duration"] / n
        local_t = 0.0
        for _ in range(n):
            time_left = max(iv["duration"] - local_t, step)
            desired_hip = max(-v_max, min(v_max, (iv["target_hip"] - hip) / time_left))
            desired_knee = max(-v_max, min(v_max, (iv["target_knee"] - knee) / time_left))
            hip_accel = max(-a_max, min(a_max, (desired_hip - hip_vel) / step))
            knee_accel = max(-a_max, min(a_max, (desired_knee - knee_vel) / step))
            x, y = forward_kinematics(hip, knee, l1, l2)
            states.append(
                SimState(
                    time_s=elapsed,
                    hip_deg=hip,
                    knee_deg=knee,
                    hip_vel=hip_vel,
                    knee_vel=knee_vel,
                    hip_accel=hip_accel,
                    knee_accel=knee_accel,
                    x=x,
                    y=y,
                    target_hip=iv["target_hip"],
                    target_knee=iv["target_knee"],
                )
            )
            new_hip_vel = hip_vel + hip_accel * step
            new_knee_vel = knee_vel + knee_accel * step
            hip += hip_vel * step + 0.5 * hip_accel * step * step
            knee += knee_vel * step + 0.5 * knee_accel * step * step
            hip_vel = new_hip_vel
            knee_vel = new_knee_vel
            elapsed += step
            local_t += step
    x, y = forward_kinematics(hip, knee, l1, l2)
    states.append(
        SimState(
            time_s=elapsed,
            hip_deg=hip,
            knee_deg=knee,
            hip_vel=hip_vel,
            knee_vel=knee_vel,
            hip_accel=0.0,
            knee_accel=0.0,
            x=x,
            y=y,
            target_hip=hips[-1],
            target_knee=knees[-1],
        )
    )
    return states
