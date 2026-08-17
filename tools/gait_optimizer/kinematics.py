"""Kinematics reused from tools/dog_gait_angle_tool.html.

The formulas intentionally mirror the existing browser tool so the Python
optimizer and the HTML preview stay consistent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def cycloid_raw(t: float, h: float, r: float) -> tuple[float, float]:
    th = (2.0 * math.pi * t) / 0.5
    progress = (th - math.sin(th)) / (2.0 * math.pi)
    raw_x = progress * r if r >= 0.0 else (1.0 - progress) * (-r)
    raw_y = h * (1.0 - math.cos(th)) * 0.5
    return raw_x, raw_y


def cycloid_component(
    t: float, bias_angle_deg: float, h: float, r: float
) -> tuple[float, float]:
    raw_x, raw_y = cycloid_raw(t, h, r)
    angle_rad = math.radians(bias_angle_deg)
    return (
        raw_x * math.cos(angle_rad) + raw_y * math.sin(angle_rad),
        raw_y * math.cos(angle_rad) - raw_x * math.sin(angle_rad),
    )


def project_trot_offset(
    t: float, bias_angle_deg: float, h: float, r: float
) -> tuple[float, float]:
    if t <= 0.5:
        return cycloid_component(t, bias_angle_deg, h, r)
    phase = t - 0.5
    return cycloid_component(phase, bias_angle_deg, 0.0, -r)


def project_trot_x(
    t: float, base_x: float, bias_angle_deg: float, h: float, r: float
) -> float:
    return base_x + project_trot_offset(t, bias_angle_deg, h, r)[0]


def project_trot_y(
    t: float, base_y: float, bias_angle_deg: float, h: float, r: float
) -> float:
    return base_y + project_trot_offset(t, bias_angle_deg, h, r)[1]


def inverse_kinematics(
    x: float, y: float, l1: float, l2: float, eps: float = 1e-9
) -> tuple[float, float]:
    """Return absolute hip/knee angles in degrees for foot position (x, y)."""
    dy = l1 + l2 - y
    ll2 = x * x + dy * dy
    ll = math.sqrt(ll2)
    if ll < eps:
        return 0.0, 0.0

    hip_base = math.atan2(x, dy)
    hip_link = math.acos(
        clamp((ll2 + l1 * l1 - l2 * l2) / (2.0 * l1 * ll), -1.0, 1.0)
    )
    knee_link = math.acos(
        clamp((l1 * l1 + l2 * l2 - ll2) / (2.0 * l1 * l2), -1.0, 1.0)
    )
    return math.degrees(hip_base - hip_link), math.degrees(math.pi - knee_link)


def forward_kinematics(
    hip_deg: float, knee_deg: float, l1: float, l2: float
) -> tuple[float, float]:
    hip = math.radians(hip_deg)
    knee = math.radians(hip_deg + knee_deg)
    return (
        l1 * math.sin(hip) + l2 * math.sin(knee),
        l1 + l2 - l1 * math.cos(hip) - l2 * math.cos(knee),
    )


@dataclass(frozen=True)
class Waypoint:
    phase: float
    x: float
    y: float
    hip_deg: float
    knee_deg: float


def build_trot_waypoints(
    *,
    base_x: float,
    base_y: float,
    bias_angle_deg: float,
    h: float,
    r: float,
    l1: float,
    l2: float,
    speed_freq: float,
) -> list[Waypoint]:
    """Build one cyclic trot cycle from the same formulas as the HTML tool."""
    phases = [i * speed_freq for i in range(round(1.0 / speed_freq))]
    waypoints: list[Waypoint] = []
    for phase in phases:
        x = project_trot_x(phase, base_x, bias_angle_deg, h, r)
        y = project_trot_y(phase, base_y, bias_angle_deg, h, r)
        hip, knee = inverse_kinematics(x, y, l1, l2)
        waypoints.append(Waypoint(phase, x, y, hip, knee))
    return waypoints
