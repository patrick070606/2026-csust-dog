"""Gurobi MIQP model for candidate-segment selection."""

from __future__ import annotations

from dataclasses import dataclass

import gurobipy as gp
from gurobipy import GRB

from .candidate import CandidatePoint, GaitParams, Segment


@dataclass
class Solution:
    status: int
    objective: float
    selected_segments: list[Segment]
    path: list[CandidatePoint]


def build_model(
    candidate_library: list[list[CandidatePoint]],
    segments: list[Segment],
    params: GaitParams,
    *,
    time_limit: float = 60.0,
    output_flag: bool = True,
):
    m = len(candidate_library)
    segment_map = {(seg.slot, seg.a.index, seg.b.index): seg for seg in segments}

    model = gp.Model("gait_candidate_segment")
    model.setParam("TimeLimit", time_limit)
    model.setParam("MIPGap", 1e-4)
    model.setParam("OutputFlag", int(output_flag))

    y: dict[tuple[int, int, int], gp.Var] = {}
    for seg in segments:
        name = f"y_{seg.slot}_{seg.a.index}_{seg.b.index}"
        y[(seg.slot, seg.a.index, seg.b.index)] = model.addVar(
            vtype=GRB.BINARY, name=name
        )
    model.update()

    # Exactly one segment per time slot.
    for i in range(m):
        keys = [
            (i, a.index, b.index)
            for a in candidate_library[i]
            for b in candidate_library[(i + 1) % m]
            if (i, a.index, b.index) in y
        ]
        model.addConstr(gp.quicksum(y[k] for k in keys) == 1, name=f"one_per_slot_{i}")

    # Command-point continuity between consecutive slots (cyclic).
    for i in range(m):
        prev = (i - 1) % m
        for a in candidate_library[i]:
            lhs = gp.quicksum(
                y[(i, a.index, b.index)]
                for b in candidate_library[(i + 1) % m]
                if (i, a.index, b.index) in y
            )
            rhs = gp.quicksum(
                y[(prev, c.index, a.index)]
                for c in candidate_library[prev]
                if (prev, c.index, a.index) in y
            )
            model.addConstr(lhs == rhs, name=f"continuity_{i}_{a.index}")

    t_sup_total = model.addVar(lb=0.0, name="t_sup_total")
    r_actual = model.addVar(lb=0.0, name="r_actual")
    d_total = model.addVar(lb=0.0, name="d_total")
    d_high = model.addVar(lb=0.0, name="d_high")
    h_short = model.addVar(lb=0.0, name="h_short")

    model.addConstr(
        t_sup_total
        == gp.quicksum(seg.t_sup * y[key] for key, seg in segment_map.items()),
        name="support_time_total",
    )
    model.addConstr(
        t_sup_total == m * params.update_period_s / 2.0, name="support_half_cycle"
    )
    model.addConstr(
        r_actual == gp.quicksum(seg.stride * y[key] for key, seg in segment_map.items()),
        name="r_actual",
    )
    model.addConstr(
        d_total
        == gp.quicksum(
            (seg.d_leg + seg.d_sup) * y[key] for key, seg in segment_map.items()
        ),
        name="d_total",
    )
    model.addConstr(
        d_high == gp.quicksum(seg.d_high * y[key] for key, seg in segment_map.items()),
        name="d_high",
    )
    model.addConstr(
        h_short >= params.ratio_high_target * d_total - d_high,
        name="high_ratio_shortfall",
    )

    # One contiguous support block per cycle.
    s = model.addVars(m, vtype=GRB.BINARY, name="s")
    u = model.addVars(m, vtype=GRB.BINARY, name="u")
    d = model.addVars(m, vtype=GRB.BINARY, name="d")
    for i in range(m):
        model.addConstr(
            s[i]
            == gp.quicksum(
                seg.sup * y[key]
                for key, seg in segment_map.items()
                if key[0] == i
            ),
            name=f"support_flag_{i}",
        )
    for i in range(m):
        prev = (i - 1) % m
        model.addConstr(u[i] >= s[i] - s[prev], name=f"u_lb_{i}")
        model.addConstr(u[i] <= s[i], name=f"u_s_{i}")
        model.addConstr(u[i] <= 1 - s[prev], name=f"u_prev_{i}")
        model.addConstr(d[i] >= s[prev] - s[i], name=f"d_lb_{i}")
        model.addConstr(d[i] <= s[prev], name=f"d_s_prev_{i}")
        model.addConstr(d[i] <= 1 - s[i], name=f"d_s_{i}")
    model.addConstr(gp.quicksum(u[i] for i in range(m)) == 1, name="one_support_in")
    model.addConstr(gp.quicksum(d[i] for i in range(m)) == 1, name="one_support_out")

    objective = gp.quicksum(
        (params.lambda_e * seg.e_cost + params.lambda_i * seg.i_cost + params.lambda_c * seg.c_cost)
        * y[key]
        for key, seg in segment_map.items()
    )
    objective += params.lambda_f * (r_actual - params.r_des) * (r_actual - params.r_des)
    objective += params.lambda_h * h_short
    model.setObjective(objective, GRB.MINIMIZE)
    model.update()

    return model, y, s, u, d, r_actual, d_total, d_high, h_short, segment_map


def extract_solution(
    model: gp.Model,
    y: dict[tuple[int, int, int], gp.Var],
    candidate_library: list[list[CandidatePoint]],
    segment_map: dict[tuple[int, int, int], Segment],
) -> Solution:
    selected: list[Segment] = []
    for i in range(len(candidate_library)):
        chosen = None
        for key, var in y.items():
            if key[0] == i and var.X > 0.5:
                chosen = segment_map[key]
                break
        if chosen is None:
            raise RuntimeError(f"no segment selected for slot {i}")
        selected.append(chosen)
    selected.sort(key=lambda seg: seg.slot)
    path = [seg.a for seg in selected]
    return Solution(
        status=model.Status,
        objective=model.ObjVal,
        selected_segments=selected,
        path=path,
    )
