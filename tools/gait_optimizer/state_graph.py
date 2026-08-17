"""State-aware candidate graph with discretized entry/exit joint velocities."""

from __future__ import annotations

from dataclasses import dataclass

import gurobipy as gp
from gurobipy import GRB

from .candidate import (
    CandidatePoint,
    GaitParams,
    Segment,
    _compute_segment,
    _segment_from_states,
)
from .servo_sim import simulate_interval


@dataclass(frozen=True)
class StateNode:
    slot: int
    candidate: CandidatePoint
    hip_bin: int
    knee_bin: int
    index: int
    hip_vel: float
    knee_vel: float


@dataclass(frozen=True)
class StateEdge:
    slot: int
    start: StateNode
    end: StateNode
    segment: Segment


VELOCITY_BINS: tuple[int, ...] = (-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5)


def _velocity_bin_center(bin_id: int, v_max: float) -> float:
    width = 2.0 * v_max / len(VELOCITY_BINS)
    if bin_id < 0:
        return bin_id * width + 0.5 * width
    return bin_id * width - 0.5 * width


def _classify_velocity(value: float, v_max: float) -> int:
    width = 2.0 * v_max / len(VELOCITY_BINS)
    index = int((max(-v_max, min(v_max, value)) + v_max) // width)
    index = max(0, min(len(VELOCITY_BINS) - 1, index))
    return index - 5
    return 0


def build_state_nodes(
    candidate_library: list[list[CandidatePoint]], v_max: float
) -> list[list[StateNode]]:
    nodes_by_slot: list[list[StateNode]] = []
    for slot, candidates in enumerate(candidate_library):
        nodes: list[StateNode] = []
        for candidate in candidates:
            for hip_bin in VELOCITY_BINS:
                for knee_bin in VELOCITY_BINS:
                    nodes.append(
                        StateNode(
                            slot=slot,
                            candidate=candidate,
                            hip_bin=hip_bin,
                            knee_bin=knee_bin,
                            index=len(nodes),
                            hip_vel=_velocity_bin_center(hip_bin, v_max),
                            knee_vel=_velocity_bin_center(knee_bin, v_max),
                        )
                    )
        nodes_by_slot.append(nodes)
    return nodes_by_slot


def build_state_edges(
    candidate_library: list[list[CandidatePoint]],
    nodes_by_slot: list[list[StateNode]],
    params: GaitParams,
) -> dict[tuple[int, int, int], StateEdge]:
    m = len(candidate_library)
    target_index: dict[tuple[int, int, int, int], StateNode] = {}
    for slot, nodes in enumerate(nodes_by_slot):
        for node in nodes:
            target_index[(slot, node.candidate.index, node.hip_bin, node.knee_bin)] = node

    edges: dict[tuple[int, int, int], StateEdge] = {}
    for slot in range(m):
        for start in nodes_by_slot[slot]:
            for target_candidate in candidate_library[(slot + 1) % m]:
                states = simulate_interval(
                    start.candidate.hip_deg,
                    start.candidate.knee_deg,
                    target_candidate.hip_deg,
                    target_candidate.knee_deg,
                    duration_s=params.update_period_s,
                    v_max=params.v_max,
                    a_max=params.a_max,
                    dt=params.sim_dt_s,
                    l1=params.l1,
                    l2=params.l2,
                    start_hip_vel=start.hip_vel,
                    start_knee_vel=start.knee_vel,
                )
                segment = _segment_from_states(
                    start.candidate, target_candidate, params, states
                )
                if not segment.feasible:
                    continue
                end = states[-1]
                end_hip_bin = _classify_velocity(end.hip_vel, params.v_max)
                end_knee_bin = _classify_velocity(end.knee_vel, params.v_max)
                end_node = target_index[
                    ((slot + 1) % m, target_candidate.index, end_hip_bin, end_knee_bin)
                ]
                key = (slot, start.index, end_node.index)
                edges[key] = StateEdge(slot, start, end_node, segment)
    return edges


def build_state_model(
    candidate_library: list[list[CandidatePoint]],
    nodes_by_slot: list[list[StateNode]],
    edges: dict[tuple[int, int, int], StateEdge],
    params: GaitParams,
    *,
    time_limit: float = 60.0,
    output_flag: bool = True,
):
    m = len(candidate_library)
    model = gp.Model("gait_state_graph")
    model.setParam("TimeLimit", time_limit)
    model.setParam("MIPGap", 1e-4)
    model.setParam("OutputFlag", int(output_flag))

    y: dict[tuple[int, int, int], gp.Var] = {}
    for key, edge in edges.items():
        y[key] = model.addVar(
            vtype=GRB.BINARY,
            name=f"y_{key[0]}_{key[1]}_{key[2]}",
        )
    model.update()

    for slot in range(m):
        model.addConstr(
            gp.quicksum(y[key] for key in edges if key[0] == slot) == 1,
            name=f"one_edge_{slot}",
        )

    for slot in range(m):
        prev = (slot - 1) % m
        for node in nodes_by_slot[slot]:
            outgoing = gp.quicksum(
                y[key] for key in edges if key[0] == slot and key[1] == node.index
            )
            incoming = gp.quicksum(
                y[key] for key in edges if key[0] == prev and key[2] == node.index
            )
            model.addConstr(
                outgoing == incoming, name=f"state_continuity_{slot}_{node.index}"
            )

    t_sup_total = model.addVar(lb=0.0, name="t_sup_total")
    r_actual = model.addVar(lb=0.0, name="r_actual")
    d_total = model.addVar(lb=0.0, name="d_total")
    d_high = model.addVar(lb=0.0, name="d_high")
    h_short = model.addVar(lb=0.0, name="h_short")

    model.addConstr(
        t_sup_total
        == gp.quicksum(edge.segment.t_sup * y[key] for key, edge in edges.items()),
        name="support_time_total",
    )
    model.addConstr(
        t_sup_total == m * params.update_period_s / 2.0, name="support_half_cycle"
    )
    model.addConstr(
        r_actual
        == gp.quicksum(edge.segment.stride * y[key] for key, edge in edges.items()),
        name="r_actual",
    )
    model.addConstr(
        d_total
        == gp.quicksum(
            (edge.segment.d_leg + edge.segment.d_sup) * y[key]
            for key, edge in edges.items()
        ),
        name="d_total",
    )
    model.addConstr(
        d_high
        == gp.quicksum(edge.segment.d_high * y[key] for key, edge in edges.items()),
        name="d_high",
    )
    model.addConstr(
        h_short >= params.ratio_high_target * d_total - d_high,
        name="high_ratio_shortfall",
    )

    s = model.addVars(m, vtype=GRB.BINARY, name="s")
    u = model.addVars(m, vtype=GRB.BINARY, name="u")
    d = model.addVars(m, vtype=GRB.BINARY, name="d")
    for slot in range(m):
        model.addConstr(
            s[slot]
            == gp.quicksum(
                edge.segment.sup * y[key]
                for key, edge in edges.items()
                if key[0] == slot
            ),
            name=f"support_flag_{slot}",
        )
    for slot in range(m):
        prev = (slot - 1) % m
        model.addConstr(u[slot] >= s[slot] - s[prev], name=f"u_lb_{slot}")
        model.addConstr(u[slot] <= s[slot], name=f"u_s_{slot}")
        model.addConstr(u[slot] <= 1 - s[prev], name=f"u_prev_{slot}")
        model.addConstr(d[slot] >= s[prev] - s[slot], name=f"d_lb_{slot}")
        model.addConstr(d[slot] <= s[prev], name=f"d_prev_{slot}")
        model.addConstr(d[slot] <= 1 - s[slot], name=f"d_s_{slot}")
    model.addConstr(gp.quicksum(u[slot] for slot in range(m)) == 1, name="one_support_in")
    model.addConstr(gp.quicksum(d[slot] for slot in range(m)) == 1, name="one_support_out")

    objective = gp.quicksum(
        (
            params.lambda_e * edge.segment.e_cost
            + params.lambda_i * edge.segment.i_cost
            + params.lambda_c * edge.segment.c_cost
        )
        * y[key]
        for key, edge in edges.items()
    )
    objective += params.lambda_f * (r_actual - params.r_des) * (r_actual - params.r_des)
    objective += params.lambda_h * h_short
    model.setObjective(objective, GRB.MINIMIZE)
    model.update()
    return model, y, s, u, d, r_actual, d_total, d_high, h_short, edges


def extract_state_solution(
    model: gp.Model,
    y: dict[tuple[int, int, int], gp.Var],
    edges: dict[tuple[int, int, int], StateEdge],
    n_slots: int,
) -> list[StateEdge]:
    selected: list[StateEdge] = []
    for slot in range(n_slots):
        chosen = None
        for key, var in y.items():
            if key[0] == slot and var.X > 0.5:
                chosen = edges[key]
                break
        if chosen is None:
            raise RuntimeError(f"no state edge selected for slot {slot}")
        selected.append(chosen)
    selected.sort(key=lambda edge: edge.slot)
    return selected
