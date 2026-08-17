"""Command-line entry point for the candidate-segment gait optimizer."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
from pathlib import Path

from .analysis import compute_cycle_metrics
from .candidate import GaitParams, generate_candidate_library, precompute_segments
from .kinematics import Waypoint, build_trot_waypoints
from .make_preview import write_preview
from .servo_sim import simulate_continuous, simulate_interval
from .solver import Solution, build_model, extract_solution
from .state_graph import (
    build_state_edges,
    build_state_model,
    build_state_nodes,
    extract_state_solution,
)


def _parse_floats(text: str) -> tuple[float, ...]:
    return tuple(float(v.strip()) for v in text.split(",") if v.strip())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--l1", type=float, default=125.0)
    parser.add_argument("--l2", type=float, default=100.0)
    parser.add_argument("--base-x", type=float, default=-20.0)
    parser.add_argument("--base-y", type=float, default=54.0)
    parser.add_argument("--bias-angle", type=float, default=0.0)
    parser.add_argument("--step-height", type=float, default=45.0)
    parser.add_argument("--step-length", type=float, default=70.0)
    parser.add_argument("--speed-freq", type=float, default=0.125)
    parser.add_argument("--update-period-ms", type=float, default=60.0)
    parser.add_argument("--v-max", type=float, default=300.0)
    parser.add_argument("--a-max", type=float, default=6000.0)
    parser.add_argument("--eps-down", type=float, default=3.0)
    parser.add_argument("--eps-up", type=float, default=3.0)
    parser.add_argument("--h-high", type=float, default=25.0)
    parser.add_argument("--h-des", type=float, default=30.0)
    parser.add_argument("--ratio-high", type=float, default=0.6)
    parser.add_argument("--r-des", type=float, default=55.0)
    parser.add_argument("--lambda-e", type=float, default=2.0)
    parser.add_argument("--lambda-i", type=float, default=3.0)
    parser.add_argument("--lambda-f", type=float, default=2.0)
    parser.add_argument("--lambda-c", type=float, default=1.0)
    parser.add_argument("--lambda-h", type=float, default=2.0)
    parser.add_argument(
        "--x-steps",
        type=str,
        default="-6,0,6",
        help="comma-separated foot x perturbations in mm",
    )
    parser.add_argument(
        "--y-steps",
        type=str,
        default="-3,0,3",
        help="comma-separated foot y perturbations in mm",
    )
    parser.add_argument("--sim-dt-ms", type=float, default=1.0)
    parser.add_argument("--time-limit", type=float, default=60.0)
    parser.add_argument("--max-iterations", type=int, default=6)
    parser.add_argument("--velocity-tolerance", type=float, default=1e-3)
    parser.add_argument("--velocity-damping", type=float, default=0.5)
    parser.add_argument(
        "--state-aware",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="use state-aware candidate edges (default: enabled)",
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("tools/gait_optimizer/output"),
    )
    return parser


def _path_waypoints(path, cycle_phase: float):
    waypoints = [
        Waypoint(phase=cp.phase, x=cp.x, y=cp.y, hip_deg=cp.hip_deg, knee_deg=cp.knee_deg)
        for cp in path
    ]
    first = waypoints[0]
    waypoints.append(
        Waypoint(
            phase=first.phase + cycle_phase,
            x=first.x,
            y=first.y,
            hip_deg=first.hip_deg,
            knee_deg=first.knee_deg,
        )
    )
    return waypoints


def _write_target_csv(path: Path, waypoints: list[Waypoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["phase", "x_mm", "y_mm", "hip_deg", "knee_deg"])
        for wp in waypoints:
            writer.writerow(
                [round(wp.phase, 6), round(wp.x, 6), round(wp.y, 6), round(wp.hip_deg, 6), round(wp.knee_deg, 6)]
            )


def _write_sim_csv(path: Path, states, support_mask: list[bool]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "time_s",
                "hip_deg",
                "knee_deg",
                "hip_vel",
                "knee_vel",
                "hip_accel",
                "knee_accel",
                "x_mm",
                "y_mm",
                "support",
            ]
        )
        for state, flag in zip(states, support_mask):
            writer.writerow(
                [
                    round(state.time_s, 6),
                    round(state.hip_deg, 6),
                    round(state.knee_deg, 6),
                    round(state.hip_vel, 6),
                    round(state.knee_vel, 6),
                    round(state.hip_accel, 6),
                    round(state.knee_accel, 6),
                    round(state.x, 6),
                    round(state.y, 6),
                    int(flag),
                ]
            )


def _boundary_velocities(
    states, n_slots: int, period_s: float
) -> dict[int, tuple[float, float]]:
    result: dict[int, tuple[float, float]] = {}
    for i in range(n_slots):
        target = i * period_s
        state = min(states, key=lambda item: abs(item.time_s - target))
        result[i] = (state.hip_vel, state.knee_vel)
    return result


def _stitch_state_lists(edge_states, period_s: float):
    states = []
    for i, edge_states_i in enumerate(edge_states):
        offset = i * period_s
        if i < len(edge_states) - 1:
            states.extend(replace(s, time_s=s.time_s + offset) for s in edge_states_i[:-1])
        else:
            states.extend(replace(s, time_s=s.time_s + offset) for s in edge_states_i)
    return states


def main() -> int:
    args = build_parser().parse_args()
    params = GaitParams(
        l1=args.l1,
        l2=args.l2,
        base_x=args.base_x,
        base_y=args.base_y,
        bias_angle_deg=args.bias_angle,
        step_height=args.step_height,
        step_length=args.step_length,
        speed_freq=args.speed_freq,
        update_period_s=args.update_period_ms / 1000.0,
        v_max=args.v_max,
        a_max=args.a_max,
        eps_down=args.eps_down,
        eps_up=args.eps_up,
        h_high=args.h_high,
        h_des=args.h_des,
        ratio_high_target=args.ratio_high,
        r_des=args.r_des,
        lambda_e=args.lambda_e,
        lambda_i=args.lambda_i,
        lambda_f=args.lambda_f,
        lambda_c=args.lambda_c,
        lambda_h=args.lambda_h,
        sim_dt_s=args.sim_dt_ms / 1000.0,
    )

    base_waypoints = build_trot_waypoints(
        base_x=params.base_x,
        base_y=params.base_y,
        bias_angle_deg=params.bias_angle_deg,
        h=params.step_height,
        r=params.step_length,
        l1=params.l1,
        l2=params.l2,
        speed_freq=params.speed_freq,
    )
    candidate_library = generate_candidate_library(
        base_waypoints,
        x_steps=_parse_floats(args.x_steps),
        y_steps=_parse_floats(args.y_steps),
        l1=params.l1,
        l2=params.l2,
        hip_min_deg=params.hip_min_deg,
        hip_max_deg=params.hip_max_deg,
        knee_min_deg=params.knee_min_deg,
        knee_max_deg=params.knee_max_deg,
    )
    if args.state_aware:
        nodes_by_slot = build_state_nodes(candidate_library, params.v_max)
        edges = build_state_edges(candidate_library, nodes_by_slot, params)
        model, y, s, u, d, r_actual, d_total, d_high, h_short, edge_map = (
            build_state_model(
                candidate_library,
                nodes_by_slot,
                edges,
                params,
                time_limit=args.time_limit,
                output_flag=not args.quiet,
            )
        )
        model.optimize()
        if model.Status != 2:
            print(f"optimization status: {model.Status}")
            if model.Status in (3, 4):
                print("model is infeasible or unbounded; check support duration and candidate library")
            return 1

        selected_edges = extract_state_solution(
            model, y, edges, len(candidate_library)
        )
        path = [edge.start.candidate for edge in selected_edges]
        solution = Solution(
            status=model.Status,
            objective=model.ObjVal,
            selected_segments=[],
            path=path,
        )
        waypoints = _path_waypoints(path, 1.0)
        edge_states = [
            simulate_interval(
                edge.start.candidate.hip_deg,
                edge.start.candidate.knee_deg,
                edge.end.candidate.hip_deg,
                edge.end.candidate.knee_deg,
                duration_s=params.update_period_s,
                v_max=params.v_max,
                a_max=params.a_max,
                dt=params.sim_dt_s,
                l1=params.l1,
                l2=params.l2,
                start_hip_vel=edge.start.hip_vel,
                start_knee_vel=edge.start.knee_vel,
            )
            for edge in selected_edges
        ]
        states = _stitch_state_lists(edge_states, params.update_period_s)
        velocity_delta = max(
            max(
                abs(edge_states[i][-1].hip_vel - nxt.start.hip_vel),
                abs(edge_states[i][-1].knee_vel - nxt.start.knee_vel),
            )
            for i, (edge, nxt) in enumerate(
                zip(selected_edges, selected_edges[1:] + selected_edges[:1])
            )
        )
        iterations = 1
    else:
        cycle_phase = 1.0
        wrap_base = base_waypoints + [
            Waypoint(
                phase=cycle_phase,
                x=base_waypoints[0].x,
                y=base_waypoints[0].y,
                hip_deg=base_waypoints[0].hip_deg,
                knee_deg=base_waypoints[0].knee_deg,
            )
        ]
        base_states = simulate_continuous(
            wrap_base,
            l1=params.l1,
            l2=params.l2,
            v_max=params.v_max,
            a_max=params.a_max,
            dt=params.sim_dt_s,
        )
        entry_velocities = _boundary_velocities(
            base_states, len(candidate_library), params.update_period_s
        )

        solution = None
        states = []
        waypoints = []
        velocity_delta = float("inf")
        iterations = 0
        for iterations in range(1, args.max_iterations + 1):
            segments = precompute_segments(
                candidate_library, params, entry_velocities=entry_velocities
            )
            model, y, s, u, d, r_actual, d_total, d_high, h_short, segment_map = build_model(
                candidate_library,
                segments,
                params,
                time_limit=args.time_limit,
                output_flag=not args.quiet,
            )
            model.optimize()
            if model.Status != 2:
                print(f"optimization status: {model.Status}")
                if model.Status in (3, 4):
                    print("model is infeasible or unbounded; check support duration and candidate library")
                return 1

            solution = extract_solution(model, y, candidate_library, segment_map)
            waypoints = _path_waypoints(solution.path, cycle_phase)
            start_hip_vel, start_knee_vel = entry_velocities[0]
            states = simulate_continuous(
                waypoints,
                l1=params.l1,
                l2=params.l2,
                v_max=params.v_max,
                a_max=params.a_max,
                dt=params.sim_dt_s,
                start_hip_vel=start_hip_vel,
                start_knee_vel=start_knee_vel,
            )
            new_entry_velocities = _boundary_velocities(
                states, len(candidate_library), params.update_period_s
            )
            velocity_delta = max(
                max(
                    abs(new_entry_velocities[i][0] - entry_velocities[i][0]),
                    abs(new_entry_velocities[i][1] - entry_velocities[i][1]),
                )
                for i in range(len(candidate_library))
            )
            alpha = args.velocity_damping
            entry_velocities = {
                i: (
                    alpha * new_entry_velocities[i][0]
                    + (1.0 - alpha) * entry_velocities[i][0],
                    alpha * new_entry_velocities[i][1]
                    + (1.0 - alpha) * entry_velocities[i][1],
                )
                for i in range(len(candidate_library))
            }
            if velocity_delta <= args.velocity_tolerance:
                break

    metrics = compute_cycle_metrics(states, params)
    half_cycle_s = params.update_period_s * round(1.0 / params.speed_freq) / 2.0
    if abs(metrics.t_sup_s - half_cycle_s) > 1e-3:
        print(
            f"warning: continuous support time {metrics.t_sup_s:.3f}s "
            f"does not match target {half_cycle_s:.3f}s"
        )
    if metrics.support_blocks != 1:
        print(
            f"warning: continuous support blocks = {metrics.support_blocks}; "
            "expected one contiguous block"
        )
    if metrics.high_ratio + 1e-3 < params.ratio_high_target:
        print(
            f"warning: high ratio {metrics.high_ratio:.3f} is below target "
            f"{params.ratio_high_target}"
        )
    if velocity_delta > args.velocity_tolerance:
        print(
            f"warning: velocity state jump = {velocity_delta:.3f} deg/s "
            "across discrete state bins"
        )
    support_mask = [
        state.y - params.base_y <= params.eps_down for state in states
    ]

    output_dir = args.output_dir
    target_csv = output_dir / "target_waypoints.csv"
    sim_csv = output_dir / "simulated_cycle.csv"
    _write_target_csv(target_csv, waypoints)
    _write_sim_csv(sim_csv, states, support_mask)
    preview_path = output_dir / "preview.html"
    write_preview(
        target_csv,
        sim_csv,
        preview_path,
        l1=params.l1,
        l2=params.l2,
        base_y=params.base_y,
        h_high=params.h_high,
        eps_down=params.eps_down,
    )

    print(f"objective = {solution.objective:.6f}")
    print(f"iterations = {iterations}, velocity_delta = {velocity_delta:.6f}")
    print(f"selected path = {', '.join(f'({cp.x:.1f},{cp.y:.1f})' for cp in solution.path)}")
    print(f"support blocks = {metrics.support_blocks}, t_sup = {metrics.t_sup_s:.3f}s")
    print(f"r_actual = {metrics.r_actual_mm:.3f} mm, target = {params.r_des} mm")
    print(f"high ratio = {metrics.high_ratio:.3f}, target = {params.ratio_high_target}")
    print(f"wrote {target_csv}")
    print(f"wrote {sim_csv}")
    print(f"wrote {preview_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
