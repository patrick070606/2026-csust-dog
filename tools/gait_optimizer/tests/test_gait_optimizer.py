from __future__ import annotations

import math
import unittest

from ..analysis import compute_cycle_metrics
from ..candidate import GaitParams, generate_candidate_library, precompute_segments
from ..kinematics import (
    Waypoint,
    build_trot_waypoints,
    forward_kinematics,
    inverse_kinematics,
)
from ..servo_sim import simulate_continuous, simulate_interval
from ..solver import build_model, extract_solution
from ..state_graph import (
    build_state_edges,
    build_state_model,
    build_state_nodes,
    extract_state_solution,
)


class GaitOptimizerTests(unittest.TestCase):
    def setUp(self):
        self.params = GaitParams()
        self.base = build_trot_waypoints(
            base_x=self.params.base_x,
            base_y=self.params.base_y,
            bias_angle_deg=self.params.bias_angle_deg,
            h=self.params.step_height,
            r=self.params.step_length,
            l1=self.params.l1,
            l2=self.params.l2,
            speed_freq=self.params.speed_freq,
        )

    def test_base_waypoint_count(self):
        self.assertEqual(len(self.base), 8)

    def test_ik_fk_roundtrip(self):
        for wp in self.base:
            hip, knee = inverse_kinematics(wp.x, wp.y, self.params.l1, self.params.l2)
            x, y = forward_kinematics(hip, knee, self.params.l1, self.params.l2)
            self.assertAlmostEqual(x, wp.x, places=6)
            self.assertAlmostEqual(y, wp.y, places=6)

    def test_simulate_interval_is_finite(self):
        states = simulate_interval(
            self.base[0].hip_deg,
            self.base[0].knee_deg,
            self.base[1].hip_deg,
            self.base[1].knee_deg,
            duration_s=self.params.update_period_s,
            v_max=self.params.v_max,
            a_max=self.params.a_max,
            dt=self.params.sim_dt_s,
        )
        for state in states:
            self.assertTrue(all(math.isfinite(v) for v in vars(state).values()))

    def test_candidate_library_and_segments(self):
        library = generate_candidate_library(
            self.base,
            x_steps=(-6.0, 0.0, 6.0),
            y_steps=(-3.0, 0.0, 3.0),
            l1=self.params.l1,
            l2=self.params.l2,
            hip_min_deg=self.params.hip_min_deg,
            hip_max_deg=self.params.hip_max_deg,
            knee_min_deg=self.params.knee_min_deg,
            knee_max_deg=self.params.knee_max_deg,
        )
        self.assertEqual(len(library), 8)
        self.assertTrue(all(len(slot) > 0 for slot in library))
        segments = precompute_segments(library, self.params)
        self.assertEqual(len(segments), 648)

    def test_solver_end_to_end(self):
        library = generate_candidate_library(
            self.base,
            x_steps=(-6.0, 0.0, 6.0),
            y_steps=(-3.0, 0.0, 3.0),
            l1=self.params.l1,
            l2=self.params.l2,
            hip_min_deg=self.params.hip_min_deg,
            hip_max_deg=self.params.hip_max_deg,
            knee_min_deg=self.params.knee_min_deg,
            knee_max_deg=self.params.knee_max_deg,
        )
        segments = precompute_segments(library, self.params)
        model, y, _, _, _, _, _, _, _, segment_map = build_model(
            library, segments, self.params, time_limit=30.0, output_flag=False
        )
        model.optimize()
        self.assertEqual(model.Status, 2)
        solution = extract_solution(model, y, library, segment_map)
        self.assertEqual(len(solution.path), 8)

        cycle_phase = 1.0
        wrapped = solution.path + [solution.path[0]]
        waypoints = [
            Waypoint(
                phase=cp.phase,
                x=cp.x,
                y=cp.y,
                hip_deg=cp.hip_deg,
                knee_deg=cp.knee_deg,
            )
            for cp in wrapped
        ]
        waypoints[-1] = Waypoint(
            phase=cycle_phase,
            x=waypoints[0].x,
            y=waypoints[0].y,
            hip_deg=waypoints[0].hip_deg,
            knee_deg=waypoints[0].knee_deg,
        )
        states = simulate_continuous(
            waypoints,
            l1=self.params.l1,
            l2=self.params.l2,
            v_max=self.params.v_max,
            a_max=self.params.a_max,
            dt=self.params.sim_dt_s,
        )
        metrics = compute_cycle_metrics(states, self.params)
        self.assertTrue(metrics.t_cycle_s > 0.47)

    def test_state_graph_end_to_end(self):
        library = generate_candidate_library(
            self.base,
            x_steps=(-6.0, 0.0, 6.0),
            y_steps=(-3.0, 0.0, 3.0),
            l1=self.params.l1,
            l2=self.params.l2,
            hip_min_deg=self.params.hip_min_deg,
            hip_max_deg=self.params.hip_max_deg,
            knee_min_deg=self.params.knee_min_deg,
            knee_max_deg=self.params.knee_max_deg,
        )
        library = [slot[:5] for slot in library]
        nodes = build_state_nodes(library, self.params.v_max)
        edges = build_state_edges(library, nodes, self.params)
        self.assertTrue(len(edges) > 0)
        model, y, _, _, _, _, _, _, _, edge_map = build_state_model(
            library, nodes, edges, self.params, time_limit=30.0, output_flag=False
        )
        model.optimize()
        self.assertEqual(model.Status, 2)
        selected = extract_state_solution(model, y, edge_map, len(library))
        self.assertEqual(len(selected), 8)
        self.assertAlmostEqual(
            sum(edge.segment.t_sup for edge in selected),
            len(library) * self.params.update_period_s / 2.0,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
