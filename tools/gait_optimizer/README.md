# Gait Candidate-Segment Optimizer

Python implementation of the candidate-segment trajectory optimizer described in
`docs/gait_trajectory_optimization_design.md`.

## Run

```powershell
python -m tools.gait_optimizer.cli --quiet --time-limit 30
```

Output:

- `tools/gait_optimizer/output/target_waypoints.csv`
- `tools/gait_optimizer/output/simulated_cycle.csv`
- `tools/gait_optimizer/output/preview.html`

## Tests

```powershell
python -m unittest tools.gait_optimizer.tests.test_gait_optimizer
```

## State-aware candidate graph

The default model uses a discrete velocity-state graph: nodes are
`candidate point + hip velocity bin + knee velocity bin`, and an edge is kept
only when the simulated exit velocity lands in the next node's velocity bins.
The CLI stitches the selected edge trajectories and validates support time
against the Gurobi model.

The velocity bins are discrete, so the CLI reports the largest velocity jump
across state boundaries. The default is 11 bins per joint. If the jump is too
large for the target servo, increase the bin count or switch back to the
non-state-aware fallback with `--no-state-aware`.

See `docs/gait_optimizer_checklist.md`, step 8.
