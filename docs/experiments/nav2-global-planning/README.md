# Global planner gate checks

Date: 2026-09-28. Read-only `ComputePathToPose` queries (no motion) against the
Unity-exported `ConstructionSiteV1` map, with Unity in Play supplying TF.

```bash
ros2 launch mobile_manipulator_navigation global_planning.launch.py footprint_profile:=<profile>
ros2 run mobile_manipulator_navigation gate_planning_check.py --label <profile> \
  --output docs/experiments/nav2-global-planning/gates-<profile>.json
```

Planners: `GridBased` (NavFn A*, point planner using inscribed-radius inflation) and
`Lattice` (Smac State Lattice, 5 cm diff-drive primitives, full-footprint SE(2) checks).
Cases use explicit start/goal poses on opposite sides of each gate:

| Case | Start -> goal (ROS map) | Opening |
|---|---|---|
| `gate_1p05` | (-5.3, -7.725) -> (-9.0, -7.725), heading pi | y in [-8.25, -7.20] on x = -7.225 |
| `gate_1p35` | (-7.325, 2.5) -> (-7.325, -2.5), heading -pi/2 | x in [-8.00, -6.65] on y = 0 |

## Results

| Profile | Case | NavFn: length / through gate | Lattice: length / through gate |
|---|---|---|---|
| `home` | 1.05 m gate | 10.80 m / no (detour) | 10.89 m / no (detour) |
| `home` | 1.35 m gate | 7.16 m / no (detour) | 7.06 m / no (detour) |
| `vertical_carry` | 1.05 m gate | 3.91 m / yes | 3.71 m / yes |
| `vertical_carry` | 1.35 m gate | 5.03 m / yes | 5.00 m / yes |

All queries succeeded (error code 0) in at most 0.02 s.

## Interpretation

- `home` (1.24 m wide) cannot use the 1.05 m gate and avoids the 1.35 m gate (0.055 m
  per side before inflation costs); both planners detour. `vertical_carry` (0.77 m wide)
  traverses both gates directly, matching the whole-robot 0.15 m gate margin measured
  in arm qualification.
- In this scene the two planners agree on gate feasibility. The difference is
  guarantees, not these outcomes: NavFn checks only a disc of the inscribed radius
  (0.53 m for `home`) and emits headings that ignore the footprint, while Lattice
  collision-checks the full polygon at every primitive pose, so its path is feasible
  for the non-circular footprint at the planned heading. Feasibility is decided before
  the controller has to discover it.
- Because alternative routes exist, "avoided" is not the same as "rejected". A forced
  scenario (gate as the only route) belongs to the Phase 1 benchmark set.

An earlier run in this session returned `INVALID_PLANNER` and a NavFn path through the
1.35 m gate for `home`; both came from a stale `planner_server` with the previous
configuration still running beside the new one. Those results were discarded; the
tables above come from a single verified instance per profile.
