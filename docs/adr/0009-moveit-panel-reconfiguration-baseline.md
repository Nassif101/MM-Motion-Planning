# ADR 0009: MoveIt panel reconfiguration at standstill (baseline B3)

Date: 2026-10-03. Status: accepted for Phase 2; amended 2026-10-04 (Phase 3 adds the dynamic footprint beside the profiles, ADR 0010).

## Context

Phase 2 adds collision-aware arm planning to the Nav2 baseline: Nav2 keeps the base and
`/cmd_vel` (ADR 0006), MoveIt plans the arm with the 3 kg panel attached. The roadmap leaves
open what the arm does during navigation, how its goals are chosen, which footprint Nav2
uses after the arm moves, and where MoveIt's collision world comes from. The B3 baseline must
stay comparable with Phase 3 (dynamic footprint) and the later whole-body planners on the
same scenarios. Design: `docs/superpowers/specs/2026-10-02-phase2-moveit-b3-design.md`.

## Options considered

- Arm motion: (A) reconfigure at standstill, then drive; (B) move the arm while driving
  (MoveGroup replanning, Servo, or whole-body MoveIt); (C) A plus a manipulation task at the goal.
- Goals: named transport poses only, or computed panel-pose goals with tolerances.
- Trigger: scripted scenario steps, route analysis, or retry after a Nav2 failure.
- Nav2 footprint after a reconfiguration: fixed for the run, switched between named profiles at
  standstill, or computed from the reached pose.
- MoveIt world: known geometry, lidar Octomap, or both.
- Structure: an action server, logic inside the scenario runner, or Nav2 behaviour-tree nodes.

## Decision

- **Reconfigure at standstill (A).** B is dropped: arm motion while driving is what the Phase 4-5
  whole-body planners do properly, and the panel was only qualified with the arm held still.
- **Computed panel-pose goals** (panel centre in `base_footprint`, rotation-vector tolerances) and
  named SRDF states for plain restores.
- **Scripted mission steps** in `scenarios.yaml` (`task: mission`), run by `mission_scenario_task`.
- **Named footprint profiles switched at standstill.** `ReconfigurePanel` projects the planned and
  then the measured robot + panel onto the ground, requires both inside the requested profile, and
  publishes it to both costmaps and the collision monitor's stop and slowdown zones (profile plus
  their margins; `navigation.launch.py footprint_mode:=profiles`, named `dynamic_monitor_zones:=true` until 2026-10-04).
- **Known geometry first** (exported Unity boxes inflated by 0.05 m, scenario boxes, a floor raised
  to the 0.15 m panel clearance); a lidar Octomap is a later scene source.
- **A C++ `ReconfigurePanel` action server** (`mobile_manipulator_manipulation`) around MoveGroup
  (OMPL RRTConnect, TOTG + Ruckig, 0.5 scaling) and the existing arm JTC; it never commands the base.

## Consequences

- B3 has one seam (`ReconfigurePanel`) that Phase 3 keeps: B4 replaces "switch to a named
  profile" with the dynamic convex hull and runs the same missions.
- The fixed-footprint limitation is measurable: a vertical-carry goal paired with the
  `vertical_carry` profile only has +/-0.01 m and +/-0.01 rad of freedom (0.02 m allowance), and a
  footprint switch must also reach the collision monitor's zones or the monitor keeps the old size.
- Everything that sizes itself from the footprint must follow the switch: costmaps (runtime
  footprint topic) and the collision monitor zones (dynamic polygon topics). The mission task
  checks both before each drive.
- MoveIt-specific behaviour the baseline relies on: request scaling 0 means 1.0 (the server sends
  the configured factors); XYZ-Euler tolerances on a subframe are dropped silently (rotation-vector
  tolerances, and the server re-checks the planned panel pose); canceling `execute_trajectory`
  does not stop the arm (the server also publishes `stop` on `/trajectory_execution_event`);
  canceling a `move_action` goal can abort move_group (the server lets a plan finish instead).
- CHOMP post-processing is not in the pipeline; it is an open B3 variant (roadmap open decisions).

## Validation

- Unit, configuration and mock-hardware tests: `mobile_manipulator_moveit_config`,
  `mobile_manipulator_manipulation` (incl. `test_reconfigure_mock.py`), `mobile_manipulator_navigation`.
- Unity: MoveIt-planned transitions met the arm acceptance limits in 19 of 20 runs, the 20th being
  a Unity feedback stall (`docs/experiments/moveit-arm`); B3 mission results are recorded in the same document.

## Revisit when

- Phase 3 replaces profile switching with the dynamic footprint.
- The arm must move while the base drives.
- Perceived obstacles must enter MoveIt's world (Octomap scene source).

## Amendment 2026-10-04: Phase 3

Phase 3 kept the `ReconfigurePanel` seam and added the dynamic footprint next to the profiles
rather than replacing them ([ADR 0010](0010-dynamic-footprint-ownership.md)): with
`footprint_mode:=profiles` (the default for missions) this ADR's profile switching is B3
unchanged; with `footprint_mode:=dynamic` (B4) `dynamic_footprint_node` owns the footprint and
`ReconfigurePanel` checks the planned and measured hull against the global costmap instead of a
profile. The goal's `footprint_profile` is then ignored.
