# Phase 3 design: dynamic convex-hull footprint (baseline B4)

**Date:** 2026-10-04
**Branch:** `dynamic-footprint` (from `moveit-arm` at `38d2221`)
**Roadmap:** Phase 3, baseline B4 (docs/roadmap.md Sections 2 and 8)

## 1. Purpose and scope

Phase 3 implements and qualifies the configuration-dependent Nav2 footprint of Sagar, Long
and Garcia Santiago (CoDIT 2026, DOI 10.1109/CoDIT70676.2026.11630999): at each update the
arm geometry is projected onto the ground plane, and the convex hull of base, arm and (for
this thesis) the attached panel becomes the Nav2 footprint.

Phase 3 is an implementation and qualification phase, not a full baseline campaign. On
the B3 missions the hull of the panel in `home` or `vertical_carry` is nearly the
rectangle of the matching B3 profile, so rerunning all 27 B3 missions would reproduce B3.
The value of the hull is generality (any arm or panel pose, no profile), which the later
phases need: B4 as a comparison row on the Phase 4-5 scenarios, the live demo with
arbitrary goals, and the geometry library for the Phase 4 MPC. Phase 3 therefore delivers
B4 as a selectable configuration, a short regression on the B3 missions, and a small
reproduction of the paper's results on this robot.

### Decisions taken in the design discussion

| Topic | Decision |
|---|---|
| Arm behaviour | As B3: the arm reconfigures only at standstill. The hull follows the arm during a reconfiguration and stays fixed while driving. The paper's simulation experiments also hold the arm still per run. |
| B4 status | A launch switch (`footprint_mode:=dynamic`) plus a regression run, not a rerun campaign. Later phases run B4 on their own scenarios. |
| Paper reproduction | A small Table II (update cost) and Table III (footprint strategies) on this robot. |
| Footprint models | Mesh hull (URDF collision primitives plus panel) by default; the paper's disc model as an option, compared offline. |
| Padding | Exact outward polygon offset instead of the paper's centroid-radial vertex push (Section 3). |
| Architecture | A geometry library independent of ROS plus a standalone `dynamic_footprint_node` that owns the footprint topics (approach 1 of 3; Section 2). |
| New B4 missions | None. The overhead course and the demo exercise the hull's generality in the later phases. |

### Out of scope

- Arm motion while the base drives (Phases 4-5).
- Footprint prediction from future arm poses (the paper's future work; roadmap Phase 5).
- A custom costmap layer for the robot's own shape (roadmap decision 6: use Nav2's native
  footprint topic).
- Lifecycle activation of the footprint node at runtime (the mode is chosen at launch).
- Octomap or other perceived obstacles in MoveIt (pre-Phase-4 checklist).

### Paper facts this design relies on

- Disc model (the paper's main method): base polygon B, a disc of calibrated radius r_i at
  the projected origin of each selected link, and discs at N_s samples along each segment
  between consecutive link origins; the convex hull of all disc points (Jarvis march);
  each hull vertex pushed by δ away from the vertex centroid (their Eq. 8). If fewer than
  3 points exist, it falls back to B.
- Mesh variant: convex hull of projected URDF collision geometry (higher fidelity, higher
  cost; shown only qualitatively, their Fig. 6).
- Published to `/global_costmap/footprint` and `/local_costmap/footprint` at 20 Hz from a
  lifecycle node reading `/joint_states` through TF.
- Table II: 689-857 µs mean, up to 2.7 ms max, 5-20 hull vertices from about 1000-1200
  points, 5000 ticks at 50 ms.
- Table III (Gazebo, 10 trials, SmacPlannerLattice + MPPI): static base-only 20-30 %
  success (arm collisions), static enlarged 40-50 % (over-conservative, blocked paths),
  dynamic 90 % (remaining failures goal or controller infeasibility). In the tucked
  configuration the dynamic footprint equals the base footprint.
- They note that reliable Nav2 operation with a changing footprint needed careful tuning
  of planner, controller, costmaps and recoveries.
- Code: github.com/KeerthiSagarSN/cs4mt_arm_dynamic_footprint_nav2 (ROS 2 Humble, C++).
  It is a reference only; this project implements its own C++ in its own packages.

## 2. Architecture

```text
/joint_states --> dynamic_footprint_node --(hull, padded)--> /global_costmap/footprint
                        |                                    /local_costmap/footprint
                        |                                    collision monitor zone topics
                        +--> /dynamic_footprint/footprint (state, RViz, mission checks)
                        uses mobile_manipulator_geometry (no ROS graph)
```

Exactly one component publishes footprints and monitor zones in a run, as for `/cmd_vel`.
The `footprint_mode` launch argument selects it:

| `footprint_mode` | Footprint and zone publisher | Used for |
|---|---|---|
| `static` | none; the profile given at launch stays | B1/B2, reproduction strategies `base_only` and `enlarged` |
| `profiles` | `ReconfigurePanel` (unchanged) | B3 |
| `dynamic` | `dynamic_footprint_node` | B4 |

A preflight check, like `check_cmd_vel_ownership`, fails the run when the publishers on the
footprint or zone topics do not match the mode.

Rejected alternatives: the hull inside `ReconfigurePanel` (the footprint would follow the
arm only while that MoveIt server runs, and the demo and later phases may not run it); a
custom costmap layer (contradicts roadmap decision 6 and the paper's "no Nav2 modification").

## 3. Geometry library `mobile_manipulator_geometry`

A new ament_cmake package: C++, Eigen, urdf, no rclcpp. It takes over the robot model from
`mobile_manipulator_navigation`, without behaviour change:

- `Shape`, `Primitive`, `Payload`, `load_primitives` (now in `lidar_robot_filter.hpp`);
- `FootprintProjector`, `contains`, `rpy_of`, `payload_from_json` (`footprint_projection.hpp`);
- the `Polygon` type (`scenario_spec.hpp`).

The lidar filter keeps its ray logic and uses the shared model. `mobile_manipulator_navigation`
and `mobile_manipulator_manipulation` link the new library; the existing projector and
filter tests move with the code and must pass unchanged.

New functions:

- `convex_hull(points)`: counter-clockwise hull, Andrew's monotone chain (deterministic
  with collinear points; same result as the paper's Jarvis march).
- `offset_outward(convex_polygon, delta)`: every edge moved outward by `delta` along its
  normal and adjacent edges re-intersected; where the mitre would extend more than
  `2 * delta` from the original vertex, the corner is bevelled. Every edge then lies
  exactly `delta` from the hull. The paper's centroid-radial push gives an edge only
  `delta * cos(angle)` of clearance, which on a long thin panel hull is far less than
  `delta`. Default `delta` = 0.02 m, the B3 profiles' perimeter allowance, so B3 and B4
  carry identical padding. Nav2's own `footprint_padding` (0.01 m) is added on top, as in B3.
- Footprint models behind one interface, `Polygon footprint(const JointMap &)`:
  - **Mesh hull** (default; the paper's URDF variant): hull of every collision primitive
    and the panel box, then the offset. Cylinders use a circumscribed polygon with a
    configurable number of sides (default 16 for footprints; the lidar filter keeps 64).
  - **Disc hull** (the paper's main method): base rectangle from the base collision box,
    discs of radius r_i at the projected link origins, discs at N_s segment samples, plus
    the projected corners of the panel box (the paper's disc model has no payload), then
    the offset. Radii are configuration, not hand-tuned: a test requires the disc hull to
    contain the mesh hull over sampled qualified poses.

## 4. `dynamic_footprint_node`

A plain rclcpp C++ node in `mobile_manipulator_navigation`.

- Input: `/joint_states`.
- A 20 Hz timer computes `footprint(joints)` with the configured model (`model:=mesh|disc`).
- It publishes only when the padded hull differs from the last published one by more than
  0.01 m (symmetric Hausdorff distance between the polygons). While the base drives the
  arm is still, so there are no updates; during a reconfiguration Nav2 re-inflates at most
  once per costmap update cycle, and this cost is measured (Section 7).
- Outputs, all latched (reliable, transient local) as `ReconfigurePanel` publishes today:
  `/global_costmap/footprint` and `/local_costmap/footprint` (`geometry_msgs/Polygon`);
  each collision monitor zone as `offset_outward(hull, zone margin)` on its
  `dynamic_polygon_topic`; `/dynamic_footprint/footprint` (`PolygonStamped`, stamped with
  the joint state used).
- Before the first complete joint state it publishes nothing; the costmaps keep the launch
  footprint, which in dynamic mode is the scenario's profile for its `arm_pose`.
- If an arm joint is missing or joint states are older than 0.5 s (the arm hardware's
  feedback timeout), it keeps the last footprint and warns; it never shrinks the footprint
  without data. (The paper's fallback to the bare base would hide the attached panel.)
- The costmap self-repair (`heal_footprints`, review minor 14: republish when a costmap
  shows another footprint for 2 s at standstill) moves here from `ReconfigurePanel`; in
  `profiles` mode `ReconfigurePanel` keeps its own copy.
- Timing: per-tick compute time, point count and hull vertex count; written as a stats
  file (path parameter) on shutdown for the runner.

## 5. `ReconfigurePanel` in dynamic mode

A `footprint_mode` parameter (`profiles` default). `profiles` is B3, unchanged. In `dynamic`:

- It publishes no footprints or zones and does not run the self-repair. It ignores the
  goal's `footprint_profile` and reports `applied_footprint_profile: "dynamic"`.
- The profile containment check becomes a hull check against the global costmap
  (`/global_costmap/costmap`): the planned final state's padded hull, posed at the current
  base pose, must contain no lethal cell, or the goal ends with the new result
  `HULL_IN_COLLISION` before the arm moves. Rationale: MoveIt plans in 3D, Nav2 judges in
  2D. A pose that is safe in 3D (panel above a low box) can leave Nav2 reporting "start
  occupied", and the check finds that before the arm moves. Each refusal is a measured
  false infeasibility of the 2D footprint.
- After execution, the same check runs on the measured state (the counterpart of B3's
  measured containment). A failure ends the goal with `HULL_IN_COLLISION`.
  `profile_violated` is always false in dynamic mode: the footprint follows the arm, so a
  failed or canceled execution cannot leave a stale footprint.
- If no costmap has been received, the check fails closed with a clear message.
- Interface additions (append-only, B3 clients unaffected): result code
  `HULL_IN_COLLISION=11`, result fields `planned_hull_clearance_m` and
  `measured_hull_clearance_m` (distance from the padded hull to the nearest lethal cell;
  NaN in `profiles` mode). In dynamic mode the containment-margin fields are NaN.

## 6. Mission task, runner and summarizer

- Mission task: before each drive it checks that both costmaps show the expected footprint
  and waits one full costmap cycle (existing). In dynamic mode the expected footprint is
  the latest `/dynamic_footprint/footprint`, compared with `footprint_matches` and Nav2's
  padding.
- Runner (`tools/run_nav_scenario.py`): `--footprint-mode static|profiles|dynamic`,
  `--footprint-model mesh|disc`, `--footprint-profile NAME` (overrides the scenario's
  profile in static mode, for `base_only` and `enlarged`). The mode, model and profile are
  recorded in each run's JSON. In dynamic mode the start-placement check uses the
  library's hull for the scenario's `arm_pose`.
- Runner contact handling: during a drive it polls Unity's scenario contacts and cancels
  the Nav2 goal at the first contact, scoring the drive `collision`, so a panel against a
  gate post does not leave Unity wedged for the next run.
- New profiles in `footprint_profiles.yaml`: `base_only` (base collision box plus 0.02 m)
  and `enlarged` (axis-aligned envelope of all qualified poses plus 0.02 m), generated and
  checked by `test_footprint_profiles.py` like the existing ones.
- Summarizer: groups by footprint mode, model and profile; adds the `collision` failure
  mode, footprint update count and compute time, and `HULL_IN_COLLISION` refusals.
- Missions are unchanged. The ±0.01 m vertical-carry tolerances stay for comparability
  with B3, although the hull no longer needs them.

## 7. Experiments

1. **Regression.** `narrow_gate_mission`, `wide_gate_mission`,
   `constrained_reconfiguration_mission` with `--footprint-mode dynamic`, RPP, 3 runs each
   (9 runs). Expected: B3's RPP result (9/9, no contact).
2. **Table III reproduction.** Arm held in `home` (the paper's "extended": the 1.24 m panel
   overhangs the base sideways) or `vertical_carry` (close to "tucked") × strategies
   `base_only`, `enlarged` (both static) and `dynamic` × the existing 1.05 m and 1.30 m
   gate drives (posts 2.4 m tall). Per cell: one plan-only query per planner the gate
   scenarios use (false infeasibility) and 5 drives with RPP (60 drives). Expected:
   base-only with `home` drives into the gate and touches (unsafe); enlarged with
   `vertical_carry` refuses or detours around a gate the robot fits (over-conservative);
   dynamic does both correctly. RPP rather than MPPI because stock MPPI's `off_goal`
   endings would blur the footprint effect.
3. **Table II reproduction and disc vs mesh.** An offline benchmark (no Unity) sweeps the
   qualified poses for both models: compute time (mean, min, max, p99), point and vertex
   count, hull area, and disc-to-mesh area ratio. The node's stats from experiments 1-2
   give the live numbers. The disc model also gets plan-only rows at both gates.

Metrics per run: success and failure mode, contact, path length, minimum clearance, time
through the gate, footprint update count and compute time, `HULL_IN_COLLISION` refusals.

## 8. Error handling

| Condition | Behaviour |
|---|---|
| URDF or payload file does not parse | `dynamic_footprint_node` exits at startup |
| Arm joint missing or joint states stale > 0.5 s | keep last footprint, warn, never shrink |
| No joint state yet | publish nothing; launch footprint stays |
| No global costmap in `ReconfigurePanel` hull check | fail closed with a message |
| Footprint or zone publishers do not match `footprint_mode` | preflight fails the run |

## 9. Testing

- Library (gtest): hull contains every point, is convex and counter-clockwise; every edge
  of `offset_outward` lies at least `delta` from the hull; for `home` and `vertical_carry`
  the mesh hull lies inside the matching B3 profile; the disc hull contains the mesh hull
  over sampled qualified poses; a timing benchmark (Section 7, experiment 3).
- Node (gtest, injected joint states and clock): publishes only on a change above 0.01 m;
  holds on stale or missing joints; silent before the first joint state; zones equal
  `offset_outward(hull, margin)`; self-repair.
- Config and launch (pytest): each `footprint_mode` gives exactly one owner; the
  `base_only` and `enlarged` profiles match their generated bounds.
- Mock hardware (`test_reconfigure_mock.py`): dynamic-mode reconfiguration, and a
  `HULL_IN_COLLISION` refusal with a scenario box under the planned panel.
- Mission task: unit test of the dynamic expected-footprint check.

## 10. Risks verified first

Nav2 Jazzy behaviours this design relies on, checked in the Nav2 source and with a short
live test before anything builds on them:

1. Smac Lattice uses a changed footprint on the next plan.
2. The inflation layer re-inflates when the footprint changes.
3. The collision monitor's `polygon_sub_topic` accepts non-rectangular polygons.
4. Nav2's `footprint_padding` (each vertex moved by the sign of its x and y) still
   contains the hull for many-vertex polygons.

If any fails, work stops for a decision with the user before a workaround.

## 11. Documentation

- `docs/experiments/dynamic-footprint/README.md` with the run data.
- ADR 0010: footprint ownership, the `footprint_mode` switch, the hull check, and the
  exact offset in place of the paper's radial padding.
- Roadmap: decision-log entry, Phase 3 section updated, B4 marked implemented in the
  baseline matrix; ADR 0009's "revisit when Phase 3 replaces profile switching" answered.
- `docs/running-the-stack.md`: the new launch and runner options.

## 12. Order of work

1. Verify the Nav2 behaviours of Section 10.
2. Move the robot model and projector into `mobile_manipulator_geometry` (no behaviour change).
3. Convex hull, outward offset, mesh and disc footprint models, with tests and the benchmark.
4. `dynamic_footprint_node`, the `footprint_mode` launch switch, and the ownership preflight.
5. `ReconfigurePanel` dynamic mode and the hull check (interface additions, mock test).
6. Mission task, runner and summarizer changes; `base_only` and `enlarged` profiles.
7. Live smoke test in Unity, then the regression (experiment 1).
8. Table III reproduction (experiment 2) and Table II / disc-vs-mesh (experiment 3).
9. Documentation (Section 11).
