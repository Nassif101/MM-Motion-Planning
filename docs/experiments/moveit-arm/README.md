# MoveIt arm reconfiguration (Phase 2, baseline B3)

Design: [Phase 2 spec](../../superpowers/specs/2026-10-02-phase2-moveit-b3-design.md). MoveIt plans
collision-free arm motions for the attached 3 kg reference panel while the base is stopped;
the `ReconfigurePanel` action checks the robot + panel ground projection against the requested
Nav2 footprint profile before and after execution and then switches the costmap footprint.

## Transition qualification

**Question:** do MoveIt-planned transitions at the configured 0.5 velocity and acceleration
scaling stay inside the arm acceptance limits that were qualified for the explicit 8 s cubic
transitions (`mobile_manipulator_control/config/qualified_payload.json`)?

**Setup (2026-10-02):** Unity 6000.5.2f1 Editor in Play, `ConstructionSiteV1`, robot placed in
the open stability-test area (`arm_test_place_open`), fresh epoch (endpoint, Play and
`arm_control.launch.py` restarted), `manipulation.launch.py use_sim_time:=true`. Each case is
one `reconfigure_panel` goal recorded by Unity's arm recorder and judged by
`tools/analyze_arm_qualification.py` with the unchanged acceptance checks (path error
< 0.15 rad, hold error < 0.06 rad, no fault or watchdog, no panel penetration, panel bottom
> 0.15 m, base tilt < 3 deg, drive saturation < 2 %). Run with:

```bash
python3 tools/run_arm_qualification.py --container <container> --suite moveit --prefix moveit --repeats 5
```

The panel-pose case uses the vertical-carry panel centre from
`panel_pose --pose vertical_carry` with +/-0.01 m and +/-0.01 rad tolerances (see "Panel-pose
tolerances" below). Evidence: `qualification/` (gzipped CSV per case, the action record, the
`reconfigure_panel` result, and `acceptance.json`).

| Transition | Runs | Passed | Planning time median / max (s) | Trajectory (s) | Max path error (rad) | Max hold error (rad) | Min panel bottom (m) | Max base tilt (deg) |
|---|---:|---:|---|---|---:|---:|---:|---:|
| home -> vertical carry (named) | 5 | 5 | 0.02 / 0.04 | 1.07-2.65 | 0.101 | 0.015 | 0.713 | 0.03 |
| vertical carry -> home | 5 | 5 | 0.02 / 0.04 | 2.65 | 0.049 | 0.014 | 0.711 | 0.03 |
| home -> vertical panel pose (computed goal) | 5 | 5 | 0.14 / 0.24 | 2.08-2.89 | 0.051 | 0.015 | 0.580 | 0.04 |
| vertical carry -> home (second) | 5 | 4 | 0.02 / 0.02 | 2.04-2.90 | 0.077 | 0.021 | 0.574 | 0.03 |

**Result:** 19 of 20 passed; the scaling stays at 0.5. MoveIt transitions take 2-3 s against
the 8 s qualified cubic transitions, with at most 0.101 rad path error (limit 0.15) and 0.021 rad
hold error (limit 0.06).

**The failed case** (`moveit-vertical-to-home-2-2`) is a Unity feedback stall, not a planning or
tracking failure: Unity's arm command age reached 4.9 s 1.3 s into the motion, the Unity arm
entered `WATCHDOG_HOLD`, ros2_control deactivated the hardware, and the recovery supervisor
re-activated it about 1 s later. `ReconfigurePanel` reported `ARM_FAULT` and did not switch the
footprint; all physical checks passed (path error 0.030 rad, no penetration). It is the stall
path documented in ADR 0005 and the ROS-TCP stall experiment; this run used a freshly opened
Editor.

## B3 missions

**Question:** with the arm reconfigured by MoveIt at standstill and the Nav2 footprint switched
between named profiles, how do the three B3 missions perform with each Phase 1 controller?

**Setup (2026-10-03):** `run_nav_scenario.py` with `--new-epoch` per controller and three runs
of each mission (runs from 21:40 UTC on 2026-10-02; earlier runs were debugging the runner). Nav2
with the persistent global obstacle layer (default) and `dynamic_monitor_zones:=true`; MoveIt
with the known scene and the scenario's boxes. Summaries: `runs/` (one per run; bags, Unity arm
recordings and MoveIt logs stay in the git-ignored `experiment_runs/`). Table from
`python3 tools/summarize_nav_runs.py docs/experiments/moveit-arm/runs/*.json`; reconfiguration
columns cover all successful reconfigurations, planned clearance is to the 0.05 m-inflated known
boxes (floor excluded), panel bottom and base tilt are Unity ground truth over the whole mission.

| Mission | Controller | Success | Contact | Total s | Drives s | Reconfig. s | Planning s | Motion s | Planned clearance m | Path error rad | Hold error rad | Panel bottom m | Base tilt deg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| narrow_gate | RPP | 3/3 | 0 | 61.8 (58.6-62.9) | 48.4 (48.2-48.7) | 13.6 (9.8-14.6) | 0.07 (0.02-0.30) | 3.58 (2.04-5.01) | 0.705 (0.025-1.358) | 0.044 (0.037-0.068) | 0.015 (0.010-0.033) | 0.704 (0.491-0.707) | 0.12 (0.10-0.12) |
| narrow_gate | DWB | 2/3 | 0 | 74.7 (73.9-75.5) | 60.6 (58.5-62.7) | 14.1 (12.8-15.4) | 0.04 (0.02-0.36) | 3.77 (2.04-5.62) | 1.335 (0.021-1.499) | 0.068 (0.026-0.068) | 0.015 (0.006-0.015) | 0.516 (0.513-0.704) | 0.11 (0.10-0.14) |
| narrow_gate | MPPI | 3/3 | 0 | 61.1 (58.5-75.1) | 44.4 (44.4-63.6) | 14.1 (11.5-16.7) | 0.07 (0.02-0.22) | 3.34 (2.42-5.51) | 0.666 (0.009-1.250) | 0.055 (0.040-0.068) | 0.012 (0.008-0.015) | 0.544 (0.520-0.700) | 0.11 (0.11-0.12) |
| wide_gate | RPP | 3/3 | 0 | 57.8 (56.9-59.6) | 47.5 (47.4-47.9) | 10.5 (9.5-11.7) | 0.03 (0.02-0.08) | 2.73 (2.04-2.74) | 0.774 (0.165-1.495) | 0.052 (0.038-0.076) | 0.015 (0.013-0.020) | 0.701 (0.562-0.704) | 0.12 (0.10-0.18) |
| wide_gate | DWB | 3/3 | 0 | 72.4 (64.4-72.5) | 61.1 (54.7-62.6) | 9.8 (9.7-11.4) | 0.06 (0.02-0.38) | 2.24 (2.04-2.34) | 0.762 (0.161-1.373) | 0.039 (0.021-0.061) | 0.016 (0.015-0.021) | 0.703 (0.699-0.707) | 0.15 (0.14-0.18) |
| wide_gate | MPPI | 3/3 | 0 | 55.6 (54.3-59.4) | 43.8 (43.2-49.4) | 10.5 (9.9-12.4) | 0.08 (0.02-0.40) | 2.04 (2.03-2.78) | 0.741 (0.194-1.411) | 0.059 (0.040-0.066) | 0.015 (0.013-0.021) | 0.699 (0.565-0.700) | 0.12 (0.10-0.13) |
| constrained | RPP | 3/3 | 0 | 30.8 (26.7-30.9) | 18.6 (15.8-18.7) | 12.1 (10.9-12.3) | 0.06 (0.02-0.26) | 3.54 (2.16-4.05) | 0.831 (0.002-1.595) | 0.040 (0.037-0.055) | 0.015 (0.011-0.019) | 0.699 (0.697-0.709) | 0.11 (0.10-0.13) |
| constrained | DWB | 3/3 | 0 | 34.5 (27.8-37.4) | 23.7 (15.2-23.8) | 12.6 (10.6-13.7) | 0.05 (0.02-0.38) | 3.42 (2.73-4.95) | 0.747 (0.024-1.629) | 0.049 (0.042-0.070) | 0.015 (0.006-0.028) | 0.675 (0.532-0.703) | 0.12 (0.11-0.14) |
| constrained | MPPI | 2/3 | 0 | 61.7 (28.7-94.7) | 48.6 (15.1-82.1) | 13.1 (12.6-13.7) | 0.10 (0.02-0.22) | 4.99 (2.51-6.30) | 0.006 (0.000-1.937) | 0.057 (0.044-0.068) | 0.015 (0.014-0.038) | 0.565 (0.318-0.694) | 0.13 (0.10-0.15) |

Cells: median (range) over runs; times over successful missions. **25 of 27 missions succeeded, no
robot-environment contact, every reconfiguration succeeded** (52 of 52 requested before a mission
ended), and all Unity arm checks passed apart from the two failed missions' action status (panel
bottom >= 0.318 m against the 0.15 m limit, tilt <= 0.18 deg, path error <= 0.076 rad).

The two failures are drives, not reconfigurations:

- `narrow_gate_mission` DWB, run 3: the drive through the 1.05 m throat ended with FollowPath
  error 105 (failed to make progress) after 11 recoveries. Phase 1 recorded this throat as
  marginal even with the panel already in vertical carry (lidar noise narrows the opening).
- `constrained_reconfiguration_mission` MPPI, run 2: stock MPPI left the start beside the box on an
  8.4 m path for the 3 m drive, recovered 11 times and aborted (error 103) 3.3 m from the goal;
  stock MPPI is kept untuned as baseline B2.

**Gate crossing against Phase 1** (reconfiguration + drive from the staging pose 3 m before the
gate to 1.8 m after it, versus Phase 1 drives over the same start and goal):

| Gate | Controller | B3 reconfigure + cross s | Phase 1 home footprint (detour) s | Phase 1 panel already vertical s |
|---|---|---|---|---|
| 1.05 m | RPP | 35.2 (32.9-35.7), 3/3 | 45.5 (43.8-47.6), 4/4, 11.0 m path | 26.8 (26.7-26.8), 2/6 |
| 1.05 m | DWB | 44.2 (42.5-45.9), 2/3 | 45.4 (45.4-45.5), 3/3 | 34.0 (33.4-34.6), 2/3 |
| 1.05 m | MPPI | 36.1 (34.7-51.1), 3/3 | 43.2 (41.8-43.7), 3/3 | 27.0, 1/3 |
| 1.30 m | RPP | 33.2 (31.9-33.8), 3/3 | 45.7 (45.0-47.5), 4/4, 11.3 m path | 26.6 (26.4-26.7), 6/6 |
| 1.30 m | DWB | 41.6 (40.6-42.6), 3/3 | 44.3 (43.8-44.5), 3/3 | 27.3 (27.2-27.6), 5/5 |
| 1.30 m | MPPI | 33.1 (33.0-33.9), 3/3 | 41.5 (41.3-42.9), 3/3 | 27.1 (27.0-27.3), 5/5 |

Phase 1 cells pool the global-costmap modes recorded in `docs/experiments/nav2-navigation/runs`
(static, live and persistent); they are a reference, not a matched rerun. Reconfiguring at the
gate is 3-12 s faster than the home-footprint detour around it and about 7-15 s slower than
arriving with the panel already vertical: the price of a standstill reconfiguration (about 3 s of
motion plus settling, verification and the footprint switch).

**Arm configurations.** KDL returned a different arm configuration for every panel-pose goal (27
distinct configurations at 0.1 rad resolution over 27 vertical-panel reconfigurations), all with
the panel within +/-0.01 m and +/-0.01 rad of the goal and inside the `vertical_carry` profile.
Computed panel goals therefore do not repeat the joint path between runs; the reported motion
time (2.0-6.3 s) and planned clearance spread reflect that. A mission that stops mid-way leaves the
arm in such a configuration; the runner returns it to a qualified pose with MoveIt before the next
scenario.

**Constrained reconfiguration.** The qualified straight transition from home to vertical carry is in
collision with the tall box in 32 of its 50 samples (`constrained-scene/naive-transition-check.json`);
MoveIt planned around it in all 9 missions (planning 0.02-0.38 s). Planned clearance to the inflated
box was often below 0.01 m, i.e. about 0.05 m from the real box.

## Octomap scene source (experimental, not usable yet)

**Question:** can MoveIt plan B3 reconfigurations from the lidar alone (`--scene-source octomap`:
static boxes and floor known, scenario boxes only perceived through `/livox/points_filtered`)?

**Setup (2026-10-03):** `constrained_reconfiguration_mission` (the tall box beside the start, now not
given to MoveIt) and `octomap_blind_zone_mission` (a 0.18 m box 1.2 m from the start, inside the
documented near-field blind zone), RPP. `ReconfigurePanel` clears the Octomap and lets it refill at
standstill before each plan. `octomap_box_check` records the occupied voxels touching each scenario
box at the start. Summaries: `runs-octomap/`.

| Mission | Runs | Success | Voxels touching the box | Highest box voxel | Planned clearance m | Panel penetration (Unity) |
|---|---:|---:|---|---|---|---|
| constrained (tall box 1.6 m) | 5 | 0 | 9-16 (of ~2400 in the Octomap) | 1.125 m | 0.23-0.36 | all 5 (0.1-1.6 mm) |
| blind zone (low box 0.18 m) | 3 | 3 | 0 | - | 6.07-6.09 | none |

**Result.** The low box is never in the Octomap: below 0.2 m within 1.5 m the Livox does not see it
(the documented blind zone). It does not matter to B3, whose panel stays above the 0.15 m raised
floor (lowest panel bottom measured in all B3 missions: 0.32 m).

The tall box is a failure of the Octomap, not of the lidar. With the robot at the start, the
filtered cloud contains the whole box face (36,662 points over 60 clouds, heights 0.27-1.62 m, sensor
at 0.39 m) and nothing beyond it, but MoveIt's Octomap holds only 9-16 voxels touching the box and a
horizontal layer of about 2,170 voxels at 1.00-1.25 m spanning the box's x range and extending 5 m
away from the robot. MoveIt therefore planned through the real box (0.23-0.36 m apparent clearance),
the panel struck it in every run, and the arm controller aborted on its path tolerance. The cause
inside MoveIt's point-cloud insertion is not identified yet (roadmap open decision); until it is,
the Octomap scene source must not be used for experiments and the B3 baseline uses known geometry.

**Updater load.** At the full cloud rate the Octomap raised `move_group` from about 5 % to about 40 %
of a core, and arm tracking degraded even with nothing nearby (path errors 0.15-0.17 rad, aborts in
the blind-zone mission). The updater now takes every second point at 2 Hz with a 1.5 s refill; the
blind-zone missions then tracked within 0.03-0.06 rad.

## Findings for the B3 baseline

- **Time parameterization scaling.** A request scaling of 0 makes MoveIt's time
  parameterization fall back to 1.0, not to the configured 0.5 default; Ruckig smoothing then
  failed on home -> vertical carry. The server sends the configured factors with every request.
- **Panel-pose tolerances.** The vertical-carry profile leaves 0.02 m around the robot and panel.
  With +/-0.05 m and +/-0.1 rad the planner chose a tilted panel outside it and the server refused
  with `PROFILE_TOO_SMALL` before moving. Computed panel goals paired with a fixed named profile
  therefore have little freedom (+/-0.01 m, +/-0.01 rad); this is the fixed-footprint limitation
  Phase 3 addresses.
- **Subframe constraints.** MoveIt rejects XYZ-Euler orientation tolerances on the panel subframe
  and then drops the panel constraints with only a log message, planning to an arbitrary state.
  The goal uses rotation-vector tolerances, and the server re-checks the planned panel pose.
- **Cancel.** Canceling MoveIt's `execute_trajectory` action alone let the trajectory run to its
  end; the server also publishes `stop` on `/trajectory_execution_event`.
- **Everything sized from the footprint must follow the switch.** The collision monitor's stop and
  slowdown zones were fixed to the launch profile: after switching to vertical carry the home-sized
  stop zone still covered the 1.30 m gate posts and stopped the base (the costmaps and RPP were
  correct). Missions launch Nav2 with `dynamic_monitor_zones:=true`; `ReconfigurePanel` publishes the
  zones, and the mission task checks the costmap footprints and zone inputs before each drive.
- **Settling.** Nav2 reports success before the base is at rest; the mission task waits until the
  base has been still for 0.75 s (1.3 s median wait after the drive, at most 2.7 s) before reconfiguring, as the server requires 0.5 s.
