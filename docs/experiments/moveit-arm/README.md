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

**Setup (2026-10-03, commit `e4dbc1d`, clean tree):** `run_nav_scenario.py` with `--new-epoch` per
controller and three runs of each mission, 17:10-18:06 UTC. Nav2 with the persistent global
obstacle layer (default) and `dynamic_monitor_zones:=true`; MoveIt with the known scene and the
scenario's boxes. The lidar draws independent range noise per point and scan and the self-filter
uses a 6 sigma noise band ("Lidar noise and the self-filter" below); `ReconfigurePanel` re-plans
up to 3 times when MoveIt rejects its own smoothed plan. Every summary records the machine
(Apple M4 host, 10-CPU aarch64 container, Unity 6000.5.2f1, Cyclone DDS); the real-time factor was
1.00-1.04 and no run spanned a host sleep. Summaries: `runs/` (one per run; bags, Unity arm
recordings and MoveIt logs stay in the git-ignored `experiment_runs/`). Table from
`python3 tools/summarize_nav_runs.py docs/experiments/moveit-arm/runs/*.json`; reconfiguration
columns cover all successful reconfigurations, planned clearance is to the 0.05 m-inflated known
boxes (floor excluded), `move_group` CPU is per reconfiguration (percent of one core), panel bottom
and base tilt are Unity ground truth over the whole mission.

| Mission | Controller | Success | Contact | Total s | Drives s | Reconfig. s | Planning s | Motion s | Planned clearance m | Path error rad | Hold error rad | move_group CPU % | Panel bottom m | Base tilt deg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| narrow_gate | RPP | 1/3 | 0 | 56.6 | 47.0 | 9.6 | 0.07 (0.02-0.20) | 2.52 (2.50-5.11) | 1.378 (0.122-1.389) | 0.048 (0.038-0.084) | 0.015 (0.009-0.015) | 5.5 (3.3-6.4) | 0.704 (0.506-0.707) | 0.10 (0.10-0.17) |
| narrow_gate | DWB | 2/3 | 0 | 116.3 (65.3-167.3) | 103.6 (50.5-156.6) | 12.7 (10.7-14.7) | 0.04 (0.02-0.40) | 2.84 (2.73-4.69) | 1.382 (0.068-1.525) | 0.066 (0.062-0.072) | 0.015 (0.007-0.016) | 3.9 (3.6-13.0) | 0.570 (0.514-0.580) | 0.17 (0.12-0.18) |
| narrow_gate | MPPI | 3/3 | 0 | 59.6 (56.7-112.2) | 48.6 (44.8-101.9) | 11.0 (10.3-12.0) | 0.05 (0.02-0.28) | 2.22 (2.05-2.43) | 0.742 (0.128-1.305) | 0.041 (0.026-0.057) | 0.015 (0.008-0.021) | 4.6 (3.6-11.7) | 0.701 (0.700-0.702) | 0.14 (0.10-0.15) |
| wide_gate | RPP | 3/3 | 0 | 58.1 (56.0-61.1) | 46.3 (46.1-46.5) | 12.1 (9.5-14.7) | 0.05 (0.02-0.30) | 3.25 (2.05-4.64) | 0.821 (0.166-1.406) | 0.051 (0.037-0.069) | 0.015 (0.011-0.032) | 4.3 (3.5-11.9) | 0.706 (0.510-0.708) | 0.11 (0.10-0.12) |
| wide_gate | DWB | 3/3 | 0 | 56.7 (55.3-59.0) | 46.9 (46.1-47.0) | 9.8 (9.1-12.0) | 0.08 (0.02-0.40) | 2.11 (2.06-3.19) | 0.856 (0.277-1.411) | 0.046 (0.036-0.064) | 0.015 (0.011-0.020) | 5.9 (3.5-12.3) | 0.705 (0.700-0.709) | 0.11 (0.10-0.17) |
| wide_gate | MPPI | 3/3 | 0 | 60.0 (55.3-71.4) | 43.4 (43.2-58.6) | 12.8 (12.1-16.6) | 0.11 (0.02-0.42) | 3.11 (2.80-5.12) | 1.232 (0.233-1.462) | 0.067 (0.063-0.078) | 0.013 (0.002-0.015) | 5.3 (3.5-11.6) | 0.564 (0.511-0.575) | 0.13 (0.12-0.14) |
| constrained | RPP | 3/3 | 0 | 31.4 (29.0-31.5) | 15.9 (15.6-17.4) | 14.1 (13.1-15.9) | 0.06 (0.02-0.18) | 5.12 (3.09-5.68) | 0.745 (0.000-1.474) | 0.063 (0.054-0.065) | 0.014 (0.009-0.016) | 4.7 (3.7-6.5) | 0.512 (0.423-0.556) | 0.11 (0.10-0.11) |
| constrained | DWB | 3/3 | 0 | 28.6 (28.5-29.4) | 16.1 (15.9-16.2) | 12.6 (12.5-13.2) | 0.05 (0.02-0.30) | 3.87 (2.90-5.26) | 0.757 (0.004-1.626) | 0.049 (0.034-0.066) | 0.014 (0.006-0.016) | 4.7 (3.8-9.2) | 0.557 (0.550-0.629) | 0.09 (0.09-0.11) |
| constrained | MPPI | 2/3 | 0 | 35.7 (32.9-38.5) | 21.5 (17.6-25.4) | 14.2 (13.0-15.3) | 0.04 (0.02-0.24) | 4.11 (2.75-4.77) | 0.034 (0.009-1.586) | 0.060 (0.033-0.086) | 0.015 (0.004-0.016) | 4.3 (3.6-8.0) | 0.535 (0.375-0.552) | 0.13 (0.10-0.16) |

Cells: median (range) over runs; times over successful missions. **23 of 27 missions succeeded, no
robot-environment contact, every reconfiguration succeeded** (50 of 50 requested before a mission
ended, none needed a re-plan), `move_group` used 3-15 % of a core and 70-72 MB per
reconfiguration, and every Unity arm check passed in all 27 runs (the 4 failed missions fail only
the mission-status check; panel bottom >= 0.375 m against the 0.15 m limit, tilt <= 0.18 deg, Unity
path error <= 0.059 rad). No global-costmap frame of any run marked a cell within 0.3 m of the robot
centre (the self-marking of the earlier series, below).

The four failures are drives, not reconfigurations:

- `narrow_gate_mission` RPP, runs 2 and 3: the drive through the 1.05 m throat stopped 2.0-2.5 m
  short after 11-13 recoveries ("Controller patience exceeded", error 104); `narrow_gate_mission` DWB,
  run 3: the same throat, error 105 (failed to make progress) after 46 recoveries. In vertical carry
  the throat leaves 0.14 m per side; with independent 0.02 m range noise the lidar returns of the
  gate posts spread about 0.09 m into the opening (about 0.07 m with the earlier repeating noise),
  enough to put a local-costmap cell into the path now and then. The throat was already marginal in
  Phase 1 (5 of 9 runs with the panel pre-rotated).
- `constrained_reconfiguration_mission` MPPI, run 2: stock MPPI left the start beside the tall box on
  an 8.2 m path for the 3 m drive, recovered 11 times and aborted (error 103) 3.6 m from the goal, as
  in the earlier series; stock MPPI is kept untuned as baseline B2.

**Gate crossing against Phase 1** (reconfiguration + drive from the staging pose 3 m before the
gate to 1.8 m after it, versus Phase 1 drives over the same start and goal). The Phase 1 cells were
recorded with the repeating lidar noise and the 4 sigma filter band and pool the global-costmap
modes in `docs/experiments/nav2-navigation/runs`; they are a reference until Phase 1 is rerun.

| Gate | Controller | B3 reconfigure + cross s | Phase 1 home footprint (detour) s | Phase 1 panel already vertical s |
|---|---|---|---|---|
| 1.05 m | RPP | 32.7, 1/3 | 45.5 (43.8-47.6), 4/4, 11.0 m path | 26.8 (26.7-26.8), 2/6 |
| 1.05 m | DWB | 89.1 (38.6-139.7), 2/3 | 45.4 (45.4-45.5), 3/3 | 34.0 (33.4-34.6), 2/3 |
| 1.05 m | MPPI | 37.8 (33.7-60.8), 3/3 | 43.2 (41.8-43.7), 3/3 | 27.0, 1/3 |
| 1.30 m | RPP | 34.2 (32.6-35.1), 3/3 | 45.7 (45.0-47.5), 4/4, 11.3 m path | 26.6 (26.4-26.7), 6/6 |
| 1.30 m | DWB | 32.6 (32.5-34.2), 3/3 | 44.3 (43.8-44.5), 3/3 | 27.3 (27.2-27.6), 5/5 |
| 1.30 m | MPPI | 35.5 (34.0-51.6), 3/3 | 41.5 (41.3-42.9), 3/3 | 27.1 (27.0-27.3), 5/5 |

Reconfiguring at the 1.30 m gate takes 33-36 s against 42-46 s for the home-footprint detour and
27 s when the panel is already vertical: the price of a standstill reconfiguration is about 6-9 s
(about 2-3 s of motion plus settling, verification and the footprint switch). At the 1.05 m gate
the reconfiguration itself works every time; the drive through the throat decides the outcome.

**Lidar noise and the self-filter (2026-10-03).** UnitySensors' noise job copies one random state
into every scheduled job, so each scan restarted the same sequence: a 20,000-point scan held about
3,200 distinct noise values, identical from scan to scan, never beyond about 3.5 sigma and fixed per
Play session by its start-up seed. In some sessions a large draw landed on the arm mount every time
the scan pattern repeated, leaked 0.08-0.10 m in front of the mount through the self-filter's
4 sigma band, and the persistent global layer marked the robot's own cell at every stop; Smac then
reported "Start occupied". The robot now uses `IidNoiseRaycastLiDARSensor` (independent noise per
point and scan; live: 19,998 distinct values per scan, sigma 0.0200, 3.3e-5 of arm-mount returns
beyond 4 sigma against 3.2e-5 for a Gaussian). With independent noise the 4 sigma band leaked 8
self-returns in 30 s, held 10 s by the costmaps' voxel decay, so the band is 6 sigma (0.12 m); no
leak in 600 scans.

**Earlier series.** The first B3 series (2026-10-02, `runs-2026-10-02/`, 25 of 27, no contact) used
the repeating noise, the 4 sigma band and no re-plan; its RPP batch ran in a session that marked the
robot's own cell (the narrow and wide gates still passed 3/3, the narrow gate in 32.9-35.7 s). An
interim rerun on 2026-10-03 (not kept) hit the same in its MPPI session (narrow gate 0/3), and 3 of 12
constrained reconfigurations there and in the next attempt failed because MoveIt rejected its own
smoothed plan, which led to the re-plan. Phase 1 results were recorded with the same noise and
band.

**Arm configurations.** KDL returned a different arm configuration for every panel-pose goal (27
distinct configurations at 0.1 rad resolution over 27 vertical-panel reconfigurations), all with
the panel within +/-0.01 m and +/-0.01 rad of the goal and inside the `vertical_carry` profile.
Computed panel goals therefore do not repeat the joint path between runs; the reported motion
time and planned clearance spread reflect that. A mission that stops mid-way leaves the arm in such
a configuration; the runner returns it to a qualified pose with MoveIt before the next scenario.

**Constrained reconfiguration.** The qualified straight transition from home to vertical carry is in
collision with the tall box in 32 of its 50 samples (`constrained-scene/naive-transition-check.json`);
MoveIt planned around it in all 9 missions (planning 0.04-0.30 s). Planned clearance to the inflated
box was 0.000-0.034 m, i.e. about 0.05 m from the real box; this is where smoothing can push a plan
into the box and MoveIt rejects it (the re-plan above; CHOMP is the open alternative).

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
the panel struck it in every run, and the arm controller aborted on its path tolerance. A recheck
with the corrected lidar noise and 6 sigma filter band (one run, 2026-10-03, kept in
`experiment_runs/`) gave the same picture: 10 voxels on the box, highest at 1.125 m, and panel
contact; the repeating noise was not the cause. The cause
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
