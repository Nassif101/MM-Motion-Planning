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

**Setup (2026-10-03/04, commit `849f19b`, clean tree):** `run_nav_scenario.py` with `--new-epoch`
per controller and three runs of each mission, 23:15-00:13 UTC. Nav2 with the persistent global
obstacle layer (default) and `dynamic_monitor_zones:=true` (since 2026-10-04 `footprint_mode:=profiles`); MoveIt with the known scene and the
scenario's boxes. The lidar draws independent range noise per point and scan and the self-filter
uses a 6 sigma noise band ("Lidar noise and the self-filter" below); `ReconfigurePanel` re-plans
up to 3 times when MoveIt rejects its own smoothed plan; after a reconfiguration the mission waits
until both costmaps have completed an update cycle on the new footprint before the next drive.
A drive counts as reached only if the base ends within the goal checker's tolerances plus 0.02
(0.17 m, 0.17 rad): Nav2's stateful goal checker stops checking the position once the base has
passed within 0.15 m, and a drive Nav2 reports reached while the base ended farther away is
`off_goal`, which fails the mission. Every summary records the machine (Apple M4 host, 10-CPU
aarch64 container, Unity 6000.5.2f1, Cyclone DDS); the real-time factor was 1.00-1.04, no run
spanned a host sleep and the container killed no process for memory. Summaries: `runs/` (one per
run; bags, Unity arm recordings and MoveIt logs stay in the git-ignored `experiment_runs/`). Table
from `python3 tools/summarize_nav_runs.py docs/experiments/moveit-arm/runs/*.json`; reconfiguration
columns cover all successful reconfigurations, planned clearance is to the 0.05 m-inflated known
boxes (floor excluded), `move_group` CPU is per reconfiguration (percent of one core), panel bottom
and base tilt are Unity ground truth over the whole mission.

| Mission | Controller | Success | Contact | Total s | Drives s | Reconfig. s | Planning s | Motion s | Planned clearance m | Path error rad | Hold error rad | move_group CPU % | Panel bottom m | Base tilt deg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| narrow_gate | RPP | 3/3 | 0 | 63.0 (59.5-63.1) | 47.1 (47.0-47.4) | 15.7 (12.5-16.0) | 0.11 (0.02-0.34) | 4.71 (2.74-5.11) | 0.723 (0.065-1.529) | 0.065 (0.046-0.068) | 0.015 (0.008-0.019) | 5.2 (3.4-12.0) | 0.515 (0.506-0.559) | 0.12 (0.10-0.15) |
| narrow_gate | DWB | 1/3 | 0 | 111.1 | 99.9 | 11.2 | 0.11 (0.02-0.24) | 2.55 (2.51-3.83) | 1.378 (0.163-1.383) | 0.039 (0.038-0.050) | 0.015 (0.009-0.015) | 5.8 (3.9-9.8) | 0.706 (0.703-0.711) | 0.14 (0.12-0.15) |
| narrow_gate | MPPI | 2/3 | 0 | 57.8 (57.5-58.1) | 44.6 (44.4-44.7) | 13.2 (13.0-13.4) | 0.14 (0.02-0.26) | 2.83 (2.68-3.92) | 0.783 (0.098-1.454) | 0.058 (0.049-0.076) | 0.015 (0.004-0.015) | 5.8 (3.3-10.2) | 0.556 (0.551-0.572) | 0.12 (0.09-0.13) |
| wide_gate | RPP | 3/3 | 0 | 57.2 (57.0-59.5) | 46.5 (46.2-46.9) | 11.0 (10.5-12.6) | 0.07 (0.02-0.18) | 2.18 (2.06-2.97) | 0.828 (0.245-1.407) | 0.039 (0.030-0.068) | 0.015 (0.012-0.021) | 5.6 (3.5-7.6) | 0.704 (0.702-0.705) | 0.12 (0.11-0.13) |
| wide_gate | DWB | 3/3 | 0 | 71.5 (67.4-71.9) | 59.7 (53.1-60.5) | 12.2 (11.0-14.4) | 0.05 (0.02-0.34) | 2.72 (2.47-3.11) | 0.853 (0.282-1.517) | 0.065 (0.037-0.076) | 0.015 (0.013-0.016) | 4.7 (3.9-11.5) | 0.582 (0.582-0.702) | 0.13 (0.13-0.15) |
| wide_gate | MPPI | 2/3 | 0 | 59.9 (59.1-60.8) | 44.6 (44.2-45.0) | 15.3 (14.1-16.5) | 0.02 (0.02-0.18) | 3.45 (2.00-5.11) | 1.303 (0.239-1.442) | 0.055 (0.044-0.088) | 0.015 (0.010-0.019) | 3.9 (3.6-7.0) | 0.570 (0.506-0.698) | 0.10 (0.09-0.12) |
| constrained | RPP | 3/3 | 0 | 28.4 (26.6-29.7) | 15.3 (15.1-17.4) | 12.3 (11.4-13.3) | 0.06 (0.02-0.72) | 3.40 (2.05-4.32) | 0.750 (0.001-1.597) | 0.057 (0.034-0.066) | 0.015 (0.014-0.020) | 4.8 (3.2-15.6) | 0.566 (0.370-0.701) | 0.12 (0.08-0.12) |
| constrained | DWB | 3/3 | 0 | 28.8 (28.3-29.9) | 16.4 (16.0-16.7) | 12.8 (11.6-13.5) | 0.05 (0.02-0.12) | 3.19 (2.31-5.60) | 0.748 (0.002-1.623) | 0.051 (0.029-0.065) | 0.015 (0.014-0.017) | 4.2 (3.7-5.7) | 0.531 (0.430-0.619) | 0.10 (0.10-0.13) |
| constrained | MPPI | 1/3 | 0 | 35.2 | 21.8 | 13.3 | 0.11 (0.02-0.24) | 4.56 (2.99-5.46) | 0.014 (0.001-1.735) | 0.044 (0.025-0.065) | 0.015 (0.012-0.015) | 5.1 (3.7-8.1) | 0.635 (0.480-0.686) | 0.15 (0.10-0.16) |

Cells: median (range) over runs; times over successful missions. **21 of 27 missions succeeded
(RPP 9/9, DWB 7/9, MPPI 5/9), no robot-environment contact, every reconfiguration succeeded** (49
of 49 requested before a mission ended, none needed a re-plan), `move_group` used 3-16 % of a core
and 70-71 MB per reconfiguration, and every Unity arm check passed in all 27 runs apart from the
mission-status check of the failed missions (panel bottom >= 0.370 m against the 0.15 m limit,
tilt <= 0.16 deg, Unity path error <= 0.066 rad, no panel penetration). No global-costmap frame of
any run marked a cell within 0.3 m of the robot centre.

The six failures are drives, not reconfigurations:

- `narrow_gate_mission` DWB, runs 1 and 3: the drive through the 1.05 m throat aborted after about
  130 s and 48-50 recoveries (error 105, failed to make progress). RPP crossed the throat 3/3 and MPPI
  3/3 in this series.
- MPPI, three drives Nav2 reported reached while the base ended 0.42 m (`narrow_gate_mission`, the
  last drive), 0.81 m (`wide_gate_mission`, the gate crossing) and 1.69 m
  (`constrained_reconfiguration_mission`) from the goal (`off_goal`): stock MPPI passes the goal
  inside the 0.15 m tolerance, the stateful goal checker latches the position, and MPPI keeps
  driving on a forward arc while it turns to the goal heading. Earlier B3 series counted such drives
  as successes (the series at `e4dbc1d` rescored: 21/27 instead of 23/27).
- `constrained_reconfiguration_mission` MPPI, run 3: stock MPPI left the start beside the tall box on
  a 6.5 m path for the 3 m drive, recovered 11 times and aborted (error 103) 3.3 m from the goal, as
  in the earlier series; stock MPPI is kept untuned as baseline B2.

**Throat plans.** The Smac Lattice plan through the 1.05 m gate often S-bends: about 0.10 m to one
side after the start, then 0.04-0.06 m off centre at the posts (RPP runs here; DWB 0.02-0.03 m to
the other side; the three MPPI plans happened to be straight), depending on the start pose to the
millimetre (the gate centreline lies on a costmap cell boundary). On a grid of starts and goals
through the gate 334 of 1925 plans bent, none at the wide gate or in open space; it is already in
the raw lattice path, and no planner setting removed it without bringing obstacle detours closer
(roadmap decision 2026-10-03, the 1.05 m gate). With 0.02-0.04 m per side for a centred robot, the
bend decides many throat drives; it is part of the baseline. A footprint-switch race found on the
way is fixed: the mission sent the drive goal 6-70 ms after the costmaps first showed the new
footprint, before the global costmap had re-inflated for it; it now waits one more cycle (about 0.5
s), and the plans bend the same way.

**Gate crossing against Phase 1** (reconfiguration + drive from the staging pose 3 m before the
gate to 1.8 m after it, versus the Phase 1 drives over the same start and goal from the full rerun at
`849f19b`, `docs/experiments/nav2-navigation/runs-2026-10-04`).

| Gate | Controller | B3 reconfigure + cross s | Phase 1 home footprint (detour) s | Phase 1 panel already vertical s |
|---|---|---|---|---|
| 1.05 m | RPP | 35.8 (34.4-35.9), 3/3 | 44.4 (44.4-45.2), 3/3, 11.0 m path | 26.7 (26.6-26.7), 2/3 |
| 1.05 m | DWB | 85.6, 1/3 | 45.2 (44.6-45.9), 3/3, 10.9 m path | 29.1 (28.2-29.4), 3/3 |
| 1.05 m | MPPI | 34.8 (34.2-35.3), 3/3 | -, 0/3 (all `off_goal`) | 27.0 (26.9-28.8), 3/3 |
| 1.30 m | RPP | 32.7 (32.2-33.6), 3/3 | 44.8 (43.7-45.1), 3/3, 11.3 m path | 26.6 (26.6-26.6), 3/3 |
| 1.30 m | DWB | 40.9 (35.7-41.6), 3/3 | 43.1, 1/3, 11.2 m path | 27.3 (27.1-27.4), 3/3 |
| 1.30 m | MPPI | 36.2 (36.2-36.3), 2/3 | -, 0/3 (all `off_goal`) | 27.1 (26.9-27.1), 3/3 |

Reconfiguring at the 1.30 m gate takes 33-41 s against 43-45 s for the home-footprint detour and
27 s when the panel is already vertical: the price of a standstill reconfiguration is about 6-14 s
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
smoothed plan, which led to the re-plan. The series at `e4dbc1d` (2026-10-03, `runs-2026-10-03/`,
reported as 23/27, 21/27 rescored with `off_goal`) had the corrected noise and the re-plan but
planned the throat drive before the global costmap had re-inflated for the new footprint; one
further batch (not kept) was cut short by a runner crash on a stray output line. Phase 1 has been
rerun on the same code (`docs/experiments/nav2-navigation`, 2026-10-04).

**Arm configurations.** KDL returned a different arm configuration for every panel-pose goal (27
distinct configurations at 0.1 rad resolution over 27 vertical-panel reconfigurations), all with
the panel within +/-0.01 m and +/-0.01 rad of the goal and inside the `vertical_carry` profile.
Computed panel goals therefore do not repeat the joint path between runs; the reported motion
time and planned clearance spread reflect that. A mission that stops mid-way leaves the arm in such
a configuration; the runner returns it to a qualified pose with MoveIt before the next scenario.

**Constrained reconfiguration.** The qualified straight transition from home to vertical carry is in
collision with the tall box in 32 of its 50 samples (`constrained-scene/naive-transition-check.json`);
MoveIt planned around it in all 9 missions (planning 0.08-0.72 s). Planned clearance to the inflated
box was 0.001-0.048 m, i.e. about 0.05 m from the real box; this is where smoothing can push a plan
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
  correct). Missions launch Nav2 with `dynamic_monitor_zones:=true` (since 2026-10-04 `footprint_mode:=profiles`); `ReconfigurePanel` publishes the
  zones, and the mission task checks the costmap footprints and zone inputs before each drive.
- **Settling.** Nav2 reports success before the base is at rest; the mission task waits until the
  base has been still for 0.75 s (1.3 s median wait after the drive, at most 2.7 s) before reconfiguring, as the server requires 0.5 s.
