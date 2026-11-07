# MuJoCo campaign plant: acceptance record

Branch `mujoco-plant`, 2026-11-05 to 2026-11-07. Design:
[spec](../../superpowers/specs/2026-11-05-mujoco-plant-design.md) and its approved
[amendment](../../superpowers/specs/2026-11-06-mujoco-plant-amendment-urdf-realistic.md) (the URDF-based realistic mode);
plan: [2026-11-05-mujoco-plant.md](../../superpowers/plans/2026-11-05-mujoco-plant.md); decision:
[ADR 0016](../../adr/0016-mujoco-campaign-plant.md). The gate (spec Section 8, amended Section 6): G, E, C, L, R and P
pass, plus the user's visual sign-off.

| Gate | Verdict | Evidence |
|---|---|---|
| **G1** read-back | pass | the compiled model's 604 site boxes and every scenario's boxes match the export within 1e-6 and the checksum (`test_site_model`, `test_plant_ideal`) |
| **G2** map | pass | MuJoCo's geoms rasterized at 5 cm in the map's 0.02-0.6 m band: 0 of 105298 cells differ from Unity's exported `.pgm` (`test_generate_site_model`) |
| **G3** distances | pass | `mj_geomDistance` against B5's FCL distances within 1.4e-7 m over the sampled near-obstacle states (`test_distance_agreement`) |
| **G4** rays | pass | noise off, every one of 20000 rays per scan (MuJoCo and BVH paths) within 1e-6 m of the exact range plus float storage, same geom (`test_lidar`) |
| **G5** fixtures | pass | the 1.05 m and 1.30 m doors and the scenario gates and beams (1.05 m, 1.23 / 1.30 m) as defined (`test_site_model`, `test_scenario_setup`) |
| **E1** equivalence | pass (reformulated, CP1) | ideal mode replays the kinematic bench open loop within 1.4e-12 m; closed loop, each cell's status lies within an ensemble of kinematic runs perturbed at rounding level (the F2+B5 loop is chaotic) (`test_equivalence`) |
| **E2** determinism | pass | the same seed gives bit-identical runs and run logs, F2 included (`test_frames`, `Session.SameSeedSameLogBytes`) |
| **C** commissioning (amended) | pass with accepted deviations (CP3b, CP3c) | [commissioning.md](commissioning.md): stops, drift, in-place turns, the arm on all 20 recorded streams pass; two turn ratios (x0.79, x0.87) and the torque-speed curve (no data) accepted; arm bandwidth 10 Hz kept (stable to 30 Hz, well damped to 15 Hz). The Unity-fitted planar profile is kept as `profile_unity.yaml` ([calibration.md](calibration.md)) |
| **L** lidar | pass | the ported noise is standard normal by a KS test and matches the C# reference (`test_lidar_noise`); against Unity at five fixture poses every class within 0.2 points, "other" within 2, self-hit Jaccard 0.87-0.89 (threshold 0.8, CP4), no range or blind-zone violations ([lidar.md](lidar.md)) |
| **R** cross-simulator (amended) | pass with findings (CP4, re-ruled 2026-11-07) | [replicas.md](replicas.md), [replicas_notes.md](replicas_notes.md): the ranking F2 >= B5 holds in every realistic cell but gate_right_swing (not significant against Unity); store_room's B5 failures are the wheel-contact base's sideslip into the door (a documented simulator difference); findings for the execution fixes |
| **P** performance | pass | Apple M4 (fanless), thermal pressure nominal, no Unity Play: the plant step 0.3-0.5 ms per 20 ms cycle (p99 <= 0.9 ms; 1.7-2.7 % of real time, limit 10 %); a lidar scan's casting p99 11.5-13.8 ms (limit 25 ms); see below |
| **V** visual | signed off (user, 2026-11-07) | Unity and the plant from the same chase camera at the 1.05 m door, before the 1.23 m beam and at the worker crossing (`tools/plant/visual_check.py`; images in `experiment_runs/visual-check-2026-11-07`, not committed) |

## P timings

`plant_campaign --profile-timing` (realistic mode, F2, seed 1, one run at a time; 2026-11-07, Mac16,12 Apple M4, the
container with 10 CPUs, `powermetrics` thermal pressure "Nominal", the Unity Editor open, not playing):

| Cell | Lidar | Plant step p50 / p99 / max (ms) | Fraction of real time | Scan cast p50 / p99 / max (ms) | Real-time factor |
|---|---|---|---:|---|---:|
| v2_door_105_nav | off | 0.53 / 0.88 / 2.60 | 0.027 | - | 3.2 |
| v2_door_105_nav | on | 3.00 / 3.60 / 4.64 | 0.149 | 12.9 / 13.8 / 15.1 | 2.3 |
| v2_low_beam_123_nav | off | 0.49 / 0.84 / 1.98 | 0.025 | - | 5.0 |
| v2_low_beam_123_nav | on | 2.62 / 3.28 / 4.44 | 0.133 | 10.7 / 11.9 / 12.9 | 3.2 |
| v2_worker_crossing_nav | off | 0.32 / 0.47 / 0.67 | 0.017 | - | 6.6 |
| v2_worker_crossing_nav | on | 2.46 / 3.09 / 9.81 | 0.121 | 10.3 / 11.5 / 16.3 | 3.9 |

The plant step with the lidar on includes its casting (the step's limit is judged with it off; the scan has its own).
The planners, not the plant, set the campaign speed: 1045 replica runs took 2.3 h with 3 jobs (23.4 h simulated, 46 %
of it in runs waiting out their timeout).

## What the plant is and is not

- A test bed for planners: B5 and F2 run in-process against it (ideal and realistic modes, the lidar, movers,
  appearing obstacles), unpaced and in parallel, logged and replayable, streamable live to a viewer.
- Not Unity's twin (CP3): the realistic mode is URDF-based (a wheel-contact skid-steer base, the arm by a bandwidth
  rule), and its differences from Unity are documented ([calibration_notes.md](calibration_notes.md),
  [commissioning_notes.md](commissioning_notes.md), [replicas_notes.md](replicas_notes.md)). MuJoCo results are their
  own profile and are never pooled with Unity's.
- Not part of the live ROS stack: no ROS bridge (spec 3), no route planner or costmap (spec 2; recorded routes are the
  input), no collision monitor, no Unity viewer (spec 4).
