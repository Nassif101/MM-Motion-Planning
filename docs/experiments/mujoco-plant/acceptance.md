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

## Problems and resolutions

Every problem the plant's development met, its cause and how it was solved (the executor's rulings, with their
costs if wrong, are in [rulings.md](rulings.md); the user's decisions in the roadmap's decision log).

| # | Problem | Cause | Resolution | Where |
|---|---|---|---|---|
| 1 | The host's Python (3.9) cannot install MuJoCo 3.17 for the viewer | MuJoCo's wheels need Python >= 3.10 | A git-ignored `.venv-plant` (uv, Python 3.12, mujoco 3.17.0); macOS's viewer needs `mjpython` | Task 1 review, CP1 |
| 2 | `site_v2.xml` was never committed | Unity's `.gitignore` entry `Models/` matched `models/` case-insensitively | A negation for the plant's models; `*.xml` with LF endings | Task 2 review |
| 3 | The F2+B5 closed loop diverges from the kinematic bench (up to 2 m in 21 s; store_room and low_beam_123 flip to timeouts) | The loop is chaotic at rounding level: a 1e-15 rad start yaw difference grows | E1 reformulated: an open-loop replay gate (1.4e-12 m) and closed-loop outcomes within a rounding ensemble; seeds perturb the start (1 mm, 1 mrad); R judges ensembles | Task 4, CP1, roadmap |
| 4 | The ideal mode's breakaway stopped moving bases at slow commands | Judged every 2 ms substep instead of once per command | Latched per command, as the kinematic plant | Task 4 review |
| 5 | The first lidar benchmark said 37-46 ms per scan, over the 25 ms rule | The new packages had built unoptimized (empty build type); MuJoCo's rays scan every geom | Builds at RelWithDebInfo; a static BVH for the site's geoms (11.5-13.6 ms; P: p99 <= 13.8 ms) | Task 5, CP2 |
| 6 | The ray test checked only rays that returned a point, with a loose tolerance | Float storage mixed into a 1e-5 m tolerance | Every one of 20000 rays: the exact range within 1e-6 m plus the float storage | Task 5 review |
| 7 | The base watchdog aged a command from its stamp, not its arrival | The stamp includes the command delay | Aged from receipt; `BaseDrive.StaleCommandRampsToZero` | Task 6 review |
| 8 | The arm fit drifted to the grid's edges | The p95 objective alone rewarded extreme gains | A combined objective (q RMSE + p95 gap), a 1 % minimum gain; the arm fitted on whole tasks; allowance 3 mrad | Task 7, CP3 |
| 9 | The plant reported the arm reference as the delayed stream | The command delay sat before the trajectory controller | The controller samples streams on receipt; the delay is on its output | Task 7 |
| 10 | The base's drift fails against Unity in long turns (overshoot, slide, over-speed) | Unity's base turns on a 0.02 friction floor: history-dependent dynamics a static map cannot fit | A wheel-contact spike, then the URDF-based realistic mode (amendment): the base on its wheels; differences documented, not tuned away (the plant is the test bed) | Task 7, spike, amendment, CP3 |
| 11 | A stiff wheel loop (kv 1000) overshot the acceleration ramp by 0.24 m/s | MuJoCo's soft contact (20 ms) at the 2 ms step | kv commissioned: the stiffest that follows the ramp within 0.02 m/s (80) | Task 7b |
| 12 | Turns need an effective track of 3.0 m (4.7x the geometric 0.64 m) | Four rigid cylinders skid-steering on soft contact; contact variants did not reduce it | Accepted and documented; Unity's sliding base and this gripping one bracket the effect; a grip ensemble in R | Task 7b, CP3b |
| 13 | The live viewer drove the robot into a wall | Without a bag the session took the straight start-goal line | Live runs follow a corpus cell's recorded route by default | Task 9 |
| 14 | The live view stutters | The single-threaded demo server stalls during medium solves (up to 100 ms) | Accepted for now; an asynchronous medium planner is Phase 6 item 2 | Task 9 |
| 15 | The viewer coloured scan points by the wrong classes | The protocol doc and the viewer assumed a different class order than the plant's enum | `HIT_CLASSES` pinned to the enum by a test; doc corrected | Task 10 |
| 16 | Tasks 7b, 8 and 9 closed without their reviewer | The executor skipped the per-task review | Retroactive reviews; all Critical and Important findings fixed: commands before a reset crashed the server, a closed log killed it, unbounded catch-up, oversized frames, Hello/Event drops, impossible move_body (reserved bodies added), wall-clock time in logs (E2), missing provenance, an untested mission path | Final(...) fixes, user's catch |
| 17 | The calibration replayed the wrong profile | plant_replay's default became the URDF-based profile | Each tool passes its own profile | Task 7b review |
| 18 | The bandwidth commissioning could not tell 5-15 Hz apart | Errors measured against the undelayed reference: the 30 ms delay's | The servo tested alone, with saturating moves and per-joint steps; 10 Hz kept (stable to 30 Hz, well damped to 15 Hz) | Task 7b review, CP3c |
| 19 | Two turn ratios outside +-10 %, a torque-speed curve without data | The wheel-contact base's understeer; no motor data | Accepted deviations, pinned to what was accepted | CP3c |
| 20 | The lidar comparison biased the site fraction by 3-5 points | Unity's points labelled by proximity (3 cm) against the plant's true labels, under 2 cm noise | Both simulators labelled alike within 8 cm (4 sigma); 'other' gated too; Jaccard 0.8 above the 0.79 noise floor | Task 10, its review, CP4 |
| 21 | The Unity export failed: no `map` frame, no `/tf_static` | Navigation stopped; the runner starts no robot_state_publisher | Unity's odom as the map frame (checked to 1.3 mm); the URDF's lidar mount | Task 10 |
| 22 | Sessions timed out at 120 s where Unity allowed 300 s | A flat default instead of the scenario's timeout_s | The scenario's timeout_s by default | Task 11 |
| 23 | The grip variants failed to load | A copied profile resolves its models beside itself | Absolute model paths in the variants | Task 11 |
| 24 | The viewer drew nothing within 14 m of the camera | Reserved bodies parked 1 km out inflated the extent MuJoCo's clip planes scale with | Clip planes set in metres | Task 12 (V) |
| 25 | The first R investigation was wrong (store_room, room_r4, gate_right_swing) | Attributions made without checking Unity's own runs and routes | Two reviews, ablations (arm delay, planar base, grip, Unity's other routes), gate diagnostics in the frames: store_room is the wheel-contact base's sideslip into the door; CP4 re-ruled | Task 11, CP4 |
| 26 | A failed reset could crash the demo server | Session replaced its plant before the steps that can throw | Built into locals, committed at the end | Final review |
| 27 | VS Code's container rebuild failed (exit 1) | Not diagnosed (no detail in the log) | Recreated with `docker compose -f .devcontainer/docker-compose.yml up -d --no-build`; volumes intact | Task 9 |

## What the plant is and is not

- A test bed for planners: B5 and F2 run in-process against it (ideal and realistic modes, the lidar, movers,
  appearing obstacles), unpaced and in parallel, logged and replayable, streamable live to a viewer.
- Not Unity's twin (CP3): the realistic mode is URDF-based (a wheel-contact skid-steer base, the arm by a bandwidth
  rule), and its differences from Unity are documented ([calibration_notes.md](calibration_notes.md),
  [commissioning_notes.md](commissioning_notes.md), [replicas_notes.md](replicas_notes.md)). MuJoCo results are their
  own profile and are never pooled with Unity's.
- Not part of the live ROS stack: no ROS bridge (spec 3), no route planner or costmap (spec 2; recorded routes are the
  input), no collision monitor, no Unity viewer (spec 4).
