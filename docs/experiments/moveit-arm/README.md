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
