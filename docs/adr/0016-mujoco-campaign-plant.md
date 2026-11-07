# ADR 0016: a MuJoCo campaign plant is the planners' test bed; Unity keeps the live stack

Date: 2026-11-07. Status: accepted (tag `mujoco-plant-accepted-2026-11-07`). Design:
[spec](../superpowers/specs/2026-11-05-mujoco-plant-design.md) with its
[amendment](../superpowers/specs/2026-11-06-mujoco-plant-amendment-urdf-realistic.md); acceptance:
[acceptance.md](../experiments/mujoco-plant/acceptance.md). Builds on [ADR 0011](0011-wholebody-mpc-command-ownership.md)
(B5 owns the commands) and [ADR 0015](0015-medium-horizon-layer.md) (the medium layer above B5).

## Context

Every phase after 5A needs fast, reproducible campaigns: perturbed ensembles of 20 runs a cell (the F2+B5 loop is
chaotic at rounding level), execution fixes that need delay and tracking errors on demand, and later perception.
Unity campaigns run at real time, one at a time with cooling pauses on the fanless host (campaign B: about 3 h for 34
runs). Options considered (roadmap, 2026-11-01): Isaac Sim / Lab (needs an RTX GPU), Gazebo, Drake, PyBullet, Newton,
MuJoCo; within MuJoCo, a headless in-process plant (chosen) against a ROS-bridged one (spec 3, later).

## Decision

- **A plant core library** (`mobile_manipulator_plant`, C++, ROS-free; MuJoCo 3.17 vendored by `mujoco_vendor`)
  behind one step API: a site model generated from Unity's V2 export, the robot from the URDF and a robot profile,
  scenario boxes and movers at reset, reserved bodies for obstacles that appear mid-run, the Livox Mid-360 lidar with
  Unity's pattern and noise (a BVH for the static geoms), ground truth (contacts, clearance).
- **Two modes.** Ideal: kinematic, equal to the bench (E1). Realistic: URDF-based (the amendment): the base on four
  driven wheels on a free joint (commissioned: wheel loop kv 80, effective track 3.0 m), the arm by a 10 Hz bandwidth
  rule with URDF damping and friction and feedforward, a 30 ms arm command delay. The Unity-fitted planar profile stays
  as `profile_unity.yaml`.
- **Thin drivers.** `mobile_manipulator_campaign`: the Session (one B5 cycle per step, missions, frames, the zlib run
  log), `plant_campaign` (one run; `tools/plant/plant_campaign.py` runs them in parallel), `plant_demo_server` (a
  versioned TCP stream to one client), `plant_replay` and `plant_lidar_fixture` (commissioning and gate L).
  `tools/plant/view_run.py` replays logs or follows a live run in MuJoCo's viewer.
- **Ownership.** MuJoCo owns nothing in the live ROS stack yet: Unity keeps rendering, physics and the simulated
  sensors there, and ROS 2 keeps navigation and planning (AGENTS.md unchanged). The plant runs B5 and F2's own code in
  process; the ROS bridge (spec 3) and a Unity viewer of the plant (spec 4, its own ADR for the ownership change) come
  later.

## Consequences

- Campaigns of a thousand runs take hours, not days (1045 runs in 2.3 h on the fanless Mac); the planners, not the
  physics, set the pace.
- MuJoCo results are a separate profile, never pooled with Unity's; differences are documented, not tuned away
  (CP3): the plant is the test bed for every baseline, not Unity's twin.
- Gate R exposed planner findings for the execution fixes: B5 ignores the base's sideslip (store_room's door), the
  slack gate locks at a margin, F2's left starts at gate_right_swing fail more often.
- Routes are recorded Nav2 routes until the plant's own route planner (spec 2): a moving obstacle that Nav2 routed
  around on Unity is not routed around here.

## Revisit

- If the execution fixes or perception need a collision monitor, odometry noise or a costmap in the loop.
- The base's sideslip magnitude and the effective track (a 4.7x multiplier) if a hardware reference appears.
- A V1 site model if V1 cells must be replicated (campaign B's 11 V1 runs were not).
