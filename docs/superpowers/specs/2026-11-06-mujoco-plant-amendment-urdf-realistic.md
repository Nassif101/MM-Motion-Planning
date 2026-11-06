# MuJoCo plant amendment: a URDF-based realistic mode

Amends `2026-11-05-mujoco-plant-design.md` (Sections 3.2, 4.3, 4.4, 7, 8 C and R, 9, 10). Status: draft for review,
2026-11-06.

## 1. Why

- **Framing (user, CP3, 2026-11-06).** The plant is the test bed for every baseline. Its realistic mode must be
  credible and consistent, not a copy of Unity. Unity is a second simulator to compare against; the differences go in
  the thesis.
- **Unity's base is not physics to copy.** Its floor friction is 0.05 static / 0.02 dynamic, a workaround that let
  PhysX's isotropic friction turn the base in place (docs/unity-skid-steer-base-controller.md). It makes the base slide
  and coast (yard_store: yaw overshoot, sideslip, 20 % over-speed for 6 s). The Task 7 skid map can't reproduce that,
  and it shouldn't have to.
- **The spike (2026-11-06) showed a wheel-contact base in MuJoCo is credible** with the URDF's settings: it turns in
  place, follows turns, stops crisply, has no overshoot, and runs about 100× real time without the lidar.
- **The arm uses Unity's PhysX drive gains** and zeroes the URDF's joint damping and friction. The URDF is the robot's
  specification, and B5 and F2 already plan against it (F2's torque budget is 60 % of the URDF efforts).

## 2. Realistic mode: the URDF plus documented assumptions

Every realistic-mode value comes from the URDF or from an entry in a new `assumptions` section of the profile. Each
entry carries its value, its source or reasoning, and the sensitivity range it is checked over. The Task 7
calibration's software-stack delays stay; they describe the ROS side, not PhysX.

| Item | Value | Source |
|---|---|---|
| Masses, inertias, payload | URDF, 107 kg + 3 kg panel | URDF, qualified_payload.json |
| Arm efforts, velocity limits | 120/160/90/40/30/20 N·m; 1.8/1.6/1.8/2.5/2.5/3.2 rad/s | URDF |
| Arm joint damping, friction | 2.0/2.2/1.8/0.9/0.8/0.6; 0.3/0.25/0.2/0.12/0.1/0.08 N·m | URDF (now on) |
| Arm armature | estimated: gear ratio² × rotor inertia per joint | assumption, cited; checked at ±50 % |
| Arm controller | JTC (ROS side) + joint PD + gravity, motion, viscous and Coulomb feedforward, torque-clamped | design rule, Section 4 |
| Wheel motors | stiff velocity loop, 85 N·m, 18 rad/s hard, 8 rad/s operating cap | URDF; controller cap kept |
| Wheel joint damping, friction | 0.35, 0.08 | URDF |
| Ground | μ 0.6 (rubber on concrete or compacted ground), elliptic cone, impratio 10 | assumption; checked at 0.4–0.8 |
| Effective track | measured in MuJoCo by commissioning (spike: about 2.5 m) | commissioning |
| Torque–speed curve | flat | assumption, documented |
| Delays | arm command 30 ms (JTC output), base 0, states 0 | Task 7 measurement and fit |

## 3. Base (replaces 3.2's base and 4.3 in realistic mode)

- `realistic.base_model: wheels | planar`. **wheels** is the new default. **planar** keeps the Task 7 Unity-calibrated
  base (skid map, lags), so today's profile stays reproducible.
- **Wheels:**
  - The chassis is a free body; the four wheels are hinge joints with the URDF's damping and friction.
  - The wheel cylinders touch the ground: elliptic friction cone, impratio 10, μ from the assumptions.
  - Wheel motors are stiff velocity servos clamped at 85 N·m.
  - The whole robot's weight presses on the wheel contacts, so load transfer from the arm, the panel, acceleration and
    turns changes each wheel's grip.
- **Base controller:** Unity's SkidSteerBaseController chain stays as controller logic: clamp, chassis rate limits,
  skid-steer mapping with the effective track, common-factor saturation at the operating cap, and the watchdog. The
  breakaway feedforward stays off unless commissioning shows the base cannot start from small commands.
- **Ideal mode is unchanged:** a planar model, set kinematically. The generator writes both robot models; the plant
  picks one by mode and `base_model`.

## 4. Arm (replaces 4.4's gains)

- The JTC is unchanged (on the ROS side, with its output delayed by the command delay; Task 7).
- The joint controller is τ = Kp(q_r − q) + Kd(q̇_r − q̇) + gravity + motion feedforward (mj_rne, armature included)
  + viscous (URDF damping · q̇_r) + Coulomb (URDF friction · sign q̇_r) feedforward, clamped to the URDF efforts.
- **Gain rule:** per joint, Kp = I·ω_n² and Kd = 2ζ·I·ω_n, where I is the joint's reflected inertia (armature plus links)
  in the qualified carry pose, ζ = 1, and ω_n = 2π·f. The bandwidth f is one documented assumption, chosen at
  commissioning: the highest value that stays stable under the clamp and on the recorded streams, with 10 Hz proposed.

## 5. Commissioning (replaces Section 7.2's fits; the Task 7 report stays as the Unity comparison)

`tools/plant/commission_plant.py` measures the plant the way a real robot is commissioned and writes
`docs/experiments/mujoco-plant/commissioning.md`:

- **Base:**
  - straight acceleration and braking;
  - a steady (v, ω) grid of achieved against commanded, including turning in place;
  - the effective track by least squares on that grid;
  - breakaway from rest (v and ω sweeps);
  - the reachable (v, ω) envelope under the wheel cap: |v| + ω·b_eff/2 ≤ 8 · 0.14 m/s;
  - drift at rest;
  - sensitivity to μ and payload pose.
- **Arm:**
  - step and sine responses per joint;
  - tracking error on the recorded streams;
  - stability at rest and under the clamp;
  - armature sensitivity.
- **Unity comparison:** `calibrate_plant.py validate` runs on the new profile, so the held-out and fresh tables show
  the documented simulator differences. They are reported, not gated.

## 6. Acceptance (amends Section 8)

- **C becomes commissioning checks:**
  - steady commands inside the envelope with |ω| ≤ 0.6 rad/s are achieved within ±10 %;
  - turning in place works from 0.2 rad/s;
  - the base stops within its deceleration limit + 0.2 s and drifts less than 1 mm/min at rest;
  - the arm is stable at rest and on every recorded stream, and stays inside the URDF limits;
  - every assumption has a source and a sensitivity result.
- **New consistency test:** the planners' limits (nav_operating_envelope.yaml, B5's and F2's configs) against the
  plant's measured envelope and breakaway. Any combination a planner may command that the base cannot reach fails it,
  until the cap is raised (the URDF allows 18 rad/s) or the planners' limits are coupled.
- **R becomes a documented comparison:**
  - outcomes are per-cell distributions over perturbed ensembles (≥ 20 runs per cell; tune-with-ensembles);
  - pass when the planner ranking holds (F2 ≥ B5 per cell) and every per-cell difference from Unity is investigated
    and attributed to a physical or model cause;
  - clearances and contacts are reported against Unity's.
- G, E, L, P and V are unchanged.

## 7. Staging (amends Section 9)

Task **7b**, after Task 7 and before Task 8, ending at **CP3b**, where you review the commissioning report:

- the generator writes both robot models;
- the plant gets the wheeled base and the arm terms;
- `commission_plant.py` and its report;
- the consistency test;
- `calibrate_plant.py` run on the new profile.

Tasks 8–12 follow unchanged; Task 11 uses the ensemble R. Effort: about one task, larger than the others.

## 8. Risks (adds to Section 10)

- **MuJoCo's skid steer is its own.** The spike's effective track (3.9× the geometric 0.64 m) is above ideal Coulomb
  (about 1.9×) and Clearpath's A300 (1.75×), probably because rigid cylinders have no tyre compliance. Commissioning
  tries a rounder wheel contact and pair-level friction along and across the wheel, and documents the result. It
  doesn't tune toward a target.
- **The reachable envelope shrinks** with a large effective track: the consistency test catches it, and the cap or the
  planners' limits decide it.
- **Armature and bandwidth are estimates.** Sensitivity results go into the report; R's ensembles show whether planner
  outcomes depend on them.
- **Load-dependent grip** makes the base respond differently with the arm extended. That's intended, and it is
  reported.
