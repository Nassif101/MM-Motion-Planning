# Base acceleration, braking, and watchdog measurements

Date: 2026-09-28 (simulation time; Unity 6000.5.2f1/PhysX, ROS 2 Jazzy container).
Purpose: replace the assumed chassis acceleration/deceleration envelope with measured
plant behaviour before any Nav2 controller, velocity smoother, or MPC is tuned.

## Setup

- `ConstructionSiteV1`, 110 kg loaded articulation, arm in controlled `home` HOLD
  (`arm_control.launch.py` active), 3 kg panel attached, 20 ms physics step.
- Base placed with `unity command arm_test_place_open` at ROS `(12, 0)`, yaw pi; the
  nearest static obstacle is 6.9 m away. Every case drives out and back.
- `ros2 run mobile_manipulator_control base_step_test.py --speeds 0.3 0.6 --yaw-rate 0.4 --prefix step --output-dir <this directory>`
- The script refuses to command unless `/cmd_vel` has no other publisher, ground-truth
  TF is fresh, and the base is stationary. Commands are published at 20 Hz in sim time.
- Velocities are centred differences of Unity ground-truth `odom -> base_footprint`
  (50 Hz); rim speeds are `0.14 m x` wheel joint velocity from `/joint_states`.
- Unity actuator limits under test: accel 0.5 m/s^2, decel 0.8 m/s^2, yaw accel/decel
  0.8/1.2 rad/s^2, reverse speed cap 0.5 m/s, 0.5 s wall-clock watchdog. Floor friction
  0.05/0.02 with `Minimum` combine.

`step-*.csv` hold the per-sample traces; `step-summary.json` holds per-segment metrics.
0.6 m/s cases characterize the plant only; that speed is not qualified with the panel.

## Results

Linear cases (two stops per case, each direction):

| Case | Steady speed (m/s) | Accel 10-90 % (m/s^2) | Stop: time (s) / distance (m) | Mean decel (m/s^2) | Peak decel (m/s^2) | Max rim - body (m/s) |
|---|---:|---:|---|---:|---:|---:|
| 0.3 explicit zero | 0.278-0.280 | 0.46-0.49 | 0.42-0.94 / 0.053-0.069 | 0.28-0.62 | 0.67-0.95 | 0.07 |
| 0.3 watchdog stop | 0.277-0.279 | 0.43-0.46 | 0.88-1.10 / 0.18-0.23 | 0.25-0.32 | 0.84-0.92 | 0.08 |
| 0.6 explicit zero | 0.54-0.55 fwd, 0.46 rev | 0.44-0.46 | 0.74-1.20 / 0.17-0.28 | 0.38-0.65 | 0.66-0.83 | 0.17 |
| 0.6 watchdog stop | 0.55 fwd, 0.46 rev | 0.46 | 1.16-1.26 / 0.36-0.45 | 0.39-0.44 | 0.79-0.82 | 0.08 |

Stop time is measured from the zero command (or the last published command) until the
chassis is below 0.01 m/s; watchdog values include the 0.5 s wall-clock timeout.

Yaw (0.4 rad/s command): steady 0.453-0.455 rad/s (+14 %), 10-90 % yaw accel
1.07-1.14 rad/s^2, stops in 0.34-0.36 s (mean 1.26-1.30 rad/s^2).
Maximum base tilt across all cases was 0.11 deg; lateral body velocity stayed below
0.022 m/s.

## Interpretation

- Acceleration follows the actuator limiter (about 0.45-0.49 of the 0.5 m/s^2 limit).
- Braking reaches the 0.8 m/s^2 limiter only transiently. Mean deceleration varies
  between runs and directions (0.28-0.65 m/s^2 for explicit stops) and wheel rims
  over-run the chassis during braking, i.e. the wheels slip. The earlier estimate that
  0.05 static friction caps traction at mu*g = 0.49 m/s^2 is not borne out as a hard
  cap (peaks reach 0.95 m/s^2), but the achieved deceleration is clearly below the
  commanded limit and not repeatable.
- Steady speed undershoots by about 7-8 % (0.3 -> 0.28 m/s), and yaw overshoots by
  about 14 % with the 1.50 m effective track, consistent with the 2026-09-03
  commissioning.
- Command loss at 0.3 m/s costs about 0.2 m of travel; at 0.6 m/s about 0.45 m.

## Limitations

Two stops per direction and speed, one arm pose (`home`), one floor region, and
normal Editor frame rate. Braking variability suggests contact-state dependence;
repeat before relying on tighter bounds, and requalify after changing payload,
friction, drive gains, or the timestep.

## 2026-09-28 low-speed breakaway

Purpose: the first Nav2 MPPI run held a steady 0.014 m/s command from rest and the base
never moved (see the nav2-navigation README). This measures the smallest command that
starts the base.

`ros2 run mobile_manipulator_control base_step_test.py --breakaway-speeds 0.01 0.02 0.03 0.0325 0.035 0.0375 0.04 0.05 0.075 0.1 --breakaway-yaw-rates 0.02 0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 --output-dir <this directory> --prefix lowspeed-<n>`

Same setup as above (open fixture, `home` HOLD, panel attached). Each case steps from rest
for 4 s, stops for 2 s, then steps the opposite way. Two runs (`lowspeed-1-*`,
`lowspeed-2-*`) give four starts per command. A start counts when the mean speed over the
last 1.5 s of the step is above 0.005 in the commanded direction.

| Command from rest | Starts (of 4) | Steady speed / command |
|---|---:|---|
| 0.01 m/s | 0 | - |
| 0.02 m/s | 1 | 0.87 |
| 0.03 m/s | 1 | 0.50 |
| 0.0325 m/s | 0 | - |
| 0.035 m/s | 3 | 0.65-0.72 |
| 0.0375 m/s | 4 | 0.71-0.91 |
| 0.04 m/s | 4 | 0.85-0.86 |
| 0.05 m/s | 4 | 0.90-0.95 |
| 0.075 and 0.1 m/s | 4 each | 0.92-0.95 |
| 0.02, 0.05, and 0.06 rad/s | 0 each | - |
| 0.07 rad/s | 4 | 0.57-0.70 |
| 0.08 and 0.09 rad/s | 4 each | 0.59-0.79 |
| 0.1 rad/s | 4 | 0.80-0.88 |
| 0.15, 0.2, and 0.3 rad/s | 4 each | 1.01-1.12 |

Interpretation:

- The base has a breakaway deadband from rest: 0.0375 m/s or 0.07 rad/s always started it;
  0.02-0.035 m/s started it only intermittently (not monotonic in the command); 0.01 m/s
  and 0.06 rad/s or less never did. Just above breakaway it creeps below the command;
  from about 0.05 m/s and 0.15 rad/s it tracks as in the 0.3 m/s and 0.4 rad/s steps.
- Recorded in the navigation operating envelope as `tracking.breakaway_linear_mps: 0.0375`
  and `tracking.breakaway_yaw_radps: 0.07`. A controller must not rely on smaller commands
  from standstill. One whose first command from rest is limited by an acceleration model
  applied to the measured velocity (Nav2 MPPI in Jazzy) cannot start with the envelope's
  0.45 m/s^2: two 0.05 s steps allow at most 0.045 m/s, and the sample average is lower.
- Only starts from rest were measured, not the smallest speed the base holds once moving.
