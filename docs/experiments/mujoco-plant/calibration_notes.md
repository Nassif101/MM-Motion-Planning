## Analysis (hand-written; appended by the generator)

### What the calibration is for

The MuJoCo plant is the test bed for all baselines (user decision, CP3, 2026-11-06). Its realistic mode needs to be
credible and consistent, not a copy of Unity: every value in the profile has a source (a Unity measurement, the
robot's specification, or this fit), the profile is versioned, and every baseline meets the same plant. Unity is a
second, independent simulator; agreement within the targets is evidence of credibility, and the places where the two
differ are recorded here as simulator differences for the thesis, not chased. Whether a difference matters is decided
on planner outcomes (Task 11's per-cell distributions over perturbed runs), not on these signal metrics.

### Data and its limits

- 109 runs with today's Unity actuators, split by cell: 97 training runs (21 cells) and 12 held-out runs (5 cells).
  The held-out runs were already looked at during the first CP3 review (the skid-map decision), so they are no longer
  untouched; the fresh check (one unused training run per training cell, read by neither fit) is the cleaner evidence.
- The skid map uses the steady parts of 24 training runs. A knot needs 30 samples from 5 distinct steady segments:
  the hard-turn knots (share 1.25 and 2.0, inner wheels reversing) have 3 and 4 segments and take their neighbour's
  value. Unity's runs contain almost no steady hard turns or in-place turns, so turning in place uses Unity's own
  measurement (0.442 / 0.4 rad/s, docs/unity-skid-steer-base-controller.md), and the turn-from-rest breakaway is not
  checked by this data.
- The odometry latency is measured: /odom arrives with the /clock tick of its own stamp (|median| < 0.03 ms on every
  fit run), so the base's state delay is 0. Unity's raw joint states are not recorded, so the arm's state delay can only
  be fitted. The signal estimates of the base delay and lags (cross-correlation, first-order fit) only seed the search;
  they are biased by the skid map's speed ratio and the collision monitor's windows.

### Fitted values

- Base: no command delay (the first-order lags absorb the transport: an earlier search without the 1 % rule chose
  20 ms, which does not improve the objective by 1 %), first-order lags 0.10 s (speed) and 0.05 s (yaw), no extra
  slip; the skid map's
  straight-line ratio 0.93 and gentle-turn yaw ratio 0.94, rising to 1.10-1.14 as the inner wheels slow.
- Arm: Unity's own drive gains on every joint (the stiffness search found no factor improving the objective by 1 %),
  a 30 ms command delay, no state delay. The JTC runs on the ROS side: it samples each stream on receipt and its output
  reaches the drives after the command delay, which is how the plant now reports its reference.

### Simulator differences (for the thesis)

1. **Turn dynamics.** Every base failure is a run with long or tight turns (chicane, yard_store, store_room, room_r4,
   the doors, the two F2 runs in low_beam_123). In yard_store, on flat ground with the arm still, Unity's base yaws
   past the command (-0.55 against -0.40 rad/s), slips sideways (up to 0.1 m/s), then runs about 20 % over the
   commanded speed for 6 s. The behaviour depends on the turn's history; a static skid map with first-order lags cannot
   produce it. Straight and gentle driving passes every base target. A wheel-contact base is spiked next (CP3).
2. **low_beam_123: F2 against B5 alone.** The four B5-only runs in this cell pass every base target; the two F2 runs
   fail all of them. F2 reconfigures the arm and takes a tighter route: 5 and 12 % of their driving commands are hard
   turns (inner wheels at rest or reversing), against 0 % for B5 alone. It is the same turn-dynamics difference, not a
   property of F2.
3. **Arm tracking error on fast motion.** The plant's joints follow Unity's to about 1 mrad, but its tracking-error p95
   is 0.5-0.8 of Unity's when the arm moves fast (door_then_beam, the F2 runs in low_beam_123). A 10 ms joint-state
   delay would close the gap (probe: 0.87-1.27 of Unity's) at the cost of the joint positions (1.6 against 1.1 mrad
   RMSE); the fit, weighing both, keeps none. Unity's JTC compares its reference with joint states that are up to one
   20 ms period stale; the plant's controller sees them on time.
4. **Arm jitter at rest.** Unity's tracking error has a 1-2.7 mrad floor while the arm holds (PhysX solver jitter);
   the plant's is about 0.1 mrad. The target's absolute allowance is 3 mrad for this reason (2 mrad in the spec until
   CP3).

### Verdict on the Section 7.3 targets

The arm meets its target on 10 of 12 held-out runs and 6 of 8 fresh runs; the remaining failures are difference 3.
The base meets its targets on straight and gentle driving and misses them on long or tight turns (difference 1). Under
the test-bed framing the calibration is accepted with these differences documented; the wheel-contact spike decides
whether the base's turn dynamics are modelled or remain a documented gap with the base targets restricted to straight
and gentle driving.
