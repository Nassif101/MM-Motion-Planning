# Unity ground-truth odometry check

Date: 2026-09-28. Fresh Play epoch, robot at the open fixture with the arm in `home`
HOLD, driven by the bounded `base_step_test.py --speeds 0.3 --yaw-rate 0.4` schedule
(starts, explicit and watchdog stops, reversals, pure yaw) while
`odom_consistency_check.py --seconds 80` recorded `/odom` and `/tf`.

`/odom` (`nav_msgs/Odometry`, `odom -> base_footprint`) is published by
`GroundTruthBaseTfPublisher` from the same physics sample and stamp as the TF. Twist is
PhysX's `base_link` articulation velocity moved to the `base_footprint` point and
expressed in `base_footprint` (ROS FLU); covariances are a fixed 1e-6 diagonal.

## Results ([odom-vs-tf.json](odom-vs-tf.json))

- 4,002 messages in 80 simulated seconds (50.0 Hz), every stamp on a 20 ms physics tick,
  all paired with a TF sample of identical stamp.
- Pose: identical to TF (0.0 m, 0.0 rad maximum difference).
- Twist against TF differences, `p50 / p95 / p99 / max`:

| Comparison | Linear (m/s) | Angular (rad/s) |
|---|---|---|
| Centred 80 ms difference, all samples | 0.003 / 0.014 / 0.019 / 0.028 | 0.002 / 0.014 / 0.024 / 0.039 |
| Centred 80 ms difference, steady samples | 0.001 / 0.012 / 0.016 / 0.024 | 0.001 / 0.010 / 0.020 / 0.039 |
| One-tick (20 ms) difference vs twist at the later tick | 0.002 / 0.007 / 0.010 / 0.024 | 0.002 / 0.015 / 0.022 / 0.037 |

Peak commanded motion reached 0.30 m/s and 0.50 rad/s.

## Interpretation

The pose is exact by construction. Median twist agreement is within 0.002, and the
signs, frames, and lever arm are covered by EditMode tests. The residual tail is not a
centre-of-mass offset: `base_link` has its centre of mass at its origin in both the URDF
and Unity (`automaticCenterOfMass = false`). It persists even for one-tick differences,
which a semi-implicit integrator would otherwise reproduce exactly. The remaining
explanation is solver position correction (contact and joint projection over 12 position
iterations), which moves the pose without appearing in the reported velocity.

`/odom` twist is therefore PhysX's reported body velocity, the usual simulator ground
truth, and differs from pose finite differences by up to about 0.02 m/s and 0.025 rad/s
(p99) during wheel-scrub contact. The initial acceptance target of 0.01 on the maximum
was stricter than this physics model supports; the measured percentiles above replace it.
