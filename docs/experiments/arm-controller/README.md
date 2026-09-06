# Arm commissioning evidence — 2026-09-06

Tests ran in the native Unity 6000.5.2f1 Editor and the existing Linux Jazzy development
container, using JointTrajectoryController 4.40.1. No MoveIt launch was used.
The tested saved configuration is in [the arm technical document](../../unity-arm-controller.md).

## Quantitative results

Each maximum is over all six joints in that record. RMS is the largest per-joint RMS,
not the RMS of the six-joint norm. Values are radians.

| Experiment | Maximum absolute position error | Largest joint RMS | Raw record |
|---|---:|---:|---|
| Unloaded startup, first 5 s | 0.00052 | 0.00040 | [unloaded-startup.csv.gz](unloaded-startup.csv.gz) |
| Loaded startup, first 5 s | 0.00046 | 0.00017 | [final-loaded-startup.csv.gz](final-loaded-startup.csv.gz) |
| Unloaded coordinated trajectory | 0.04132 | 0.00832 | [unloaded-trajectory.csv.gz](unloaded-trajectory.csv.gz) |
| Final loaded coordinated trajectory | 0.03946 | 0.01049 | [final-loaded-trajectory.csv.gz](final-loaded-trajectory.csv.gz) |
| Unloaded upright base disturbances | 0.00243 | 0.00055 | [unloaded-disturbance.csv.gz](unloaded-disturbance.csv.gz) |
| Loaded upright base disturbances | 0.01870 | 0.00340 | [final-loaded-disturbance.csv.gz](final-loaded-disturbance.csv.gz) |
| Loaded contact-free extended HOLD | 0.05269 | 0.02622 | [final-loaded-extended-hold.csv.gz](final-loaded-extended-hold.csv.gz) |
| Loaded extended base disturbances | 0.05222 | 0.02613 | [final-loaded-extended-disturbance.csv.gz](final-loaded-extended-disturbance.csv.gz) |
| Cancellation | 0.00377 | 0.00053 | [cancellation.csv.gz](cancellation.csv.gz) |
| Command-loss watchdog | 0.00091 | 0.00059 | [watchdog.csv.gz](watchdog.csv.gz) |

Loaded startup was measured from the first fixed tick in the authored upright pose;
all initial targets were the sampled physical positions with zero desired velocity.
This establishes no zero-target snap in that scene, not arbitrary imported-pose validation.

Final loaded extension and return, coordinated waypoints, independent +0.05 rad commands
for all six joints, and the one-radian wrist test returned JTC status 4/SUCCESS.
The one-radian sample was 1.0000304 rad, with drive target 57.29578 degrees and local
quaternion `(0,-0.479442,0,0.877574)`, independently confirming the physical rotation.
All six signs are +1 after mapping URDF +Z to Unity local -Y and URDF +Y to Unity +X.

Cancellation returned status 5/CANCELED after about 2.05 wall seconds. The arm stopped
near 0.0271 rad on the pan joint, rather than completing the 0.3 rad target. See
[cancel-result.json](cancel-result.json). JTC continued finite position HOLD afterward.
Stopping the arm launch triggered WATCHDOG_HOLD; desired velocities stayed zero and
positions were captured from actual joints rather than reset to zero.

Disabling Unity's arm transport separately stopped physical-state publication. The
hardware reported an error, returned to unconfigured state, and all command/state
interfaces became unavailable and unclaimed. Unity remained WATCHDOG_HOLD with less
than 0.00039 rad maximum error at the follow-up sample. Restarting the manager and a
fresh Play session is the documented recovery. Invalid unknown-name, NaN and out-of-
limit packets were rejected by the live actuator; pure tests also cover duplicate
names and arbitrary packet ordering. No valid target was partially applied.

## Loaded HOLD details and practical limits

At the contact-free 1.3 rad shoulder pose the panel bottom was approximately 0.242 m
above ground. The shoulder's RMS error was about 0.0262 rad stationary and 0.0261 rad
during base disturbance. Instantaneous maxima reached 0.0527 and 0.0522 rad; this residual
numerical/contact jitter is not hidden by reporting only settled samples. The shoulder
briefly reached its 160 Nm limit for about 0.10%/0.08% of the stationary/disturbance
records. It did not freely swing or lose HOLD. The accepted 0.04 rad JTC goal tolerance
is an action condition, not a promise that every subsequent physical sample stays
inside that error.

Final loaded trajectory maximum shoulder drive contribution was 118.29 Nm, with zero
near-limit samples in that record. The earlier capacity-selection trial had larger
transient error (0.1102 rad), remained inside the 0.15 rad path envelope and completed
successfully. These are simulation observations, not actuator hardware ratings.

The full horizontal straight-arm panel touched the floor. Its apparently reduced
holding torque includes ground support and is **excluded** from unsupported gravity
validation. The source-model gravity bound is still calculated at full extension;
physical validation uses the difficult contact-free pose. Payload shape/attachment and
base height must be reconsidered before requiring that straight horizontal pose.

The upright disturbance sequence commanded ±0.2 m/s, ±0.3 rad/s and a 0.15/0.15 curve.
Measured loaded peaks were 0.1991 m/s and 0.3730 rad/s. The extended sequence commanded
±0.1 m/s and ±0.1 rad/s; measured peaks were 0.1138 m/s and 0.1328 rad/s. Base controller
acceleration/braking settings stayed 0.5/0.8 m/s² and 0.8/1.2 rad/s². Raw finite-difference
acceleration peaks include contact/solver jitter (up to 1.264 m/s² and 3.279 rad/s² in
loaded upright tests), so they do not equal the commanded rate-limit values.

## Tuning trials retained

- `loaded-extended`: provisional 120 Nm shoulder, 6/1 solver; loaded goal tolerance
  failed with roughly 0.05 rad shoulder droop.
- `loaded-extended-solver12`: unchanged gains/torques, 12/4 solver; difficult extension
  succeeded with roughly 0.026 rad shoulder droop.
- `loaded-extended-disturbance` includes the first attempted return and its path failure;
  it is not the final disturbance acceptance record.
- `loaded-return-slow` and `loaded-return-dt10ms`: 120 Nm trials failed JTC path tolerance;
  neither slow timing alone nor a smaller timestep established adequate loaded return.
- `loaded-capacity160`: finite 160 Nm shoulder with the same gains and 20 ms step passed
  the loaded out-and-back action. Final saved torque limits are 120/160/90/40/30/20 Nm.
- `loaded-horizontal-hold`: floor contact; retained as rejected test geometry.

Earlier files use EXTERNAL_CONTROL for stationary ROS targets. Final code labels a
zero-velocity supplied target HOLD; both physically maintain the desired pose and both
remain subject to the watchdog. This state-label change adds no trajectory semantics.

## Timing and analysis

Configured manager and physics rates are 50 Hz; observed command/state delivery was
27.73/45.72 Hz during Editor operations and 29.93/46.59 Hz in the quiet benchmark.
Maximum interarrival gaps were 0.304/0.257 seconds under Editor activity. See
[transport-rates.json](transport-rates.json) and
[transport-rates-idle.json](transport-rates-idle.json). These are bounded receiver
benchmarks, not latency guarantees or real-time bus certification.

[metrics.json](metrics.json) includes per-joint error, velocity error, effort, estimated
saturation, final two-second mean, envelope overshoot and settling observations.
A null settling value means the record does not establish two seconds continuously
inside both the position and speed criterion; it is not silently treated as settled.
No exact coupled-system damping-ratio identification or motor-current measurement is
claimed. Explicit gravity feedforward remains a future option if these residuals do
not meet a later manipulation accuracy requirement.

Records retain the first 120 simulated seconds at most, to exclude prolonged idle
logging during an interrupted session. Startup acceptance uses the first 5 seconds.
The final upright disturbance record uses its first 30 seconds so it excludes a later
extension command. [record-windows.json](record-windows.json) records original/retained
row counts and window lengths. Retained samples are losslessly gzip-compressed without
numeric rounding or downsampling. Failed-trial and final records are distinguished.

Reproduce the summaries from the repository root:

```bash
python3 tools/analyze_arm_experiments.py docs/experiments/arm-controller
python3 tools/calculate_arm_actuators.py
```

## Implementation inventory and checks

Created ROS package: `ros2_ws/src/mobile_manipulator_control` (hardware plugin, model
publisher, launch/configuration, packet tests, bounded action client and rate benchmark).
Controllers: `arm_controller` and `arm_joint_state_broadcaster`. Interfaces and full
per-joint gain table are in [the technical document](../../unity-arm-controller.md).

Created Unity runtime scripts: `ArmActuatorController`, `ArmRosTransport`,
`ArmTrackingRecorder`; Editor composition: `ArmControlSetup`; tests: `ArmCommandTests`.
Modified `MobileManipulatorRosSetup` and `ConstructionSiteV1` to use the new actuator.
The old standalone hold script remains for legacy assets but is removed from the active
scene and rejected as a competing enabled owner. No base-controller source changed.

Updated the authoritative description generator, generated ROS/Unity URDFs and
validation expectation for the 160 Nm shoulder assumption. Added calculation/analysis
tools, this evidence set, the dedicated controller document and ADR 0004. Updated the
design ledger, stack runbook and base-controller integration note.

Verification: ROS description/control build and colcon tests passed; the available
workspace test-result summary reported 10 tests with no failures. All 41 project Unity
EditMode tests passed, including the three new mapping/unit tests. Runtime checks
covered the scenarios above. MoveIt configuration, full-workspace collision/grasp
semantics, real actuator identification and guaranteed 50 Hz delivery remain outside
this commissioned baseline.


## Extension and shutdown follow-up

The requested extension needs an explicit panel orientation. With every distal joint
at zero, pitching the shoulder to 90 degrees also pitches the fixed panel vertically.
The shoulder axis is 0.62 m above the nominal ground and half the panel is 0.60 m:
only 0.02 m remains before sag or base pitch. The observed floor contact is therefore
consistent with the model, rather than evidence of insufficient actuator torque.

For shoulder angle `s`, wrist-2 angle `w`, and other arm joints zero, the current URDF
and saved collider give the following nominal planar geometry (metres, radians):

```text
pitch = s + w
x_tool_from_shoulder = 0.70 sin(s) + 0.19 sin(pitch)
z_tool = 0.62 + 0.70 cos(s) + 0.19 cos(pitch)
z_panel_bottom = z_tool + 0.035 cos(pitch)
                 - 0.60 abs(sin(pitch)) - 0.02 abs(cos(pitch))
```

| Pose | Shoulder / wrist-2 | Tool reach from shoulder | Panel bottom |
| --- | --- | --- | --- |
| Straight arm, vertical panel | +90 / 0 degrees | 0.890 m | 0.020 m |
| Previous extension test | 1.3 / 0 radians | 0.858 m | 0.284 m |
| Extended proximal arm, level panel | +90 / -90 degrees | 0.700 m | 0.825 m |

These are nominal calculations, not new dynamic acceptance results. The previous
1.3 rad trial measured about 0.242 m after physical deflection, versus 0.284 m nominal.
The level-panel option remains to be tested for the entire swept path, self-collision,
scene clearance, tracking and stability. It sacrifices 0.19 m of tool-centre reach;
the panel's forward edge reaches 1.30 m from the shoulder.

Recommended next work:

1. Keep the current robot geometry and validate coordinated shoulder/wrist motion
   that keeps the panel level. Include the panel as an attached collision object in
   the ROS planning scene, plus the floor and obstacles, and enforce an explicit
   clearance margin across the path. MoveIt supports
   [attached collision objects](https://moveit.picknik.ai/main/doc/examples/subframes/subframes_tutorial.html).
2. If a straight arm with a vertical panel is a task requirement, evaluate raising
   the arm pedestal by approximately 0.15–0.20 m or changing the grasp position.
   That would give 0.17–0.22 m nominal floor clearance for the straight pose before
   deflection. Update mass/inertia, collision geometry and both robot descriptions;
   recheck tipping margin and loaded base disturbance before selecting a design.
3. Investigate timing with simultaneous simulation-time and wall-time counters at
   each boundary. `/clock` currently publishes in `Update`, and the connector drains
   incoming messages in `Update`, while the arm runs in `FixedUpdate`. These are
   concrete places to investigate the observed 28–30 Hz command reception. Existing
   receiver tests use depth one and may miss messages themselves; they do not prove
   the controller's actual tick rate. Compare a quiet Editor with a standalone player
   before changing scheduling or choosing a guaranteed operating rate.
4. Revalidate loaded tracking in the chosen collision-free extension, including
   stationary hold and base starts/stops. Define allowable continuous hold error
   separately from JTC's action-completion tolerance before further gain changes.

On 2026-09-06 the captured error log contained 570 repeated editor-owned
`UnityConnectWebRequestException: Token Exchange failed due a failure with the web request`
entries and one project-owned `IndexOutOfRangeException`. The latter stack was
`ArmActuatorController.OnDisable -> CaptureHold -> Position -> ArticulationReducedSpace.get_Item`.
The lifecycle fix skips physical hold capture during application/Play shutdown and
validates each native reduced-coordinate buffer before reading it. Transport also
skips hold during shutdown and tolerates an already-destroyed connection; recording
skips faulted actuators.

Verification after the fix: two Play start/stop cycles, controller disable/re-enable,
transport disable/re-enable, finite six-joint hold targets with zero desired velocities,
and all 41 project EditMode tests passed. The captured error buffer was empty after
the two cycles. The editor cloud-authentication failures did not recur during that
check, but no account/network repair was made and their root cause is not established.
