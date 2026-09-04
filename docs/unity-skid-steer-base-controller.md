# Unity skid-steer base controller

## Purpose and ownership

`SkidSteerBaseController` is the low-level physical actuator for the existing four-wheel mobile base. It receives `geometry_msgs/msg/Twist` on `/cmd_vel`; it does not plan, localize, follow paths, arbitrate command sources, or depend on Nav2. ROS 2 owns those policies. Unity owns command-boundary safeguards, the wheel drives, PhysX contact, and measured simulation motion.

The runtime path is:

`ROS publisher -> /cmd_vel -> Unity limiter/watchdog -> skid-steer mapping -> wheel drives -> PhysX contact -> chassis motion`

Nav2 and a future whole-body MPC/QP therefore connect by publishing the same Twist interface.

## Model-derived geometry and mass

| Quantity | Value | Classification |
| --- | ---: | --- |
| Wheel radius | 0.14 m | URDF/model geometry |
| Geometric track | 0.64 m | URDF wheel-centre spacing |
| Wheelbase | 0.60 m | URDF wheel-centre spacing |
| Chassis mass | 55 kg | URDF engineering estimate |
| Wheel mass | 4 kg each | URDF engineering estimate |
| Bare robot mass | 107 kg | Sum of URDF inertials |
| Reference payload | 3 kg | Project simulation assumption |
| Loaded articulation | about 110 kg | Unity/URDF calculation |
| Effective model track | 1.50 m | Experimentally identified compromise |

All four URDF wheel axes are `+Y`. Inspection of the imported articulation frames confirmed that a positive drive velocity rolls every wheel forward, so all four controller direction multipliers are `+1`; there are no hidden side-specific minus signs. Positive ROS yaw commands the left wheels backward and right wheels forward. Because Unity is left-handed, positive ROS yaw appears as negative rotation around Unity local Y; telemetry converts it back to ROS sign.

## Kinematics and command shaping

For limited chassis velocity `v` and yaw rate `w`, the controller uses:

`v_left = v - w * b_eff / 2`

`v_right = v + w * b_eff / 2`

`wheel_left = v_left / r`, `wheel_right = v_right / r`

The 0.64 m geometric track remains visible in configuration. `b_eff = 1.50 m` is a separate kinematic parameter; it does not alter physical wheel locations.

The fixed-timestep sequence is velocity clamp, chassis-space rate limit, inverse kinematics, common-factor wheel saturation, and drive update. Reversal brakes to zero before accelerating in the opposite direction. Unsupported Twist components are ignored.

| Limit | Value |
| --- | ---: |
| Maximum forward speed | 0.8 m/s |
| Maximum reverse speed | 0.5 m/s |
| Maximum yaw rate | 0.8 rad/s |
| Linear acceleration / deceleration | 0.5 / 0.8 m/s^2 |
| Angular acceleration / deceleration | 0.8 / 1.2 rad/s^2 |
| Operating wheel speed | 8 rad/s |
| Hard articulation speed | 18 rad/s |

At the maximum combined `v = 0.8 m/s`, `w = 0.8 rad/s`, the effective model requests `(1.429, 10.0) rad/s`. The operating cap applies a common 0.8 scale, producing `(1.143, 8.0) rad/s` and preserving curvature. The hard 18 rad/s articulation limit is a second guard, not the normal operating limit.

Angular calculations and diagnostics use radians/second. Unity's revolute `ArticulationDrive.targetVelocity` boundary uses degrees/second, so the controller performs one explicit `rad/s * 180/pi` conversion when writing each drive. `ArticulationBody.jointVelocity` remains radians/second.

## Watchdog and thread boundary

The expected command rate is 20 Hz (50 ms period). A valid message stores only `linear.x`, `angular.z`, and a monotonic `Stopwatch` timestamp under a lock; the ROS callback does not touch Unity physics. `FixedUpdate` treats an absent or older-than-0.5 s command as stale and rate-limits toward zero. Five hundred milliseconds permits ten expected command periods while still bounding a disconnect. Disabling the component unsubscribes and writes zero wheel targets.

Non-finite commands are rejected and counted. A valid zero Twist is distinct from command loss, although both ultimately request rest.

## Physical drive and contact assumptions

| Setting | Value | Basis |
| --- | ---: | --- |
| Drive type | Force | Finite-torque physical response |
| Stiffness | 0 | Velocity actuation, no position servo |
| Damping | 20 N m s/rad | Initial stable velocity-error gain |
| Wheel torque limit | 20 N m each | Simulation assumption, below 85 N m URDF hard effort |
| Joint friction | 0.08 | Simulation assumption |
| Wheel body damping | 0.05 | Simulation assumption |
| Wheel material static/dynamic | 0.9 / 0.8 | High-grip tire-side material assumption |
| Floor material static/dynamic | 0.05 / 0.02 | Experimentally selected isotropic scrub compromise |
| Friction combine | Minimum | Makes the lower floor value the contact-pair coefficient |
| Bounce | 0 | Non-bouncing rolling contact |

Exact motor and tire data are unavailable. For the 110 kg loaded model, the ideal force for the 0.5 m/s^2 acceleration limit is `F = ma = 55 N`. Even distribution gives an ideal per-wheel torque of `55 * 0.14 / 4 = 1.93 N m`; the 20 N m limit provides margin for inertia, contact loss, rolling geometry, and skid scrub without using the 85 N m URDF ceiling or an infinite-gain velocity drive.

The existing convex, simplified wheel collision meshes and the construction-floor `BoxCollider` remain in use. No `WheelCollider`, transform motion, direct chassis-velocity assignment, or custom tire force is used.

The low floor coefficients are a simulation-model result, not a claim about real compacted soil. Isotropic PhysX friction couples longitudinal grip and lateral scrub; trials at 0.7/0.6 and 0.35/0.25 locked in-place rotation, 0.15/0.10 with 85 N m remained slow/asymmetric, and 0.08/0.05 increased drift while reducing yaw. The selected 0.05/0.02 pair gave the best stable compromise in this collider model. A future anisotropic tire model may replace it if thesis experiments require hardware fidelity.

## Inspector and setup

The component exposes ROS topic/watchdog, measured and effective geometry, chassis limits, wheel limits, drive parameters, explicit chassis/wheel references, URDF direction multipliers, and optional CSV logging. Run `Tools > Motion Planning > Configure Mobile Manipulator ROS` after reimporting the robot; it wires the controller, optional keyboard tester, torque-limited arm hold, and third-person camera in the sensorized prefab and active scene, and assigns the two project-owned Physics Materials.

CSV diagnostics are off by default. When enabled they log at 10 Hz:

`time, limited v/w, target left/right, actual FL/RL/FR/RR, actual chassis v/w, watchdog, saturation`

## Commissioning procedure

Start the endpoint and Unity as described in `docs/running-the-stack.md`, then publish at 20 Hz. For each test, record a settled interval and allow the watchdog to stop the robot after the publisher exits.

1. Zero and command-loss stop.
2. Straight `(+0.2, 0)` and reverse `(-0.2, 0)`.
3. Pure yaw `(0, +0.4)` and `(0, -0.4)`.
4. One-metre nominal arcs `(+0.2, +0.2)` and `(+0.2, -0.2)`.
5. Step/ramp and reversal.
6. Out-of-envelope velocity clamp and combined-command wheel saturation.

During base-only commissioning, leave `Arm Joint Hold Controller > Hold Arm Joints` enabled. It captures the current six arm-joint positions at Play Mode startup, applies stiffness and damping while preserving the URDF effort limits, and restores the passive drives when disabled. Turn it off before enabling a real ROS arm controller. The base actuator itself still never commands arm joints.

For a Unity-only drive test, enable `Skid Steer Keyboard Teleop > Enable Keyboard Teleop` on the robot root. `W/S` or `Up/Down` command translation, `A/D` or `Left/Right` command yaw, and `Space` commands zero. This explicit test override is disabled by default and returns control to the watchdog-protected ROS input as soon as it is disabled.

## Measured 2026-09-03 results

Measurements used the 110 kg payload-equipped `ConstructionSiteV1` articulation, 20 Hz ROS commands, 0.02 s physics steps, the final contact values above, and a temporary torque-limited arm hold. Values are means over settled CSV samples.

| Test | Command `(v, w)` | Measured `(v, w)` | Additional result |
| --- | --- | --- | --- |
| Forward | `(0.2, 0)` | `(0.176, 0.001)` | Four wheels averaged 1.27-1.31 rad/s |
| Reverse | `(-0.2, 0)` | `(-0.177, 0.002)` | Four wheels averaged -1.26 to -1.28 rad/s |
| Positive pure yaw | `(0, 0.4)` | `(-0.020, 0.442)` | 1.50 m effective-track trial |
| Negative pure yaw | `(0, -0.4)` | `(-0.052, -0.403)` | Both turn directions verified |
| Forward-left arc | `(0.2, 0.2)` | `(0.175, 0.166)` | Radius 1.054 m versus 1.0 m ideal |
| Forward-right arc | `(0.2, -0.2)` | `(0.190, -0.161)` | Radius 1.181 m versus 1.0 m ideal |

The 1.50 m effective track is a compromise: a 1.36 m pure-yaw fit measured 0.400 rad/s exactly but produced 1.21-1.27 m arc radii; a 1.60 m trial produced 0.472 rad/s pure yaw. Curvature- and direction-dependent residuals remain visible, so the effective track must not be mistaken for physical geometry.

The watchdog transitioned in every bounded live test and brought wheel targets back to zero. EditMode tests verify exact 0.02 s rate-limit increments, separate acceleration/deceleration, reversal through zero, clamp values, timeout boundary, drive-unit conversion, and saturation ratio.

## Limitations

- The coefficients and motor torque are simulation assumptions, not measured hardware properties.
- A single effective track cannot model all speeds, radii, payload configurations, terrain, or direction asymmetry.
- The quantitative tests do not replace visual inspection for wheel penetration, high-frequency jitter, or long-duration thermal/contact stability.
- Arm control, command arbitration, odometry-message publication, and Nav2 controller integration are separate work.
- The current controller ignores unsupported Twist DOFs rather than warning on them.
