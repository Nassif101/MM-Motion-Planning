# Mobile manipulator design ledger

## Robot metadata

- Robot name: `mobile_manipulator`
- Target consumers: Unity URDF Importer/ArticulationBody, RViz, `robot_state_publisher`, and later MoveIt 2.
- Units: URDF uses metres, kilograms, seconds, and radians. CAD and STL sources use millimetres.
- Frame convention: REP-103 body convention: +X forward, +Y left, +Z up.
- Dimension source: first-pass project assumptions chosen for a compact indoor/outdoor research platform.
- Mobility: four-wheel skid steer. All four wheel joints rotate about +Y; positive joint rotation produces forward chassis motion under the no-slip convention.
- Arm layout: six revolute joints with neutral-axis sequence Z-Y-Y-Z-Y-Z.

## CAD brief

- Model: four-wheel mobile base with a six-axis serial arm, new parametric assembly.
- Coordinate convention: assembly origin at `base_link`; +X forward, +Y left, +Z up.
- Base chassis: 850 x 550 x 220 mm; chassis origin at its centre.
- Wheels: 280 mm diameter, 90 mm width; axle centres at X +/-300 mm, Y +/-320 mm, Z -70 mm relative to `base_link`.
- Base ground clearance: 100 mm. `base_link` is 210 mm above `base_footprint`.
- Arm pedestal: mounted 80 mm rearward of chassis centre; shoulder-pan axis is 290 mm above the chassis centre.
- Arm reach from shoulder-lift axis to `tool0`: 890 mm in the upright neutral pose (1,010 mm from the shoulder-pan axis).
- Outputs: labeled neutral-pose STEP assembly, link-local STEP/STL visual geometry, generated URDF, Unity asset copy.
- Validation targets: assembly dimensions and labels; ten movable joints; connected URDF tree; normalized axes; positive inertials; mesh scale 0.001; Unity importer load.

## Link ledger

| Link | Role and frame definition | Parent joint | Geometry | Inertial source |
|---|---|---|---|---|
| `base_footprint` | Frame-only root at ground directly below `base_link` | none | none | omitted intentionally |
| `base_link` | Physical chassis frame at chassis geometric centre | `base_footprint_joint` | link-local CAD mesh; box collision | estimated box |
| `front_left_wheel_link` | Wheel centre, axes parallel to base | `front_left_wheel_joint` | shared wheel CAD mesh; cylinder collision | estimated Y-axis cylinder |
| `front_right_wheel_link` | Wheel centre, axes parallel to base | `front_right_wheel_joint` | shared wheel CAD mesh; cylinder collision | estimated Y-axis cylinder |
| `rear_left_wheel_link` | Wheel centre, axes parallel to base | `rear_left_wheel_joint` | shared wheel CAD mesh; cylinder collision | estimated Y-axis cylinder |
| `rear_right_wheel_link` | Wheel centre, axes parallel to base | `rear_right_wheel_joint` | shared wheel CAD mesh; cylinder collision | estimated Y-axis cylinder |
| `arm_mount_link` | Pedestal datum on chassis top | `arm_mount_joint` | pedestal CAD mesh; cylinder collision | estimated cylinder |
| `shoulder_pan_link` | J1 axis at pedestal top | `shoulder_pan_joint` | rotary housing CAD mesh; cylinder collision | estimated cylinder |
| `upper_arm_link` | J2 axis; link extends along local +Z | `shoulder_lift_joint` | upper-arm CAD mesh; box collision | estimated box |
| `forearm_link` | J3 axis; link extends along local +Z | `elbow_joint` | forearm CAD mesh; box collision | estimated box |
| `wrist_1_link` | J4 roll axis; link extends along local +Z | `wrist_1_joint` | wrist CAD mesh; cylinder collision | estimated cylinder |
| `wrist_2_link` | J5 pitch axis; link extends along local +Z | `wrist_2_joint` | wrist CAD mesh; cylinder collision | estimated cylinder |
| `wrist_3_link` | J6 tool-roll axis; flange extends along local +Z | `wrist_3_joint` | wrist/flange CAD mesh; cylinder collision | estimated cylinder |
| `tool0` | Frame-only tool centre at flange face | `tool0_joint` | none | omitted intentionally |
| `top_sensor_mount_link` | Frame-only sensor datum on front deck | `top_sensor_mount_joint` | none | omitted intentionally |
| `livox_frame` | Livox Mid-360 measurement/raycast frame, 47 mm above the top mechanical mount | `livox_joint` | none; sensor frame only | omitted intentionally |
| `front_sensor_mount_link` | Frame-only sensor datum on front face | `front_sensor_mount_joint` | none | omitted intentionally |
| `tool_sensor_mount_link` | Frame-only sensor datum coincident with `tool0` | `tool_sensor_mount_joint` | none | omitted intentionally |

## Joint ledger

| Joint | Type | Parent -> child | Origin xyz (m) | Axis | Limits (rad) | Positive motion |
|---|---|---|---|---|---|---|
| `base_footprint_joint` | fixed | footprint -> base | 0 0 0.21 | - | - | fixed |
| `front_left_wheel_joint` | continuous | base -> FL wheel | 0.30 0.32 -0.07 | 0 1 0 | continuous | drives forward |
| `front_right_wheel_joint` | continuous | base -> FR wheel | 0.30 -0.32 -0.07 | 0 1 0 | continuous | drives forward |
| `rear_left_wheel_joint` | continuous | base -> RL wheel | -0.30 0.32 -0.07 | 0 1 0 | continuous | drives forward |
| `rear_right_wheel_joint` | continuous | base -> RR wheel | -0.30 -0.32 -0.07 | 0 1 0 | continuous | drives forward |
| `arm_mount_joint` | fixed | base -> mount | -0.08 0 0.11 | - | - | fixed |
| `shoulder_pan_joint` | revolute | mount -> shoulder | 0 0 0.18 | 0 0 1 | +/-2.967 | CCW viewed from +Z |
| `shoulder_lift_joint` | revolute | shoulder -> upper arm | 0 0 0.12 | 0 1 0 | -1.745..1.745 | tips arm toward +X |
| `elbow_joint` | revolute | upper arm -> forearm | 0 0 0.32 | 0 1 0 | -2.356..2.356 | bends forearm toward +X |
| `wrist_1_joint` | revolute | forearm -> wrist 1 | 0 0 0.28 | 0 0 1 | +/-3.142 | CCW viewed from +Z |
| `wrist_2_joint` | revolute | wrist 1 -> wrist 2 | 0 0 0.10 | 0 1 0 | +/-2.094 | tips tool toward +X |
| `wrist_3_joint` | revolute | wrist 2 -> wrist 3 | 0 0 0.10 | 0 0 1 | +/-6.283 | CCW viewed from +Z |
| `tool0_joint` | fixed | wrist 3 -> tool0 | 0 0 0.09 | - | - | fixed |
| `livox_joint` | fixed | top sensor mount -> Livox Mid-360 measurement frame | 0 0 0.047 | - | - | fixed |
| sensor mount joints | fixed | named physical parent -> mount frame | see generator | - | - | fixed |

Joint origins are expressed in the parent frame. Each child link frame is coincident with its joint frame. Movable axes are expressed in the joint frame.

### Arm motion-limit contract

The URDF position, velocity, and effort values are hard description limits. Acceleration and jerk are provisional planning/controller limits for the generic research arm; they are not certified actuator ratings. Deceleration uses the same magnitude as acceleration unless a future controller contract specifies a smaller safe-stop value.

| Joint | Position range (rad) | Max velocity (rad/s) | Max acceleration/deceleration (rad/s^2) | Max jerk (rad/s^3) | Max effort (N m) |
|---|---:|---:|---:|---:|---:|
| `shoulder_pan_joint` | -2.967..2.967 | 1.8 | 2.0 | 10.0 | 120 |
| `shoulder_lift_joint` | -1.745..1.745 | 1.6 | 1.5 | 7.5 | 160 |
| `elbow_joint` | -2.356..2.356 | 1.8 | 2.0 | 10.0 | 90 |
| `wrist_1_joint` | -3.142..3.142 | 2.5 | 3.0 | 15.0 | 40 |
| `wrist_2_joint` | -2.094..2.094 | 2.5 | 3.0 | 15.0 | 30 |
| `wrist_3_joint` | -6.283..6.283 | 3.2 | 4.0 | 20.0 | 20 |

With the reference panel attached, motion planning starts with both maximum velocity and maximum acceleration scaling factors at 0.5. Jerk limits are not scaled independently in the initial contract. `mobile_manipulator_moveit_config/config/joint_limits.yaml` carries the acceleration and jerk values (kept equal to this table by `test_moveit_config.py`); the later ros2_control hardware/controller configuration must enforce the final hardware-qualified limits. The URDF cannot represent acceleration or jerk.

### Standard DH arm model

The Denavit-Hartenberg representation uses the standard convention

`A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`.

The DH base frame is located at the `shoulder_pan_joint` axis with fixed transform `base_link -> dh_base = xyz(-0.08, 0, 0.29)`, `rpy(0, 0, 0)`. Joint variables `q_i` have the same sign and zero values as the corresponding URDF joints. Distances are metres and angles are radians.

| i | URDF joint | `theta_i` | `d_i` | `a_i` | `alpha_i` |
|---:|---|---:|---:|---:|---:|
| 1 | `shoulder_pan_joint` | `q1 + pi` | 0.12 | 0 | `+pi/2` |
| 2 | `shoulder_lift_joint` | `q2 + pi/2` | 0 | 0.32 | 0 |
| 3 | `elbow_joint` | `q3 + pi/2` | 0 | 0 | `+pi/2` |
| 4 | `wrist_1_joint` | `q4 + pi` | 0.38 | 0 | `+pi/2` |
| 5 | `wrist_2_joint` | `q5 + pi` | 0 | 0 | `+pi/2` |
| 6 | `wrist_3_joint` | `q6` | 0.19 | 0 | 0 |

The final DH frame is `tool0`; no additional tool transform is required. At `q = [0, 0, 0, 0, 0, 0]`, `base_link -> tool0` is `xyz(-0.08, 0, 1.30)`, `rpy(0, 0, 0)`. The DH origins deliberately do not all coincide with URDF joint origins: intersecting coaxial/perpendicular axes allow the J3-to-J5 axial distances to combine into `d4 = 0.38`, and the J5-to-J6/tool distances to combine into `d6 = 0.19`. Automated forward-kinematics comparison guards the equivalence. The generated URDF remains authoritative if a discrepancy is ever found.

### Base geometry and motion-limit contract

- Drive model: four-wheel skid steer, represented to planners/controllers as nonholonomic differential drive; commanded lateral velocity is always zero.
- Wheel radius: 0.14 m. Longitudinal wheel-centre separation: 0.60 m. Transverse wheel-centre separation: 0.64 m.
- Bare collision extents, including wheels: X = +/-0.44 m and Y = +/-0.365 m, a bounding rectangle of 0.88 x 0.73 m.
- Bare operational footprint with 0.02 m perimeter allowance: `[[0.46, 0.385], [0.46, -0.385], [-0.46, -0.385], [-0.46, 0.385]]` in `base_footprint`.
- Hard wheel-joint limits remain 18 rad/s and 85 N m per wheel. The body-level limits below are the normal operating envelope and take precedence for command generation.

| Base quantity | Positive maximum | Negative maximum | Unit |
|---|---:|---:|---|
| Longitudinal velocity `vx` | 0.8 | -0.5 | m/s |
| Lateral velocity `vy` | 0 | 0 | m/s |
| Yaw velocity `wz` | 0.8 | -0.8 | rad/s |
| Longitudinal acceleration/deceleration | 0.5 | -0.8 | m/s^2 |
| Yaw acceleration/deceleration | 0.8 | -1.2 | rad/s^2 |
| Longitudinal jerk | 1.0 | -1.0 | m/s^3 |
| Yaw jerk | 2.0 | -2.0 | rad/s^3 |

The Unity drive controller ignores unsupported Twist DOFs, limits wheel commands consistently with the body envelope, and treats a command older than 0.5 s as stale before braking to rest within the deceleration limits. Commissioning identified a 1.50 m effective track for the initial isotropic-friction skid model while retaining the measured 0.64 m wheel spacing. Simultaneous maximum forward and yaw commands request 10 rad/s at the faster-side wheels with that model track, so the 8 rad/s operating wheel cap scales both sides by 0.8 and preserves their ratio. The 18 rad/s value remains a hard joint guard.

## Geometry and inertial ledger

- All visual meshes are generated in millimetres at the owning link frame and referenced with scale `0.001 0.001 0.001`.
- All collision geometry is deliberately simplified to URDF boxes or cylinders.
- Mass, centre of mass, and inertia values are engineering estimates, not measured hardware data.
- The estimated URDF mass is 107 kg before sensors and payload: 55 kg chassis, four 4 kg wheels, 10 kg arm mount, and 26 kg across the six arm links.
- Box and cylinder inertias are calculated analytically in SI units around each declared COM.
- Off-diagonal inertia terms are zero because the approximations are symmetric about the declared inertial frame.

## Assumptions and limitations

- This is a research simulation platform, not a certified mechanical design.
- Real tire coefficients and motor/transmission data remain unavailable. Unity uses explicitly documented simulation assumptions for wheel-ground friction and drive torque; these are not hardware specifications.
- The arm is not based on a named commercial robot; limits and effort ratings are provisional.
- The self-collision matrix is in `mobile_manipulator_moveit_config/config/mobile_manipulator.srdf`, generated with the MoveIt Setup Assistant's sampling (`generate_collision_matrix`, 10000 trials); regenerate it after any collision-geometry change.
- The Unity copy is generated from this ROS package and must not become a second source of truth.
- The `livox_frame` offset is measured from the installed UnitySensorsROS Mid-360 prefab: its sensor/raycast child is 47 mm above the prefab's mechanical root.

## Sensor-frame contract

| Frame | Parent | Parent-relative xyz (m) | Status and semantic role |
|---|---|---|---|
| `top_sensor_mount_link` | `base_link` | 0.24 0 0.13 | Mechanical mounting datum; not a measurement frame |
| `livox_frame` | `top_sensor_mount_link` | 0 0 0.047 | Active Livox Mid-360 point-cloud measurement/raycast frame |
| `front_sensor_mount_link` | `base_link` | 0.44 0 0.02 | Reserved mechanical datum; no sensor or topic assigned |
| `tool_sensor_mount_link` | `tool0` | 0 0 0 | Reserved tool-sensor datum; no sensor or topic assigned |

The active `livox_frame` is at `xyz(0.24, 0, 0.177)` relative to `base_link`, or 0.387 m above `base_footprint` in the neutral chassis pose. `/livox/lidar` uses this exact frame. Reserved mount frames must not be used as message `frame_id` values until a concrete sensor and its measurement-origin transform are defined. The deferred IMU will receive a dedicated `imu_link`; it must not reuse a generic mount-frame name.

## Unity-ROS runtime contract

- Unity is the simulation-time authority and publishes `/clock`.
- Unity publishes all ten movable joints on `/joint_states`: six arm joints and four continuous wheel joints.
- `robot_state_publisher` owns the URDF-derived fixed and movable link transforms.
- Unity publishes only the ground-truth dynamic transform `odom -> base_footprint`; it does not publish per-joint TF.
- Unity also publishes ground-truth `nav_msgs/Odometry` on `/odom` (`odom -> base_footprint`, 50 Hz) from the same physics sample and stamp as that transform. Pose equals the TF exactly; twist is the PhysX body velocity at `base_footprint`, expressed in `base_footprint`, with a fixed 1e-6 diagonal covariance. It agrees with TF finite differences to 0.002 (p50) and about 0.02 m/s / 0.025 rad/s (p99), the tail coming from solver position correction ([evidence](../../../../docs/experiments/odometry/README.md)). No ROS node publishes `/odom` or fuses odometry.
- ROS publishes a static identity transform `map -> odom`.
- The Unity world origin is coincident with `map` and `odom` for an experiment run.
- `odom -> base_footprint` is derived from the physical `base_link` articulation pose and the fixed `base_footprint_joint`, not from the non-articulated Unity parent transform.
- A scene/robot reset starts a new simulation epoch. The initial implementation restarts the ROS simulation nodes rather than preserving odometry continuity across a teleport or backward clock jump.
- UnitySensors `TFLink` components are not used on this robot, preventing a second TF authority.
- Unity does not subscribe to `/tf`: the ROS-TCP-Connector TF listener (`listenForTFMessages`) is disabled in `ROSConnectionPrefab`, so ROS transforms are not streamed back into Unity.
- Every Unity-published stamp uses the canonical integer-nanosecond physics clock that also drives `/clock` (`RosTimeUtility.PhysicsTimeSeconds`): `/tf`, `/joint_states`, `/arm/state`, and `/livox/lidar`. `/livox/lidar` carries the physics tick of the sample's raycasts through `PhysicsClockSensorTime`, not render-frame `Time.time`; its single-precision UnitySensors time interface limits resolution to about 0.24 ms after one simulated hour. Stamps are never ahead of the latest `/clock` tick (verified live 2026-09-28: TF and joint states exactly on 20 ms ticks, lidar within 1 us).

## Initial panel transport and construction-site experiment contract

- The initial payload proxy is a 1.20 x 1.20 x 0.04 m panel attached to the Unity `tool0` transform. Its broad face lies in tool-local XZ, so the panel plane is orthogonal to the tool's local Y axis; this supersedes the earlier, incorrect local-Z-normal assumption.
- The arm begins upright and planning initially treats the full 1.20 x 1.20 m panel projection as the limiting footprint envelope. This deliberately supersedes the smaller bare-base footprint for clearance checks even where the exact panel projection at a particular arm pose is narrower. The panel is centred on the arm axis, 0.08 m behind `base_footprint`, so the envelope is offset rearward (see the footprint-profile contract below).
- A 0.30 m design margin on each side gives a nominal straight-passage requirement of 1.80 m. The scene's primary transport lane is 2.40 m wide, leaving 0.60 m on each side of the initially oriented panel.
- The square panel's in-plane swept radius is `sqrt(0.60^2 + 0.60^2) = 0.849 m`, and its swept diameter is 1.697 m. With a 0.30 m radial margin, the nominal turning-pocket requirement is 2.297 m; the scene provides a minimum 2.90 m pocket.
- The 1.80 m chicane meets the nominal initial-pose passage requirement. The 1.35 m controlled gate is geometrically passable in the ideal centered initial pose but leaves only 0.075 m per side, below the design margin.
- The 1.05 m manipulation gate cannot admit the initial 1.20 m projected width. It is an intentional experiment feature: a later planner must change panel pose and coordinate base/arm motion to reduce the projected obstruction width.
- `ConstructionSiteV1` uses explicit `Environment`, `Experiment`, `SimulationROS`, and `MobileManipulator` roots. Unity owns the scene geometry, collision proxies, rendering, physics, and sensors; ROS 2 remains responsible for navigation and coordinated motion planning.
- Navigation-relevant scan assets use simple explicit collision proxies. Small rubble, debris, and cones are primarily visual set dressing unless promoted to planning obstacles in a later experiment contract.
- The downloaded FBX models and source textures remain local under the ignored Unity `Assets/Models` tree. The generated scene and material references are reproducible only on workstations that have the same imported asset library; project-owned primitive proxies preserve the clearance geometry without those visuals.
- Current limitation: the panel remains a rigid Unity scene attachment. Its reference mass and inertia are simulated, but grasp/attachment dynamics and configuration-dependent footprint updates remain deferred. Since Phase 2 (2026-10-03) MoveIt carries the panel as a collision object attached to `tool0` (touch links `tool0` and `wrist_3_link`, `panel` subframe at its centre) with the same centre and size (`planning_scene_loader`).

### Reference payload physical properties

- Payload class: lightweight reference panel; this is not a claim about a particular commercial construction panel.
- Collision/visual box: 1.20 x 0.04 x 1.20 m in Unity tool-local XYZ, with the broad face in tool-local XZ.
- Mass: 3.0 kg. Combined robot plus reference payload mass is nominally 110 kg, excluding sensor mass.
- Centre of mass relative to `tool0`: `(0, 0.035, 0)` m, including the existing 15 mm mounting standoff.
- Principal inertia at the payload COM, aligned to `tool0`: `(Ixx, Iyy, Izz) = (0.3604, 0.7200, 0.3604)` kg m^2, calculated as a uniform box.
- The 3.0 kg value is the initial simulated payload and planning reference. Payloads with different mass, COM, inertia, or geometry require a named payload profile; geometry-only scaling is not permitted.

The Unity `tool0` articulation represents the attached panel mass while the panel is rigidly attached. A future grasp/attach implementation must replace that scene-specific fixed assumption and update the MoveIt 2 attached collision object. Arm commissioning revised shoulder lift from 120 to 160 N m after the 105.43 N m loaded gravity calculation and saturated return trials. The tested fully horizontal panel touches the ground; the contact-free difficult pose is 1.3 rad shoulder lift. The 3.0 kg value remains a simulation reference, not a certified full-workspace rating.

## 2026-09-03 Unity skid-steer actuator commissioning

- **Generic command boundary:** accepted `/cmd_vel` Twist instead of a Nav2-specific API or direct planner dependency. Any manual, Nav2, or future MPC/QP publisher can command the same actuator; ROS must provide arbitration if multiple sources exist.
- **Physical actuation:** retained the imported revolute `ArticulationBody` wheels instead of transform motion, direct chassis velocity, or `WheelCollider`. This preserves payload-dependent PhysX response at the cost of contact-model tuning.
- **Finite Force drive:** selected Force mode, zero stiffness, 20 N m per-wheel torque, 20 N m s/rad damping, 0.08 joint friction, and 0.05 wheel-body damping. The alternative 85 N m URDF ceiling did not cure scrub lock and was rejected as the nominal setting. With a 110 kg loaded model, ideal straight acceleration at 0.5 m/s^2 needs only about 1.93 N m per wheel before losses; 20 N m is a provisional simulation margin, not a motor specification.
- **Unity unit boundary:** controller calculations remain rad/s, but revolute drive targets are explicitly converted to degrees/s at the `ArticulationDrive` write. Omitting this conversion produced target tracking near zero in the live scene.
- **Watchdog and shaping:** selected a 0.5 s monotonic timeout for an expected 20 Hz command stream, chassis-space acceleration/deceleration limiting, and common-factor wheel saturation. Holding the last command, independently clipping wheels, and frame-rate-dependent `Update` control were rejected because they respectively permit runaway motion, alter curvature, and make results timing-dependent.
- **Contact model:** retained conventional Physics Materials and rejected a custom tire model for this baseline. Wheel material is 0.9/0.8 static/dynamic; the floor is 0.05/0.02 with `Minimum` combine and zero bounce. Higher floor trials (0.7/0.6, 0.35/0.25, and 0.08/0.05) locked yaw or increased drift; 0.15/0.10 plus 85 N m also remained slow and asymmetric. The selected low floor values are an empirical workaround for isotropic four-wheel scrub, not real soil coefficients.
- **Effective track:** retained the measured 0.64 m track and introduced a distinct 1.50 m model track. Trials found 1.36 m fit 0.4 rad/s pure yaw but under-turned 1 m arcs, while 1.60 m over-rotated; 1.50 m is the initial compromise. Consequence: the maximum combined body command requests 10 rad/s, so the 8 rad/s operating cap engages and scales both sides together.
- **Arm ownership:** the base controller never writes arm drives. Commissioning used a temporary torque-limited arm hold because the current arm drives are otherwise passive and visibly swing during base acceleration. Integrated operation requires a separate ROS-owned arm controller.
- **Observed residuals:** settled +/-0.2 m/s straight tests measured +0.176/-0.177 m/s. +/-0.4 rad/s pure-turn tests measured +0.442/-0.403 rad/s with -0.020/-0.052 m/s longitudinal drift. Nominal 1 m left/right arcs measured 1.054/1.181 m radii. These direction- and curvature-dependent errors are accepted for the initial conventional-friction model and are revisit evidence for a richer tire model.

## 2026-09-28 base braking and acceleration measurement

- **Evidence:** [base-controller measurements](../../../../docs/experiments/base-controller/README.md), loaded robot with `home` arm HOLD, 0.3 and 0.6 m/s out-and-back step tests plus 0.4 rad/s yaw, ground truth from Unity TF.
- **Acceleration:** 0.43-0.49 m/s^2 (10-90 %), tracking the 0.5 m/s^2 limiter.
- **Braking:** mean 0.28-0.65 m/s^2, peaks up to 0.95 m/s^2, with wheel slip; the 0.8 m/s^2 limiter is reached only transiently. Explicit-zero stops from 0.3 m/s travel 0.05-0.07 m; watchdog stops travel 0.18-0.23 m (0.36-0.45 m from 0.6 m/s).
- **Breakaway from rest:** steps of 0.0375 m/s or 0.07 rad/s always start the base; 0.02-0.035 m/s start it only intermittently, and 0.01 m/s or 0.06 rad/s never. Just above breakaway the base creeps at 57-91 % of the command.
- **Goal heading consequence:** the 0.15 rad goal heading tolerance is close to what a sub-breakaway final turn can leave uncorrected; DWB ended 0.144-0.151 rad off and once aborted just short of the goal (nav2-navigation README).
- **Consequence:** the body-level deceleration in the base motion-limit table is an actuator command limit, not a plant capability. ROS-side controllers and safety margins must use the measured values (see the navigation operating envelope). Unity actuator limits are unchanged under the Phase 0 freeze.

## Navigation operating envelope

`mobile_manipulator_navigation/config/nav_operating_envelope.yaml` is the upper bound for every ROS-side base command source (Nav2 controller, velocity smoother, collision monitor, later MPC) while the panel is attached. It is stricter than the Unity actuator limits in the base motion-limit table, which remain guards rather than operating targets.

| Quantity | `home` / `vertical_carry` | Basis |
|---|---:|---|
| Forward / reverse speed | 0.3 / 0.3 m/s | Payload qualification (vertical carry 2026-09-06, home 2026-09-28) |
| Yaw rate | 0.4 rad/s | Same qualification runs |
| Linear accel / decel | 0.45 / 0.5 m/s^2 | Base step measurements 2026-09-28 |
| Yaw accel / decel | 0.8 / 1.0 rad/s^2 | Actuator limiter and measured yaw stops |
| Worst-case planning decel | 0.25 m/s^2 | Slowest measured mean braking |
| Watchdog stop distance from 0.3 m/s | 0.23 m | Measured maximum |
| Breakaway from rest (plant property, not a limit) | 0.0375 m/s, 0.07 rad/s | Low-speed steps 2026-09-28 |

Higher speeds require new payload qualification; `test_operating_envelope.py` rejects an envelope above the recorded `base_commands_tested`.

## Phase 1 navigation stack

`navigation.launch.py` composes global planning, the Livox robot filter, and the ADR 0006 command chain: `controller_server` (20 Hz, STVL local costmap, and one controller selected by `controller:=`, always loaded as `FollowPath` so the behaviour tree and the rest of the chain are shared: Regulated Pure Pursuit for bring-up (0.3 m/s, rotate-to-heading 0.4 rad/s, collision checking to the carrot), DWB as baseline B1 (ObstacleFootprint critic), or Nav2 MPPI as baseline B2 (DiffDrive, 2000 x 56 x 0.05 s, footprint-aware CostCritic); all forward-only and at or below the envelope's velocities, with the velocity smoother enforcing its accelerations on every controller's commands. MPPI keeps the stock forward and yaw acceleration models (3.0 m/s^2, 3.5 rad/s^2) because its first command from rest is bounded by those models applied to the measured velocity and would stay below the base's breakaway; its braking model uses the envelope) and `behavior_server` (Wait only) publish `cmd_vel_nav`; `velocity_smoother` (open loop, 0.3 m/s, 0.4 rad/s, 0.45/0.5 m/s^2, 0.8/1.0 rad/s^2) publishes `cmd_vel_smoothed`; `collision_monitor` (filtered Livox, 0.05-2.0 m) stops inside footprint + 0.05 m, halves speed inside footprint + 0.30 m, applies a footprint approach projection 1.2 s along the current command (including rotation), and alone publishes `/cmd_vel`. Navigation scenario runs record robot-environment contact with the Unity `RobotContactMonitor` and fail on any contact. By default (since 2026-09-30) the global costmap also carries an STVL obstacle layer fed by the filtered Livox cloud (same sensor contract as the local layer), so unmapped obstacles invalidate the global path and trigger replanning; `global_obstacles:=false` restores the static-map-only global costmap used for the Phase 1 controller comparison. `global_planning.launch.py` on its own (planner-only queries) stays static-map only. `bt_navigator` loads only `NavigateToPose`; its default tree keeps the path until the goal changes or the path becomes invalid (a 1 Hz replanning tree is kept as `behavior_tree:=replan_1hz`), uses the Lattice planner (cost penalty 1.0, non-straight penalty 1.2 for straight, centred paths through narrow gaps), and limits recovery to costmap clearing and waiting. `test_navigation_config.py` keeps these values inside the operating envelope.

## Unity-derived Nav2 static-map contract

- The experiment uses no SLAM or sensor-derived mapping. `ConstructionSiteV1` is the static environment source of truth, and Unity exports a standard PGM/YAML occupancy map for ROS 2.
- Only enabled, active, non-trigger colliders below `Environment/NavigationObstacles` contribute static occupancy. Rendered meshes, visual set dressing, the robot, payload, experiment markers, and the ground plane do not.
- The map covers 40 x 40 m at 0.05 m/cell, producing an 800 x 800 grid with origin `[-20, -20, 0]` in `map`. The collider inclusion band is Unity Y = 0.02 through 3.20 m.
- Planar coordinate conversion matches the existing FLU bridge: ROS X = Unity Z and ROS Y = -Unity X. PGM rows are written top-down while ROS occupancy cells are indexed from the lower-left map origin.
- Unity writes `construction_site.pgm`, `construction_site.yaml`, and deterministic metadata into the `mobile_manipulator_navigation` package. The scene build command also regenerates these artifacts, and `validate_nav2_map` rejects stale files.
- Nav2 `map_server` owns `/map` with frame `map`; the global costmap consumes it through a transient-local static layer. Inflation and footprint padding are ROS configuration and are not baked into the exported image.
- No AMCL is started. The simulation continues to use Unity ground-truth `odom -> base_footprint` and the ROS-owned identity `map -> odom`; a planner requires that complete TF chain before activation.
- The global costmap footprint is a named arm-pose profile from `mobile_manipulator_navigation/config/footprint_profiles.yaml`, selected by the `footprint_profile` launch argument, with 0.01 m padding. A later configuration-dependent footprint must use the convex hull of the chassis, arm projection, and panel projection.
- 2026-09-28 correction: the earlier origin-centred 1.20 x 1.20 m square was **not** conservative. Forward kinematics of the home and vertical-carry poses place the panel at X = -0.68..0.52 m in `base_footprint`, 0.07 m behind the old rear edge, and the true circumscribed radius exceeded the 0.90 m inflation radius.

### Footprint-profile contract

Each profile is the axis-aligned ground projection of every URDF collision primitive plus the reference panel in a qualified arm pose, enlarged by the 0.02 m perimeter allowance. `test_footprint_profiles.py` recomputes the bounds from the URDF and `qualified_payload.json` and fails if a profile no longer contains the robot or drifts from the generated bound.

| Profile | Arm pose | Projected bound X / Y (m) | Polygon X / Y (m) | Role |
|---|---|---|---|---|
| `home` | `[0, 0, 0, 0, 0, 0]` | -0.68..0.52 / +/-0.60 | -0.70..0.54 / +/-0.62 | Default conservative Phase 1 baseline; horizontal panel |
| `vertical_carry` | `[pi/2, 0, 0, 0, pi/2, 0]` | -0.68..0.52 / +/-0.365 | -0.70..0.54 / +/-0.385 | Narrow-passage transport profile; panel along chassis X |

The planner server exposes two global planners selected by `planner_id`: `GridBased` (NavFn A*, a point planner that sees the footprint only through inscribed-radius inflation; retained baseline) and `Lattice` (Smac State Lattice with the installed 5 cm differential-drive primitives, full-footprint SE(2) collision checking, in-place rotation allowed, no reverse expansion, no unknown traversal). Gate checks on 2026-09-28: with `home` both planners route around the 1.05 m and 1.35 m gates; with `vertical_carry` both traverse them ([evidence](../../../../docs/experiments/nav2-global-planning/README.md)).

The global inflation radius is 1.00 m, at least the largest profile circumscribed radius (0.935 m) plus padding. Level extension (panel to X = 1.22 m) is not a transport profile.
- `/livox/lidar` remains excluded from the static global map. Its rolling local-costmap consumer follows the local-costmap perception contract below; MoveIt plans against the known geometry (exported boxes, scenario boxes, raised floor); a filtered MoveIt Octomap consumer is the next scene source (Phase 2 plan, Task 8).

## Local-costmap perception contract

Evidence: [Livox observations](../../../../docs/experiments/lidar-local-costmap/README.md) and [local-costmap qualification](../../../../docs/experiments/local-costmap/README.md) (2026-09-28, both footprint profiles). Applies to every local costmap fed by `/livox/lidar`.

- **Filter:** navigation consumers read `/livox/points_filtered` from `livox_robot_filter` (C++, `rclcpp` + Eigen, since 2026-09-30; byte-identical output to the earlier Python node over 819 compared scans, p50 1.6 ms and p99 under 5 ms per scan alone, 3.9 / 6.6 ms under full Nav2 load, against about 15 / 22-25 ms in Python), which removes zero-point misses and self-returns and republishes in `livox_frame`. Self-returns come from a ray-based geometric self-model (URDF collision primitives plus the reference panel, posed from TF): inside a primitive + 0.03 m, or on a ray that first hits the robot and within 0.08 m (4 sigma range noise) in front of that surface. Since 2026-09-28 it no longer removes the whole footprint column, which had hidden obstacles that entered the footprint rectangle (see the Phase 1 navigation evidence). It does not apply a height band: ground returns below the marking height are the rays that clear low voxels near the robot. Consumers apply their own heights.

- **Layer:** Spatio-Temporal Voxel Layer (`spatio_temporal_voxel_layer`, since 2026-09-28), followed by `InflationLayer`. Voxels decay linearly over 10 s, and faster (5 1/s^2) inside a clearing frustum of full azimuth, symmetric +/-7.2 deg, 1.5-6.0 m. The 2D `ObstacleLayer` is not used: a ray passing over a low obstacle would clear its cell, and low obstacles leave the sensor's view as the robot approaches (they are visible only beyond `(0.387 - h) / tan(7.2 deg)`, about 1.5 m for a 0.2 m object). The `VoxelLayer` was qualified first and rejected because removed obstacles stayed marked in open directions (see below); STVL's frustum starts at 1.5 m so blind-zone obstacles are remembered for the full 10 s decay.
- **Misses:** UnitySensors encodes misses and out-of-range returns as zero points at the `livox_frame` origin (about 55 % of each scan). The filter removes them before any consumer. Misses provide no max-range clearing; stale voxels in open directions clear only through rays to real returns. Converting misses to max-range clearing rays is a possible later filter, not part of the baseline. Misses are removed in ROS, not in Unity: dropping them before publishing cut lidar bytes by 57 % but raised endpoint CPU by about 20 points and doubled small-message p99 jitter, because the message length then varies every scan ([transport evidence](../../../../docs/experiments/lidar-transport/README.md)).
- **Height band:** marking uses `min_obstacle_height` 0.05 m (ground returns lie below it) and `max_obstacle_height` 2.0 m (robot and panel top reach 1.92 m in vertical carry), 0.05 m voxels, 5.0 m marking range.
- **Self-returns:** 11-14 % of each scan returns from the chassis deck, arm, and panel (0.32-1.57 m), all inside the matching footprint profile. `footprint_clearing_enabled: true` removes them only while the footprint profile matches the arm pose held in Unity; *Update:* `livox_robot_filter` now removes self-returns with the URDF collision primitives and the panel posed from TF, so returns from any arm pose are filtered, including poses outside the active footprint polygon.
- **Timing and frame:** clouds are stamped with the canonical physics tick and use `livox_frame`; costmap `transform_tolerance` must not be widened to mask missing TF.
- **Measured behaviour:** no self-marking in either profile; obstacles in view marked within 0.3 s (simulation time); a 0.2 m obstacle 1.0 m ahead never seen. With the VoxelLayer, 50-65 % of a removed 0.5 m obstacle's cells stayed lethal in open directions because misses give no clearing rays; with STVL every removed obstacle cleared within 11 s (2-6 s in view).
- **Near-field blind zone:** the ground is visible only beyond about 3.0 m. Controller and collision-monitor margins must not assume that obstacles closer than about 1.5 m below 0.2 m height remain observable; the static map covers static obstacles, and moving low obstacles near the robot are a documented limitation for Phase 1.

## Deferred IMU notes

An IMU is intentionally omitted from the initial robot setup because planning and ground-truth odometry do not require it. When added, use a project-owned implementation rather than the current UnitySensors `IMUSensor` unchanged:

- sample on the Unity physics timestep;
- compute angular velocity and specific force in the IMU-local frame;
- subtract gravity with consistent world/local transforms;
- suppress the uninitialized first measurement;
- use the shared Unity simulation timestamp;
- expose configurable noise and covariance;
- mount through a dedicated fixed `imu_link` only when the physical pose is known.

## 2026-09-06 ros2_control arm commissioning

- **Execution ownership:** adopted Jazzy controller_manager and standard JointTrajectoryController;
  rejected a custom Unity trajectory interpolator. ROS owns time, action status, tolerances,
  cancellation and replacement. Unity owns instantaneous finite-torque physical actuation.
- **Hardware and transport:** added UnityArmSystem with six named position/velocity command and
  state interfaces over the existing ROS-TCP stack. JointState packets carry explicit names;
  no array-order assumption, second transport stack, or fabricated effort feedback is introduced.
- **State authority:** retained Unity's ten-joint `/joint_states`; the standard arm broadcaster
  uses local topics. `/arm/robot_description` carries the control augmentation without another
  TF publisher. Existing ROS description, TF, clock and base-command contracts remain intact.
- **HOLD/watchdog:** replaced the scene's temporary equal-gain hold with ArmActuatorController.
  Startup captures actual position. A 0.5 s monotonic watchdog captures physical position on
  command loss. Drives remain engaged; stale targets, arbitrary zero resets, gravity removal,
  kinematic joints and transform locking were rejected. Invalid configuration is a fault.
- **Gravity and physical properties:** restored gravity on the arm mount, six arm links, tool0
  payload and tool sensor frame. Imported COM and inertias agreed with coordinate-converted
  model data. Retained physical 3 kg panel mass/COM/inertia and existing articulation hierarchy.
- **Actuator assumptions:** finite torque limits are 120/160/90/40/30/20 N m. The old shoulder
  120 N m value had only 14% reserve over the 105.43 N m loaded gravity bound and saturated
  during loaded returns; 160 N m gives 52% reserve and passed the corresponding trajectory.
  These are engineering simulation assumptions, not manufacturer specifications.
- **Per-joint feedback gains:** Kp = 2000/6000/3500/500/650/300 N m/rad and Kd =
  240/360/180/40/35/28 N m s/rad. Initial values follow gravity-error and inertia/damping-ratio
  calculations. Generic angular drag is 0.05, distinct from retained URDF joint friction.
- **Solver evidence:** retained the 20 ms timestep. Default 6/1 solver iterations failed loaded
  extension tolerance; 12/4 reduced shoulder droop without changing gains. A 10 ms trial did
  not resolve the provisional 120 N m return failure. Runtime applies 12/4 explicitly because
  articulation solver settings are not serialized.
- **Base/payload envelope:** base code and acceleration limits are unchanged. Tests include
  unloaded/loaded startup and trajectories, upright acceleration/braking/reversal/yaw/curves,
  and loaded 1.3 rad extension under low-speed base acceleration/yaw. Full horizontal straight
  extension contacts the ground and is excluded from unsupported gravity-hold acceptance.
- **Measured timing:** configuration is 50 Hz; observed command delivery was about 28–30 Hz and
  feedback 46–47 Hz on the commissioning host. TCP and Editor scheduling limit timing claims.
- **Future integration:** MoveIt can use the standard FollowJointTrajectory action with no new
  Unity trajectory code. An alternative ROS MPC controller can reuse instantaneous actuator
  commands. Explicit gravity feedforward remains deferred unless tighter error/payload
  requirements justify it; current operation accepts measured physical droop.

Alternatives, trade-offs and revisit conditions are in [ADR 0004](../../../../docs/adr/0004-ros2-control-unity-arm-actuation.md).
The [arm technical document](../../../../docs/unity-arm-controller.md) includes calculations,
limits, state flow and reproducible tests; [experiment evidence](../../../../docs/experiments/arm-controller/README.md)
records quantitative results and failed trials.
