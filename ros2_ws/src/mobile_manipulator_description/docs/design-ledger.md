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
| `shoulder_lift_joint` | -1.745..1.745 | 1.6 | 1.5 | 7.5 | 120 |
| `elbow_joint` | -2.356..2.356 | 1.8 | 2.0 | 10.0 | 90 |
| `wrist_1_joint` | -3.142..3.142 | 2.5 | 3.0 | 15.0 | 40 |
| `wrist_2_joint` | -2.094..2.094 | 2.5 | 3.0 | 15.0 | 30 |
| `wrist_3_joint` | -6.283..6.283 | 3.2 | 4.0 | 20.0 | 20 |

With the reference panel attached, motion planning starts with both maximum velocity and maximum acceleration scaling factors at 0.5. Jerk limits are not scaled independently in the initial contract. A later MoveIt 2 `joint_limits.yaml` must carry the acceleration and jerk values, and the later ros2_control hardware/controller configuration must enforce the final hardware-qualified limits. The URDF cannot represent acceleration or jerk.

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
- No self-collision matrix is defined yet; that belongs to the later SRDF/MoveIt 2 phase.
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
- ROS publishes a static identity transform `map -> odom`.
- The Unity world origin is coincident with `map` and `odom` for an experiment run.
- `odom -> base_footprint` is derived from the physical `base_link` articulation pose and the fixed `base_footprint_joint`, not from the non-articulated Unity parent transform.
- A scene/robot reset starts a new simulation epoch. The initial implementation restarts the ROS simulation nodes rather than preserving odometry continuity across a teleport or backward clock jump.
- UnitySensors `TFLink` components are not used on this robot, preventing a second TF authority.

## Initial panel transport and construction-site experiment contract

- The initial payload proxy is a 1.20 x 1.20 x 0.04 m panel attached to the Unity `tool0` transform. Its broad face lies in tool-local XZ, so the panel plane is orthogonal to the tool's local Y axis; this supersedes the earlier, incorrect local-Z-normal assumption.
- The arm begins upright and planning initially treats the panel as a conservative limiting footprint envelope of 1.20 x 1.20 m. This deliberately supersedes the smaller bare-base footprint for clearance checks even where the exact panel projection at a particular arm pose is narrower.
- A 0.30 m design margin on each side gives a nominal straight-passage requirement of 1.80 m. The scene's primary transport lane is 2.40 m wide, leaving 0.60 m on each side of the initially oriented panel.
- The square panel's in-plane swept radius is `sqrt(0.60^2 + 0.60^2) = 0.849 m`, and its swept diameter is 1.697 m. With a 0.30 m radial margin, the nominal turning-pocket requirement is 2.297 m; the scene provides a minimum 2.90 m pocket.
- The 1.80 m chicane meets the nominal initial-pose passage requirement. The 1.35 m controlled gate is geometrically passable in the ideal centered initial pose but leaves only 0.075 m per side, below the design margin.
- The 1.05 m manipulation gate cannot admit the initial 1.20 m projected width. It is an intentional experiment feature: a later planner must change panel pose and coordinate base/arm motion to reduce the projected obstruction width.
- `ConstructionSiteV1` uses explicit `Environment`, `Experiment`, `SimulationROS`, and `MobileManipulator` roots. Unity owns the scene geometry, collision proxies, rendering, physics, and sensors; ROS 2 remains responsible for navigation and coordinated motion planning.
- Navigation-relevant scan assets use simple explicit collision proxies. Small rubble, debris, and cones are primarily visual set dressing unless promoted to planning obstacles in a later experiment contract.
- The downloaded FBX models and source textures remain local under the ignored Unity `Assets/Models` tree. The generated scene and material references are reproducible only on workstations that have the same imported asset library; project-owned primitive proxies preserve the clearance geometry without those visuals.
- Current limitation: the panel remains a rigid Unity scene attachment. Its reference mass and inertia are simulated, but grasp/attachment dynamics, self-collision allowances, configuration-dependent footprint updates, and the corresponding MoveIt 2 attached collision object are deferred until the manipulation planning contract is implemented.

### Reference payload physical properties

- Payload class: lightweight reference panel; this is not a claim about a particular commercial construction panel.
- Collision/visual box: 1.20 x 0.04 x 1.20 m in Unity tool-local XYZ, with the broad face in tool-local XZ.
- Mass: 3.0 kg. Combined robot plus reference payload mass is nominally 110 kg, excluding sensor mass.
- Centre of mass relative to `tool0`: `(0, 0.035, 0)` m, including the existing 15 mm mounting standoff.
- Principal inertia at the payload COM, aligned to `tool0`: `(Ixx, Iyy, Izz) = (0.3604, 0.7200, 0.3604)` kg m^2, calculated as a uniform box.
- The 3.0 kg value is the initial simulated payload and planning reference. Payloads with different mass, COM, inertia, or geometry require a named payload profile; geometry-only scaling is not permitted.

The Unity `tool0` articulation represents the attached panel mass while the panel is rigidly attached. A future grasp/attach implementation must replace that scene-specific fixed assumption and update the MoveIt 2 attached collision object. Full horizontal extension has limited shoulder-torque margin under the provisional 120 N m effort bound, so controller commissioning must include gravity/dynamic torque analysis rather than treating the 3.0 kg value as a certified full-workspace rating.

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

## Unity-derived Nav2 static-map contract

- The experiment uses no SLAM or sensor-derived mapping. `ConstructionSiteV1` is the static environment source of truth, and Unity exports a standard PGM/YAML occupancy map for ROS 2.
- Only enabled, active, non-trigger colliders below `Environment/NavigationObstacles` contribute static occupancy. Rendered meshes, visual set dressing, the robot, payload, experiment markers, and the ground plane do not.
- The map covers 40 x 40 m at 0.05 m/cell, producing an 800 x 800 grid with origin `[-20, -20, 0]` in `map`. The collider inclusion band is Unity Y = 0.02 through 3.20 m.
- Planar coordinate conversion matches the existing FLU bridge: ROS X = Unity Z and ROS Y = -Unity X. PGM rows are written top-down while ROS occupancy cells are indexed from the lower-left map origin.
- Unity writes `construction_site.pgm`, `construction_site.yaml`, and deterministic metadata into the `mobile_manipulator_navigation` package. The scene build command also regenerates these artifacts, and `validate_nav2_map` rejects stale files.
- Nav2 `map_server` owns `/map` with frame `map`; the global costmap consumes it through a transient-local static layer. Inflation and footprint padding are ROS configuration and are not baked into the exported image.
- No AMCL is started. The simulation continues to use Unity ground-truth `odom -> base_footprint` and the ROS-owned identity `map -> odom`; a planner requires that complete TF chain before activation.
- The initial global costmap footprint is a conservative 1.20 x 1.20 m square with 0.01 m padding. A later configuration-dependent footprint must use the convex hull of the chassis, arm projection, and panel projection.
- `/livox/lidar` remains excluded from the static global map. Its rolling local-costmap and filtered MoveIt planning-scene consumers are deferred until base control, odometry-message, sensor height/range, self-filter, and payload-filter contracts are implemented.

## Deferred IMU notes

An IMU is intentionally omitted from the initial robot setup because planning and ground-truth odometry do not require it. When added, use a project-owned implementation rather than the current UnitySensors `IMUSensor` unchanged:

- sample on the Unity physics timestep;
- compute angular velocity and specific force in the IMU-local frame;
- subtract gravity with consistent world/local transforms;
- suppress the uninitialized first measurement;
- use the shared Unity simulation timestamp;
- expose configurable noise and covariance;
- mount through a dedicated fixed `imu_link` only when the physical pose is known.
