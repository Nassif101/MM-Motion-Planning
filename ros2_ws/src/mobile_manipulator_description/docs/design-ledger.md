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
- Arm reach from shoulder-lift axis to `tool0`: approximately 990 mm in the upright neutral pose.
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

## Geometry and inertial ledger

- All visual meshes are generated in millimetres at the owning link frame and referenced with scale `0.001 0.001 0.001`.
- All collision geometry is deliberately simplified to URDF boxes or cylinders.
- Mass, centre of mass, and inertia values are engineering estimates, not measured hardware data.
- Box and cylinder inertias are calculated analytically in SI units around each declared COM.
- Off-diagonal inertia terms are zero because the approximations are symmetric about the declared inertial frame.

## Assumptions and limitations

- This is a research simulation platform, not a certified mechanical design.
- Wheel-ground friction, drive controllers, transmissions, ros2_control tags, and sensor payload mass are intentionally deferred.
- The arm is not based on a named commercial robot; limits and effort ratings are provisional.
- No self-collision matrix is defined yet; that belongs to the later SRDF/MoveIt 2 phase.
- The Unity copy is generated from this ROS package and must not become a second source of truth.
- The `livox_frame` offset is measured from the installed UnitySensorsROS Mid-360 prefab: its sensor/raycast child is 47 mm above the prefab's mechanical root.

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

## Deferred IMU notes

An IMU is intentionally omitted from the initial robot setup because planning and ground-truth odometry do not require it. When added, use a project-owned implementation rather than the current UnitySensors `IMUSensor` unchanged:

- sample on the Unity physics timestep;
- compute angular velocity and specific force in the IMU-local frame;
- subtract gravity with consistent world/local transforms;
- suppress the uninitialized first measurement;
- use the shared Unity simulation timestamp;
- expose configurable noise and covariance;
- mount through a dedicated fixed `imu_link` only when the physical pose is known.
