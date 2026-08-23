# ADR 0001: Unity-ROS time, robot state, and TF ownership

## Status

Accepted for the initial mobile-manipulator simulation.

## Context

Unity owns physics and sensor realization. ROS 2 will own later motion planning through MoveIt 2. The simulation has no SLAM or localization system, and duplicate TF publishers would make frame authority ambiguous.

## Options considered

1. Add UnitySensors `TFLink` components to every robot link and publish the complete tree from Unity.
2. Publish joint states and base ground truth from Unity, then let ROS `robot_state_publisher` derive the URDF tree.
3. Implement kinematics and the complete TF tree in project-owned Unity code.

## Decision

Choose option 2.

- Unity publishes `/clock`, `/joint_states`, and dynamic `odom -> base_footprint`.
- `/joint_states` contains the six arm joints and four wheel joints.
- ROS publishes static identity `map -> odom`.
- ROS `robot_state_publisher` owns all transforms described by the URDF.
- UnitySensors `TFLink` components are removed from the configured robot.
- The Unity world origin is both `map` and `odom` during an experiment run.

The clock is scene-owned and unique. Joint-state and base-ground-truth publishers are robot-owned.

## Consequences

- The TF tree has one authority per edge.
- Wheel transforms remain complete and observable without affecting the arm-only MoveIt group.
- ROS planning consumes standard `sensor_msgs/JointState` and TF interfaces.
- Scene reset or robot teleport is treated as a new simulation epoch; the initial workflow restarts time/TF-dependent ROS nodes.
- The base publisher derives `base_footprint` from the physical `base_link` articulation and the fixed URDF offset.

## Validation

- Verify monotonically increasing `/clock`.
- Verify ten unique names and finite values on `/joint_states`.
- Verify `map -> odom -> base_footprint -> base_link` and both sensor/arm branches with TF2.
- Verify no UnitySensors `TFLink` remains in the robot setup.
- Verify lidar messages use `velodyne_link`.

## Revisit when

- Hot resets must preserve a running ROS graph.
- SLAM/localization is introduced.
- Multiple robots share one scene.
- ros2_control becomes the state or command authority.
