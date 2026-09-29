# ADR 0008: Cyclone DDS as the ROS 2 middleware

Date: 2026-09-30. Status: accepted.

## Context

All Unity streams (`/clock`, `/arm/state`, `/tf`, `/odom`, `/joint_states`, the lidar
cloud) reach ROS through one ROS-TCP endpoint thread that publishes each message before
reading the next. With Fast DDS (`rmw_fastrtps_cpp`, the previous default) that
`publish()` blocked for about 0.3 s, process-wide, whenever ROS processes started or
exited uncleanly; bursts of these paused every Unity stream for 0.5-3 s, which tripped
the 0.5 s arm feedback timeout at run transitions (docs/experiments/ros-tcp-stalls).
Transport settings (asynchronous publishing, UDP only) and clean Nav2 shutdown did not
remove the pauses.

## Options considered

1. Keep Fast DDS and rely on automatic arm recovery (ADR 0005 amendment).
2. Cyclone DDS (`rmw_cyclonedds_cpp`), already installed.
3. Zenoh (`rmw_zenoh_cpp`), already installed; needs a router process.

## Decision

Use Cyclone DDS for every ROS process in the container: the Dev Container sets
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Keep the arm recovery supervisor as a safety net.

In the same idle, churn, and Nav2 start/stop phases, Cyclone DDS had no Unity stream
pause over 0.15 s, Zenoh one 0.83 s pause at idle, and Fast DDS its usual 0.3 s pauses
(docs/experiments/ros-tcp-stalls). Zenoh also adds a router to start and supervise.

## Consequences

- All ROS processes must use the same middleware; mixing Fast DDS and Cyclone DDS nodes
  is not supported. After changing the Dev Container configuration, rebuild the container
  so every shell and process inherits the setting.
- Earlier evidence recorded with Fast DDS (arm transport rates, Phase 1 navigation runs)
  stays valid as recorded. Re-measured under Cyclone DDS: 50 arm commands/s with gaps up
  to 0.061 s (Fast DDS: 49.7/s, up to 0.094 s), and one round of the Phase 1 comparison
  with no Nav2 relaunch, arm fault, or `/clock` gap over 0.18 s
  (docs/experiments/ros-tcp-stalls).
- Revisit if Cyclone DDS shows pauses under a larger graph (MoveIt, Phase 2) or when
  moving to real hardware networking.
