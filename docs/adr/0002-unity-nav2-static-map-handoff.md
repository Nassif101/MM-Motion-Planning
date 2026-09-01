# ADR 0002: Unity-to-Nav2 static map handoff

## Status

Accepted for the construction-site global-planning baseline.

## Context

The experiment does not use SLAM or another mapping tool. Unity owns authoritative environment geometry, ROS 2 owns navigation, and the static map must be available before live sensor processing. The existing frame contract makes Unity world, `map`, and `odom` coincident for an experiment epoch.

## Options considered

1. Render a top-down Unity camera image and convert its colours to occupancy.
2. Publish a full `nav_msgs/OccupancyGrid` from Unity at runtime through ROS-TCP.
3. Export a standard PGM/YAML map offline from explicit Unity navigation colliders and load it with Nav2 `map_server`.

## Decision

Choose option 3.

- `Environment/NavigationObstacles` colliders are the sole static occupancy source.
- A project-owned Unity editor command performs deterministic 0.05 m rasterization.
- Generated PGM/YAML/metadata files live in `mobile_manipulator_navigation/maps`.
- Nav2 `map_server` publishes `/map`; the global costmap consumes it through its static layer.
- Live `/livox/lidar` data is not fused into the static map.

## Consequences

- The committed map is inspectable, reproducible, and available independently of Unity Play mode.
- Rendering, materials, ignored FBX assets, and sensor noise cannot change global occupancy.
- Map artifacts must be regenerated when authoritative navigation colliders change.
- Collider bounds intentionally over-approximate rotated geometry.
- The initial global footprint is conservative and cannot express coordinated panel reorientation by itself.

## Validation

- Unity EditMode tests verify handedness, grid dimensions, collider projection, and PGM row order.
- `validate_nav2_map` detects stale PGM/YAML artifacts.
- ROS package tests verify dimensions, metadata, a known free start cell, and a known occupied fence cell.
- RViz must show `/map`, robot TF, and lidar aligned before planning experiments.

## Revisit when

- Runtime construction geometry changes during an experiment.
- Multiple or rotated map frames are introduced.
- Height-dependent navigation cannot be represented conservatively in 2D.
- A coordinated base-arm planner replaces the fixed-footprint Nav2 baseline.
