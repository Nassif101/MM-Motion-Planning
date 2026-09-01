# Mobile manipulator navigation

This package contains the deterministic 2D map exported from Unity and the first project-owned Nav2 configuration. It deliberately launches only the static map server and global planner. Base command execution, a rolling lidar local costmap, and coordinated base-arm planning remain separate follow-up contracts.

Build and source the workspace, start the description/TF stack, then run:

```bash
ros2 launch mobile_manipulator_navigation global_planning.launch.py
```

The launch activates `map_server` and `planner_server`, publishes the Unity-derived map on `/map` in the `map` frame, and exposes Nav2's path-computation actions. See `docs/unity-map-export.md` for regeneration and validation details.
