# Unity to Nav2 static-map handoff

## Ownership

`ConstructionSiteV1` and its project-owned collision proxies are the source of truth. Unity exports a standard ROS occupancy-map artifact; ROS 2 loads that artifact and owns global path planning. The map is not reconstructed from lidar, camera rendering, imported scan meshes, or SLAM.

## Export contract

- Source scene: `Assets/Scenes/ConstructionSiteV1.unity`
- Source hierarchy: `Environment/NavigationObstacles`
- Input: enabled, active, non-trigger colliders below that hierarchy
- Bounds: 40 x 40 m
- Resolution: 0.05 m/cell, producing 800 x 800 cells
- Unity vertical inclusion band: 0.02 through 3.20 m
- ROS frame: `map`
- Coordinate conversion: ROS X = Unity Z, ROS Y = -Unity X, ROS Z = Unity Y
- Encoding: PGM, black occupied and white free, plus standard trinary map YAML
- Inflation: not baked into the image; Nav2 owns footprint padding and inflation

Collider world-space bounds are rasterized conservatively. This is exact for the current axis-aligned proxy geometry and safely over-approximates a future rotated collider rather than under-reporting it.

## Regenerate

With the native Unity Editor and Pipeline connection ready, run either:

```bash
unity command build_construction_site --project-path ./motion-planning-sim
```

which rebuilds the scene and exports the map, or export the current scene alone:

```bash
unity command export_nav2_map --project-path ./motion-planning-sim
```

The generated files are written to this package's `maps/` directory. Validate that committed artifacts still match the scene with:

```bash
unity command validate_nav2_map --project-path ./motion-planning-sim
```

## ROS usage

After rebuilding and sourcing the ROS workspace:

```bash
ros2 launch mobile_manipulator_navigation global_planning.launch.py
```

The launch starts lifecycle-managed `map_server` and `planner_server`. It does not start AMCL: the simulation contract supplies ground-truth `odom -> base_footprint`, while `mobile_manipulator_description` supplies the identity `map -> odom` transform.

The global costmap uses the static map and a conservative initial 1.2 x 1.2 m footprint. The 1.05 m manipulation gate should therefore be rejected in the initial pose. Future coordinated planning may publish a configuration-dependent footprint, but standard 2D Nav2 alone does not decide how to reorient the panel.

## Deferred local and arm perception

`/livox/lidar` is reserved for a rolling Nav2 local voxel/obstacle costmap and a separately filtered MoveIt occupancy input. Those consumers require validated controller, odometry-message, height-filter, self-filter, and payload-filter contracts before they are enabled. The static global map remains unchanged by live sensor observations.
