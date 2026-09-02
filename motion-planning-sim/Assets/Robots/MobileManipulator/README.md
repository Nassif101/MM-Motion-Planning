# Mobile manipulator description

This package is the source of truth for the project's four-wheel, six-axis mobile manipulator.

## Regeneration

Use the repository-local `.venv-cad` Python environment and the installed text-to-CAD launchers.
Generate CAD before regenerating the URDF whenever mesh geometry changes.

The generated URDF must not be edited directly; edit `urdf/mobile_manipulator.py`.

## Unity

Run `tools/sync_mobile_manipulator_to_unity.py` from the repository root after regenerating the model.
In Unity, import:

`Assets/Robots/MobileManipulator/urdf/mobile_manipulator.urdf`

The synchronization script translates standard ROS package URIs only in the Unity copy because the installed Unity URDF Importer resolves mesh paths relative to the URDF directory.

After synchronization, use Unity's `Tools > Motion Planning > Import Mobile Manipulator` command. The project-owned importer validates the hierarchy and repairs the installed URDF Importer's Windows mesh-reference issue before saving the prefab. Then run `Tools > Motion Planning > Configure Mobile Manipulator ROS` to build the sensor-equipped prefab and wire the active scene.

The installed URDF Importer may log `meshes cannot be created! It may already exist` warnings while reusing its generated cylinder-collision asset. These warnings originate in the package's existing-folder branch; the project importer validates the resulting colliders and prefab before reporting success.

The configuration command mounts the Livox Mid-360 on `livox_frame`, removes all UnitySensors `TFLink` components, and publishes its `sensor_msgs/msg/PointCloud2` data on `/livox/lidar`. It preserves the UnitySensors Mid-360 defaults of 20,000 points at 10 Hz with a 0.1-70 m range. Unity also publishes `/clock`, `/joint_states`, and only the dynamic `odom -> base_footprint` TF edge. Drive control and arm command subscribers remain deferred.

## Simulation runtime

Build and launch the ROS-side description stack:

```bash
cd ros2_ws
colcon build --packages-select mobile_manipulator_description --symlink-install
source install/setup.bash
ros2 launch mobile_manipulator_description simulation.launch.py
```

Start Unity Play mode after the ROS TCP endpoint is available. The launch owns the identity `map -> odom` transform and all URDF-derived transforms through `robot_state_publisher`; Unity owns simulation time, joint states, and ground-truth base pose.

To start the same simulation-description stack together with RViz, using simulation time and a preconfigured `/livox/lidar` PointCloud2 display, run:

```bash
ros2 launch mobile_manipulator_description simulation_rviz.launch.py
```

The RViz display uses `map` as its fixed frame and retains 0.5 seconds of Livox scans so the non-repeating scan pattern is easier to see.
