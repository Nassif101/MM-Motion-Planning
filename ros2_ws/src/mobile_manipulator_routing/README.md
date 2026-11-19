# Mobile manipulator routing

Portable C++17 map, footprint and state-lattice routing for spec 2. The core has no ROS, TF, Nav2,
Unity, URDF or bag dependency. Callers provide resolved map/footprint values and motion data.
YAML and JSON parsing use yaml-cpp and nlohmann JSON; tests use system GTest.

Standalone build (with those development dependencies available):

```sh
cmake -S ros2_ws/src/mobile_manipulator_routing -B build/routing -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/routing --parallel 2
ctest --test-dir build/routing --output-on-failure
```

The default package build uses plain CMake. Optional ament export requires explicit
`-DROUTING_WITH_AMENT=ON`; it does not change runtime ownership. Data is installed under
`share/mobile_manipulator_routing`. See `data/README.md` for the pinned motion input and licence.

Implementation is in progress. Configured work budgets are not a measured timing guarantee.
