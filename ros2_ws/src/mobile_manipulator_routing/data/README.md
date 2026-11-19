# Motion data provenance

`diff_5cm.json` is the unmodified Nav2 differential-drive lattice input installed by
`ros-jazzy-nav2-smac-planner` version `1.3.13-1noble.20260921.071143`.
The relative package path is `sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`.
Upstream: [ROS Navigation / Navigation2](https://github.com/ros-navigation/navigation2).

SHA-256: `105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9`.
Size: 71,540 bytes. Format version 1.0; 16 nonuniform headings, 112 trajectories, including 32 pure rotations.
Pure rotations have zero translational `trajectory_length`; Smac charges them its rotation penalty.

The installed package's copyright metadata declares Apache-2.0. `LICENSE.nav2` contains the full
Apache License 2.0 supplied by the distribution; `COPYRIGHT.nav2` retains that package metadata.
Since spec 2 Amendment 1 (2026-11-19) the search is a port of Smac Lattice 1.3.13, which reads this file as Smac does;
see `../third_party/NOTICE.md`.
No ROS runtime or download is needed to use the provisioned file.
