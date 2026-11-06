## Notes

- **Recording (2026-11-06):** `compare_lidar.py record` with the Unity Editor on ConstructionSiteV2; Play stopped
  afterwards. Navigation stopped, so no `map -> odom` exists: Unity's `odom` is the map frame (its odom -> base_footprint
  is the placed pose, within 1.3 mm at every fixture). The runner starts no `robot_state_publisher`, so the bags hold
  no `/tf_static`; the lidar mount is the URDF's (the plant's header, generated from the same URDF), which is what
  `/tf_static` would carry.
- **Labelling:** both simulators' points are labelled by the nearest plant primitive within 8 cm (4 sigma of the
  0.02 m range noise). A 3 cm tolerance for Unity's points against the plant's true labels biased the site fraction by
  3-5 points from the noise alone (the self-comparison smoke); labelled alike, plant scans posing as Unity's give
  0.0-point gaps and a Jaccard of 1.00, and the proximity labels agree with the plant's ground truth on 99.7-99.9 %.
- **Self-hit Jaccard 0.87-0.89:** the robot and panel return the same regions in both simulators; the voxels that
  differ are the 5 cm cells the 2 cm noise moves points across (different noise draws). The 0.7 threshold is to be
  confirmed at CP4.
- **Point counts** differ by under 0.4 %: rays that graze an edge hit in one simulator and miss in the other.
