#pragma once
// The known collision world shared by MoveIt (B3) and the whole-body MPC: the boxes
// exported from Unity plus the scenario's unmapped obstacles. No ROS graph, no MoveIt.
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_navigation
{
// Box in the map frame: centre, full size along its own axes, yaw of those axes about +z.
struct Box
{
  std::string name;
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  double yaw = 0.0;
};

// A scenarios.yaml obstacle {name, x, y, size_x, size_y, height[, z]}: the box bottom is at
// `z` (default 0, on the ground), so center.z = z + height / 2.
Box scenario_box(const Json & obstacle);

// Boxes of a site's maps/<site>.boxes.yaml.
std::vector<Box> load_boxes(const std::string & yaml_path);

// The map boxes plus the scenario's obstacles (null scenario: map boxes only). Movers are
// excluded: they are not known in advance.
std::vector<Box> known_boxes(const std::string & boxes_file, const Json * scenario);
}  // namespace mobile_manipulator_navigation
