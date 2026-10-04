#pragma once
// The known MoveIt collision world for B3 (no ROS graph): static boxes exported from Unity,
// the scenario's unmapped boxes, a floor raised to the panel ground-clearance limit, and
// the reference panel attached to tool0 with a `panel` subframe at its centre.
#include <string>
#include <vector>

#include <Eigen/Core>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_manipulation
{
using mobile_manipulator_navigation::Json;
using mobile_manipulator_geometry::Payload;

// Box in the map frame: centre, full size along its own axes, yaw of those axes about +z.
struct Box
{
  std::string name;
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  double yaw = 0.0;
};

struct SceneInputs
{
  std::vector<Box> static_boxes;
  std::vector<Box> scenario_boxes;
  double floor_top_m = 0.15;        // qualified_payload.json min_panel_ground_clearance_m
  double floor_size_m = 40.0;       // the exported map's extent
  double floor_thickness_m = 0.3;
  // Added to every side of the known boxes (not the floor): plans keep this physical
  // margin rather than grazing an obstacle.
  double box_margin_m = 0.05;
  Payload panel;
};

// Links that legitimately sit below the raised floor top.
const std::vector<std::string> & floor_contact_links();

// World objects (map frame) plus the attached panel, as a planning-scene diff.
moveit_msgs::msg::PlanningScene build_scene_diff(const SceneInputs & inputs);

// `current` with a `floor` entry that may touch only floor_contact_links(). Sent with the
// diff because a diff carrying an ACM replaces MoveIt's whole matrix.
moveit_msgs::msg::AllowedCollisionMatrix allow_floor_contacts(
  const moveit_msgs::msg::AllowedCollisionMatrix & current);

// A scenarios.yaml obstacle {name, x, y, size_x, size_y, height} standing on the ground.
Box scenario_box(const Json & obstacle);

// Boxes of maps/construction_site.boxes.yaml.
std::vector<Box> load_boxes(const std::string & yaml_path);
}  // namespace mobile_manipulator_manipulation
