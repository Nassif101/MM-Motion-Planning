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
#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_manipulation
{
using mobile_manipulator_navigation::Json;
using mobile_manipulator_geometry::Payload;

// The known-world loader lives in mobile_manipulator_navigation so the MPC shares it.
using Box = mobile_manipulator_navigation::Box;
using mobile_manipulator_navigation::load_boxes;
using mobile_manipulator_navigation::scenario_box;

struct SceneInputs
{
  std::vector<Box> static_boxes;
  std::vector<Box> scenario_boxes;
  double floor_top_m = 0.15;        // qualified_payload.json min_panel_ground_clearance_m
  mobile_manipulator_navigation::MapExtent floor_extent{-20.0, -20.0, 20.0, 20.0};  // the site map's extent
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
}  // namespace mobile_manipulator_manipulation
