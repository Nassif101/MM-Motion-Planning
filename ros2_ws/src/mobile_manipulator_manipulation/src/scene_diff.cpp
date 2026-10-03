#include "mobile_manipulator_manipulation/scene_diff.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace mobile_manipulator_manipulation
{
namespace
{
moveit_msgs::msg::CollisionObject box_object(const std::string & id, const std::string & frame,
                                             const Eigen::Vector3d & center, const Eigen::Vector3d & size,
                                             double yaw = 0.0)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {size.x(), size.y(), size.z()};
  geometry_msgs::msg::Pose pose;
  pose.position.x = center.x();
  pose.position.y = center.y();
  pose.position.z = center.z();
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  return object;
}
}  // namespace

const std::vector<std::string> & floor_contact_links()
{
  static const std::vector<std::string> links = {
    "base_link", "front_left_wheel_link", "front_right_wheel_link",
    "rear_left_wheel_link", "rear_right_wheel_link", "arm_mount_link"};
  return links;
}

moveit_msgs::msg::PlanningScene build_scene_diff(const SceneInputs & inputs)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  for (const auto * boxes : {&inputs.static_boxes, &inputs.scenario_boxes}) {
    for (const auto & box : *boxes) {
      const Eigen::Vector3d padded = box.size + Eigen::Vector3d::Constant(2.0 * inputs.box_margin_m);
      scene.world.collision_objects.push_back(box_object(box.name, "map", box.center, padded, box.yaw));
    }
  }
  scene.world.collision_objects.push_back(box_object(
    "floor", "map", {0.0, 0.0, inputs.floor_top_m - inputs.floor_thickness_m / 2.0},
    {inputs.floor_size_m, inputs.floor_size_m, inputs.floor_thickness_m}));

  moveit_msgs::msg::AttachedCollisionObject panel;
  panel.link_name = inputs.panel.link;
  panel.touch_links = {"tool0", "wrist_3_link"};
  panel.object = box_object("panel", inputs.panel.link, Eigen::Vector3d::Zero(), inputs.panel.size);
  // The object pose is the panel centre; its single primitive and subframe sit at that pose.
  panel.object.pose.position.x = inputs.panel.center.x();
  panel.object.pose.position.y = inputs.panel.center.y();
  panel.object.pose.position.z = inputs.panel.center.z();
  panel.object.subframe_names = {"panel"};
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  panel.object.subframe_poses = {identity};
  scene.robot_state.attached_collision_objects.push_back(panel);
  return scene;
}

moveit_msgs::msg::AllowedCollisionMatrix allow_floor_contacts(
  const moveit_msgs::msg::AllowedCollisionMatrix & current)
{
  auto acm = current;
  const auto index_of = [&acm](const std::string & name) {
    const auto found = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
    if (found != acm.entry_names.end()) return static_cast<size_t>(found - acm.entry_names.begin());
    // New entry: checked against everything until set below.
    for (auto & row : acm.entry_values) row.enabled.push_back(false);
    acm.entry_names.push_back(name);
    moveit_msgs::msg::AllowedCollisionEntry row;
    row.enabled.assign(acm.entry_names.size(), false);
    acm.entry_values.push_back(row);
    return acm.entry_names.size() - 1;
  };
  const size_t floor = index_of("floor");
  for (const auto & link : floor_contact_links()) {
    const size_t i = index_of(link);
    acm.entry_values[floor].enabled[i] = true;
    acm.entry_values[i].enabled[floor] = true;
  }
  return acm;
}

Box scenario_box(const Json & obstacle)
{
  const double height = obstacle.at("height").get<double>();
  return {obstacle.at("name").get<std::string>(),
          {obstacle.at("x").get<double>(), obstacle.at("y").get<double>(), height / 2.0},
          {obstacle.at("size_x").get<double>(), obstacle.at("size_y").get<double>(), height}};
}

std::vector<Box> load_boxes(const std::string & yaml_path)
{
  const auto root = YAML::LoadFile(yaml_path);
  if (root["frame_id"].as<std::string>() != "map") throw std::invalid_argument("planning boxes must be in map");
  std::vector<Box> boxes;
  for (const auto & box : root["boxes"]) {
    const auto c = box["center"], s = box["size"];
    boxes.push_back({box["name"].as<std::string>(),
                     {c[0].as<double>(), c[1].as<double>(), c[2].as<double>()},
                     {s[0].as<double>(), s[1].as<double>(), s[2].as<double>()},
                     box["yaw"] ? box["yaw"].as<double>() : 0.0});
  }
  return boxes;
}
}  // namespace mobile_manipulator_manipulation
