#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "mobile_manipulator_manipulation/scene_diff.hpp"

namespace mmm = mobile_manipulator_manipulation;

namespace
{
mmm::SceneInputs inputs()
{
  mmm::SceneInputs in;
  in.static_boxes = {{"Environment/NavigationObstacles/Wall", {5.0, 1.0, 1.5}, {0.2, 4.0, 3.0}}};
  in.scenario_boxes = {{"box", {1.0, 2.0, 0.4}, {0.6, 0.6, 0.8}}};
  in.panel = {"tool0", {1.2, 1.2, 0.04}, {0.0, 0.0, 0.035}};
  return in;
}

const moveit_msgs::msg::CollisionObject & object(const moveit_msgs::msg::PlanningScene & scene,
                                                 const std::string & id)
{
  const auto & objects = scene.world.collision_objects;
  const auto found = std::find_if(objects.begin(), objects.end(), [&](const auto & o) { return o.id == id; });
  if (found == objects.end()) throw std::runtime_error("missing object " + id);
  return *found;
}
}  // namespace

TEST(SceneDiff, IsADiffInTheMapFrame)
{
  const auto scene = mmm::build_scene_diff(inputs());
  EXPECT_TRUE(scene.is_diff);
  ASSERT_EQ(scene.world.collision_objects.size(), 3u);  // wall, scenario box, floor
  for (const auto & o : scene.world.collision_objects) {
    EXPECT_EQ(o.header.frame_id, "map") << o.id;
    EXPECT_EQ(o.operation, moveit_msgs::msg::CollisionObject::ADD) << o.id;
  }
}

TEST(SceneDiff, FloorTopAtClearanceHeight)
{
  const auto scene = mmm::build_scene_diff(inputs());
  const auto & floor = object(scene, "floor");
  ASSERT_EQ(floor.primitives.size(), 1u);
  const double half_height = floor.primitives[0].dimensions[2] / 2.0;
  EXPECT_NEAR(floor.primitive_poses[0].position.z + half_height, 0.15, 1e-12);
  EXPECT_DOUBLE_EQ(floor.primitives[0].dimensions[0], 40.0);
}

TEST(SceneDiff, PanelAttachedWithSubframe)
{
  const auto scene = mmm::build_scene_diff(inputs());
  ASSERT_EQ(scene.robot_state.attached_collision_objects.size(), 1u);
  const auto & panel = scene.robot_state.attached_collision_objects[0];
  EXPECT_TRUE(scene.robot_state.is_diff);
  EXPECT_EQ(panel.link_name, "tool0");
  EXPECT_EQ(panel.object.id, "panel");
  EXPECT_EQ(panel.object.header.frame_id, "tool0");
  EXPECT_EQ(std::set<std::string>(panel.touch_links.begin(), panel.touch_links.end()),
            (std::set<std::string>{"tool0", "wrist_3_link"}));
  ASSERT_EQ(panel.object.primitives.size(), 1u);
  const auto & size = panel.object.primitives[0].dimensions;
  EXPECT_EQ(std::vector<double>(size.begin(), size.end()), (std::vector<double>{1.2, 1.2, 0.04}));
  EXPECT_DOUBLE_EQ(panel.object.pose.position.z, 0.035);
  EXPECT_EQ(panel.object.subframe_names, (std::vector<std::string>{"panel"}));
  EXPECT_DOUBLE_EQ(panel.object.subframe_poses.at(0).orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(panel.object.subframe_poses.at(0).position.z, 0.0);
}

// A diff carrying an ACM replaces MoveIt's whole matrix, so the floor entries are added to
// the current matrix (with the SRDF's disabled pairs) rather than sent alone.
TEST(SceneDiff, FloorAllowedOnlyForBaseLinksAndExistingPairsKept)
{
  moveit_msgs::msg::AllowedCollisionMatrix current;
  current.entry_names = {"base_link", "upper_arm_link", "wrist_3_link"};
  current.entry_values.resize(3);
  current.entry_values[0].enabled = {false, true, false};
  current.entry_values[1].enabled = {true, false, false};
  current.entry_values[2].enabled = {false, false, false};

  const auto acm = mmm::allow_floor_contacts(current);
  const auto index = [&](const std::string & name) {
    const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
    EXPECT_NE(it, acm.entry_names.end()) << name;
    return static_cast<size_t>(it - acm.entry_names.begin());
  };
  const auto allowed = [&](const std::string & a, const std::string & b) {
    return static_cast<bool>(acm.entry_values.at(index(a)).enabled.at(index(b)));
  };
  ASSERT_EQ(acm.entry_values.size(), acm.entry_names.size());
  for (const auto & row : acm.entry_values) ASSERT_EQ(row.enabled.size(), acm.entry_names.size());

  for (const std::string link : {"base_link", "front_left_wheel_link", "front_right_wheel_link",
                                 "rear_left_wheel_link", "rear_right_wheel_link", "arm_mount_link"}) {
    EXPECT_TRUE(allowed("floor", link)) << link;
    EXPECT_TRUE(allowed(link, "floor")) << link;
  }
  EXPECT_FALSE(allowed("floor", "upper_arm_link"));
  EXPECT_FALSE(allowed("floor", "wrist_3_link"));
  EXPECT_TRUE(allowed("base_link", "upper_arm_link"));     // SRDF pair kept
  EXPECT_FALSE(allowed("base_link", "wrist_3_link"));
  EXPECT_FALSE(allowed("front_left_wheel_link", "wrist_3_link"));  // new entries default to checked
}

TEST(SceneDiff, ScenarioBoxesStandOnTheGround)
{
  EXPECT_DOUBLE_EQ(mmm::scenario_box({{"name", "b"}, {"x", 1.0}, {"y", 2.0}, {"size_x", 0.6},
                                      {"size_y", 0.6}, {"height", 0.8}}).center.z(), 0.4);
}
