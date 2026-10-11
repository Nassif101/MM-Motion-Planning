#include <gtest/gtest.h>

#include "mobile_manipulator_navigation/known_world.hpp"

namespace mmn = mobile_manipulator_navigation;

namespace
{
std::string boxes_file() { return std::string(PACKAGE_SOURCE_DIR) + "/maps/construction_site.boxes.yaml"; }

mmn::Json obstacle(const mmn::Json & changes = mmn::Json::object())
{
  mmn::Json box = {{"name", "beam"}, {"x", 12.0}, {"y", 0.5}, {"size_x", 0.3}, {"size_y", 2.0}, {"height", 0.2}};
  box.update(changes);
  return box;
}
}  // namespace

TEST(KnownWorld, ScenarioBoxStandsOnGroundByDefault)
{
  const auto box = mmn::scenario_box(obstacle({{"height", 0.8}}));
  EXPECT_DOUBLE_EQ(box.center.z(), 0.4);
  EXPECT_DOUBLE_EQ(box.size.z(), 0.8);
  EXPECT_DOUBLE_EQ(box.center.x(), 12.0);
  EXPECT_DOUBLE_EQ(box.center.y(), 0.5);
}

TEST(KnownWorld, ScenarioBoxFloatsAtZ)
{
  const auto box = mmn::scenario_box(obstacle({{"z", 1.1}}));
  EXPECT_DOUBLE_EQ(box.center.z(), 1.2);
  EXPECT_DOUBLE_EQ(box.size.z(), 0.2);
}

TEST(KnownWorld, KnownBoxesIncludesMapAndScenario)
{
  EXPECT_EQ(mmn::known_boxes(boxes_file(), nullptr).size(), 28u);
  const mmn::Json scenario = {{"obstacles", {obstacle(), obstacle({{"name", "low"}, {"z", 0.0}})}},
                              {"movers", {{{"name", "walker"}}}}};
  const auto boxes = mmn::known_boxes(boxes_file(), &scenario);
  ASSERT_EQ(boxes.size(), 30u);
  EXPECT_EQ(boxes[28].name, "beam");
  EXPECT_EQ(boxes[29].name, "low");
  const mmn::Json empty = {{"start", {0, 0, 0}}};
  EXPECT_EQ(mmn::known_boxes(boxes_file(), &empty).size(), 28u);
}
