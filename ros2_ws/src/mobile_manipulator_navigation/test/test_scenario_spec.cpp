#include <gtest/gtest.h>
#include <cmath>
#include <set>

#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmn = mobile_manipulator_navigation;

namespace
{
const mmn::ScenarioConfig & config()
{
  static const mmn::ScenarioConfig instance(PACKAGE_SOURCE_DIR);
  return instance;
}

mmn::Polygon home()
{
  return mmn::polygon_of(config().load("footprint_profiles.yaml")["profiles"]["home"]["polygon"]);
}

// Open band x 5.2-18 m, |y| < 3 m; mapped walls cross it at x 2.0-2.2 and 4.8-5.1 m.
mmn::Json obstacle_scenario(const mmn::Json & changes = mmn::Json::object())
{
  mmn::Json box = {{"name", "box"}, {"x", 12.0}, {"y", 0.0}, {"size_x", 0.6}, {"size_y", 0.6}, {"height", 0.8}};
  box.update(changes);
  return {{"start", {16.0, 0.0, M_PI}}, {"goal", {8.0, 0.0, M_PI}}, {"obstacles", {box}}};
}

bool contains(const std::vector<std::string> & problems, const std::string & text, size_t index = 0)
{
  return problems.size() > index && problems[index].find(text) != std::string::npos;
}
}  // namespace

TEST(ScenarioSpec, EveryScenarioIsConsistentAndStartsInFreeSpace)
{
  const auto scenarios = config().load("scenarios.yaml").at("scenarios");
  ASSERT_FALSE(scenarios.empty());
  for (const auto & item : scenarios.items()) {
    SCOPED_TRACE(item.key());
    const auto resolved = config().resolve(item.key());
    const auto & scenario = resolved.at("scenario");
    EXPECT_TRUE(resolved.at("start_free").get<bool>()) << resolved.at("blocked_cells").dump();
    const auto task = scenario.at("task").get<std::string>();
    ASSERT_TRUE(task == "compute_path" || task == "navigate_to_pose");
    if (task == "compute_path") {
      for (const auto & planner : scenario.at("planners")) {
        EXPECT_TRUE(planner == "GridBased" || planner == "Lattice") << planner;
      }
    } else {
      EXPECT_GT(scenario.at("timeout_s").get<double>(), 0);
    }
    EXPECT_EQ(scenario.at("start").size(), 3u);
    EXPECT_EQ(scenario.at("goal").size(), 3u);
    EXPECT_EQ(scenario.at("name"), item.key());
  }
}

TEST(ScenarioSpec, UnknownScenarioIsRejected)
{
  EXPECT_THROW(config().resolve("no_such_scenario"), mmn::UnknownScenario);
}

TEST(ScenarioSpec, FreeSpaceCheckRejectsAStartInsideTheGatePost)
{
  // ManipulationRequiredGate_1p05m left post centre: Unity (7.0, -7.225) -> ROS (-7.225, -7.0).
  const auto check = mmn::start_is_free(config().map(), home(), {-7.225, -7.0, M_PI});
  EXPECT_FALSE(check.free);
  EXPECT_FALSE(check.blocked.empty());
}

TEST(ScenarioSpec, FreeSpaceCheckUsesTheRotatedFootprint)
{
  // East lane: 2.4 m between fences, running along ROS x, centred on y = -10.5.
  // The 1.24 x 1.24 m home footprint fits at either heading.
  EXPECT_TRUE(mmn::start_is_free(config().map(), home(), {13.0, -10.5, 0.0}).free);
  EXPECT_TRUE(mmn::start_is_free(config().map(), home(), {13.0, -10.5, M_PI / 2}).free);
  // A footprint 2.6 m wide in body y fits only when body y points along the lane.
  const mmn::Polygon wide = {{0.54, 1.3}, {0.54, -1.3}, {-0.70, -1.3}, {-0.70, 1.3}};
  EXPECT_TRUE(mmn::start_is_free(config().map(), wide, {13.0, -10.5, M_PI / 2}).free);
  EXPECT_FALSE(mmn::start_is_free(config().map(), wide, {13.0, -10.5, 0.0}).free);
}

TEST(ScenarioSpec, ObstacleOnTheOpenRouteIsAccepted)
{
  EXPECT_TRUE(mmn::obstacle_problems(config().map(), obstacle_scenario(), home()).empty());
}

TEST(ScenarioSpec, ObstacleNearStartOrGoalOrInAWallIsRejected)
{
  const auto & map = config().map();
  EXPECT_TRUE(contains(mmn::obstacle_problems(map, obstacle_scenario({{"x", 15.0}}), home()), "start footprint"));
  EXPECT_TRUE(contains(mmn::obstacle_problems(map, obstacle_scenario({{"x", 8.9}}), home()), "goal footprint"));
  const auto in_wall = mmn::obstacle_problems(map, obstacle_scenario({{"x", 5.0}}), home());
  bool mapped = false;
  for (const auto & problem : in_wall) mapped |= problem.find("mapped obstacle") != std::string::npos;
  EXPECT_TRUE(mapped);
  EXPECT_FALSE(mmn::obstacle_problems(map, obstacle_scenario({{"height", 4.0}}), home()).empty());
}

TEST(ScenarioSpec, ObstacleNeedsExactlyTheDocumentedFields)
{
  auto scenario = obstacle_scenario();
  scenario["obstacles"][0].erase("height");
  EXPECT_TRUE(contains(mmn::obstacle_problems(config().map(), scenario, home()), "exactly"));
}

TEST(ScenarioSpec, BoxPointsCoverTheBoxAtTheSpacing)
{
  const auto points = mmn::box_points({{"x", 1.0}, {"y", 2.0}, {"size_x", 0.6}, {"size_y", 0.3}});
  ASSERT_EQ(points.size(), 13u * 7u);
  EXPECT_NEAR(points.front()[0], 0.7, 1e-12);
  EXPECT_NEAR(points.front()[1], 1.85, 1e-12);
  EXPECT_NEAR(points.back()[0], 1.3, 1e-12);
  EXPECT_NEAR(points.back()[1], 2.15, 1e-12);
}

TEST(YamlJson, ScalarsAreTypedLikePyYaml)
{
  const auto json = mmn::to_json(YAML::Load(
    "a: 1\nb: 1.5\nc: true\nd: off\ne: '12'\nf: text\ng: ~\nh: 1e3\ni: -.5\nlist: [1, 2.0, x]\n"));
  EXPECT_TRUE(json["a"].is_number_integer());
  EXPECT_TRUE(json["b"].is_number_float());
  EXPECT_EQ(json["c"], true);
  EXPECT_EQ(json["d"], false);
  EXPECT_EQ(json["e"], "12");
  EXPECT_EQ(json["f"], "text");
  EXPECT_TRUE(json["g"].is_null());
  EXPECT_EQ(json["h"], "1e3");  // YAML 1.1 floats need a dot
  EXPECT_DOUBLE_EQ(json["i"].get<double>(), -0.5);
  EXPECT_EQ(json["list"].dump(), "[1,2.0,\"x\"]");
  std::vector<std::string> keys;
  for (const auto & item : json.items()) keys.push_back(item.key());
  EXPECT_EQ(keys.front(), "a");  // document order kept
  EXPECT_EQ(keys.back(), "list");
}
