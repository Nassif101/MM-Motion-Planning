#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>

#include "mobile_manipulator_navigation/mission.hpp"
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
    ASSERT_TRUE(task == "compute_path" || task == "navigate_to_pose" || task == "mission");
    if (task == "compute_path") {
      for (const auto & planner : scenario.at("planners")) {
        EXPECT_TRUE(planner == "GridBased" || planner == "Lattice") << planner;
      }
    } else {
      EXPECT_GT(scenario.at("timeout_s").get<double>(), 0);
    }
    if (task == "mission") {
      const auto profiles = config().load("footprint_profiles.yaml").at("profiles");
      EXPECT_EQ(mmn::mission_problems(config().map(), scenario, profiles), std::vector<std::string>{});
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

TEST(ScenarioSpec, ObstacleClearanceOverride)
{
  // Start at x = 16 facing -x: the home footprint's front edge is at x = 15.46. A 0.6 m box
  // centred at x = 15.06 stands 0.1 m in front of it.
  const auto & map = config().map();
  EXPECT_TRUE(contains(mmn::obstacle_problems(map, obstacle_scenario({{"x", 15.06}}), home()), "start footprint"));
  EXPECT_TRUE(mmn::obstacle_problems(map, obstacle_scenario({{"x", 15.06}, {"clearance_m", 0.05}}), home()).empty());
  EXPECT_FALSE(mmn::obstacle_problems(map, obstacle_scenario({{"clearance_m", -0.1}}), home()).empty());
}

TEST(ScenarioSpec, ElevatedObstacleSkipsGroundFreeCheckAboveBaseBand)
{
  const auto & map = config().map();
  // x = 5.0 is inside a mapped wall: a ground box there is rejected, a beam above 0.60 m is not.
  EXPECT_FALSE(mmn::obstacle_problems(map, obstacle_scenario({{"x", 5.0}}), home()).empty());
  EXPECT_FALSE(mmn::obstacle_problems(map, obstacle_scenario({{"x", 5.0}, {"z", 0.3}}), home()).empty());
  EXPECT_TRUE(mmn::obstacle_problems(map, obstacle_scenario({{"x", 5.0}, {"z", 0.6}}), home()).empty());
  EXPECT_TRUE(mmn::obstacle_problems(map, obstacle_scenario({{"x", 5.0}, {"z", 1.1}, {"height", 0.2}}), home()).empty());
  // Over the open route, with z below the band it still has to be free map space (it is).
  EXPECT_TRUE(mmn::obstacle_problems(map, obstacle_scenario({{"z", 1.1}, {"height", 0.2}}), home()).empty());
  // Above the band it still keeps clear of the start footprint.
  EXPECT_TRUE(contains(mmn::obstacle_problems(map, obstacle_scenario({{"x", 15.0}, {"z", 1.1}}), home()), "start footprint"));
}

TEST(ScenarioSpec, RejectsNegativeZ)
{
  const auto map = config().map();
  EXPECT_TRUE(contains(mmn::obstacle_problems(map, obstacle_scenario({{"z", -0.1}}), home()), "z must be"));
  EXPECT_TRUE(contains(mmn::obstacle_problems(map, obstacle_scenario({{"z", 2.6}}), home()), "z must be"));
  EXPECT_TRUE(mmn::obstacle_problems(map, obstacle_scenario({{"z", 0.0}}), home()).empty());
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

namespace
{
mmn::Json mover_scenario(const mmn::Json & changes = mmn::Json::object())
{
  mmn::Json worker = {{"name", "worker"}, {"start", {11.0, 2.5}}, {"end", {11.0, -2.5}}, {"size_x", 0.5},
                      {"size_y", 0.5}, {"height", 1.8}, {"speed_mps", 0.8}, {"trigger_distance_m", 2.5},
                      {"crossings", 1}};
  worker.update(changes);
  return {{"start", {16.0, 0.0, M_PI}}, {"goal", {8.0, 0.0, M_PI}}, {"movers", {worker}}};
}

bool any_contains(const std::vector<std::string> & problems, const std::string & text)
{
  for (const auto & problem : problems) {
    if (problem.find(text) != std::string::npos) return true;
  }
  return false;
}
}  // namespace

TEST(ScenarioSpec, MoverCrossingTheOpenRouteIsAccepted)
{
  EXPECT_TRUE(mmn::mover_problems(config().map(), mover_scenario(), home()).empty());
}

TEST(ScenarioSpec, MoverThroughAWallOrWaitingOnTheRobotIsRejected)
{
  const auto & map = config().map();
  // A walk along y = 0 from x = 7 to x = 3 crosses the mapped walls at x 4.8-5.1 m.
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"start", {7.0, 0.0}}, {"end", {3.0, 0.0}}}), home()),
                           "walked area overlaps mapped obstacle"));
  // Waiting next to the robot's start pose, or stopping on the goal.
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"start", {15.0, 1.0}}}), home()),
                           "waiting position within 0.3 m of the start footprint"));
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"end", {8.0, 0.5}}}), home()),
                           "final position within 0.3 m of the goal footprint"));
  // With two crossings the worker ends back at its start, clear of both footprints.
  EXPECT_TRUE(mmn::mover_problems(map, mover_scenario({{"end", {8.0, 0.5}}, {"crossings", 2}}), home()).empty());
}

TEST(ScenarioSpec, MoverFieldsAndLimitsAreChecked)
{
  const auto & map = config().map();
  auto missing = mover_scenario();
  missing["movers"][0].erase("crossings");
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, missing, home()), "exactly"));
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"speed_mps", 3.0}}), home()), "speed_mps"));
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"crossings", 1.5}}), home()), "crossings"));
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"end", {11.0, 2.45}}}), home()), "0.1 m long"));
  EXPECT_TRUE(any_contains(mmn::mover_problems(map, mover_scenario({{"start", {11.0}}}), home()), "[x, y]"));
}

TEST(ScenarioSpec, WorkerCrossingScenarioResolves)
{
  const auto resolved = config().resolve("worker_crossing_nav");
  EXPECT_TRUE(resolved.at("start_free").get<bool>());
  EXPECT_EQ(resolved.at("scenario").at("movers").size(), 1u);
}

namespace
{
const std::vector<std::string> kPhase4Scenarios = {
  "worker_late_crossing_nav", "worker_stops_in_lane_nav", "overhead_course_nav",
  "overhead_course_mission", "overhead_course_worker_nav", "overhead_course_worker_mission"};
}  // namespace

// Phase 4 (spec Section 7): the worker interaction scenarios and the overhead course.
TEST(ScenarioSpec, NewPhase4ScenariosValid)
{
  const auto & map = config().map();
  for (const auto & name : kPhase4Scenarios) {
    SCOPED_TRACE(name);
    mmn::Json resolved;
    ASSERT_NO_THROW(resolved = config().resolve(name));
    EXPECT_TRUE(resolved.at("start_free").get<bool>()) << resolved.at("blocked_cells").dump();
    const auto & scenario = resolved.at("scenario");
    // B5 drives every one of them from home (the drive versions with --arm-pose home): the home
    // footprint also fits at the start and goal and keeps clear of the boxes and the workers.
    EXPECT_TRUE(mmn::start_is_free(map, home(), mmn::pose_of(scenario.at("start"))).free);
    EXPECT_TRUE(mmn::start_is_free(map, home(), mmn::pose_of(scenario.at("goal"))).free);
    EXPECT_EQ(mmn::obstacle_problems(map, scenario, home()), std::vector<std::string>{});
    EXPECT_EQ(mmn::mover_problems(map, scenario, home()), std::vector<std::string>{});
  }
  for (const std::string name : {"worker_late_crossing_nav", "worker_stops_in_lane_nav"}) {
    const auto scenario = config().resolve(name).at("scenario");
    EXPECT_EQ(scenario.at("arm_pose"), "home") << name;
    EXPECT_EQ(scenario.at("task"), "navigate_to_pose") << name;
    EXPECT_EQ(scenario.at("movers").size(), 1u) << name;
    EXPECT_FALSE(scenario.contains("obstacles")) << name;
  }
  // The drive version starts in vertical carry (B1/B2); the mission version starts in home (B3/B4).
  EXPECT_EQ(config().resolve("overhead_course_nav").at("scenario").at("arm_pose"), "vertical_carry");
  EXPECT_EQ(config().resolve("overhead_course_nav").at("scenario").at("task"), "navigate_to_pose");
  EXPECT_EQ(config().resolve("overhead_course_mission").at("scenario").at("arm_pose"), "home");
  EXPECT_EQ(config().resolve("overhead_course_mission").at("scenario").at("task"), "mission");
  // The worker variants are the course plus one worker.
  for (const std::string base : {"overhead_course_nav", "overhead_course_mission"}) {
    const auto course = config().resolve(base).at("scenario");
    auto with_worker = config().resolve(base.substr(0, 15) + "_worker" + base.substr(15)).at("scenario");
    EXPECT_EQ(with_worker.at("movers").size(), 1u) << base;
    EXPECT_FALSE(course.contains("movers")) << base;
    for (const char * key : {"start", "goal", "obstacles", "arm_pose", "task"}) {
      EXPECT_EQ(with_worker.at(key), course.at(key)) << base << " " << key;
    }
  }
}

TEST(ScenarioSpec, OverheadCourseBeamFloatsAboveTheBaseBand)
{
  // Exactly one box floats (the beam): above the 0.60 m base band, so B5's route passes under it,
  // and below 2.0 m, so the B1-B4 band (0.05-2.0 m) sees a wall.
  const auto scenario = config().resolve("overhead_course_nav").at("scenario");
  int floating = 0;
  for (const auto & obstacle : scenario.at("obstacles")) {
    const double z = obstacle.value("z", 0.0);
    if (z == 0.0) continue;
    ++floating;
    EXPECT_EQ(obstacle.at("name"), "beam");
    EXPECT_GE(z, 0.60);
    EXPECT_LT(z, 2.0);
  }
  EXPECT_EQ(floating, 1);
}

TEST(ScenarioSpec, ArmPoseOverrideUsesThatPoseAndItsProfile)
{
  // run_nav_scenario.py --arm-pose: B5 drives overhead_course_nav from home.
  const auto resolved = config().resolve("overhead_course_nav", std::string("home"));
  EXPECT_EQ(resolved.at("scenario").at("arm_pose"), "home");
  EXPECT_EQ(resolved.at("scenario").at("footprint_profile"), "home");
  EXPECT_EQ(mmn::polygon_of(resolved.at("polygon")), home());
  EXPECT_TRUE(resolved.at("start_free").get<bool>());
  EXPECT_EQ(config().resolve("overhead_course_nav").at("scenario").at("arm_pose"), "vertical_carry");
  // B5 with the bent carry: its profile is home's polygon, so the home drives fit it too.
  for (const std::string name : {"overhead_course_nav", "narrow_gate_home_nav"}) {
    const auto carry = config().resolve(name, std::string("carry"));
    EXPECT_EQ(carry.at("scenario").at("arm_pose"), "carry") << name;
    EXPECT_EQ(carry.at("scenario").at("footprint_profile"), "carry") << name;
    EXPECT_EQ(mmn::polygon_of(carry.at("polygon")), home()) << name;
    EXPECT_TRUE(carry.at("start_free").get<bool>()) << name;
  }
  // Only a profile that describes a qualified arm pose of the same name qualifies.
  EXPECT_THROW(config().resolve("overhead_course_nav", std::string("base_only")), mmn::InvalidScenario);
  EXPECT_THROW(config().resolve("overhead_course_nav", std::string("no_such_pose")), mmn::InvalidScenario);
}

namespace
{
namespace fs = std::filesystem;

// A temporary package root with two tiny 20 x 20 cell sites; "site_b" has one occupied cell
// at (2.0, 2.0) m that the default site lacks.
class ScenarioSites : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / ("scenario_sites_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                                         "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(root_);
    fs::create_directories(root_ / "config");
    fs::create_directories(root_ / "maps");
    fs::copy_file(fs::path(PACKAGE_SOURCE_DIR) / "config/footprint_profiles.yaml", root_ / "config/footprint_profiles.yaml");
    write_site("construction_site", "Assets/Scenes/A.unity", false);
    write_site("site_b", "Assets/Scenes/B.unity", true);
    write("config/scenarios.yaml", "scenarios:\n" + scenario("a_nav"));
    write("config/scenarios_site_b.yaml", "site: site_b\nscenarios:\n" + scenario("b_nav"));
  }
  void TearDown() override { fs::remove_all(root_); }

  void write(const std::string & relative, const std::string & text) const
  {
    std::ofstream(root_ / relative, std::ios::binary) << text;
  }
  static std::string scenario(const std::string & name)
  {
    return "  " + name + ":\n    arm_pose: home\n    footprint_profile: home\n    task: compute_path\n"
           "    planners: [GridBased]\n    start: [2.0, 2.0, 0.0]\n    goal: [2.5, 2.0, 0.0]\n";
  }
  void write_site(const std::string & site, const std::string & scene, bool occupied_centre) const
  {
    write("maps/" + site + ".yaml", "image: " + site + ".pgm\nresolution: 0.2\norigin: [0.0, 0.0, 0.0]\n");
    write("maps/" + site + ".metadata.json", "{\"sourceScene\": \"" + scene + "\"}");
    std::string pixels(400, static_cast<char>(254));
    if (occupied_centre) pixels[(19 - 10) * 20 + 10] = 0;  // cell (col 10, row 10)
    write("maps/" + site + ".pgm", "P5\n20 20\n255\n" + pixels);
  }
  fs::path root_;
};
}  // namespace

TEST_F(ScenarioSites, FileWithoutSiteIsDefault)
{
  EXPECT_EQ(mmn::ScenarioConfig(root_.string()).site_of("a_nav"), "construction_site");
}

TEST_F(ScenarioSites, FileSiteIsUsed)
{
  const auto resolved = mmn::ScenarioConfig(root_.string()).resolve("b_nav");
  EXPECT_EQ(resolved.at("site"), "site_b");
  EXPECT_EQ(resolved.at("scene"), "Assets/Scenes/B.unity");
  EXPECT_EQ(resolved.at("disturbances"), false);
}

TEST_F(ScenarioSites, NamesSpanAllFiles)
{
  EXPECT_EQ(mmn::ScenarioConfig(root_.string()).names(), (std::vector<std::string>{"a_nav", "b_nav"}));
}

TEST_F(ScenarioSites, NamesOfOneSite)
{
  const mmn::ScenarioConfig config(root_.string());
  EXPECT_EQ(config.names("construction_site"), (std::vector<std::string>{"a_nav"}));
  EXPECT_EQ(config.names("site_b"), (std::vector<std::string>{"b_nav"}));
  EXPECT_TRUE(config.names("no_such_site").empty());
}

TEST_F(ScenarioSites, DuplicateNameThrows)
{
  write("config/scenarios_site_b.yaml", "site: site_b\nscenarios:\n" + scenario("a_nav") + scenario("b_nav"));
  try {
    mmn::ScenarioConfig(root_.string()).resolve("a_nav");
    FAIL() << "expected InvalidScenario";
  } catch (const mmn::InvalidScenario & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("scenarios.yaml"), std::string::npos) << message;
    EXPECT_NE(message.find("scenarios_site_b.yaml"), std::string::npos) << message;
  }
}

TEST_F(ScenarioSites, ChecksUseTheScenarioSitesMap)
{
  const mmn::ScenarioConfig config(root_.string());
  EXPECT_TRUE(config.resolve("a_nav").at("start_free").get<bool>());
  EXPECT_FALSE(config.resolve("b_nav").at("start_free").get<bool>());
}

// The V2 scenario set (spec Section 6; Phase 4B-A adds v2_gate_right_swing_nav): 21 scenarios named v2_*. resolve() checks the obstacles, movers
// and mission steps and reports start and goal free for the scenario's own footprint profile.
TEST(ScenarioSet, InstalledV2ScenariosResolve)
{
  const auto names = config().names();
  size_t count = 0;
  for (const auto & name : names) {
    if (name.rfind("v2_", 0) != 0) continue;
    ++count;
    SCOPED_TRACE(name);
    const auto resolved = config().resolve(name);
    EXPECT_EQ(resolved.at("site"), "construction_site_v2");
    EXPECT_TRUE(resolved.at("start_free").get<bool>()) << resolved.at("blocked_cells").dump();
    EXPECT_FALSE(resolved.at("scenario").at("description").get<std::string>().empty());
    // Every V2 scenario starts in the bent carry pose (user decision 2026-10-17).
    EXPECT_EQ(resolved.at("scenario").at("arm_pose"), "carry");
    EXPECT_EQ(resolved.at("scenario").at("footprint_profile"), "carry");
  }
  EXPECT_EQ(count, 21u);
  EXPECT_NE(std::find(names.begin(), names.end(), "open_space"), names.end());  // V1 stays listed
  EXPECT_TRUE(config().resolve("v2_spine_rough_nav").at("disturbances").get<bool>());
  EXPECT_TRUE(config().resolve("v2_scaffold_tunnel_rough_nav").at("disturbances").get<bool>());
  EXPECT_FALSE(config().resolve("v2_spine_nav").at("disturbances").get<bool>());
}

TEST(EffectiveSite, EmptyRequestTakesScenarioSite)
{
  EXPECT_EQ(mmn::effective_site("", std::string("construction_site_v2")), "construction_site_v2");
  EXPECT_EQ(mmn::effective_site("construction_site_v2", std::string("construction_site_v2")),
            "construction_site_v2");
}

TEST(EffectiveSite, NoScenarioTakesRequestOrDefault)
{
  EXPECT_EQ(mmn::effective_site("", std::nullopt), "construction_site");
  EXPECT_EQ(mmn::effective_site("construction_site_v2", std::nullopt), "construction_site_v2");
}

TEST(EffectiveSite, MismatchThrows)
{
  EXPECT_THROW(mmn::effective_site("construction_site", std::string("construction_site_v2")),
               mmn::InvalidScenario);
}

TEST(EffectiveSite, BoxesFileLivesInTheSiteMaps)
{
  EXPECT_EQ(mmn::site_boxes_file("/root", "construction_site_v2"), "/root/maps/construction_site_v2.boxes.yaml");
}

TEST(EffectiveSite, ExtentOfTheMap)
{
  mmn::StaticMap map{0.05, -25.0, -20.0, 1000, 800, {}};
  const auto extent = mmn::extent_of(map);
  EXPECT_DOUBLE_EQ(extent.min_x, -25.0);
  EXPECT_DOUBLE_EQ(extent.min_y, -20.0);
  EXPECT_DOUBLE_EQ(extent.max_x, 25.0);
  EXPECT_DOUBLE_EQ(extent.max_y, 20.0);
}
