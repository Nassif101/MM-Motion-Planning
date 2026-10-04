#include <gtest/gtest.h>
#include <cmath>
#include <map>

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

mmn::Json profiles() { return config().load("footprint_profiles.yaml").at("profiles"); }

mmn::Polygon polygon(const std::string & name) { return mmn::polygon_of(profiles().at(name).at("polygon")); }

// 1.05 m gate on y = -7.725: start in front of it, the throat at x = -7.2.
mmn::Json gate_mission(const mmn::Json & steps, const mmn::Json & goal)
{
  return {{"task", "mission"}, {"arm_pose", "home"}, {"footprint_profile", "home"},
          {"start", {-4.2, -7.725, M_PI}}, {"goal", goal}, {"timeout_s", 90}, {"steps", steps}};
}

mmn::Json named(const std::string & state, const std::string & profile)
{
  return {{"reconfigure", {{"named_state", state}, {"footprint_profile", profile}}}};
}

mmn::Json panel(const std::string & profile)
{
  return {{"reconfigure", {{"panel_pose", {{"xyz", {-0.08, 0.225, 1.32}}, {"rpy", {-M_PI / 2, M_PI / 2, 0.0}}}},
                           {"position_tolerance", {0.01, 0.01, 0.01}},
                           {"orientation_tolerance", {0.01, 0.01, 0.01}},
                           {"footprint_profile", profile}}}};
}

mmn::Json drive(double x) { return {{"navigate", {x, -7.725, M_PI}}}; }

bool mentions(const std::vector<std::string> & problems, const std::string & text)
{
  for (const auto & p : problems) if (p.find(text) != std::string::npos) return true;
  return false;
}
}  // namespace

TEST(Mission, ParsesStepsInOrder)
{
  const auto steps = mmn::parse_mission(gate_mission({panel("vertical_carry"), drive(-9.0), named("home", "home")},
                                                     {-9.0, -7.725, M_PI}));
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_EQ(steps[0].kind, mmn::MissionStep::Kind::Reconfigure);
  EXPECT_EQ(steps[0].reconfigure.at("footprint_profile"), "vertical_carry");
  EXPECT_EQ(steps[1].kind, mmn::MissionStep::Kind::Navigate);
  EXPECT_DOUBLE_EQ(steps[1].pose[0], -9.0);
  EXPECT_EQ(steps[2].kind, mmn::MissionStep::Kind::Reconfigure);
  EXPECT_EQ(steps[2].reconfigure.at("named_state"), "home");
}

TEST(Mission, ValidMissionHasNoProblems)
{
  const auto mission = gate_mission({panel("vertical_carry"), drive(-9.0), named("home", "home")}, {-9.0, -7.725, M_PI});
  EXPECT_EQ(mmn::mission_problems(config().map(), mission, profiles()), std::vector<std::string>{});
}

TEST(Mission, RejectsReconfigureWithBothTargets)
{
  auto both = panel("vertical_carry");
  both["reconfigure"]["named_state"] = "vertical_carry";
  const auto problems = mmn::mission_problems(
    config().map(), gate_mission({both, drive(-9.0)}, {-9.0, -7.725, M_PI}), profiles());
  EXPECT_TRUE(mentions(problems, "exactly one of named_state or panel_pose"));
}

TEST(Mission, RejectsReconfigureWithUnknownProfile)
{
  const auto problems = mmn::mission_problems(
    config().map(), gate_mission({named("vertical_carry", "no_such_profile"), drive(-9.0)}, {-9.0, -7.725, M_PI}),
    profiles());
  EXPECT_TRUE(mentions(problems, "unknown footprint profile"));
}

TEST(Mission, RejectsPanelPoseWithoutTolerances)
{
  auto loose = panel("vertical_carry");
  loose["reconfigure"].erase("orientation_tolerance");
  const auto problems = mmn::mission_problems(
    config().map(), gate_mission({loose, drive(-9.0)}, {-9.0, -7.725, M_PI}), profiles());
  EXPECT_TRUE(mentions(problems, "orientation_tolerance"));
}

TEST(Mission, RejectsReconfigurePoseBlockedForTheWiderProfile)
{
  // In the throat the vertical-carry footprint fits but the home footprint does not.
  ASSERT_TRUE(mmn::start_is_free(config().map(), polygon("vertical_carry"), {-7.2, -7.725, M_PI}).free);
  ASSERT_FALSE(mmn::start_is_free(config().map(), polygon("home"), {-7.2, -7.725, M_PI}).free);
  const auto problems = mmn::mission_problems(
    config().map(), gate_mission({panel("vertical_carry"), drive(-7.2), named("home", "home")}, {-7.2, -7.725, M_PI}),
    profiles());
  EXPECT_TRUE(mentions(problems, "step 3"));
  EXPECT_TRUE(mentions(problems, "home"));
}

TEST(Mission, RejectsADriveBlockedForTheActiveProfile)
{
  // Driving into the throat while still in home.
  const auto problems = mmn::mission_problems(
    config().map(), gate_mission(mmn::Json::array({drive(-7.2)}), {-7.2, -7.725, M_PI}), profiles());
  EXPECT_TRUE(mentions(problems, "step 1"));
}

TEST(Mission, LastDriveMustEndAtTheGoal)
{
  const auto problems = mmn::mission_problems(
    config().map(), gate_mission({panel("vertical_carry"), drive(-9.0)}, {-11.0, -7.725, M_PI}), profiles());
  EXPECT_TRUE(mentions(problems, "goal"));
}

// Review Focus 4: the costmaps must actually carry the new footprint before the next drive.
TEST(FootprintMatch, PaddingAndOrderTolerant)
{
  const mmn::Polygon expected = polygon("vertical_carry");  // 0.54/-0.70, +/-0.385
  const mmn::Polygon published = {{-0.71, -0.395}, {-0.71, 0.395}, {0.55, 0.395}, {0.55, -0.395}};
  EXPECT_TRUE(mmn::footprint_matches(published, expected, 0.01));
  EXPECT_FALSE(mmn::footprint_matches(published, expected, 0.0));
}

TEST(FootprintMatch, RejectsOtherProfile)
{
  EXPECT_FALSE(mmn::footprint_matches(polygon("home"), polygon("vertical_carry"), 0.0));
  EXPECT_FALSE(mmn::footprint_matches({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, polygon("vertical_carry"), 0.0));
}

// Costmaps publish the footprint posed in their global frame (odom or map), not in
// base_footprint; it is mapped back with the robot pose before comparing.
TEST(FootprintMatch, WorldPolygonBackToBaseFrame)
{
  // Published /local_costmap/published_footprint with the robot at (-2.2, -7.725, pi), home profile.
  const mmn::Polygon world = {{-2.748, -8.355}, {-2.748, -7.095}, {-1.488, -7.095}, {-1.488, -8.355}};
  const auto base = mmn::to_base_frame(world, {-2.2, -7.725, M_PI});
  EXPECT_TRUE(mmn::footprint_matches(base, polygon("home"), 0.01));
  EXPECT_FALSE(mmn::footprint_matches(base, polygon("vertical_carry"), 0.01));
}

// Nav2 reports success while the base is still settling; ReconfigurePanel needs 0.5 s at rest.
TEST(Stillness, MeasuresTimeSinceTheBaseLastMoved)
{
  mmn::Stillness still(0.01, 0.02);
  EXPECT_DOUBLE_EQ(still.still_for(1.0), 0.0);  // no samples yet
  still.add(1.0, 0.05, 0.0);
  still.add(1.2, 0.005, 0.0);
  EXPECT_NEAR(still.still_for(1.7), 0.5, 1e-12);
  still.add(1.8, 0.0, 0.03);  // turning in place counts as moving
  EXPECT_NEAR(still.still_for(2.0), 0.0, 1e-12);
  still.add(2.1, 0.0, 0.0);
  EXPECT_NEAR(still.still_for(2.9), 0.8, 1e-12);
}

// The collision monitor's stop and slowdown zones are the profile grown by each zone's
// margin (navigation.launch.py padded()); B3 republishes them when the profile switches.
TEST(MonitorZones, PaddedRectangleMatchesTheLaunchZones)
{
  const auto stop = mmn::padded_rectangle(polygon("home"), 0.05);
  EXPECT_TRUE(mmn::footprint_matches(stop, {{0.59, 0.67}, {0.59, -0.67}, {-0.75, -0.67}, {-0.75, 0.67}}, 0.0, 1e-9));
}

TEST(MonitorZones, DynamicZonesComeFromTheNavigationConfig)
{
  const auto zones = mmn::monitor_zones(config().load("nav2_navigation.yaml"));
  ASSERT_EQ(zones.size(), 2u);
  EXPECT_EQ(zones[0].name, "StopZone");
  EXPECT_DOUBLE_EQ(zones[0].margin_m, 0.05);
  EXPECT_EQ(zones[0].polygon_topic, "/collision_monitor/stop_zone_in");
  EXPECT_EQ(zones[0].visual_topic, "/collision_monitor/stop_zone");
  EXPECT_EQ(zones[1].name, "SlowdownZone");
  EXPECT_DOUBLE_EQ(zones[1].margin_m, 0.30);
  EXPECT_EQ(zones[1].polygon_topic, "/collision_monitor/slowdown_zone_in");
}

// Minor 8: a collision monitor launched with static zones must fail the mission preflight.
TEST(MonitorZones, PreflightRequiresTheMonitorToFollowTheZoneInputs)
{
  const auto zones = mmn::monitor_zones(config().load("nav2_navigation.yaml"));
  EXPECT_TRUE(mmn::monitor_zone_problems(zones, {{"StopZone", "/collision_monitor/stop_zone_in"},
                                                 {"SlowdownZone", "collision_monitor/slowdown_zone_in"}}).empty());
  const auto static_zones = mmn::monitor_zone_problems(zones, {});
  ASSERT_EQ(static_zones.size(), 2u);
  EXPECT_NE(static_zones[0].find("footprint_mode:=profiles or dynamic"), std::string::npos);
  EXPECT_EQ(mmn::monitor_zone_problems(zones, {{"StopZone", "/collision_monitor/stop_zone_in"},
                                               {"SlowdownZone", "/other"}}).size(), 1u);
  EXPECT_EQ(mmn::monitor_zone_problems(zones, {{"StopZone", ""},
                                               {"SlowdownZone", "/collision_monitor/slowdown_zone_in"}}).size(), 1u);
}

// Follow-up 3c: the costmaps must publish the new footprint after the switch; a cached
// message from before it (a dead costmap) must not count, even when the profile is unchanged.
TEST(FootprintsApplied, RequiresMessagesReceivedAfterTheSwitch)
{
  const auto home = polygon("home");
  const mmn::Pose2 robot{0.0, 0.0, 0.0};
  const auto padded = [&](double pad) {
    mmn::Polygon p;
    for (const auto & [x, y] : home) p.push_back({x + std::copysign(pad, x), y + std::copysign(pad, y)});
    return p;
  };
  const std::vector<mmn::MonitorZone> zones = {{"StopZone", 0.05, "/stop_in", "/stop"}};
  std::map<std::string, mmn::PublishedPolygon> latest = {
    {"/local_costmap/published_footprint", {padded(0.01), "map", 5.0}},
    {"/global_costmap/published_footprint", {padded(0.01), "map", 5.0}},
    {"/stop_in", {mmn::padded_rectangle(home, 0.05), "base_footprint", 5.0}}};
  const std::vector<std::string> costmaps = {"/local_costmap/published_footprint", "/global_costmap/published_footprint"};
  EXPECT_TRUE(mmn::footprints_applied(latest, costmaps, zones, home, robot, 4.0, 0.01));
  EXPECT_FALSE(mmn::footprints_applied(latest, costmaps, zones, home, robot, 6.0, 0.01));  // stale costmaps
  latest["/local_costmap/published_footprint"].received = 7.0;
  latest["/global_costmap/published_footprint"].received = 7.0;
  EXPECT_TRUE(mmn::footprints_applied(latest, costmaps, zones, home, robot, 6.0, 0.01));   // latched zone ok
  latest.erase("/stop_in");
  EXPECT_FALSE(mmn::footprints_applied(latest, costmaps, zones, home, robot, 6.0, 0.01));  // zone missing
}

// The first costmap message showing a new footprint can close a cycle inflated for the old one;
// the drive may start only after a later matching message on every topic.
TEST(FootprintRefresh, WaitsForACycleCompletedOnTheNewFootprint)
{
  const auto home = polygon("home");
  const mmn::Pose2 robot{0.0, 0.0, 0.0};
  mmn::Polygon padded;
  for (const auto & [x, y] : home) padded.push_back({x + std::copysign(0.01, x), y + std::copysign(0.01, y)});
  const std::vector<std::string> costmaps = {"/local_costmap/published_footprint", "/global_costmap/published_footprint"};
  std::map<std::string, mmn::PublishedPolygon> latest = {
    {costmaps[0], {padded, "map", 1.0}}, {costmaps[1], {padded, "map", 1.0}}};
  const auto applied = [&](const mmn::FootprintRefresh & r) {
    return mmn::footprints_applied(latest, costmaps, {}, home, robot, r.since(), 0.01);
  };

  mmn::FootprintRefresh refresh(2.0);                      // switched at 2.0
  EXPECT_FALSE(refresh.observe(applied(refresh), 2.1));    // only messages from before the switch
  latest[costmaps[0]].received = 2.2;                      // local costmap shows it (5 Hz)
  latest[costmaps[0]].received = 2.4;
  EXPECT_FALSE(refresh.observe(applied(refresh), 2.45));   // global costmap has not yet
  latest[costmaps[1]].received = 2.5;                      // global shows it: stale inflation
  EXPECT_FALSE(refresh.observe(applied(refresh), 2.52));
  EXPECT_TRUE(refresh.shown());
  EXPECT_DOUBLE_EQ(refresh.since(), 2.52);
  latest[costmaps[0]].received = 2.6;
  EXPECT_FALSE(refresh.observe(applied(refresh), 2.62));   // the global costmap's next cycle is due
  latest[costmaps[1]].received = 3.0;
  EXPECT_TRUE(refresh.observe(applied(refresh), 3.02));
  EXPECT_TRUE(refresh.observe(false, 3.1));                // stays refreshed
}
