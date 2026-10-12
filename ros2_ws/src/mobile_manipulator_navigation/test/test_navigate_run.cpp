#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_navigation/navigate_run.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mmn = mobile_manipulator_navigation;

// Final review I2: CPU and memory over an interval, for drives and for reconfigurations.
TEST(UsageReport, CpuPercentOfACoreAndPeakMemory)
{
  const std::map<std::string, mmn::Usage> before = {{"move_group", {1.0, 150.0}}};
  const std::map<std::string, mmn::Usage> after = {{"move_group", {1.5, 210.0}},
                                                   {"reconfigure_panel_server", {0.2, 30.0}}};
  const auto report = mmn::usage_report(before, after, 2.0);
  EXPECT_DOUBLE_EQ(report.at("cpu_percent_of_core").at("move_group").get<double>(), 25.0);
  EXPECT_DOUBLE_EQ(report.at("cpu_percent_of_core").at("reconfigure_panel_server").get<double>(), 10.0);
  EXPECT_DOUBLE_EQ(report.at("max_rss_mb").at("move_group").get<double>(), 210.0);
}

// A goal Nav2 reports reached counts only if the base ended there (stateful goal checker).
TEST(DriveStatus, NavSuccessFarFromTheGoalIsOffGoal)
{
  const mmn::GoalTolerance tolerance{0.15, 0.15};
  EXPECT_EQ(mmn::drive_status("succeeded", 0.148, 0.10, tolerance), "succeeded");
  EXPECT_EQ(mmn::drive_status("succeeded", 0.169, 0.169, tolerance), "succeeded");  // settling margin
  EXPECT_EQ(mmn::drive_status("succeeded", 3.78, 0.143, tolerance), "off_goal");
  EXPECT_EQ(mmn::drive_status("succeeded", 0.10, 0.20, tolerance), "off_goal");
  EXPECT_EQ(mmn::drive_status("succeeded", std::nullopt, std::nullopt, tolerance), "off_goal");
  EXPECT_EQ(mmn::drive_status("aborted", 0.05, 0.0, tolerance), "aborted");
}

TEST(DriveStatus, TolerancesComeFromTheGoalChecker)
{
  const auto share = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const auto tolerance = mmn::goal_tolerance(mmn::load_yaml_file(share + "/config/nav2_navigation.yaml"));
  EXPECT_DOUBLE_EQ(tolerance.xy_m, 0.15);
  EXPECT_DOUBLE_EQ(tolerance.yaw_rad, 0.15);
}

// Phase 4 Task 3 carry-forward: the 2D base metric covers ground obstacles and ones the base can
// drive under only up to the base band; a floating box (an overhead beam) is the 3D clearance's.
TEST(GroundObstaclePoints, FloatingBoxesAtOrAboveTheBaseBandAreExcluded)
{
  const mmn::Json obstacles = {
    {{"name", "post"}, {"x", 1.0}, {"y", 2.0}, {"size_x", 0.4}, {"size_y", 0.4}, {"height", 2.0}},
    {{"name", "low_shelf"}, {"x", 5.0}, {"y", 0.0}, {"size_x", 0.4}, {"size_y", 0.4}, {"height", 0.2}, {"z", 0.5}},
    {{"name", "beam"}, {"x", 9.0}, {"y", 0.0}, {"size_x", 1.0}, {"size_y", 3.0}, {"height", 0.2}, {"z", 0.6}},
    {{"name", "high_beam"}, {"x", 12.0}, {"y", 0.0}, {"size_x", 1.0}, {"size_y", 3.0}, {"height", 0.2}, {"z", 1.5}}};
  const auto points = mmn::ground_obstacle_points(obstacles);
  const size_t post = mmn::box_outline(1.0, 2.0, 0.4, 0.4).size(), shelf = mmn::box_outline(5.0, 0.0, 0.4, 0.4).size();
  ASSERT_EQ(points.size(), post + shelf);
  for (const auto & p : points) EXPECT_LT(p[0], 6.0) << "a point of the beam";
  EXPECT_TRUE(mmn::ground_obstacle_points(mmn::Json::array()).empty());
}
