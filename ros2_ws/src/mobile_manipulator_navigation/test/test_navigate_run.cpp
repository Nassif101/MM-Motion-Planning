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
