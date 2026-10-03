#include <gtest/gtest.h>

#include "mobile_manipulator_navigation/navigate_run.hpp"

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
