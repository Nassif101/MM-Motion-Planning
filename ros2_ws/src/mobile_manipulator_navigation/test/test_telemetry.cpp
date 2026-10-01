#include <gtest/gtest.h>
#include <cmath>
#include "mobile_manipulator_navigation/telemetry.hpp"

using namespace mobile_manipulator_navigation;

TEST(Telemetry, DownsampleKeepsEndsAndSpacing)
{
  std::vector<PathPoint> points;
  for (int i = 0; i < 41; ++i) points.push_back({i * 0.05, 0.0, 0.0});  // 2 m at 5 cm
  const auto kept = downsample(points, 0.1);
  ASSERT_EQ(kept.size(), 21u);
  EXPECT_EQ(kept.front(), 0u);
  EXPECT_EQ(kept.back(), 40u);
  for (size_t i = 1; i + 1 < kept.size(); ++i) {
    EXPECT_GE(points[kept[i]].x - points[kept[i - 1]].x, 0.1 - 1e-9);
  }
  EXPECT_TRUE(downsample({}, 0.1).empty());
  EXPECT_EQ(downsample({{1.0, 2.0, 0.0}}, 0.1), std::vector<size_t>{0});
}

TEST(Telemetry, PathLengthAndCrossTrack)
{
  const std::vector<PathPoint> path = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0}};
  EXPECT_NEAR(path_length(path), 4.0, 1e-12);
  EXPECT_NEAR(*cross_track(1.0, 0.3, path), 0.3, 1e-12);
  EXPECT_NEAR(*cross_track(2.5, 1.0, path), 0.5, 1e-12);
  EXPECT_NEAR(*cross_track(-1.0, 0.0, path), 1.0, 1e-12);  // beyond the start
  EXPECT_FALSE(cross_track(0.0, 0.0, {}).has_value());
}

TEST(Telemetry, EventLogIsBoundedAndOrdered)
{
  EventLog log(3);
  for (int i = 0; i < 5; ++i) log.add(i, "e" + std::to_string(i));
  ASSERT_EQ(log.events().size(), 3u);
  EXPECT_EQ(log.events()[0].text, "e2");
  EXPECT_EQ(log.events()[2].text, "e4");
}

TEST(Telemetry, GapCounterRateAndLongGaps)
{
  GapCounter counter(0.5);
  for (int i = 0; i <= 10; ++i) EXPECT_FALSE(counter.add(0.1 * i).has_value());  // 10 Hz for 1 s
  EXPECT_NEAR(counter.rate(), 10.0, 1e-9);
  const auto gap = counter.add(1.8);  // 0.8 s gap
  ASSERT_TRUE(gap.has_value());
  EXPECT_NEAR(*gap, 0.8, 1e-9);
  EXPECT_EQ(counter.long_gaps(), 1);
  EXPECT_NEAR(counter.max_gap(), 0.8, 1e-9);
}

TEST(Telemetry, StatusNames)
{
  EXPECT_EQ(goal_status_name(2), "executing");
  EXPECT_EQ(goal_status_name(9), "9");
  EXPECT_EQ(monitor_action_name(2), "slowdown");
  EXPECT_DOUBLE_EQ(round_to(1.23456, 2), 1.23);
}
