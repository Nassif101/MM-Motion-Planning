// Expected values were computed with the Python scenario-task helpers these functions replace.
#include <gtest/gtest.h>

#include <cmath>

#include "mobile_manipulator_navigation/scenario_metrics.hpp"

namespace mmn = mobile_manipulator_navigation;

TEST(ScenarioMetrics, OccupiedPointsAreCellCentresAtOrAbove65)
{
  const std::vector<int8_t> data = {0, 100, -1, 65, 64, 0, 100, 0};  // 4 x 2, unknown is -1
  const auto points = mmn::occupied_points(data, 4, 2, 0.5, -1.0, 2.0);
  const std::vector<mmn::Point2> expected = {{-0.25, 2.25}, {0.75, 2.25}, {0.25, 2.75}};
  EXPECT_EQ(points, expected);
}

TEST(ScenarioMetrics, MinClearanceAndPathLength)
{
  const std::vector<mmn::Point2> path = {{0.0, 0.0}, {1.0, 0.5}, {2.0, 0.25}};
  EXPECT_EQ(mmn::min_clearance(path, {{1.2, 1.7}, {5.0, -0.3}, {-2.5, 0.1}, {9.0, 9.0}}), 1.217);
  EXPECT_EQ(mmn::min_clearance(path, {{20.0, 20.0}}), 3.0);  // nothing within 3 m of the path box
  EXPECT_FALSE(mmn::min_clearance({}, {{1.0, 1.0}}).has_value());
  EXPECT_FALSE(mmn::min_clearance(path, {}).has_value());
  EXPECT_DOUBLE_EQ(mmn::round_digits(mmn::polyline_length(path), 3), 2.149);
}

TEST(ScenarioMetrics, RectangleDistanceToTheFootprint)
{
  const mmn::Bounds home{-0.7, 0.54, -0.62, 0.62};
  const std::vector<mmn::Point2> points = {{1.0, 0.0}, {0.6, 0.7}, {-1.0, -1.0}, {0.0, 0.0}, {0.2, -0.9}};
  const std::vector<double> expected = {0.46, 0.1, 0.484148737476, 0.0, 0.28};
  for (size_t i = 0; i < points.size(); ++i) EXPECT_NEAR(mmn::rectangle_distance(points[i], home), expected[i], 1e-12);
  // Posed at (10, 5) facing +y: the map point (10, 6.0) is 1.0 m ahead, 0.46 m beyond the front.
  EXPECT_NEAR(*mmn::footprint_clearance({{10.0, 6.0}, {30.0, 5.0}}, 10.0, 5.0, M_PI / 2, home, 8.0), 0.46, 1e-12);
  EXPECT_FALSE(mmn::footprint_clearance({{30.0, 5.0}}, 10.0, 5.0, 0.0, home, 8.0).has_value());
}

TEST(ScenarioMetrics, BoxOutlineSamplesEachEdgeWithRoundHalfToEven)
{
  const auto points = mmn::box_outline(1.0, -2.0, 0.1, 0.05);
  // x: 6 samples per edge; y: 0.05 / 0.02 = 2.5 rounds to 2, so 3 samples per edge.
  ASSERT_EQ(points.size(), 18u);
  const std::vector<mmn::Point2> expected = {
    {0.95, -2.025}, {0.97, -2.025}, {0.99, -2.025}, {1.01, -2.025}, {1.03, -2.025}, {1.05, -2.025},
    {0.95, -1.975}, {0.97, -1.975}, {0.99, -1.975}, {1.01, -1.975}, {1.03, -1.975}, {1.05, -1.975},
    {0.95, -2.025}, {0.95, -2.0}, {0.95, -1.975}, {1.05, -2.025}, {1.05, -2.0}, {1.05, -1.975}};
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(points[i][0], expected[i][0], 1e-12);
    EXPECT_NEAR(points[i][1], expected[i][1], 1e-12);
  }
}

TEST(ScenarioMetrics, SummaryStatistics)
{
  const auto summary = mmn::summarize({0.3, 0.1, 0.2, 0.5, 0.4, 0.05, 0.9, 0.7, 0.6, 0.8, 1.0, 0.15, 0.25, 0.35,
                                       0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 0.33});
  ASSERT_TRUE(summary.has_value());
  EXPECT_DOUBLE_EQ(summary->mean, 0.5157);
  EXPECT_DOUBLE_EQ(summary->p95, 0.95);
  EXPECT_DOUBLE_EQ(summary->max, 1.0);
  EXPECT_DOUBLE_EQ(mmn::summarize({2.0})->p95, 2.0);
  EXPECT_FALSE(mmn::summarize({}).has_value());
  EXPECT_NEAR(mmn::yaw_of(0.0, 0.0, std::sin(1.2), std::cos(1.2)), 2.4, 1e-12);
  EXPECT_DOUBLE_EQ(mmn::round_digits(std::abs(std::remainder(3.1 - (-3.1), 2 * M_PI)), 3), 0.083);
}
