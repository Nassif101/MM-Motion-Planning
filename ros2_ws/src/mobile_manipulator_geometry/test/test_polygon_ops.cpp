#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

#include "mobile_manipulator_geometry/polygon_ops.hpp"

namespace mmg = mobile_manipulator_geometry;

namespace
{
double cross(const mmg::Point2 & o, const mmg::Point2 & a, const mmg::Point2 & b)
{
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

// Outward distance of p from the line of the counter-clockwise edge a -> b (positive outside).
double outward(const mmg::Point2 & a, const mmg::Point2 & b, const mmg::Point2 & p)
{
  return -cross(a, b, p) / std::hypot(b[0] - a[0], b[1] - a[1]);
}

double segment_distance(const mmg::Point2 & a, const mmg::Point2 & b, const mmg::Point2 & p)
{
  const double ex = b[0] - a[0], ey = b[1] - a[1];
  const double t = std::clamp(((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / (ex * ex + ey * ey), 0.0, 1.0);
  return std::hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ey);
}

double boundary_distance(const mmg::Polygon & polygon, const mmg::Point2 & p)
{
  double best = INFINITY;
  for (size_t i = 0; i < polygon.size(); ++i) {
    best = std::min(best, segment_distance(polygon[i], polygon[(i + 1) % polygon.size()], p));
  }
  return best;
}

mmg::Polygon rectangle(double half_x, double half_y)
{
  return {{half_x, -half_y}, {half_x, half_y}, {-half_x, half_y}, {-half_x, -half_y}};
}

// The paper's padding (Sagar et al., Eq. 8): each vertex pushed by delta away from the centroid.
mmg::Polygon centroid_push(const mmg::Polygon & polygon, double delta)
{
  double cx = 0.0, cy = 0.0;
  for (const auto & v : polygon) {
    cx += v[0] / polygon.size();
    cy += v[1] / polygon.size();
  }
  mmg::Polygon pushed;
  for (const auto & v : polygon) {
    const double n = std::hypot(v[0] - cx, v[1] - cy);
    pushed.push_back({v[0] + delta * (v[0] - cx) / n, v[1] + delta * (v[1] - cy) / n});
  }
  return pushed;
}

constexpr double kDelta = 0.02;
}  // namespace

TEST(Hull, ContainsEveryPointConvexCcw)
{
  std::mt19937 random(7);
  std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
  std::vector<mmg::Point2> points;
  for (int i = 0; i < 200; ++i) points.push_back({coordinate(random), coordinate(random)});
  const auto hull = mmg::convex_hull(points);
  ASSERT_GE(hull.size(), 3u);
  // The hull's own vertices lie on its boundary: allow rounding.
  EXPECT_GE(mmg::contains(hull, points).margin_m, -1e-12);
  for (size_t i = 0; i < hull.size(); ++i) {
    EXPECT_GT(cross(hull[i], hull[(i + 1) % hull.size()], hull[(i + 2) % hull.size()]), 1e-12);
  }
}

TEST(Hull, DropsCollinearAndDuplicates)
{
  const std::vector<mmg::Point2> points{{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.5, 0}, {1, 0.5},
                                        {0.5, 1}, {0, 0.5}, {0, 0}, {1, 1}};
  EXPECT_EQ(mmg::convex_hull(points).size(), 4u);
}

TEST(Hull, ThrowsOnDegenerate)
{
  EXPECT_THROW(mmg::convex_hull({{0, 0}, {1, 1}}), std::invalid_argument);
  EXPECT_THROW(mmg::convex_hull({{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}}), std::invalid_argument);
}

TEST(Offset, EveryEdgeAtDelta)
{
  const mmg::Polygon hexagon{{1.0, 0.0}, {0.6, 0.7}, {-0.4, 0.8}, {-1.0, 0.1}, {-0.7, -0.6}, {0.3, -0.8}};
  for (const auto & polygon : {rectangle(0.62, 0.02), hexagon}) {
    const auto offset = mmg::offset_outward(polygon, kDelta);
    // Each original edge's line moved out by delta supports the offset polygon.
    for (size_t i = 0; i < polygon.size(); ++i) {
      const auto & a = polygon[i];
      const auto & b = polygon[(i + 1) % polygon.size()];
      double farthest = -INFINITY;
      for (const auto & v : offset) farthest = std::max(farthest, outward(a, b, v));
      EXPECT_NEAR(farthest, kDelta, 1e-9);
    }
    EXPECT_GE(mmg::contains(offset, polygon).margin_m, kDelta - 1e-9);
  }
}

TEST(Offset, BevelsSharpCorners)
{
  // A 20 degree apex at the origin.
  const double half = 10.0 * M_PI / 180.0;
  const mmg::Polygon triangle{{0.0, 0.0}, {std::cos(half), -std::sin(half)}, {std::cos(half), std::sin(half)}};
  const auto offset = mmg::offset_outward(triangle, kDelta);
  for (const auto & v : offset) {
    EXPECT_LE(boundary_distance(triangle, v), 2.0 * kDelta + 1e-9);
  }
  // The bevel keeps every edge at least delta from the polygon.
  EXPECT_GE(mmg::contains(offset, triangle).margin_m, kDelta - 1e-9);
}

TEST(Offset, BeatsCentroidPushOnThinHull)
{
  const auto panel = rectangle(0.62, 0.02);
  EXPECT_LT(mmg::contains(centroid_push(panel, kDelta), panel).margin_m, 0.1 * kDelta);
  EXPECT_NEAR(mmg::contains(mmg::offset_outward(panel, kDelta), panel).margin_m, kDelta, 1e-9);
}

TEST(Hausdorff, ShiftedSquare)
{
  const auto square = rectangle(0.5, 0.5);
  mmg::Polygon shifted;
  for (const auto & v : square) shifted.push_back({v[0] + 0.03, v[1]});
  EXPECT_NEAR(mmg::hausdorff(square, shifted), 0.03, 1e-3);
  EXPECT_NEAR(mmg::hausdorff(square, square), 0.0, 1e-12);
}

TEST(Measures, AreaAndCircumscribedRadius)
{
  EXPECT_NEAR(mmg::area(rectangle(0.5, 0.25)), 0.5, 1e-12);
  EXPECT_NEAR(mmg::circumscribed_radius(rectangle(0.3, 0.4)), 0.5, 1e-12);
}
