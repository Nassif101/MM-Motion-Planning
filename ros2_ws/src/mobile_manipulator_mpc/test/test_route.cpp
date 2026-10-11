#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "mobile_manipulator_mpc/route.hpp"

using mobile_manipulator_mpc::Route;
using Eigen::Vector2d;

TEST(Route, ProjectsOntoNearestSegment)
{
  const Route route({Vector2d(0, 0), Vector2d(4, 0), Vector2d(4, 3)});
  EXPECT_DOUBLE_EQ(route.length(), 7.0);
  const auto a = route.project(Vector2d(1.5, 0.4), 1.5);
  EXPECT_NEAR(a.s, 1.5, 1e-12);
  EXPECT_NEAR((a.point - Vector2d(1.5, 0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(a.heading, 0.0, 1e-12);
  EXPECT_NEAR(a.lateral, 0.4, 1e-12);  // left of +x
  const auto b = route.project(Vector2d(4.5, 1.0), 5.0);
  EXPECT_NEAR(b.s, 5.0, 1e-12);
  EXPECT_NEAR(b.heading, M_PI / 2, 1e-12);
  EXPECT_NEAR(b.lateral, -0.5, 1e-12);  // right of +y
  // Beyond the ends the projection clamps to the route.
  const auto before = route.project(Vector2d(-2, 0), 0.0);
  EXPECT_NEAR(before.s, 0.0, 1e-12);
  const auto after = route.project(Vector2d(4, 9), 7.0);
  EXPECT_NEAR(after.s, 7.0, 1e-12);
  EXPECT_NEAR(route.at(100.0).y(), 3.0, 1e-12);
  EXPECT_NEAR(route.at(-1.0).x(), 0.0, 1e-12);
  EXPECT_NEAR((route.at(5.5) - Vector2d(4, 1.5)).norm(), 0.0, 1e-12);
}

TEST(Route, ProjectStaysOnBranchNearHint)
{
  // A U: out along y = 0, back along y = 0.5; the point between is 0.25 m from both branches.
  const Route route({Vector2d(0, 0), Vector2d(10, 0), Vector2d(10, 0.5), Vector2d(0, 0.5)});
  const Vector2d p(5.0, 0.2);
  const auto out = route.project(p, 5.0);
  EXPECT_NEAR(out.s, 5.0, 1e-12);
  EXPECT_NEAR(out.heading, 0.0, 1e-12);
  const auto back = route.project(p, 15.5);
  EXPECT_NEAR(back.s, 10.5 + 5.0, 1e-12);
  EXPECT_NEAR(std::abs(back.heading), M_PI, 1e-12);
  EXPECT_NEAR(back.lateral, 0.3 * 1.0, 1e-12);  // heading -x: left is -y, the point is at y < 0.5 -> left
  // The global range search takes the nearer branch.
  EXPECT_NEAR(route.project_range(p, 0.0, route.length()).s, 5.0, 1e-12);
}

TEST(Route, HeadingAtCorners)
{
  const Route route({Vector2d(0, 0), Vector2d(2, 0), Vector2d(2, 2), Vector2d(0, 2)});
  EXPECT_NEAR(route.heading_at(0.0), 0.0, 1e-12);
  EXPECT_NEAR(route.heading_at(1.0), 0.0, 1e-12);
  EXPECT_NEAR(route.heading_at(2.0), M_PI / 2, 1e-12);  // corner: the outgoing segment
  EXPECT_NEAR(route.heading_at(3.0), M_PI / 2, 1e-12);
  EXPECT_NEAR(std::abs(route.heading_at(4.0)), M_PI, 1e-12);
  EXPECT_NEAR(std::abs(route.heading_at(6.0)), M_PI, 1e-12);  // the end: the last segment
  EXPECT_NEAR(std::abs(route.heading_at(9.0)), M_PI, 1e-12);  // clamped
}

TEST(Route, DegenerateRoutesThrow)
{
  EXPECT_THROW(Route({Vector2d(1, 1)}), std::invalid_argument);
  EXPECT_THROW(Route({Vector2d(1, 1), Vector2d(1, 1)}), std::invalid_argument);
  const Route route({Vector2d(0, 0), Vector2d(0, 0), Vector2d(1, 0)});
  EXPECT_DOUBLE_EQ(route.length(), 1.0);
}
