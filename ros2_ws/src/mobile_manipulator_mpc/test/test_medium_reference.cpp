// B5's medium-layer references (Phase 5A Task 10): the medium plan as B5's route and its arc-length lookups, and the
// per-cycle fallback to the frozen behaviour.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "mobile_manipulator_mpc/medium_reference.hpp"

namespace
{
using namespace mobile_manipulator_mpc;
using Eigen::Vector2d;
using Joints = Eigen::Matrix<double, 6, 1>;

Joints joints(double v) { return Joints::Constant(v); }

// Samples every 0.1 m along +x from the origin to `length`, with yaw, speed and q rising linearly with s.
std::vector<MediumSample> straight(double length)
{
  std::vector<MediumSample> out;
  for (int i = 0; i * 0.1 <= length + 1e-9; ++i) {
    const double s = 0.1 * i;
    out.push_back({s, s, 0.0, 0.1 * s, 0.2 + 0.1 * s, joints(s)});
  }
  return out;
}

TEST(MediumReference, RouteSplicesNav2Remainder)
{
  // The plan's 1 m along +x, then the Nav2 route's remainder from its arc length 1.0 (the corner, then 3 m north).
  const Route nav2({Vector2d(0.0, 0.0), Vector2d(1.0, 0.0), Vector2d(1.0, 3.0)});
  const MediumReference ref(straight(1.0), nav2, 1.0, joints(0.0));
  EXPECT_NEAR(ref.route().length(), 4.0, 1e-9);
  EXPECT_NEAR(ref.goal_s(), 4.0, 1e-9);
  EXPECT_TRUE(ref.route().at(2.0).isApprox(Vector2d(1.0, 1.0), 1e-9));
  // a plan ending off the Nav2 route joins it at the splice point
  std::vector<MediumSample> off = straight(1.0);
  off.back().y = 0.1;
  const MediumReference joined(off, nav2, 1.0, joints(0.0));
  EXPECT_TRUE(joined.route().at(joined.samples_length()).isApprox(Vector2d(1.0, 0.1), 1e-9));
  EXPECT_NEAR(joined.route().length(), joined.samples_length() + 0.1 + 3.0, 1e-9);
}

TEST(MediumReference, SpliceWithNoRemainder)
{
  const Route nav2({Vector2d(0.0, 0.0), Vector2d(1.0, 0.0)});
  const MediumReference ref(straight(1.0), nav2, 1.0, joints(0.0));
  EXPECT_NEAR(ref.route().length(), 1.0, 1e-9);
  EXPECT_NEAR(ref.goal_s(), 1.0, 1e-9);
  EXPECT_THROW(MediumReference({straight(1.0).front()}, nav2, 0.0, joints(0.0)), std::invalid_argument);
}

TEST(MediumReference, LookupsByArcLength)
{
  const Route nav2({Vector2d(0.0, 0.0), Vector2d(1.0, 0.0), Vector2d(1.0, 3.0)});
  const MediumReference ref(straight(1.0), nav2, 1.0, joints(-1.0));
  EXPECT_NEAR(ref.yaw_at(0.55), 0.055, 1e-9);
  EXPECT_NEAR(ref.speed_at(0.55), 0.255, 1e-9);
  EXPECT_TRUE(ref.posture_at(0.55).isApprox(joints(0.55), 1e-9));
  // beyond the samples: the route's tangent, B5's own speed (infinity: no cap), the comfort pose
  EXPECT_NEAR(ref.yaw_at(2.0), M_PI / 2, 1e-9);
  EXPECT_TRUE(std::isinf(ref.speed_at(2.0)));
  EXPECT_TRUE(ref.posture_at(2.0).isApprox(joints(-1.0), 1e-12));
  // at equal s (a turn on the spot) a lookup takes the last sample
  std::vector<MediumSample> turn = straight(0.5);
  turn.push_back({0.5, 0.5, 0.0, 0.8, 0.0, joints(2.0)});
  const MediumReference spot(turn, nav2, 0.5, joints(0.0));
  EXPECT_NEAR(spot.yaw_at(0.5), 0.8, 1e-12);
  EXPECT_TRUE(spot.posture_at(0.5).isApprox(joints(2.0), 1e-12));
}

TEST(MediumReference, InPlaceTurnKeepsGoalHeading)
{
  // A plan ending at the goal with a turn on the spot to the goal heading: the route ends there (the goal), and its
  // last yaw is the turned one, so B5's goal-heading term near the goal sees no other target.
  const Route nav2({Vector2d(0.0, 0.0), Vector2d(1.0, 0.0)});
  std::vector<MediumSample> turn = straight(1.0);
  for (const double yaw : {0.4, 0.8, 1.2}) turn.push_back({1.0, 1.0, 0.0, yaw, 0.0, joints(0.0)});
  const MediumReference ref(turn, nav2, 1.0, joints(0.0));
  EXPECT_NEAR(ref.goal_s(), 1.0, 1e-9);
  EXPECT_NEAR(ref.yaw_at(1.0), 1.2, 1e-12);
  EXPECT_NEAR(ref.yaw_at(5.0), 1.2, 1e-12);  // beyond the goal: still the last sample (no remainder to follow)
}

TEST(MediumReference, StartHintIsTheNearestSampleWithinTheFirstMetre)
{
  const Route nav2({Vector2d(0.0, 0.0), Vector2d(3.0, 0.0)});
  const MediumReference ref(straight(3.0), nav2, 3.0, joints(0.0));
  EXPECT_NEAR(ref.start_hint(Vector2d(0.41, 0.05)), 0.4, 1e-9);
  EXPECT_NEAR(ref.start_hint(Vector2d(2.5, 0.0)), 1.0, 1e-9);  // only the first metre
}

TEST(MediumFallback, EachReason)
{
  MpcConfig c{};
  c.medium_stale_s = 0.5;
  c.medium_max_offset_m = 0.3;
  const double none = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(medium_fallback(0.1, 7, 7, 0.05, c), MediumFallback::None);
  EXPECT_EQ(medium_fallback(none, 7, 7, 0.05, c), MediumFallback::NoPlan);
  EXPECT_EQ(medium_fallback(0.51, 7, 7, 0.05, c), MediumFallback::Stale);
  EXPECT_EQ(medium_fallback(0.1, 6, 7, 0.05, c), MediumFallback::RouteId);
  EXPECT_EQ(medium_fallback(0.1, 7, 7, 0.31, c), MediumFallback::Offset);
  EXPECT_STREQ(to_string(MediumFallback::None), "");
  EXPECT_STREQ(to_string(MediumFallback::NoPlan), "no_plan");
  EXPECT_STREQ(to_string(MediumFallback::Stale), "stale");
  EXPECT_STREQ(to_string(MediumFallback::RouteId), "route_id");
  EXPECT_STREQ(to_string(MediumFallback::Offset), "offset");
}
}  // namespace

TEST(MediumReference, RejectsNonFiniteSamples)
{
  // A NaN would reach B5's QP and fail it until the plan went stale; the node counts the plan as rejected instead.
  const Route nav2({Vector2d(0.0, 0.0), Vector2d(1.0, 0.0)});
  for (int field = 0; field < 5; ++field) {
    std::vector<MediumSample> bad = straight(1.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (field == 0) bad[3].x = nan;
    if (field == 1) bad[3].y = nan;
    if (field == 2) bad[3].yaw = nan;
    if (field == 3) bad[3].speed = nan;
    if (field == 4) bad[3].q(2) = nan;
    EXPECT_THROW(MediumReference(bad, nav2, 1.0, joints(0.0)), std::invalid_argument) << field;
  }
}
