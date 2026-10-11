#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include "mobile_manipulator_geometry/polygon_ops.hpp"
#include "mobile_manipulator_navigation/dynamic_footprint.hpp"
#include "mobile_manipulator_navigation/mission.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;

namespace
{
const std::vector<std::string> kArm = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

// A square of half-width 0.3 m + shoulder_pan_joint.
class SquareModel : public mmg::FootprintModel
{
public:
  std::vector<mmg::Point2> points(const mmg::JointMap & joints) const override
  {
    const double h = 0.3 + joints.at("shoulder_pan_joint");
    return {{h, -h}, {h, h}, {-h, h}, {-h, -h}};
  }
};

// A rectangle reaching 0.3 m + shoulder_pan_joint forward, 0.3 m + shoulder_lift_joint to each
// side and 0.3 m behind, with the panel's lever on wrist_1_joint (1 m/rad) added to both reaches.
class ReachModel : public mmg::FootprintModel
{
public:
  std::vector<mmg::Point2> points(const mmg::JointMap & joints) const override
  {
    const double sway = joints.at("wrist_1_joint");
    const double x = 0.3 + joints.at("shoulder_pan_joint") + sway;
    const double y = 0.3 + joints.at("shoulder_lift_joint") + sway;
    return {{x, -y}, {x, y}, {-0.3, y}, {-0.3, -y}};
  }
};

mmn::DynamicFootprint make(double inflation_radius_m = 1.0)
{
  return mmn::DynamicFootprint(std::make_shared<SquareModel>(), kArm, {0.05, 0.25}, 0.02, 0.01, 0.5, inflation_radius_m);
}

void all_joints(mmn::DynamicFootprint & footprint, double t, double pan)
{
  footprint.joints(t, kArm, {pan, 0, 0, 0, 0, 0});
}

mmn::DynamicFootprint make_reach()
{
  return mmn::DynamicFootprint(std::make_shared<ReachModel>(), kArm, {0.05}, 0.02, 0.01, 0.5, 1.0);
}

void reach(mmn::DynamicFootprint & footprint, double t, double pan, double lift, double wrist_1)
{
  footprint.joints(t, kArm, {pan, lift, 0, wrist_1, 0, 0});
}

// Whether `point` lies inside (or on, within `tolerance`) the counter-clockwise convex polygon.
bool contains(const mmg::Polygon & polygon, const mmg::Point2 & point, double tolerance = 1e-9)
{
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]);
    if (cross < -tolerance) return false;
  }
  return true;
}

bool contains_all(const mmg::Polygon & outer, const mmg::Polygon & inner)
{
  for (const auto & point : inner) {
    if (!contains(outer, point)) return false;
  }
  return true;
}

double max_x(const mmg::Polygon & polygon)
{
  double x = -INFINITY;
  for (const auto & p : polygon) x = std::max(x, p[0]);
  return x;
}

double half_width(const mmg::Polygon & polygon)
{
  double h = 0.0;
  for (const auto & [x, y] : polygon) h = std::max(h, std::abs(y));
  return h;
}
}  // namespace

TEST(DynamicFootprint, SilentBeforeFirstJointState)
{
  auto footprint = make();
  EXPECT_FALSE(footprint.tick(0.0));
  EXPECT_EQ(footprint.health(0.0), mmn::DynamicFootprint::Health::NoData);
  EXPECT_FALSE(footprint.current());
}

TEST(DynamicFootprint, PublishesFirstAndOnChange)
{
  auto footprint = make();
  all_joints(footprint, 0.0, 0.0);
  const auto first = footprint.tick(0.05);
  ASSERT_TRUE(first);
  const auto expected = mmg::offset_outward(mmg::convex_hull(SquareModel().points({{"shoulder_pan_joint", 0.0}})), 0.02);
  EXPECT_NEAR(mmg::hausdorff(first->footprint, expected), 0.0, 1e-9);
  ASSERT_EQ(first->zones.size(), 2u);
  EXPECT_NEAR(mmg::hausdorff(first->zones[0], mmg::offset_outward(first->footprint, 0.05)), 0.0, 1e-9);
  EXPECT_NEAR(mmg::hausdorff(first->zones[1], mmg::offset_outward(first->footprint, 0.25)), 0.0, 1e-9);
  EXPECT_FALSE(footprint.tick(0.10));  // unchanged
  all_joints(footprint, 0.12, 0.02);
  const auto moved = footprint.tick(0.15);
  ASSERT_TRUE(moved);
  EXPECT_NEAR(half_width(moved->footprint), 0.3 + 0.02 + 0.02, 1e-9);
  EXPECT_EQ(footprint.health(0.15), mmn::DynamicFootprint::Health::Fresh);
}

TEST(DynamicFootprint, IgnoresSubThresholdJitter)
{
  auto footprint = make();
  all_joints(footprint, 0.0, 0.0);
  ASSERT_TRUE(footprint.tick(0.0));
  std::mt19937 random(3);
  std::uniform_real_distribution<double> jitter(-0.004, 0.004);
  for (int i = 1; i <= 100; ++i) {
    all_joints(footprint, i * 0.05, jitter(random));
    EXPECT_FALSE(footprint.tick(i * 0.05)) << i;
  }
}

TEST(DynamicFootprint, MergesPartialJointStates)
{
  auto footprint = make();
  footprint.joints(0.0, kArm, {0.1, 0.5, 0.5, 0.5, 0.5, 0.5});
  ASSERT_TRUE(footprint.tick(0.0));
  // A message with only the wheel joints, then one with only shoulder_pan_joint.
  footprint.joints(0.02, {"front_left_wheel_joint"}, {3.0});
  footprint.joints(0.04, {"shoulder_pan_joint"}, {0.15});
  const auto update = footprint.tick(0.05);
  ASSERT_TRUE(update);
  EXPECT_NEAR(half_width(update->footprint), 0.3 + 0.15 + 0.02, 1e-9);
}

TEST(DynamicFootprint, HoldsWhenAnArmJointIsMissing)
{
  auto footprint = make();
  footprint.joints(0.0, {"shoulder_pan_joint", "shoulder_lift_joint"}, {0.0, 0.0});
  EXPECT_FALSE(footprint.tick(0.05));
  EXPECT_EQ(footprint.health(0.05), mmn::DynamicFootprint::Health::MissingJoint);
}

TEST(DynamicFootprint, HoldsWhenStale)
{
  auto footprint = make();
  all_joints(footprint, 0.0, 0.2);
  ASSERT_TRUE(footprint.tick(0.0));
  const auto held = *footprint.current();
  // Only shoulder_pan_joint keeps arriving; the other arm joints go stale.
  footprint.joints(0.6, {"shoulder_pan_joint"}, {0.0});
  EXPECT_FALSE(footprint.tick(0.6));
  EXPECT_EQ(footprint.health(0.6), mmn::DynamicFootprint::Health::Stale);
  EXPECT_NEAR(mmg::hausdorff(*footprint.current(), held), 0.0, 1e-12);  // never shrinks without data
}

TEST(DynamicFootprint, FlagsHullBeyondInflationRadius)
{
  auto footprint = make(0.5);
  all_joints(footprint, 0.0, 0.3);  // half-width 0.6 m
  ASSERT_TRUE(footprint.tick(0.0));
  EXPECT_TRUE(footprint.last_stats().beyond_inflation);
  EXPECT_EQ(footprint.last_stats().points, 4u);
  EXPECT_EQ(footprint.last_stats().vertices, 4u);
  EXPECT_GE(footprint.last_stats().compute_s, 0.0);
}

TEST(FootprintRepair, RepublishesAfterTwoSecondsAtStandstill)
{
  mmn::FootprintDriftGuard guard(2.0);
  EXPECT_FALSE(guard.republish(true, 0.0));
  EXPECT_FALSE(guard.republish(false, 1.0));  // a switch in progress shows within a cycle
  EXPECT_FALSE(guard.republish(true, 2.0));
  EXPECT_FALSE(guard.republish(false, 3.0));
  EXPECT_FALSE(guard.republish(false, 4.0));
  EXPECT_TRUE(guard.republish(false, 5.0));
  EXPECT_FALSE(guard.republish(false, 6.0));  // give the costmap time to take it
  EXPECT_TRUE(guard.republish(false, 7.0));   // still wrong: again
  EXPECT_FALSE(guard.republish(true, 8.0));
  EXPECT_FALSE(guard.republish(false, 9.0));
}

TEST(DynamicFootprint, IgnoresNonFiniteJointValues)
{
  auto footprint = make();
  // A NaN from the first message never counts as received.
  footprint.joints(0.0, kArm, {std::nan(""), 0, 0, 0, 0, 0});
  EXPECT_FALSE(footprint.tick(0.0));
  EXPECT_EQ(footprint.health(0.0), mmn::DynamicFootprint::Health::MissingJoint);
  all_joints(footprint, 0.05, 0.1);
  ASSERT_TRUE(footprint.tick(0.05));
  const auto held = *footprint.current();
  // A later NaN or infinity neither replaces the last good value nor refreshes it.
  footprint.joints(0.10, {"shoulder_pan_joint"}, {std::nan("")});
  footprint.joints(0.15, {"shoulder_pan_joint"}, {INFINITY});
  EXPECT_FALSE(footprint.tick(0.15));
  EXPECT_NEAR(mmg::hausdorff(*footprint.current(), held), 0.0, 1e-12);
  EXPECT_EQ(footprint.health(0.6), mmn::DynamicFootprint::Health::Stale);
}

TEST(DynamicFootprint, InflationCheckIncludesNav2Padding)
{
  // Padded square corner at (0.32, 0.32): radius 0.4525 m; with Nav2's 0.01 m 0.4667 m.
  for (const auto & [nav2_padding, beyond] : {std::pair{0.0, false}, std::pair{0.01, true}}) {
    mmn::DynamicFootprint footprint(std::make_shared<SquareModel>(), kArm, {0.05}, 0.02, 0.01, 0.5, 0.46, nav2_padding);
    all_joints(footprint, 0.0, 0.0);
    ASSERT_TRUE(footprint.tick(0.0));
    EXPECT_EQ(footprint.last_stats().beyond_inflation, beyond) << nav2_padding;
  }
}

TEST(DynamicFootprint, MovingFollowsOdometryThresholdsAndHold)
{
  auto footprint = make();
  EXPECT_FALSE(footprint.moving(0.0));
  footprint.base_motion(1.0, 0.005, 0.01);  // below both thresholds
  EXPECT_FALSE(footprint.moving(1.0));
  footprint.base_motion(2.0, 0.01, 0.0);
  EXPECT_TRUE(footprint.moving(2.0));
  EXPECT_TRUE(footprint.moving(2.0 + mmn::DynamicFootprint::kMotionHoldS));
  EXPECT_FALSE(footprint.moving(2.0 + mmn::DynamicFootprint::kMotionHoldS + 0.01));
  footprint.base_motion(3.0, 0.0, -0.02);  // turning counts
  EXPECT_TRUE(footprint.moving(3.1));
  footprint.base_motion(3.2, 0.0, 0.0);  // a still sample does not end the hold early
  EXPECT_TRUE(footprint.moving(3.4));
}

TEST(DynamicFootprint, GrowOnlyIgnoresSwayWhileMoving)
{
  auto footprint = make_reach();
  // Published at the sway peak, then the panel swings +-0.019 rad (1 m lever: +-0.019 m, far
  // above the 0.01 m threshold between the extremes) while the base drives.
  reach(footprint, 0.0, 0.0, 0.0, 0.019);
  footprint.base_motion(0.0, 0.3, 0.0);
  ASSERT_TRUE(footprint.tick(0.0));
  for (int i = 1; i <= 40; ++i) {
    const double t = i * 0.05;
    reach(footprint, t, 0.0, 0.0, i % 2 ? -0.019 : 0.019);
    footprint.base_motion(t, 0.3, 0.0);
    ASSERT_TRUE(footprint.moving(t));
    EXPECT_FALSE(footprint.tick(t)) << i;
  }
}

TEST(DynamicFootprint, GrowsWhileMovingWhenHullExtends)
{
  auto footprint = make_reach();
  reach(footprint, 0.0, 0.0, 0.0, 0.0);
  footprint.base_motion(0.0, 0.3, 0.0);
  const auto first = footprint.tick(0.0);
  ASSERT_TRUE(first);
  // Within the 0.01 m allowance: held.
  reach(footprint, 0.05, 0.008, 0.0, 0.0);
  footprint.base_motion(0.05, 0.3, 0.0);
  EXPECT_FALSE(footprint.tick(0.05));
  // Forward reach past the allowance: one update.
  reach(footprint, 0.10, 0.03, 0.0, 0.0);
  footprint.base_motion(0.10, 0.3, 0.0);
  const auto forward = footprint.tick(0.10);
  ASSERT_TRUE(forward);
  EXPECT_TRUE(contains_all(forward->footprint, first->footprint));
  EXPECT_NEAR(max_x(forward->footprint), 0.3 + 0.03 + 0.02, 1e-9);
  // The arm now reaches sideways instead of forward: the update holds both extents.
  reach(footprint, 0.15, 0.0, 0.03, 0.0);
  footprint.base_motion(0.15, 0.3, 0.0);
  const auto sideways = footprint.tick(0.15);
  ASSERT_TRUE(sideways);
  const auto sideways_hull = mmg::offset_outward(mmg::convex_hull(ReachModel().points(
    {{"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", 0.03}, {"wrist_1_joint", 0.0}})), 0.02);
  EXPECT_TRUE(contains_all(sideways->footprint, forward->footprint));
  EXPECT_TRUE(contains_all(sideways->footprint, sideways_hull));
  ASSERT_EQ(sideways->zones.size(), 1u);
  EXPECT_NEAR(mmg::hausdorff(sideways->zones[0], mmg::offset_outward(sideways->footprint, 0.05)), 0.0, 1e-9);
  EXPECT_NEAR(mmg::hausdorff(*footprint.current(), sideways->footprint), 0.0, 1e-12);
}

TEST(DynamicFootprint, ShrinksAtStandstill)
{
  auto footprint = make();
  all_joints(footprint, 0.0, 0.1);
  footprint.base_motion(0.0, 0.3, 0.0);
  const auto grown = footprint.tick(0.0);
  ASSERT_TRUE(grown);
  // The arm retracts while the base still moves (the hold covers 0.5 s after the last motion).
  all_joints(footprint, 0.3, 0.0);
  EXPECT_FALSE(footprint.tick(0.3));
  EXPECT_NEAR(half_width(*footprint.current()), 0.3 + 0.1 + 0.02, 1e-9);
  // The hold has expired: the smaller hull follows by the Hausdorff rule.
  all_joints(footprint, 0.6, 0.0);
  const auto shrunk = footprint.tick(0.6);
  ASSERT_TRUE(shrunk);
  EXPECT_NEAR(half_width(shrunk->footprint), 0.3 + 0.02, 1e-9);
}

TEST(DynamicFootprint, NeverSmallerThanArmWhileMoving)
{
  auto footprint = make_reach();
  std::mt19937 random(7);
  std::uniform_real_distribution<double> sway(-0.04, 0.06);
  for (int i = 0; i < 400; ++i) {
    const double t = i * 0.05;
    const double pan = sway(random), lift = sway(random), wrist = 0.5 * sway(random);
    reach(footprint, t, pan, lift, wrist);
    footprint.base_motion(t, 0.3, 0.0);
    footprint.tick(t);
    ASSERT_TRUE(footprint.current());
    const auto arm = mmg::convex_hull(ReachModel().points(
      {{"shoulder_pan_joint", pan}, {"shoulder_lift_joint", lift}, {"wrist_1_joint", wrist}}));
    ASSERT_TRUE(contains_all(*footprint.current(), arm)) << i;
  }
}
