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

mmn::DynamicFootprint make(double inflation_radius_m = 1.0)
{
  return mmn::DynamicFootprint(std::make_shared<SquareModel>(), kArm, {0.05, 0.25}, 0.02, 0.01, 0.5, inflation_radius_m);
}

void all_joints(mmn::DynamicFootprint & footprint, double t, double pan)
{
  footprint.joints(t, kArm, {pan, 0, 0, 0, 0, 0});
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
