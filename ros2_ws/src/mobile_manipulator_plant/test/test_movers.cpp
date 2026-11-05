// The scenario mover's motion, a port of Unity's ScenarioMoverMotion (MuJoCo plant plan Task 4): the C# EditMode tests
// (ScenarioMoverMotionTests.cs) mirrored, plus the spec's validation ranges.
#include <gtest/gtest.h>

#include <stdexcept>

#include "mobile_manipulator_plant/movers.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
// Worker crossing the open band: (11, 2.5) -> (11, -2.5) at 0.8 m/s, trigger at 2.5 m.
mmp::MoverSpec worker_spec(int crossings = 1)
{
  mmp::MoverSpec s;
  s.name = "worker";
  s.start = {11.0, 2.5};
  s.end = {11.0, -2.5};
  s.size = {0.5, 0.5, 1.8};
  s.speed_mps = 0.8;
  s.trigger_distance_m = 2.5;
  s.crossings = crossings;
  return s;
}

void step(mmp::MoverMotion & motion, double t, double rx, double ry)
{
  const Eigen::Vector2d p = motion.next(t, {rx, ry});
  motion.commit(t, p);
}
}  // namespace

TEST(Movers, WaitsUntilTriggered)
{
  mmp::MoverMotion motion(worker_spec());
  for (int k = 0; k < 250; ++k) step(motion, 0.02 * k, 16.0 - 0.3 * 0.02 * k, 0.0);
  EXPECT_FALSE(motion.trigger_time().has_value());
  EXPECT_EQ(motion.position(), Eigen::Vector2d(11.0, 2.5));
}

TEST(Movers, WalksAtSpeedAfterTrigger)
{
  mmp::MoverMotion motion(worker_spec());
  step(motion, 10.0, 13.4, 0.0);  // triggers
  ASSERT_TRUE(motion.trigger_time().has_value());
  EXPECT_DOUBLE_EQ(*motion.trigger_time(), 10.0);
  step(motion, 12.5, 13.0, 0.0);  // 2.0 m walked
  EXPECT_NEAR(motion.position().y(), 0.5, 1e-9);
}

TEST(Movers, StaysAtLastEnd)
{
  mmp::MoverMotion motion(worker_spec());
  step(motion, 10.0, 13.4, 0.0);
  step(motion, 20.0, 12.0, 0.0);  // 8.0 m would exceed the 5.0 m crossing
  EXPECT_TRUE(motion.finished());
  EXPECT_NEAR(motion.position().y(), -2.5, 1e-9);
  EXPECT_NEAR(motion.walked(), 5.0, 1e-9);
}

TEST(Movers, AlternatesCrossings)
{
  const mmp::MoverMotion motion(worker_spec(2));
  EXPECT_NEAR(motion.position_at(7.5).y(), 0.0, 1e-9);  // halfway back
  EXPECT_NEAR(motion.position_at(10.0).y(), 2.5, 1e-9);
  EXPECT_NEAR(motion.position_at(99.0).y(), 2.5, 1e-9);
}

TEST(Movers, HeldTimeCountsAsBlockedNotWalking)
{
  mmp::MoverMotion motion(worker_spec());
  step(motion, 10.0, 13.0, 0.0);
  motion.next(11.0, {13.0, 0.0});
  motion.hold(11.0);  // robot in the way for 1 s
  EXPECT_NEAR(motion.blocked_s(), 1.0, 1e-9);
  EXPECT_EQ(motion.walked(), 0.0);
  step(motion, 11.5, 13.0, 0.0);
  EXPECT_NEAR(motion.walked(), 0.4, 1e-9);
}

TEST(Movers, RejectsInvalidSpecs)
{
  auto with = [](auto change) {
    mmp::MoverSpec s = worker_spec();
    change(s);
    return s;
  };
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.end = {11.0, 2.45}; })), std::out_of_range);  // < 0.1 m
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.speed_mps = 0.0; })), std::out_of_range);
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.speed_mps = 2.1; })), std::out_of_range);
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.trigger_distance_m = 0.0; })), std::out_of_range);
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.trigger_distance_m = 21.0; })), std::out_of_range);
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.crossings = 0; })), std::out_of_range);
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.crossings = 11; })), std::out_of_range);
  EXPECT_THROW(mmp::MoverMotion(with([](auto & s) { s.start.x() = std::nan(""); })), std::out_of_range);
}
