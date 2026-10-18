// The executed base command (the collision monitor's /cmd_vel) as B5's stage-0 previous command
// (Phase 4B-A spec Section 4), and the base braking ramp shared by holds and goal stops.
#include <gtest/gtest.h>

#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/executed_command.hpp"
#include "test_support.hpp"

using namespace mpc_test;
using Eigen::Vector2d;

TEST(ExecutedCommand, PassThroughUsesTheLatestSent)
{
  ExecutedCommand e;
  e.on_sent(0.00, {0.10, 0.0});
  e.on_sent(0.02, {0.12, 0.0});
  e.on_executed(0.021, {0.10, 0.0});  // the monitor forwarded the older one
  EXPECT_TRUE(e.previous(0.03, 0.04).isApprox(Vector2d(0.12, 0.0)));
}

TEST(ExecutedCommand, ScaledCommandIsUsedAsIs)
{
  // FootprintApproach scaled 0.12 -> 0.06.
  ExecutedCommand e;
  e.on_sent(0.0, {0.12, 0.0});
  e.on_executed(0.001, {0.06, 0.0});
  EXPECT_TRUE(e.previous(0.01, 0.04).isApprox(Vector2d(0.06, 0.0)));
}

TEST(ExecutedCommand, OnlyTheLastFiveSentCountAsPassThrough)
{
  ExecutedCommand e;
  e.on_sent(0.00, {0.05, 0.0});
  for (int i = 1; i <= 5; ++i) e.on_sent(0.02 * i, {0.05 + 0.01 * i, 0.0});
  e.on_executed(0.11, {0.05, 0.0});  // equals the sixth newest: not a pass-through any more
  EXPECT_TRUE(e.previous(0.12, 0.04).isApprox(Vector2d(0.05, 0.0)));
}

TEST(ExecutedCommand, SilenceIsZero)
{
  ExecutedCommand e;
  e.on_sent(0.0, {0.3, 0.2});
  e.on_executed(0.0, {0.3, 0.2});
  EXPECT_TRUE(e.executed(0.05, 0.04).isZero());
  EXPECT_TRUE(e.previous(0.05, 0.04).isZero());
}

TEST(ExecutedCommand, NeverReceivedIsZero)
{
  ExecutedCommand e;
  e.on_sent(0.0, {0.3, 0.2});
  EXPECT_TRUE(e.previous(1.0, 0.04).isZero());
  EXPECT_TRUE(e.executed(1.0, 0.04).isZero());
}

TEST(ExecutedCommand, ExecutedIsTheRawLatest)
{
  ExecutedCommand e;
  e.on_sent(0.0, {0.12, 0.1});
  e.on_executed(0.001, {0.12, 0.1});
  EXPECT_TRUE(e.executed(0.01, 0.04).isApprox(Vector2d(0.12, 0.1)));
}

TEST(BrakeBase, RampsAtTheEnvelopeDecel)
{
  const Limits & l = limits();
  const Vector2d next = brake_base({0.3, 0.4}, 0.1, l, l.breakaway);
  EXPECT_NEAR(next(0), 0.3 - l.decel * 0.1, 1e-12);
  EXPECT_NEAR(next(1), 0.4 - l.yaw_decel * 0.1, 1e-12);
  const Vector2d reverse = brake_base({0.3, -0.4}, 10.0, l, l.breakaway);
  EXPECT_TRUE(reverse.isZero());  // never past zero
  const Vector2d negative = brake_base({0.0, -0.4}, 0.1, l, l.breakaway);
  EXPECT_NEAR(negative(1), -0.4 + l.yaw_decel * 0.1, 1e-12);
}

TEST(ExecutedCommand, NodeStaleTimeIsAboveTheMonitorJitter)
{
  // /cmd_vel inter-arrival p99 was 31-51 ms with gaps beyond 40 ms in every live run (Task 3 review): an
  // echo 100 ms old is still the executed command. Only a monitor that died mid-command (it publishes
  // zeros before going silent after a stop) leaves a non-zero command older than kExecutedStaleS.
  EXPECT_GE(kExecutedStaleS, 0.25);
  ExecutedCommand e;
  e.on_sent(0.0, {0.3, 0.0});
  e.on_executed(0.001, {0.3, 0.0});
  EXPECT_TRUE(e.previous(0.1, kExecutedStaleS).isApprox(Vector2d(0.3, 0.0)));
  EXPECT_TRUE(e.previous(0.001 + kExecutedStaleS + 0.01, kExecutedStaleS).isZero());
}
