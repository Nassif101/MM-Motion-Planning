// The blocked detector (Phase 4B-A spec Section 2): B5 commanding while the collision monitor passes
// nothing holds, and fails FollowPath after 3 s of blocked time without base motion.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <tuple>

#include "mobile_manipulator_mpc/blocked_detector.hpp"

using namespace mobile_manipulator_mpc;
using Eigen::Vector2d;
using State = BlockedDetector::State;

namespace
{
constexpr double kDt = 0.02;
const Vector2d kMoving(0.3, 0.0), kZero(0.0, 0.0), kHere(1.0, 2.0);

// Steps the detector at 50 Hz from t0 for `seconds` with the given inputs; returns the last state and
// the time of the first change of state (NaN if none).
struct Outcome
{
  State last;
  double changed_at;
};

Outcome run(BlockedDetector & d, double t0, double seconds, const Vector2d & own, const Vector2d & executed,
        bool monitor_stop, const Vector2d & xy = kHere)
{
  Outcome r{d.state(), std::numeric_limits<double>::quiet_NaN()};
  const State start = d.state();
  for (double t = t0; t < t0 + seconds - 1e-9; t += kDt) {
    r.last = d.update(t, own, executed, monitor_stop, xy);
    if (std::isnan(r.changed_at) && r.last != start) r.changed_at = t;
  }
  return r;
}
}  // namespace

TEST(BlockedDetector, DetectsAfterOneSecond)
{
  BlockedDetector d;
  const Outcome r = run(d, 0.0, 1.2, kMoving, kZero, true);
  EXPECT_EQ(r.last, State::Blocked);
  EXPECT_NEAR(r.changed_at, 1.0, kDt + 1e-9);
}

TEST(BlockedDetector, ScaledIsNotBlocked)
{
  BlockedDetector d;  // FootprintApproach scaled 0.3 -> 0.06: still moving
  EXPECT_EQ(run(d, 0.0, 3.0, kMoving, Vector2d(0.06, 0.0), false).last, State::Clear);
}

TEST(BlockedDetector, NotCommandingIsNotBlocked)
{
  BlockedDetector d;
  EXPECT_EQ(run(d, 0.0, 3.0, Vector2d(0.03, 0.05), kZero, true).last, State::Clear);
}

TEST(BlockedDetector, FirstMessageWithinDetectTimeNeverBlocks)
{
  BlockedDetector d;  // no executed command yet (zero) for 0.5 s, then the monitor follows
  EXPECT_EQ(run(d, 0.0, 0.5, kMoving, kZero, false).last, State::Clear);
  EXPECT_EQ(run(d, 0.5, 3.0, kMoving, kMoving, false).last, State::Clear);
}

TEST(BlockedDetector, ClearsAfterMonitorNotStopForClearS)
{
  BlockedDetector d;
  ASSERT_EQ(run(d, 0.0, 1.5, kMoving, kZero, true).last, State::Blocked);
  // The MPC holds (zero command); the monitor's STOP ends at 1.5 s.
  const Outcome r = run(d, 1.5, 1.0, kZero, kZero, false);
  EXPECT_EQ(r.last, State::Clear);
  EXPECT_NEAR(r.changed_at, 2.0, kDt + 1e-9);
}

TEST(BlockedDetector, StaysBlockedWhileTheMonitorStops)
{
  BlockedDetector d;
  run(d, 0.0, 1.5, kMoving, kZero, true);
  EXPECT_EQ(run(d, 1.5, 1.0, kZero, kZero, true).last, State::Blocked);
}

TEST(BlockedDetector, AccumulatesAcrossHoldsAndFailsAtThreeSeconds)
{
  // Each round (2.7 s): commanding 1.1 s into a STOP (blocked from 1.0 s), holding 1.0 s, the STOP ends and
  // the hold clears 0.5 s later (the round's last 0.1 s is clear): 1.6 s blocked per round, no base motion.
  // 3.0 s are reached 1.4 s into round 2's block (round 2 starts at 2.7 s, blocks from 3.7 s).
  BlockedDetector d;
  double t = 0.0;
  double failed_at = std::numeric_limits<double>::quiet_NaN();
  for (int round = 0; round < 3 && d.state() != State::Failed; ++round) {
    for (const auto & [seconds, own, stop] :
         {std::tuple{1.1, kMoving, true}, std::tuple{1.0, kZero, true}, std::tuple{0.6, kZero, false}}) {
      const Outcome r = run(d, t, seconds, own, kZero, stop);
      if (r.last == State::Failed && std::isnan(failed_at)) failed_at = r.changed_at;
      t += seconds;
      if (r.last == State::Failed) break;
    }
  }
  EXPECT_EQ(d.state(), State::Failed);
  EXPECT_NEAR(failed_at, 2.7 + 1.0 + 1.4, 2 * kDt + 1e-9);
  EXPECT_NEAR(d.blocked_time(), 3.0, kDt + 1e-9);
}

TEST(BlockedDetector, BaseMotionRestartsTheAccount)
{
  BlockedDetector d;
  run(d, 0.0, 3.0, kMoving, kZero, true);  // 2 s blocked
  run(d, 3.0, 0.5, kZero, kZero, false);   // cleared
  // The base moved 0.06 m before the next block: the account starts again.
  const Vector2d moved = kHere + Vector2d(0.06, 0.0);
  run(d, 3.5, 0.2, kMoving, kMoving, false, moved);
  EXPECT_EQ(run(d, 3.7, 3.0, kMoving, kZero, true, moved).last, State::Blocked);
  EXPECT_LT(d.blocked_time(), 2.1);
}

TEST(BlockedDetector, FailedLatchesUntilRestart)
{
  BlockedDetector d;
  run(d, 0.0, 4.5, kMoving, kZero, true);
  ASSERT_EQ(d.state(), State::Failed);
  EXPECT_EQ(run(d, 4.5, 1.0, kMoving, kMoving, false).last, State::Failed);
  d.restart();
  EXPECT_EQ(d.state(), State::Clear);
  EXPECT_DOUBLE_EQ(d.blocked_time(), 0.0);
}

TEST(BlockedDetector, TheBreakawayCountsAsCommanding)
{
  // With the executed-command feedback a blocked MPC starting from rest sends exactly the breakaway.
  BlockedDetector d;
  const BlockedConfig & c = d.config();
  EXPECT_EQ(run(d, 0.0, 1.2, Vector2d(c.own_v, 0.0), kZero, true).last, State::Blocked);
  BlockedDetector turning;
  EXPECT_EQ(run(turning, 0.0, 1.2, Vector2d(0.0, -c.own_w), kZero, true).last, State::Blocked);
}
