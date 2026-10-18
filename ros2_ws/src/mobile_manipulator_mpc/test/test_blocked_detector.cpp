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
            bool monitor_stop, const Vector2d & xy = kHere, bool at_rest = true)
{
  Outcome r{d.state(), std::numeric_limits<double>::quiet_NaN()};
  const State start = d.state();
  for (double t = t0; t < t0 + seconds - 1e-9; t += kDt) {
    r.last = d.update(t, own, executed, monitor_stop, xy, at_rest);
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
  EXPECT_EQ(run(d, 0.0, 3.0, kMoving, Vector2d(0.06, 0.0), false, kHere, false).last, State::Clear);
}

TEST(BlockedDetector, SubBreakawayExecutedWithTheBaseAtRestIsBlocked)
{
  // FootprintApproach scales the breakaway command to 0.02 m/s: below the breakaway the base does not start
  // (Task 5 review): with the base measured at rest it is blocked.
  BlockedDetector d;
  const BlockedConfig & c = d.config();
  EXPECT_EQ(run(d, 0.0, 1.2, Vector2d(c.own_v, 0.0), Vector2d(0.02, 0.0), false).last, State::Blocked);
  BlockedDetector turning;  // a 0.07 rad/s turn scaled to 0.04 rad/s
  EXPECT_EQ(run(turning, 0.0, 1.2, Vector2d(0.0, c.own_w), Vector2d(0.0, 0.04), false).last, State::Blocked);
  BlockedDetector moving;  // the same command while the base is measured moving: not blocked
  EXPECT_EQ(run(moving, 0.0, 1.2, Vector2d(c.own_v, 0.0), Vector2d(0.02, 0.0), false, kHere, false).last,
            State::Clear);
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
  EXPECT_EQ(run(d, 0.5, 3.0, kMoving, kMoving, false, kHere, false).last, State::Clear);
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
  // Blocked time counts from the first commanding cycle without execution (the detection second counts, Task 5
  // review: a block that is not a STOP clears 0.5 s after the hold and would otherwise count 0.5 s per round).
  // Each round (2.7 s): commanding 1.1 s, holding 1.0 s, the STOP ends and the hold clears 0.5 s later, the
  // round's last 0.1 s clear: 2.6 s per round. Round 2 starts at 2.7 s; its detection at 3.7 s brings 1.0 s
  // more: 3.6 s >= 3.0, Failed on entering Blocked.
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
  EXPECT_NEAR(failed_at, 3.7, 2 * kDt + 1e-9);
}

TEST(BlockedDetector, ApproachBlocksCountTheirDetection)
{
  // A FootprintApproach block (never STOP): detect 1 s, held, cleared 0.5 s later, re-detected...: each round
  // counts 1.5 s, so the goal fails in the second round, not after six.
  BlockedDetector d;
  double t = 0.0;
  for (int round = 0; round < 6 && d.state() != State::Failed; ++round) {
    run(d, t, 1.05, kMoving, kZero, false);
    run(d, t + 1.05, 0.6, kZero, kZero, false);
    t += 1.65;
  }
  EXPECT_EQ(d.state(), State::Failed);
  EXPECT_LT(t, 3.5);
}

TEST(BlockedDetector, BaseMotionRestartsTheAccount)
{
  BlockedDetector d;
  run(d, 0.0, 2.0, kMoving, kZero, true);  // 2 s blocked (with the detection second)
  run(d, 2.0, 0.5, kZero, kZero, false);   // cleared
  // The base moved 0.06 m before the next block: the account starts again.
  const Vector2d moved = kHere + Vector2d(0.06, 0.0);
  run(d, 2.5, 0.2, kMoving, kMoving, false, moved, false);
  EXPECT_EQ(run(d, 2.7, 2.0, kMoving, kZero, true, moved).last, State::Blocked);
  EXPECT_LT(d.blocked_time(), 2.1);
}

TEST(BlockedDetector, FailedLatchesUntilRestart)
{
  BlockedDetector d;
  run(d, 0.0, 3.5, kMoving, kZero, true);
  ASSERT_EQ(d.state(), State::Failed);
  EXPECT_EQ(run(d, 3.5, 1.0, kMoving, kMoving, false, kHere, false).last, State::Failed);
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

TEST(BlockedDetector, BriefDipsInTheCommandDoNotRestartDetection)
{
  // Live (v2_gate_right_swing_nav, Task 9): from rest with the executed feedback at zero every fifth MPC command
  // fell to zero or below the breakaway; detection must not restart on such dips (up to kCommandGapS).
  BlockedDetector d;
  const Vector2d breakaway(d.config().own_v, 0.0);
  State last = State::Clear;
  double blocked_at = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 75; ++i) {
    const double t = i * kDt;
    last = d.update(t, i % 5 == 4 ? kZero : breakaway, kZero, true, kHere, true);
    if (last == State::Blocked && std::isnan(blocked_at)) blocked_at = t;
  }
  EXPECT_EQ(last, State::Blocked);
  EXPECT_NEAR(blocked_at, 1.0, 2 * kDt + 1e-9);
}

TEST(BlockedDetector, AStopOfTheCommandEndsDetection)
{
  BlockedDetector d;  // the MPC stops commanding for longer than the gap: not blocked
  run(d, 0.0, 0.6, kMoving, kZero, true);
  run(d, 0.6, 0.5, kZero, kZero, true);
  EXPECT_EQ(run(d, 1.1, 0.6, kMoving, kZero, true).last, State::Clear);
}
