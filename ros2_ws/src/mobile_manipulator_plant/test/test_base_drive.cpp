// The realistic base drive (MuJoCo plant plan Task 6): Unity's SkidSteerBaseController limiter (clamp, chassis rate
// limits, braking before a reversal, the 0.5 s watchdog, common wheel-speed saturation), then optional breakaway, a
// first-order lag and slip; and the time-based delay queue.
#include <gtest/gtest.h>

#include "mobile_manipulator_plant/base_drive.hpp"
#include "mobile_manipulator_plant/delay_queue.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
mmp::BaseDriveParams unity()
{
  mmp::BaseDriveParams p;
  p.limits = {0.8, 0.5, 0.8, 0.5, 0.8, 0.8, 1.2, 0.5};
  p.wheel_radius_m = 0.14;
  p.effective_track_m = 1.5;
  p.wheel_speed_max = 8.0;
  return p;
}

// `n` steps of 2 ms with the command (v, w) stamped now; the drive's last target.
Eigen::Vector2d run(mmp::BaseDrive & drive, double v, double w, int n, double age = 0.0)
{
  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  for (int k = 0; k < n; ++k) out = drive.target(Eigen::Vector2d(v, w), age, 0.002);
  return out;
}
}  // namespace

TEST(DelayQueue, ReturnsTheLatestDueValue)
{
  mmp::DelayQueue<int> q(0.04);
  EXPECT_FALSE(q.at(0.0).has_value());
  q.push(0.0, 1);
  q.push(0.02, 2);
  EXPECT_FALSE(q.at(0.039).has_value());
  EXPECT_EQ(*q.at(0.04), 1);
  EXPECT_EQ(*q.at(0.059), 1);
  EXPECT_EQ(*q.at(0.06), 2);
  EXPECT_EQ(*q.at(1.0), 2);  // the last value holds
}

TEST(DelayQueue, ZeroDelayIsImmediate)
{
  mmp::DelayQueue<int> q(0.0);
  q.push(0.5, 7);
  EXPECT_EQ(*q.at(0.5), 7);
}

TEST(BaseDrive, ClampsSpeeds)
{
  mmp::BaseDrive d(unity());
  EXPECT_NEAR(run(d, 2.0, 0.0, 2000)(0), 0.8, 1e-12);
  mmp::BaseDrive r(unity());
  EXPECT_NEAR(run(r, -2.0, 0.0, 2000)(0), -0.5, 1e-12);
  mmp::BaseDrive y(unity());
  EXPECT_NEAR(run(y, 0.0, 3.0, 2000)(1), 0.8, 1e-12);
}

TEST(BaseDrive, RateLimitsLikeUnity)
{
  mmp::BaseDrive d(unity());
  EXPECT_NEAR(run(d, 0.3, 0.0, 300)(0), 0.3, 1e-9);  // 0.6 s at 0.5 m/s^2
  mmp::BaseDrive e(unity());
  EXPECT_NEAR(run(e, 0.3, 0.0, 250)(0), 0.25, 1e-9);  // 0.5 s: not yet
  EXPECT_NEAR(run(e, 0.0, 0.0, 100)(0), 0.25 - 0.8 * 0.2, 1e-9);  // braking at 0.8 m/s^2
}

TEST(BaseDrive, BrakesBeforeReversal)
{
  mmp::BaseDrive d(unity());
  run(d, 0.3, 0.0, 600);
  // to -0.3: brake at the deceleration limit to zero, then accelerate the other way
  const Eigen::Vector2d after = run(d, -0.3, 0.0, 150);  // 0.3 s: 0.3 - 0.8 * 0.3 = 0.06 m/s, still forward
  EXPECT_NEAR(after(0), 0.06, 1e-9);
}

TEST(BaseDrive, StaleCommandRampsToZero)
{
  mmp::BaseDrive d(unity());
  run(d, 0.3, 0.0, 600);
  EXPECT_NEAR(run(d, 0.3, 0.0, 50, 0.6)(0), 0.3 - 0.8 * 0.1, 1e-9);  // 0.6 s old: treated as zero
}

TEST(BaseDrive, WheelSaturationKeepsCurvature)
{
  mmp::BaseDrive d(unity());
  const Eigen::Vector2d out = run(d, 0.8, 0.8, 3000);
  // wheel speeds (0.8 -+ 0.8 * 0.75) / 0.14 = (1.43, 10.0) rad/s: a common 0.8 scale
  EXPECT_NEAR(out(0), 0.64, 1e-9);
  EXPECT_NEAR(out(1), 0.64, 1e-9);
}

TEST(BaseDrive, BreakawayFromRestWhenOn)
{
  auto p = unity();
  p.breakaway = true;
  p.breakaway_linear = 0.0375;
  p.breakaway_yaw = 0.07;
  p.rest_linear = 0.01;
  p.rest_yaw = 0.02;
  mmp::BaseDrive d(p);
  EXPECT_NEAR(run(d, 0.03, 0.0, 500)(0), 0.0, 1e-12);
  EXPECT_GT(run(d, 0.05, 0.0, 500)(0), 0.04);
  mmp::BaseDrive off(unity());  // Unity's friction feedforward: every command starts
  EXPECT_NEAR(run(off, 0.03, 0.0, 500)(0), 0.03, 1e-9);
}

TEST(BaseDrive, LagReachesSixtyThreePercentAtTau)
{
  auto p = unity();
  p.tau_v = 0.1;
  p.limits.accel = 100.0;  // a near step, so the lag alone shapes the response
  mmp::BaseDrive d(p);
  EXPECT_NEAR(run(d, 0.3, 0.0, 50)(0), 0.3 * (1.0 - std::exp(-1.0)), 0.005);  // the 3 ms ramp shifts it slightly
}

TEST(BaseDrive, SlipScalesTheAchievedTwist)
{
  auto p = unity();
  p.k_v = 0.9;
  p.k_w = 0.8;
  mmp::BaseDrive d(p);
  const Eigen::Vector2d out = run(d, 0.3, 0.4, 2000);
  EXPECT_NEAR(out(0), 0.27, 1e-9);
  EXPECT_NEAR(out(1), 0.32, 1e-9);
}
