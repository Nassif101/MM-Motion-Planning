#include <gtest/gtest.h>
#include "mobile_manipulator_control/feedback_freshness.hpp"

using mobile_manipulator_control::FeedbackFreshness;

namespace
{
// States at 50 Hz with advancing stamps from `start` for `seconds`; returns the end time.
double stream(FeedbackFreshness & freshness, double start, double seconds, double & stamp)
{
  double now = start;
  for (; now <= start + seconds + 1e-9; now += 0.02) freshness.on_state(now, stamp += 0.02);
  return now - 0.02;
}
}  // namespace

TEST(FeedbackFreshness, FreshAfterTheWindowOfAdvancingStates)
{
  FeedbackFreshness freshness(1.0);
  double stamp = 10.0;
  EXPECT_FALSE(freshness.fresh(0.0));
  const double half = stream(freshness, 0.0, 0.5, stamp);
  EXPECT_FALSE(freshness.fresh(half));
  const double full = stream(freshness, half + 0.02, 0.6, stamp);
  EXPECT_TRUE(freshness.fresh(full));
  EXPECT_FALSE(freshness.fresh(full + 0.25));  // the stream stopped
}

TEST(FeedbackFreshness, GapRestartsTheWindow)
{
  FeedbackFreshness freshness(1.0);
  double stamp = 0.0;
  const double end = stream(freshness, 0.0, 1.2, stamp);
  ASSERT_TRUE(freshness.fresh(end));
  const double resumed = stream(freshness, end + 0.5, 0.5, stamp);  // 0.5 s gap
  EXPECT_FALSE(freshness.fresh(resumed));
  EXPECT_TRUE(freshness.fresh(stream(freshness, resumed + 0.02, 0.6, stamp)));
}

TEST(FeedbackFreshness, RepeatedOrRegressingStampsRestartTheWindow)
{
  FeedbackFreshness freshness(1.0);
  double stamp = 5.0;
  double now = stream(freshness, 0.0, 1.2, stamp);
  ASSERT_TRUE(freshness.fresh(now));
  freshness.on_state(now += 0.02, stamp);  // repeated stamp
  EXPECT_FALSE(freshness.fresh(now));
  freshness.on_state(now += 0.02, 0.1);  // new epoch: stamps regress
  EXPECT_FALSE(freshness.fresh(now));
  for (int i = 0; i < 60; ++i) freshness.on_state(now += 0.02, 0.2 + i * 0.02);
  EXPECT_FALSE(freshness.fresh(now));  // stamps below the old maximum never count
}
