#include <gtest/gtest.h>
#include <limits>
#include "mobile_manipulator_control/feedback_check.hpp"
using mobile_manipulator_control::Feedback;
using mobile_manipulator_control::check_feedback;
using mobile_manipulator_control::stamp_regressed;
TEST(Feedback, FreshWithinTimeoutAndClockWindow) {
  EXPECT_EQ(check_feedback(0.02, 10.0, 10.02, 0.5), Feedback::Fresh);
}
TEST(Feedback, OldOrMissingStateIsStale) {
  EXPECT_EQ(check_feedback(0.5, 10.0, 10.0, 0.5), Feedback::Stale);
  EXPECT_EQ(check_feedback(std::numeric_limits<double>::infinity(), 10.0, 10.0, 0.5), Feedback::Stale);
}
TEST(Feedback, ClockPauseEitherWayIsStaleNotEpoch) {
  EXPECT_EQ(check_feedback(0.01, 10.0, 10.6, 0.5), Feedback::Stale);  // state behind /clock
  EXPECT_EQ(check_feedback(0.01, 10.6, 10.0, 0.5), Feedback::Stale);  // /clock paused behind state
}
TEST(Feedback, OnlyStampRegressionIsAnEpochChange) {
  EXPECT_FALSE(stamp_regressed(-1, 0.0));
  EXPECT_FALSE(stamp_regressed(10.0, 10.02));
  EXPECT_TRUE(stamp_regressed(10.0, 0.02));
}
