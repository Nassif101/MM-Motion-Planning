// The wall timer's schedule as rcl keeps it (Phase 4B-B Task 1 review): a late tick does not shift the phase.
#include <gtest/gtest.h>

#include "mobile_manipulator_mpc/timer_schedule.hpp"

using mobile_manipulator_mpc::TimerSchedule;

TEST(TimerSchedule, OnTimeTicksAreNotLate)
{
  TimerSchedule s(0.0, 0.02);
  EXPECT_NEAR(s.tick(0.02), 0.0, 1e-12);
  EXPECT_NEAR(s.tick(0.0401), 0.0001, 1e-12);
  EXPECT_NEAR(s.tick(0.06), 0.0, 1e-12);
}

TEST(TimerSchedule, LateTickKeepsThePhase)
{
  TimerSchedule s(0.0, 0.02);
  s.tick(0.02);
  EXPECT_NEAR(s.tick(0.046), 0.006, 1e-12);
  EXPECT_NEAR(s.tick(0.0605), 0.0005, 1e-12) << "due at 0.06, not 0.066";
  // A stall past whole periods: rcl skips the missed ticks and keeps the grid.
  EXPECT_NEAR(s.tick(0.1060), 0.026, 1e-12) << "due at 0.08";
  EXPECT_NEAR(s.tick(0.1203), 0.0003, 1e-12) << "next due at 0.12 (0.10 was skipped)";
}

TEST(TimerSchedule, FirstTickLateMeasuresFromCreation)
{
  TimerSchedule s(1.0, 0.02);
  EXPECT_NEAR(s.tick(1.05), 0.03, 1e-12);
  EXPECT_NEAR(s.tick(1.06), 0.0, 1e-12);
}
