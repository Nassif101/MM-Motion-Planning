#pragma once
// The node's wall timer schedule as rcl keeps it (Phase 4B-B MpcCycle timer_lateness_s): ticks are due every period
// from the timer's creation; a late tick does not move the grid, and ticks a stall skipped are skipped (rcl's next
// call time is the first grid point after the late call).
#include <cmath>

namespace mobile_manipulator_mpc
{
class TimerSchedule
{
public:
  // A timer created at created_s (seconds, the tick times' clock) with period_s.
  TimerSchedule(double created_s, double period_s) : next_(created_s + period_s), period_(period_s) {}
  // A tick at now_s: its lateness after its due time (0 when early), then the next due time.
  double tick(double now_s)
  {
    const double late = now_s - next_;
    if (late >= period_) next_ += std::floor(late / period_) * period_;
    next_ += period_;
    return late > 0.0 ? late : 0.0;
  }

private:
  double next_, period_;
};
}  // namespace mobile_manipulator_mpc
