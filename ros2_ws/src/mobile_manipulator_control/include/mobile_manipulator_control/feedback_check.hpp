#pragma once
#include <cmath>

namespace mobile_manipulator_control
{
// Stale feedback is a delivery pause: the hardware deactivates but may be re-activated once
// fresh feedback returns. A timestamp regression means a new simulation epoch and stays
// latched until arm control restarts (ADR 0005).
enum class Feedback { Fresh, Stale };

// age_s: monotonic time since the last accepted state; state_stamp_s: its simulation stamp;
// clock_s: the controller manager's simulation time.
inline Feedback check_feedback(double age_s, double state_stamp_s, double clock_s, double timeout_s)
{
  if (!std::isfinite(age_s) || age_s >= timeout_s) return Feedback::Stale;
  if (std::abs(clock_s - state_stamp_s) > timeout_s) return Feedback::Stale;
  return Feedback::Fresh;
}

inline bool stamp_regressed(double previous_stamp_s, double stamp_s)
{
  return previous_stamp_s >= 0 && stamp_s < previous_stamp_s;
}
}
