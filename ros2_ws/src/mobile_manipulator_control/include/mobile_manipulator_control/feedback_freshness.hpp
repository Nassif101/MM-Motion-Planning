#pragma once
// Freshness of Unity's arm feedback for the recovery supervisor (no ROS dependencies).
//
// Feedback is fresh once states with advancing stamps have arrived without a gap longer
// than `max_gap_s` for at least `fresh_for_s`. A repeated or regressing stamp, or a gap,
// restarts the window. Times are wall-clock seconds so a /clock pause cannot stop recovery.
#include <optional>

namespace mobile_manipulator_control
{
class FeedbackFreshness
{
public:
  explicit FeedbackFreshness(double fresh_for_s, double max_gap_s = 0.2)
  : fresh_for_s_(fresh_for_s), max_gap_s_(max_gap_s) {}

  void on_state(double now, double stamp)
  {
    if (stamp <= last_stamp_ || (last_state_ && now - *last_state_ > max_gap_s_)) {
      fresh_since_.reset();  // regression, repeat, or a gap
    }
    if (stamp > last_stamp_) {
      if (!fresh_since_) fresh_since_ = now;
      last_stamp_ = stamp;
    }
    last_state_ = now;
  }

  bool fresh(double now) const
  {
    return last_state_ && now - *last_state_ < max_gap_s_ && fresh_since_ &&
           now - *fresh_since_ >= fresh_for_s_;
  }

private:
  double fresh_for_s_, max_gap_s_;
  std::optional<double> last_state_, fresh_since_;
  double last_stamp_ = -1.0;
};
}  // namespace mobile_manipulator_control
