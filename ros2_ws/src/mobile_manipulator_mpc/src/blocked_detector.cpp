#include "mobile_manipulator_mpc/blocked_detector.hpp"

#include <cmath>

namespace mobile_manipulator_mpc
{
namespace
{
constexpr double kEps = 1e-9;
}

BlockedDetector::BlockedDetector(BlockedConfig config) : config_(config) {}

void BlockedDetector::restart()
{
  state_ = State::Clear;
  since_ = free_since_ = last_ = std::numeric_limits<double>::quiet_NaN();
  blocked_ = 0.0;
  anchored_ = false;
}

BlockedDetector::State BlockedDetector::update(double now, const Eigen::Vector2d & own, const Eigen::Vector2d & executed,
                                               bool monitor_stop, const Eigen::Vector2d & base_xy, bool base_at_rest)
{
  const double step = std::isnan(last_) ? 0.0 : std::max(0.0, now - last_);
  last_ = now;
  if (!anchored_ || (base_xy - anchor_).norm() >= config_.progress_m) {
    anchor_ = base_xy;  // the base moved: the blocked time starts again
    anchored_ = true;
    if (state_ != State::Failed) blocked_ = 0.0;
  }
  if (state_ == State::Failed) return state_;

  if (state_ == State::Blocked) {
    blocked_ += step;
    if (blocked_ >= config_.fail_s - kEps) {
      state_ = State::Failed;
      return state_;
    }
    if (monitor_stop) {
      free_since_ = std::numeric_limits<double>::quiet_NaN();
    } else if (std::isnan(free_since_)) {
      free_since_ = now;
    }
    if (!std::isnan(free_since_) && now - free_since_ >= config_.clear_s - kEps) {
      state_ = State::Clear;
      since_ = free_since_ = std::numeric_limits<double>::quiet_NaN();
    }
    return state_;
  }

  const bool commanding = std::abs(own(0)) >= config_.own_v - kEps || std::abs(own(1)) >= config_.own_w - kEps;
  const bool stopped =
    base_at_rest && std::abs(executed(0)) < config_.own_v - kEps && std::abs(executed(1)) < config_.own_w - kEps;
  if (!(commanding && stopped)) {
    since_ = std::numeric_limits<double>::quiet_NaN();
    return state_;
  }
  if (std::isnan(since_)) since_ = now;
  if (now - since_ >= config_.detect_s - kEps) {
    state_ = State::Blocked;
    free_since_ = std::numeric_limits<double>::quiet_NaN();
    blocked_ += now - since_;  // the detection window counts
    if (blocked_ >= config_.fail_s - kEps) state_ = State::Failed;
  }
  return state_;
}
}  // namespace mobile_manipulator_mpc
