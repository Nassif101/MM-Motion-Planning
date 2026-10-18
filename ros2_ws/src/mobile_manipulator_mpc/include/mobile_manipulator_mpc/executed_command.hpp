#pragma once
// The base command the robot was actually told (Phase 4B-A spec Sections 2 and 4): the collision
// monitor is the only /cmd_vel publisher and may scale or zero what the MPC sent. B5 uses it as its
// stage-0 previous base command, and the blocked detector compares it with the MPC's own command.
#include <array>
#include <cstddef>
#include <limits>

#include <Eigen/Core>

namespace mobile_manipulator_mpc
{
class ExecutedCommand
{
public:
  static constexpr std::size_t kSentKept = 5;     // the last commands the MPC sent, for the pass-through test
  static constexpr double kPassThroughTol = 1e-3;  // per component

  // A command the MPC sent (base v, omega) at time t (seconds, one clock for both inputs).
  void on_sent(double t, const Eigen::Vector2d & vw);
  // A /cmd_vel message received at time t.
  void on_executed(double t, const Eigen::Vector2d & vw);

  // The latest executed command, or zero when none arrived within max_age_s before now (the
  // monitor stops publishing stop_pub_timeout after a stop, so silence means stopped).
  Eigen::Vector2d executed(double now, double max_age_s) const;
  // The stage-0 previous base command: the latest sent command when the executed one equals one of
  // the last kSentKept sent commands (the monitor passed it through; its output lags the MPC's by a
  // message), else executed(now, max_age_s).
  Eigen::Vector2d previous(double now, double max_age_s) const;

private:
  std::array<Eigen::Vector2d, kSentKept> sent_{};
  std::size_t sent_count_ = 0, sent_next_ = 0;
  Eigen::Vector2d latest_sent_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d executed_ = Eigen::Vector2d::Zero();
  double executed_at_ = -std::numeric_limits<double>::infinity();
};
}  // namespace mobile_manipulator_mpc
