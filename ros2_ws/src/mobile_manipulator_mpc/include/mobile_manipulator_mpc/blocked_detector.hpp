#pragma once
// Blocked detection (Phase 4B-A spec Section 2): B5 commands motion while the collision monitor passes
// nothing (a StopZone or a FootprintApproach at zero). The MPC cannot see this in its own plan (its margin
// keeps the plan clear), so without it the base sits still with status ok while the monitor is silent.
//  - Blocked: for detect_s in a row the MPC's sent base command reaches own_v or own_w while the base is
//    measured at rest and the executed command (ExecutedCommand::executed) stays below own_v and own_w. own_*
//    are the base breakaway: with the executed-command feedback a blocked MPC starting from rest sends exactly
//    the breakaway, and an executed command below it does not start the base (a FootprintApproach scaling it
//    down stalls the base as a STOP does). The blocked time counts from the first such cycle.
//  - Clear again once the monitor's /collision_monitor_state has not been STOP for clear_s (holding sends
//    zero, so the executed command cannot show when the way is free).
//  - Failed (latched until restart) when the blocked time, summed over holds and resumes, reaches fail_s;
//    it starts again whenever the base has moved progress_m since the last start.
#include <limits>

#include <Eigen/Core>

namespace mobile_manipulator_mpc
{
struct BlockedConfig
{
  double own_v = 0.0375, own_w = 0.07;  // nav_operating_envelope.yaml tracking.breakaway_* (the node sets them)
  double detect_s = 1.0, fail_s = 3.0, clear_s = 0.5, progress_m = 0.05;
};

class BlockedDetector
{
public:
  enum class State { Clear, Blocked, Failed };

  explicit BlockedDetector(BlockedConfig config = {});
  // One cycle at `now` (seconds, monotonic): `own` the base command the MPC sent last, `executed` the executed
  // one, `monitor_stop` whether the monitor's latest state is STOP, `base_xy` the measured base position,
  // `base_at_rest` whether the measured base is at rest (mobile_manipulator_mpc::base_at_rest).
  State update(double now, const Eigen::Vector2d & own, const Eigen::Vector2d & executed, bool monitor_stop,
               const Eigen::Vector2d & base_xy, bool base_at_rest);
  // A new goal (not a preemption): Clear, no blocked time, the position anchor taken at the next update.
  void restart();

  State state() const { return state_; }
  double blocked_time() const { return blocked_; }
  const BlockedConfig & config() const { return config_; }

private:
  BlockedConfig config_;
  State state_ = State::Clear;
  double since_ = std::numeric_limits<double>::quiet_NaN();     // commanding without execution since
  double free_since_ = std::numeric_limits<double>::quiet_NaN();  // blocked, monitor not STOP since
  double last_ = std::numeric_limits<double>::quiet_NaN();      // the previous update
  double blocked_ = 0.0;
  bool anchored_ = false;
  Eigen::Vector2d anchor_ = Eigen::Vector2d::Zero();
};
}  // namespace mobile_manipulator_mpc
