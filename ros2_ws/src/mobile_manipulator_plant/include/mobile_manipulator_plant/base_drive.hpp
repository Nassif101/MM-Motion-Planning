#pragma once
// The realistic base drive (MuJoCo plant spec Section 4.3): the command through Unity's SkidSteerBaseController limiter
// (clamp, chassis-space rate limits with braking to zero before a reversal, the stale-command watchdog, common-factor
// wheel-speed saturation), then optional breakaway from rest, a first-order lag and slip: the base twist the planar
// velocity servo is asked for.
#include <Eigen/Core>

#include "mobile_manipulator_plant/robot_profile.hpp"

namespace mobile_manipulator_plant
{
struct BaseDriveParams
{
  BaseLimits limits;
  double wheel_radius_m = 0.14, effective_track_m = 1.5, wheel_speed_max = 8.0;  // Unity's skid-steer mapping
  bool breakaway = false;  // off: Unity's base friction feedforward (Phase 5A) starts every command
  double breakaway_linear = 0.0, breakaway_yaw = 0.0, rest_linear = 0.0, rest_yaw = 0.0;
  double tau_v = 0.0, tau_w = 0.0;  // first-order lag (s); 0: none
  double k_v = 1.0, k_w = 1.0;      // slip: achieved / target
};

class BaseDrive
{
public:
  explicit BaseDrive(BaseDriveParams params);
  void reset();
  // The commanded (v, omega), `command_age_s` since it arrived (older than limits.stale_s: zero), over `dt`.
  Eigen::Vector2d target(const Eigen::Vector2d & command, double command_age_s, double dt);

private:
  BaseDriveParams p_;
  Eigen::Vector2d limited_ = Eigen::Vector2d::Zero(), output_ = Eigen::Vector2d::Zero(), lagged_ = Eigen::Vector2d::Zero();
};
}  // namespace mobile_manipulator_plant
