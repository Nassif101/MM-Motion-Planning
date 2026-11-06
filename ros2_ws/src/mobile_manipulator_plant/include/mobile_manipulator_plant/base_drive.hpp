#pragma once
// The realistic base drive (MuJoCo plant spec Section 4.3): the command through Unity's SkidSteerBaseController limiter
// (clamp, chassis-space rate limits with braking to zero before a reversal, the stale-command watchdog, common-factor
// wheel-speed saturation), then optional breakaway from rest, a first-order lag and slip: the base twist the planar
// velocity servo is asked for.
#include <Eigen/Core>
#include <vector>

#include "mobile_manipulator_plant/arm_servo.hpp"
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
  // The skid map (calibrated from Unity): achieved / commanded speed and yaw rate, piecewise linear in the inner wheels'
  // share |omega| (b_eff / 2) / |v| at the knots skid_share (ascending; clamped beyond them); below 0.02 m/s the base
  // turns in place with skid_w_in_place. Identity by default.
  std::vector<double> skid_share{0.0}, skid_v{1.0}, skid_w{1.0};
  double skid_w_in_place = 1.0;
};

// The skid-steer mapping onto the wheels: (left, right) wheel speeds (rad/s) of the chassis twist `u` (already limited and
// saturated by BaseDrive): (v -+ omega b_eff / 2) / r.
Eigen::Vector2d wheel_speeds(const Eigen::Vector2d & u, double effective_track_m, double wheel_radius_m);

class BaseDrive
{
public:
  explicit BaseDrive(BaseDriveParams params);
  void reset();
  // The commanded (v, omega), `command_age_s` since it arrived (older than limits.stale_s: zero), over `dt`.
  Eigen::Vector2d target(const Eigen::Vector2d & command, double command_age_s, double dt);
  // The limiter's reference motion after the last target() (for the arm's feedforward, as Unity's arm takes it).
  const BaseMotion & motion() const { return motion_; }

private:
  BaseMotion motion_;
  BaseDriveParams p_;
  Eigen::Vector2d limited_ = Eigen::Vector2d::Zero(), output_ = Eigen::Vector2d::Zero(), lagged_ = Eigen::Vector2d::Zero();
};
}  // namespace mobile_manipulator_plant
