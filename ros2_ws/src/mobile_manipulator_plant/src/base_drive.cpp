#include "mobile_manipulator_plant/base_drive.hpp"

#include <algorithm>
#include <cmath>

namespace mobile_manipulator_plant
{
namespace
{
// Unity's chassis-space rate limit for one component: a reversal (or a stop) brakes to zero at the deceleration limit
// first; speeding up uses the acceleration limit, slowing down the deceleration limit.
double rate_limit(double current, double target, double accel, double decel, double dt)
{
  auto toward = [](double from, double to, double step) { return from < to ? std::min(to, from + step) : std::max(to, from - step); };
  if (current != 0.0 && (target == 0.0 || std::signbit(target) != std::signbit(current)))
    return toward(current, 0.0, decel * dt);
  if (std::abs(target) > std::abs(current)) return toward(current, target, accel * dt);
  return toward(current, target, decel * dt);
}
}  // namespace

BaseDrive::BaseDrive(BaseDriveParams params) : p_(params) {}

void BaseDrive::reset()
{
  limited_.setZero();
  output_.setZero();
  lagged_.setZero();
  motion_ = BaseMotion{};
}

Eigen::Vector2d BaseDrive::target(const Eigen::Vector2d & command, double command_age_s, double dt)
{
  const BaseLimits & l = p_.limits;
  // the watchdog: a stale command is no command
  Eigen::Vector2d c = command_age_s > l.stale_s ? Eigen::Vector2d::Zero() : command;
  c(0) = std::clamp(c(0), -l.v_max_rev, l.v_max_fwd);
  c(1) = std::clamp(c(1), -l.w_max, l.w_max);
  const Eigen::Vector2d before = limited_;
  limited_(0) = rate_limit(limited_(0), c(0), l.accel, l.decel, dt);
  limited_(1) = rate_limit(limited_(1), c(1), l.yaw_accel, l.yaw_decel, dt);
  motion_ = {limited_(0), (limited_(0) - before(0)) / dt, limited_(1), (limited_(1) - before(1)) / dt};
  // inverse kinematics and the common-factor saturation of the faster wheel (curvature kept)
  Eigen::Vector2d u = limited_;
  const double half = 0.5 * p_.effective_track_m;
  const double fastest = std::max(std::abs(u(0) - u(1) * half), std::abs(u(0) + u(1) * half)) / p_.wheel_radius_m;
  if (fastest > p_.wheel_speed_max) u *= p_.wheel_speed_max / fastest;
  // breakaway from rest (off when Unity's friction feedforward starts every command)
  if (p_.breakaway && std::abs(output_(0)) < p_.rest_linear && std::abs(output_(1)) < p_.rest_yaw &&
      std::abs(u(0)) < p_.breakaway_linear && std::abs(u(1)) < p_.breakaway_yaw)
    u.setZero();
  // the drive's first-order lag, then slip
  const double av = p_.tau_v > 0.0 ? 1.0 - std::exp(-dt / p_.tau_v) : 1.0;
  const double aw = p_.tau_w > 0.0 ? 1.0 - std::exp(-dt / p_.tau_w) : 1.0;
  lagged_(0) += av * (u(0) - lagged_(0));
  lagged_(1) += aw * (u(1) - lagged_(1));
  output_ = Eigen::Vector2d(p_.k_v * lagged_(0), p_.k_w * lagged_(1));
  return output_;
}
}  // namespace mobile_manipulator_plant
