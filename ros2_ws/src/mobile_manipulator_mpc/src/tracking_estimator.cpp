#include "mobile_manipulator_mpc/tracking_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace mobile_manipulator_mpc
{
namespace
{
// the loader's ranges (tracking_model.hpp): the states are the deltas themselves, g = 1 + delta, so the prior comes back
// bit for bit
constexpr double kInfLow = -0.5, kInfHigh = 0.5, kZeroLow = -0.99, kZeroHigh = 0.5, kSlipLow = -0.5, kSlipHigh = 0.5;
}  // namespace

TrackingEstimator::TrackingEstimator(const TrackingModel & prior, EstimatorConfig config)
: prior_(prior), config_(config)
{
  reset();
}

void TrackingEstimator::reset()
{
  turn_ = Eigen::Vector2d(prior_.turn_delta_inf, prior_.turn_delta_zero);
  p_turn_ = Eigen::Vector2d(config_.sigma_g_inf * config_.sigma_g_inf, config_.sigma_g_0 * config_.sigma_g_0).asDiagonal();
  k_ = prior_.sideslip;
  p_k_ = config_.sigma_k * config_.sigma_k;
  clear_window();
}

void TrackingEstimator::clear_window() { t_ = yaw_ = lateral_ = a_ = b_ = h_ = 0.0; }

void TrackingEstimator::observe(const Eigen::Vector3d & before, const Eigen::Vector3d & after, const Control & sent,
                                double dt)
{
  if (!(dt > 0.0) || dt > config_.max_step_s || !before.allFinite() || !after.allFinite() || !std::isfinite(sent(0)) ||
      !std::isfinite(sent(1))) {
    clear_window();
    return;
  }
  const double c = prior_.turn_scale, w = sent(1), v = sent(0);
  const double dyaw = std::remainder(after(2) - before(2), 2.0 * M_PI);
  const double mid = before(2) + 0.5 * dyaw;
  const double dx = after(0) - before(0), dy = after(1) - before(1);
  // the turn map's regressors for this command (yaw rate = g_inf a + g_0 b) and the sideslip's (lateral = k vf wf),
  // with the current turn estimate
  const double b = c * std::tanh(w / c), a = w - b;
  const double wf = (1.0 + turn_(0)) * a + (1.0 + turn_(1)) * b;
  const double vf = v * (1.0 - prior_.speed_turn * (std::sqrt(wf * wf + 1e-4) - 0.01));
  t_ += dt;
  yaw_ += dyaw;
  lateral_ += -std::sin(mid) * dx + std::cos(mid) * dy;
  a_ += a * dt;
  b_ += b * dt;
  h_ += vf * wf * dt;
  if (t_ >= config_.window_s - 1e-9) {
    update();
    clear_window();
  }
}

void TrackingEstimator::update()
{
  const double scale = config_.window_s / config_.settle_s;
  // the random walk: each state drifts a full prior spread over settle_s of updates
  p_turn_(0, 0) += config_.sigma_g_inf * config_.sigma_g_inf * scale;
  p_turn_(1, 1) += config_.sigma_g_0 * config_.sigma_g_0 * scale;
  p_k_ += config_.sigma_k * config_.sigma_k * scale;
  // the turn gains: the window's mean yaw rate = H (1 + delta_inf, 1 + delta_zero), so mean yaw rate - a - b = H delta
  const Eigen::RowVector2d h(a_ / t_, b_ / t_);
  const double r = config_.meas_yaw * config_.meas_yaw;
  const double s = (h * p_turn_ * h.transpose())(0, 0) + r;
  const Eigen::Vector2d gain = p_turn_ * h.transpose() / s;
  turn_ += gain * (yaw_ / t_ - h(0) - h(1) - h * turn_);
  p_turn_ = (Eigen::Matrix2d::Identity() - gain * h) * p_turn_;
  p_turn_ = 0.5 * (p_turn_ + p_turn_.transpose());
  turn_(0) = std::clamp(turn_(0), kInfLow, kInfHigh);
  turn_(1) = std::clamp(turn_(1), kZeroLow, kZeroHigh);
  // the sideslip: the window's mean lateral speed = k x mean(vf wf)
  const double hk = h_ / t_;
  const double sk = hk * p_k_ * hk + config_.meas_lateral * config_.meas_lateral;
  const double gk = p_k_ * hk / sk;
  k_ = std::clamp(k_ + gk * (lateral_ / t_ - hk * k_), kSlipLow, kSlipHigh);
  p_k_ *= (1.0 - gk * hk);
  // the cap: no state's variance above the prior's (no windup without excitation)
  const double cap_inf = config_.sigma_g_inf * config_.sigma_g_inf, cap_zero = config_.sigma_g_0 * config_.sigma_g_0;
  const double s0 = p_turn_(0, 0) > cap_inf ? std::sqrt(cap_inf / p_turn_(0, 0)) : 1.0;
  const double s1 = p_turn_(1, 1) > cap_zero ? std::sqrt(cap_zero / p_turn_(1, 1)) : 1.0;
  const Eigen::Matrix2d shrink = Eigen::Vector2d(s0, s1).asDiagonal();
  p_turn_ = shrink * p_turn_ * shrink;
  p_k_ = std::min(p_k_, config_.sigma_k * config_.sigma_k);
}

TrackingModel TrackingEstimator::model() const
{
  TrackingModel m = prior_;
  m.turn_delta_inf = turn_(0);
  m.turn_delta_zero = turn_(1);
  m.sideslip = k_;
  return m;
}

Eigen::Vector3d TrackingEstimator::estimate() const { return Eigen::Vector3d(1.0 + turn_(0), 1.0 + turn_(1), k_); }

Eigen::Matrix3d TrackingEstimator::covariance() const
{
  Eigen::Matrix3d p = Eigen::Matrix3d::Zero();
  p.topLeftCorner<2, 2>() = p_turn_;
  p(2, 2) = p_k_;
  return p;
}
}  // namespace mobile_manipulator_mpc
