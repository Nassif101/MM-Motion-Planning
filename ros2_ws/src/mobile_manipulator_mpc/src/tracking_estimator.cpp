#include "mobile_manipulator_mpc/tracking_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mobile_manipulator_mpc
{
namespace
{
// the loader's ranges (tracking_model.hpp); the states are the deltas, so the prior comes back bit for bit
constexpr double kInfLow = -0.5, kInfHigh = 0.5, kSlipLow = -0.5, kSlipHigh = 0.5;

void check(bool ok, const char * what)
{
  if (!ok) throw std::invalid_argument(std::string("TrackingEstimator: ") + what);
}
}  // namespace

TrackingEstimator::TrackingEstimator(const TrackingModel & prior, EstimatorConfig config)
: prior_(prior), config_(config)
{
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  const auto non_negative = [](double v) { return std::isfinite(v) && v >= 0.0; };
  check(positive(config_.sigma_g_inf) && positive(config_.sigma_k), "the spreads must be finite and positive");
  check(positive(config_.initial_fraction) && config_.initial_fraction <= 1.0, "initial_fraction must be in (0, 1]");
  check(positive(config_.settle_s), "settle_s must be finite and positive");
  check(positive(config_.meas_yaw) && positive(config_.meas_lateral), "the noises must be finite and positive");
  check(positive(config_.window_s) && positive(config_.max_step_s), "the window and step must be finite and positive");
  check(non_negative(config_.ramp_gate) && non_negative(config_.slow_band) &&
          non_negative(config_.min_lateral_regressor) && positive(config_.nis_gate) && non_negative(config_.blocked_share) &&
          non_negative(config_.steady_hold_s),
        "the gates must be finite and not negative");
  reset();
}

void TrackingEstimator::reset()
{
  turn_ = prior_.turn_delta_inf;
  k_ = prior_.sideslip;
  p_turn_ = config_.initial_fraction * config_.sigma_g_inf * config_.sigma_g_inf;
  p_k_ = config_.initial_fraction * config_.sigma_k * config_.sigma_k;
  steady_s_ = 0.0;
  w_ref_ = 0.0;
  clear_window();
}

void TrackingEstimator::interrupt()
{
  clear_window();
  steady_s_ = 0.0;
}

void TrackingEstimator::clear_window()
{
  t_ = yaw_ = lateral_ = a_ = b_ = h_ = offset_ = 0.0;
  w_min_ = w_max_ = 0.0;
  empty_ = true;
}

void TrackingEstimator::observe(const Eigen::Vector3d & before, const Eigen::Vector3d & after, const Control & sent,
                                double dt)
{
  if (prior_.turn_offset != 0.0 || prior_.turn_offset_com != 0.0)
    throw std::logic_error("TrackingEstimator: a model with a turning-centre offset needs the centre of mass (observe "
                           "with com_x)");
  observe(before, after, sent, dt, 0.0);
}

void TrackingEstimator::observe(const Eigen::Vector3d & before, const Eigen::Vector3d & after, const Control & sent,
                                double dt, double com_x)
{
  if (!(dt > 0.0) || dt > config_.max_step_s || !before.allFinite() || !after.allFinite() || !std::isfinite(sent(0)) ||
      !std::isfinite(sent(1)) || !std::isfinite(com_x)) {
    interrupt();  // after a gap the base's state is unknown: wait for a steady command again
    return;
  }
  const double c = prior_.turn_scale, g_0 = 1.0 + prior_.turn_delta_zero, w = sent(1), v = sent(0);
  const double dyaw = std::remainder(after(2) - before(2), 2.0 * M_PI);
  const double mid = before(2) + 0.5 * dyaw;
  const double dx = after(0) - before(0), dy = after(1) - before(1);
  // the turn map's regressors for this command (yaw rate = g_inf a + g_0 b) and the sideslip's (lateral = k vf wf),
  // with the current turn estimate
  const double b = c * std::tanh(w / c), a = w - b;
  const double wf = (1.0 + turn_) * a + g_0 * b;
  const double vf = v * (1.0 - prior_.speed_turn * (std::sqrt(wf * wf + 1e-4) - 0.01));
  if (std::abs(w - w_ref_) > config_.ramp_gate) {  // the command moved: its steady clock restarts
    w_ref_ = w;
    steady_s_ = 0.0;
  } else {
    steady_s_ += dt;
  }
  w_min_ = empty_ ? w : std::min(w_min_, w);
  w_max_ = empty_ ? w : std::max(w_max_, w);
  empty_ = false;
  t_ += dt;
  yaw_ += dyaw;
  lateral_ += -std::sin(mid) * dx + std::cos(mid) * dy;
  a_ += a * dt;
  b_ += b * dt;
  h_ += vf * wf * dt;
  offset_ += (prior_.turn_offset + prior_.turn_offset_com * com_x) * wf * dt;
  if (t_ >= config_.window_s - 1e-9) {
    update();
    clear_window();
  }
}

void TrackingEstimator::update()
{
  const double scale = config_.window_s / config_.settle_s;
  const double cap_turn = config_.sigma_g_inf * config_.sigma_g_inf, cap_k = config_.sigma_k * config_.sigma_k;
  // the random walk, then the cap (no windup without excitation)
  p_turn_ = std::min(cap_turn, p_turn_ + cap_turn * scale);
  p_k_ = std::min(cap_k, p_k_ + cap_k * scale);
  // a ramp (the base's lag dominates) is not learned at all (review C1)
  if (w_max_ - w_min_ > config_.ramp_gate || steady_s_ < config_.steady_hold_s) return;
  const double g_0 = 1.0 + prior_.turn_delta_zero;
  const double ha = a_ / t_, hb = b_ / t_, measured = yaw_ / t_;
  const double predicted = (1.0 + turn_) * ha + g_0 * hb;
  const double fastest = std::max(std::abs(w_min_), std::abs(w_max_));
  // the turn gain: the window's mean yaw rate - a - g_0 b = delta_inf a; only above the stiction band, with a plausible
  // innovation, and when the base turned at all (review C1, M3)
  const bool blocked = std::abs(measured) < config_.blocked_share * std::abs(predicted);
  if (fastest >= config_.slow_band && !blocked) {
    const double innovation = measured - predicted;
    const double s = ha * p_turn_ * ha + config_.meas_yaw * config_.meas_yaw;
    if (innovation * innovation / s <= config_.nis_gate) {
      const double gain = p_turn_ * ha / s;
      turn_ = std::clamp(turn_ + gain * innovation, kInfLow, kInfHigh);
      p_turn_ *= (1.0 - gain * ha);
    }
  }
  // the sideslip: the window's mean lateral speed = k x mean(vf wf), when the turn at speed excites it
  const double hk = h_ / t_;
  if (std::abs(hk) >= config_.min_lateral_regressor && !blocked) {
    const double innovation = (lateral_ - offset_) / t_ - hk * k_;
    const double s = hk * p_k_ * hk + config_.meas_lateral * config_.meas_lateral;
    if (innovation * innovation / s <= config_.nis_gate) {
      const double gain = p_k_ * hk / s;
      k_ = std::clamp(k_ + gain * innovation, kSlipLow, kSlipHigh);
      p_k_ *= (1.0 - gain * hk);
    }
  }
}

TrackingModel TrackingEstimator::model() const
{
  TrackingModel m = prior_;
  m.turn_delta_inf = turn_;
  m.sideslip = k_;
  return m;
}

Eigen::Vector3d TrackingEstimator::estimate() const
{
  return Eigen::Vector3d(1.0 + turn_, 1.0 + prior_.turn_delta_zero, k_);
}

Eigen::Matrix3d TrackingEstimator::covariance() const
{
  Eigen::Matrix3d p = Eigen::Matrix3d::Zero();
  p(0, 0) = p_turn_;
  p(2, 2) = p_k_;
  return p;
}
}  // namespace mobile_manipulator_mpc
