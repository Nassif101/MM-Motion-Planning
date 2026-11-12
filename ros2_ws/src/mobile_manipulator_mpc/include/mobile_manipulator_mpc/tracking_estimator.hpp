#pragma once
// E2 (spec Amendment 2; plan Task 6b): the online correction of the tracking model. The base's turn response and
// sideslip move with the ground's grip (e2.md "Identification": g_inf 1.27 / 0.98 / 0.80 at mu 0.4 / 0.6 / 0.8), so a
// model identified at one grip mispredicts at another. The estimator starts from the identified model and corrects
// g_inf, g_0 and the sideslip k from the base's measured motion: a Kalman filter with random-walk states, updated once
// per window from the window's mean yaw rate (g_inf, g_0: linear in them for the fixed turn scale c) and mean lateral
// speed (k: linear given the turn). Directions the commands do not excite do not move; the states stay inside the
// loader's ranges and the covariance under the prior's. Deterministic. ROS-free.
#include <Eigen/Core>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/tracking_model.hpp"

namespace mobile_manipulator_mpc
{
struct EstimatorConfig
{
  double sigma_g_inf = 0.3, sigma_g_0 = 0.5, sigma_k = 0.2;  // the prior's spreads (1 sigma) and the covariance cap
  double settle_s = 60.0;       // random walk: a full prior spread of drift over this many seconds of updates (steadiness against tracking speed)
  double meas_yaw = 0.03;       // rad/s: the noise of a window's mean yaw rate (the turning chatter: about 1-2 mrad per 20 ms cycle)
  double meas_lateral = 0.005;  // m/s: the noise of a window's mean lateral speed
  double window_s = 0.1;        // the measurement window
  double max_step_s = 0.03;     // a transition longer than this (a gap) restarts the window
};

class TrackingEstimator
{
public:
  explicit TrackingEstimator(const TrackingModel & prior, EstimatorConfig config = {});

  // One transition of the base: its measured pose (x, y, yaw) before and after, the command that drove it (v, omega
  // in the first two entries) and its duration. A transition without positive duration, longer than max_step_s or
  // with non-finite values restarts the window and changes nothing else.
  void observe(const Eigen::Vector3d & before, const Eigen::Vector3d & after, const Control & sent, double dt);

  TrackingModel model() const;       // the prior with the current g_inf, g_0 and k
  Eigen::Vector3d estimate() const;  // g_inf, g_0, k
  Eigen::Matrix3d covariance() const;
  void reset();                      // back to the prior

private:
  void update();
  void clear_window();

  TrackingModel prior_;
  EstimatorConfig config_;
  Eigen::Vector2d turn_;     // turn_delta_inf, turn_delta_zero (g = 1 + delta)
  Eigen::Matrix2d p_turn_;
  double k_ = 0.0, p_k_ = 0.0;
  // the window's sums: time, yaw change, body-lateral displacement, and the regressors integrated over time
  double t_ = 0.0, yaw_ = 0.0, lateral_ = 0.0, a_ = 0.0, b_ = 0.0, h_ = 0.0;
};
}  // namespace mobile_manipulator_mpc
