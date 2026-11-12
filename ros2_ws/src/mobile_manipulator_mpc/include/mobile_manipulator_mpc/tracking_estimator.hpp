#pragma once
// E2 (spec Amendment 2, narrowed after the Task 6b review; plan Task 6b): the online correction of the tracking model.
// The base's turn response and sideslip move with the ground's grip (e2.md "Identification": g_inf 1.27 / 0.98 / 0.80
// at mu 0.4 / 0.6 / 0.8), so a model identified at one grip mispredicts at another. The estimator starts from the
// identified model and corrects g_inf and the sideslip k from the base's measured motion: a Kalman filter with
// random-walk states, updated once per window from the window's mean yaw rate (linear in g_inf for the fixed turn scale
// c and the fixed g_0) and mean lateral speed (linear in k given the turn).
//
// The static map cannot explain two kinds of window, and they are not learned (review C1): a window whose command ramps
// (the base's lag dominates: the start of motion), and turns slower than the stiction band (the base executes a fraction
// of them in place). g_0 is not estimated: it matters only below c, inside that band, and is weakly identified (review
// I3); it stays at the identified value. A window whose innovation is implausible (normalized innovation squared above
// the gate) or whose base did not turn while commanded to (blocked wheels, a contact) is not learned either (review M3).
// Unexcited states do not move; the states stay inside the loader's ranges and the covariance under the cap.
// Deterministic. ROS-free.
#include <Eigen/Core>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/tracking_model.hpp"

namespace mobile_manipulator_mpc
{
struct EstimatorConfig
{
  double sigma_g_inf = 0.3, sigma_k = 0.2;  // the covariance cap (1 sigma): the spread to the mu 0.4 and 0.8 fits
  double initial_fraction = 0.1;  // the initial covariance as a share of the cap (the prior is an identified model)
  // Random walk: a full cap of drift over this many seconds of updates. Under turning the estimate still re-settles
  // fast (review I2: within 0.1 of a new grip after about 0.5 s at 0.2 rad/s at speed); the drift sets how quickly an
  // unexcited estimate regains its full uncertainty.
  double settle_s = 60.0;
  // rad/s and m/s: the noise of a window's mean yaw rate and lateral speed. The prior model's real residual is about
  // 0.01-0.02 rad/s per 0.1 s window and correlated (review I2: model error, not white chatter); 0.03 is a hedge.
  double meas_yaw = 0.03;
  double meas_lateral = 0.005;
  double window_s = 0.1;         // the measurement window
  double max_step_s = 0.03;      // a transition longer than this (a gap) restarts the window
  double ramp_gate = 0.03;       // rad/s: a window whose commanded yaw rate spans more is a ramp, not learned
  double steady_hold_s = 0.3;    // and a window is learned only once the command has stayed within ramp_gate this long
                                 // (the base's lag tail after a ramp: review C1)
  double slow_band = 0.08;       // rad/s: a window whose largest commanded |yaw rate| is below this is not learned
  double min_lateral_regressor = 0.005;  // m rad/s^2: k learns only from windows with |mean vf wf| at least this
  double nis_gate = 9.0;         // a window with a normalized innovation squared above this (3 sigma) is not learned
  double blocked_share = 0.2;    // a turn window measuring less than this share of the predicted turn is not learned
};

class TrackingEstimator
{
public:
  // Throws std::invalid_argument for a non-finite or non-positive spread, time, noise or window, or a negative gate.
  explicit TrackingEstimator(const TrackingModel & prior, EstimatorConfig config = {});

  // One transition of the base: its measured pose (x, y, yaw) before and after, the command that drove it (v, omega
  // in the first two entries) and its duration. A transition without positive duration, longer than max_step_s or
  // with non-finite values restarts the window and changes nothing else.
  void observe(const Eigen::Vector3d & before, const Eigen::Vector3d & after, const Control & sent, double dt);

  TrackingModel model() const;       // the prior with the current g_inf and k (g_0, c and the speed term the prior's)
  Eigen::Vector3d estimate() const;  // g_inf, g_0 (the prior's), k
  Eigen::Matrix3d covariance() const;  // diagonal: g_inf, 0 (g_0 is not estimated), k
  void reset();                      // back to the prior

private:
  void update();
  void clear_window();

  TrackingModel prior_;
  EstimatorConfig config_;
  double turn_ = 0.0, p_turn_ = 0.0;  // turn_delta_inf and its variance (g_inf = 1 + turn_)
  double k_ = 0.0, p_k_ = 0.0;
  // the window's sums: time, yaw change, body-lateral displacement, the regressors integrated over time, and the
  // commanded yaw rate's range
  double t_ = 0.0, yaw_ = 0.0, lateral_ = 0.0, a_ = 0.0, b_ = 0.0, h_ = 0.0, w_min_ = 0.0, w_max_ = 0.0;
  bool empty_ = true;
  // the steady-command clock: the time since the commanded yaw rate last moved more than ramp_gate from its reference
  double steady_s_ = 0.0, w_ref_ = 0.0;
};
}  // namespace mobile_manipulator_mpc
