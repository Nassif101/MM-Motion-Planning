#include "mobile_manipulator_mpc/formulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <urdf/model.h>

#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_mpc
{
namespace
{
namespace mmn = mobile_manipulator_navigation;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr int kNz = Formulation::kNz;
constexpr int kYaw = 2, kQ = 3;  // state indices: yaw, first arm joint
constexpr int kPrev = kNx;       // QP state index of the previous control
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPadLower = -1.0;  // an unused collision row: 0 >= -1, always satisfied

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

const mmn::Json & field(const mmn::Json & node, const char * key, const std::string & where)
{
  if (!node.is_object() || !node.contains(key)) throw std::runtime_error(where + ": missing '" + key + "'");
  return node.at(key);
}

double number(const mmn::Json & node, const char * key, const std::string & where)
{
  const auto & value = field(node, key, where);
  if (!value.is_number()) throw std::runtime_error(where + ": '" + key + "' is not a number");
  return value.get<double>();
}

Eigen::Matrix<double, 6, 1> six(const mmn::Json & node, const char * key, const std::string & where)
{
  const auto & list = field(node, key, where);
  if (!list.is_array() || list.size() != 6) throw std::runtime_error(where + ": '" + key + "' needs six values");
  Eigen::Matrix<double, 6, 1> out;
  for (int i = 0; i < 6; ++i) out(i) = list.at(i).get<double>();
  return out;
}

// Residual r(z + dz, u + du) ~ r + Jz dz + Ju du, cost 0.5 w |r|^2.
void add_state_residual(StageQp & st, const Eigen::Ref<const MatrixXd> & Jz, const Eigen::Ref<const VectorXd> & r,
                        double w)
{
  if (w == 0.0) return;
  st.Q.noalias() += w * Jz.transpose() * Jz;
  st.q.noalias() += w * Jz.transpose() * r;
}

// Unit tangent of the route at s: the chord over s +- kTangentChordM, so the direction turns
// continuously through a corner instead of jumping between the incoming and outgoing segments.
Vector2d tangent(const Route & route, double s)
{
  const Vector2d chord = route.at(s + kTangentChordM) - route.at(s - kTangentChordM);
  if (chord.norm() < 1e-9) {
    const double h = route.heading_at(s);
    return {std::cos(h), std::sin(h)};
  }
  return chord.normalized();
}

// SO(3) inverse left Jacobian at the rotation vector phi (angle clamped below pi, where it is singular).
Matrix3d inverse_left_jacobian(const Vector3d & phi)
{
  const double angle = phi.norm();
  Matrix3d W;
  W << 0, -phi.z(), phi.y(), phi.z(), 0, -phi.x(), -phi.y(), phi.x(), 0;
  if (angle < 1e-6) return Matrix3d::Identity() - 0.5 * W;
  const double a = std::min(angle, 2.8);
  const double coefficient = 1.0 / (a * a) - (1.0 + std::cos(a)) / (2.0 * a * std::sin(a));
  return Matrix3d::Identity() - 0.5 * W + coefficient * W * W;
}

struct OrientationError
{
  Vector3d e;
  Eigen::Matrix<double, 3, kNx> J;
};

// Panel orientation error in base_footprint as the rotation vector log(R_ref^T R) and its
// Jacobian. Unlike vee(0.5 (R_ref^T R - R^T R_ref)) (whose norm sin(angle) falls again beyond
// 90 degrees, so a Gauss-Newton step there turns away from the reference), it decreases towards
// the reference from any angle below 180 degrees; home -> vertical_carry is 120 degrees.
OrientationError orientation_error(const State & x, const Matrix3d & R_ref)
{
  State xb = x;
  xb.head<3>().setZero();  // base_footprint coordinates
  const int panel = link_index("panel");
  const Matrix3d R = link_pose(panel, xb).linear();
  const PointJacobian origin = point_jacobian(panel, xb, Vector3d::Zero());
  Eigen::Matrix<double, 3, kNx> Jw = Eigen::Matrix<double, 3, kNx>::Zero();
  for (int i = 0; i < 3; ++i) {
    const PointJacobian axis = point_jacobian(panel, xb, Vector3d::Unit(i));
    const Eigen::Matrix<double, 3, kNx> dr = axis.J - origin.J;  // d(R e_i)/dx = w x (R e_i)
    const Vector3d r = R.col(i);
    for (int c = 0; c < kNx; ++c) Jw.col(c) += 0.5 * r.cross(dr.col(c).eval());
  }
  const Eigen::AngleAxisd aa(R_ref.transpose() * R);
  OrientationError out;
  out.e = aa.angle() * aa.axis();
  out.J = inverse_left_jacobian(out.e) * R_ref.transpose() * Jw;
  out.J.leftCols<3>().setZero();
  return out;
}
}  // namespace

// ---- loading ----

Limits load_limits(const std::string & envelope_yaml, const std::string & urdf_text)
{
  const mmn::Json env = mmn::load_yaml_file(envelope_yaml);
  const std::string where = "operating envelope";
  Limits l{};
  const auto & profiles = field(env, "profiles", where);
  if (!profiles.is_object() || profiles.empty()) throw std::runtime_error(where + ": no profiles");
  l.v_max = kInf;
  l.omega_max = kInf;
  for (const auto & item : profiles.items()) {
    l.v_max = std::min(l.v_max, number(item.value(), "max_forward_mps", where + " profile " + item.key()));
    l.omega_max = std::min(l.omega_max, number(item.value(), "max_yaw_radps", where + " profile " + item.key()));
  }
  const auto & acc = field(env, "acceleration", where);
  l.accel = number(acc, "linear_accel_mps2", where);
  l.decel = number(acc, "linear_decel_mps2", where);
  l.yaw_accel = number(acc, "yaw_accel_radps2", where);
  l.yaw_decel = number(acc, "yaw_decel_radps2", where);
  const auto & tracking = field(env, "tracking", where);
  l.breakaway.linear_mps = number(tracking, "breakaway_linear_mps", where);
  l.breakaway.yaw_radps = number(tracking, "breakaway_yaw_radps", where);
  const auto & arm = field(env, "wholebody_arm", where);
  l.qd_max = six(arm, "max_joint_velocity_radps", where);
  l.qdd_max = six(arm, "max_joint_accel_radps2", where);

  urdf::Model model;
  if (!model.initString(urdf_text)) throw std::runtime_error("load_limits: could not parse the URDF");
  for (int i = 0; i < 6; ++i) {
    const auto joint = model.getJoint(kArmJoints[i]);
    if (!joint || !joint->limits) throw std::runtime_error(std::string("load_limits: no limits for ") + kArmJoints[i]);
    l.q_min(i) = joint->limits->lower;
    l.q_max(i) = joint->limits->upper;
  }
  if (arm.contains("joint_bounds")) {
    const auto & bounds = arm.at("joint_bounds");
    if (!bounds.is_array() || bounds.size() != 6) throw std::runtime_error(where + ": joint_bounds needs six pairs");
    for (int i = 0; i < 6; ++i) {
      l.q_min(i) = std::max(l.q_min(i), bounds.at(i).at(0).get<double>());
      l.q_max(i) = std::min(l.q_max(i), bounds.at(i).at(1).get<double>());
      if (l.q_min(i) > l.q_max(i)) throw std::runtime_error(where + ": empty joint range for " + kArmJoints[i]);
    }
  }
  return l;
}

MpcConfig load_mpc_config(const std::string & yaml_path)
{
  const mmn::Json root = mmn::load_yaml_file(yaml_path);
  const std::string where = yaml_path;
  MpcConfig c{};
  c.horizon = static_cast<int>(number(root, "horizon", where));
  c.dt = number(root, "dt_s", where);
  const auto & w = field(root, "weights", where);
  Weights & out = c.weights;
  out.contour = number(w, "contour", where);
  out.lag = number(w, "lag", where);
  out.progress = number(w, "progress", where);
  out.heading = number(w, "heading", where);
  out.goal_heading = number(w, "goal_heading", where);
  out.panel_position = number(w, "panel_position", where);
  out.panel_orientation = number(w, "panel_orientation", where);
  out.posture = number(w, "posture", where);
  if (w.contains("slack_inside")) out.slack_inside = number(w, "slack_inside", where);
  out.effort_base = number(w, "effort_base", where);
  out.effort_arm = number(w, "effort_arm", where);
  out.smooth_base = number(w, "smooth_base", where);
  out.smooth_arm = number(w, "smooth_arm", where);
  out.slack = number(w, "slack", where);
  out.slack_quadratic = number(w, "slack_quadratic", where);
  const auto & clearance = field(root, "clearance", where);
  out.clearance = number(clearance, "weight", where);
  out.clearance_target_m = number(clearance, "target_m", where);
  if (!(out.clearance >= 0.0) || !std::isfinite(out.clearance) || !(out.clearance_target_m > 0.0))
    throw std::runtime_error(where + ": clearance weight must be >= 0 and target_m > 0");
  const auto & posture = field(root, "posture", where);
  const auto & mode = field(posture, "mode", where);
  const auto & comfort = field(posture, "comfort_pose", where);
  if (!mode.is_string() || !comfort.is_string())
    throw std::runtime_error(where + ": posture mode and comfort_pose must be names");
  c.posture_mode = parse_posture_mode(mode.get<std::string>());
  c.comfort_pose = comfort.get<std::string>();
  if (posture.contains("tight_poses")) {
    const auto & tight = posture.at("tight_poses");
    if (!tight.is_array() || tight.empty()) throw std::runtime_error(where + ": posture.tight_poses must be a non-empty list");
    c.tight_poses.clear();
    for (const auto & name : tight) {
      if (!name.is_string()) throw std::runtime_error(where + ": posture.tight_poses entries must be pose names");
      c.tight_poses.push_back(name.get<std::string>());
    }
  }
  const auto & solver = field(root, "solver", where);
  c.solver_eps = number(solver, "eps", where);
  c.solver_max_iter = static_cast<int>(number(solver, "max_iter", where));
  c.fallback_s = number(root, "fallback_s", where);
  const auto & gate = field(root, "slack_gate", where);
  if (!gate.is_boolean()) throw std::runtime_error(where + ": slack_gate must be true or false");
  c.slack_gate.enabled = gate.get<bool>();
  c.slack_gate.horizon_s = number(root, "slack_gate_horizon_s", where);
  c.slack_gate.tolerance_m = number(root, "slack_gate_tolerance_m", where);
  c.slack_gate.resume_horizon_s = number(root, "slack_gate_resume_horizon_s", where);
  c.slack_gate.resume_dwell_s = number(root, "slack_gate_resume_dwell_s", where);
  if (!(c.slack_gate.horizon_s > 0.0) || !(c.slack_gate.resume_horizon_s > 0.0) ||
      !(c.slack_gate.resume_dwell_s >= 0.0) || !(c.slack_gate.tolerance_m >= 0.0) || !std::isfinite(c.slack_gate.tolerance_m))
    throw std::runtime_error(where + ": slack_gate_horizon_s and slack_gate_resume_horizon_s must be positive, "
                             "slack_gate_resume_dwell_s >= 0 and slack_gate_tolerance_m finite and >= 0");
  const auto & collision = field(root, "collision", where);
  c.collision.base_margin_m = number(collision, "base_margin_m", where);
  if (!(c.collision.base_margin_m >= c.collision.margin_m) || !std::isfinite(c.collision.base_margin_m))
    throw std::runtime_error(where + ": collision.base_margin_m must be finite and at least the 0.05 m margin");
  const auto & blocked = field(root, "blocked", where);
  c.blocked.detect_s = number(blocked, "detect_s", where);
  c.blocked.fail_s = number(blocked, "fail_s", where);
  c.blocked.clear_s = number(blocked, "clear_s", where);
  c.blocked.progress_m = number(blocked, "progress_m", where);
  if (!(c.blocked.detect_s > 0.0) || !(c.blocked.fail_s > 0.0) || !(c.blocked.clear_s > 0.0) ||
      !(c.blocked.progress_m > 0.0))
    throw std::runtime_error(where + ": every blocked value must be positive");
  if (c.horizon < 1 || c.dt <= 0.0) throw std::runtime_error(where + ": horizon and dt_s must be positive");
  if (!(c.fallback_s > 0.0)) throw std::runtime_error(where + ": fallback_s must be positive");
  return c;
}

// ---- limits of the controls ----

Control lower_control(const Limits & l)
{
  Control u;
  u << 0.0, -l.omega_max, -l.qd_max;
  return u;
}

Control upper_control(const Limits & l)
{
  Control u;
  u << l.v_max, l.omega_max, l.qd_max;
  return u;
}

Control lower_rate(const Limits & l, double dt)
{
  Control r;
  r << -l.decel * dt, -l.yaw_accel * dt, -l.qdd_max * dt;
  return r;
}

Control upper_rate(const Limits & l, double dt)
{
  Control r;
  r << l.accel * dt, l.yaw_accel * dt, l.qdd_max * dt;
  return r;
}

// ---- side picker ----

const char * to_string(PostureChoice choice)
{
  switch (choice) {
    case PostureChoice::None:
      return "none";
    case PostureChoice::Comfort:
      return "comfort";
    case PostureChoice::Tight0:
      return "left";
    case PostureChoice::Tight1:
      return "right";
  }
  return "unknown";
}

PostureChoice SidePicker::update(double alpha, const std::vector<double> & comfort,
                                 const std::vector<std::vector<double>> & tight, double shoulder_pan,
                                 double route_turn_sign)
{
  if (alpha < kPickAlphaOff) {
    choice_ = PostureChoice::None;
    scores_.clear();
    return choice_;
  }
  if ((choice_ != PostureChoice::None && choice_ != PostureChoice::Comfort) || alpha < kPickAlphaOn - 1e-12 ||
      comfort.empty())
    return choice_;
  const auto conflict = std::find_if(comfort.begin(), comfort.end(), [](double c) { return c < kTightComfortM; });
  if (conflict == comfort.end()) return choice_;
  const int k = static_cast<int>(conflict - comfort.begin());
  auto score = [&](const std::vector<double> & v) {
    double s = std::numeric_limits<double>::infinity();
    for (int i = std::max(0, k - 1); i <= std::min(static_cast<int>(v.size()) - 1, k + 1); ++i)
      s = std::min(s, v[static_cast<size_t>(i)]);
    return s;
  };
  const bool from_comfort = choice_ == PostureChoice::Comfort;
  const double comfort_min = *std::min_element(comfort.begin(), comfort.end());
  scores_ = {score(comfort)};
  int best = -1;
  for (size_t i = 0; i < tight.size() && i < 2; ++i) {
    scores_.push_back(score(tight[i]));
    if (!(scores_.back() > scores_[0] + kPickMarginM)) continue;  // the guard: must beat the comfort pose
    if (from_comfort && !(*std::min_element(tight[i].begin(), tight[i].end()) > comfort_min + kPickMarginM)) {
      scores_.back() = -std::numeric_limits<double>::infinity();  // better only on the approach: not eligible
      continue;
    }
    if (best < 0 || scores_.back() > scores_[static_cast<size_t>(best) + 1]) best = static_cast<int>(i);
  }
  if (best < 0) {
    choice_ = PostureChoice::Comfort;
    return choice_;
  }
  const bool both = scores_.size() == 3 && scores_[1] > scores_[0] + kPickMarginM && scores_[2] > scores_[0] + kPickMarginM;
  if (both && std::abs(scores_[1] - scores_[2]) <= kPickMarginM) {  // a tie
    if (std::abs(shoulder_pan) > kPickPanRad) best = shoulder_pan > 0.0 ? 0 : 1;
    else if (route_turn_sign != 0.0) best = route_turn_sign > 0.0 ? 0 : 1;
    else best = 0;
  }
  choice_ = best == 0 ? PostureChoice::Tight0 : PostureChoice::Tight1;
  return choice_;
}

// ---- tightness ----

TightnessFilter::TightnessFilter(double comfort_m, double margin_m, double tau_s)
: comfort_m_(comfort_m), margin_m_(margin_m), tau_s_(tau_s)
{
  if (!(comfort_m > margin_m) || !(tau_s > 0.0)) throw std::invalid_argument("TightnessFilter: bad parameters");
}

double TightnessFilter::update(double home_clearance_m, double dt)
{
  const double target = std::clamp((comfort_m_ - home_clearance_m) / (comfort_m_ - margin_m_), 0.0, 1.0);
  alpha_ += (1.0 - std::exp(-std::max(dt, 0.0) / tau_s_)) * (target - alpha_);
  return alpha_;
}

// ---- formulation ----

Formulation::Formulation(int horizon, double dt, Limits limits, Weights weights, CollisionConfig collision)
: horizon_(horizon), dt_(dt), limits_(limits), weights_(weights), collision_(collision)
{
  if (horizon < 1 || !(dt > 0.0)) throw std::invalid_argument("Formulation: horizon and dt must be positive");
  if (collision.pairs_per_stage < 0) throw std::invalid_argument("Formulation: negative row budget");
}

double Formulation::progress_window() const { return std::max(1.0, 3.0 * limits_.v_max * dt_); }

double Formulation::stage0_progress(const Inputs & in) const
{
  if (in.route == nullptr) throw std::invalid_argument("Formulation::stage0_progress: no route");
  const Vector2d p = in.x0.head<2>();
  if (std::isnan(in.s0_hint)) return in.route->project_range(p, 0.0, in.route->length()).s;
  return in.route->project(p, in.s0_hint, progress_window()).s;
}

QpProblem Formulation::build(const Inputs & in, const Plan & nominal, const std::vector<DistanceRow> & rows) const
{
  const int N = horizon_;
  const int Rc = collision_.pairs_per_stage;
  if (in.route == nullptr) throw std::invalid_argument("Formulation::build: no route");
  if (static_cast<int>(nominal.x.size()) != N + 1 || static_cast<int>(nominal.u.size()) != N)
    throw std::invalid_argument("Formulation::build: nominal does not match the horizon");
  const Route & route = *in.route;
  const Weights & w = weights_;
  // The posture attractor: blend moves it from the comfort pose to the tight pose with the
  // tightness; comfort_only keeps the comfort pose; none has no posture term.
  const bool tight_choice = in.posture_choice == PostureChoice::Tight0 || in.posture_choice == PostureChoice::Tight1;
  const double alpha =
    in.posture.mode == PostureMode::Blend && tight_choice ? std::clamp(in.tightness, 0.0, 1.0) : 0.0;
  const size_t tight_index = in.posture_choice == PostureChoice::Tight1 ? 1 : 0;
  const Eigen::Matrix<double, 6, 1> & tight =
    tight_index < in.posture.tights.size() ? in.posture.tights[tight_index] : in.posture.tight;
  const Eigen::Matrix<double, 6, 1> posture_target = (1.0 - alpha) * in.posture.comfort + alpha * tight;
  const double posture_weight = in.posture.mode == PostureMode::None ? 0.0 : w.posture;
  const Control u_lo = lower_control(limits_), u_hi = upper_control(limits_);
  const Control rate_lo = lower_rate(limits_, dt_), rate_hi = upper_rate(limits_, dt_);
  const Control last = in.last_command.cwiseMax(u_lo).cwiseMin(u_hi);
  const double goal_s = std::clamp(in.goal_s, 0.0, route.length());
  const Vector2d goal_point = route.at(goal_s);
  const double align = std::clamp((kGoalAlignStartM - (in.x0.head<2>() - goal_point).norm()) /
                                    (kGoalAlignStartM - kGoalAlignFullM), 0.0, 1.0);

  // Collision rows per stage (stage 0 is fixed and gets none).
  std::vector<std::vector<const DistanceRow *>> by_stage(N + 1);
  for (const auto & row : rows) {
    if (row.stage <= 0 || row.stage > N) continue;
    by_stage[row.stage].push_back(&row);
    if (static_cast<int>(by_stage[row.stage].size()) > Rc)
      throw std::invalid_argument("Formulation::build: more collision rows in a stage than the budget");
  }

  // Route progress of each nominal stage and the progress reference.
  auto progress_speed = [&](double s) {
    if (s >= goal_s) return 0.0;
    return std::max(limits_.breakaway.linear_mps, limits_.v_max * std::clamp((goal_s - s) / kProgressTaperM, 0.0, 1.0));
  };
  const double window = progress_window();
  std::vector<double> s(N + 1), s_ref(N + 1);
  s[0] = stage0_progress(in);
  for (int k = 1; k <= N; ++k) s[k] = route.project(nominal.x[k].head<2>(), s[k - 1], window).s;
  // Lag reference: one step at the tapered progress speed ahead of the stage's own projection,
  // never beyond the goal. Far from the goal it is a bounded forward pull at full speed (the
  // speed itself is the progress term's); near the goal it follows the taper (ruling R25: a
  // full-speed pull outweighed the taper, and the base reached the goal tolerance at 0.15 m/s).
  // A reference running ahead from the measured progress pulled up to 0.6 m into an obstacle
  // that blocked the base, and the large multipliers that followed stalled the QP solver.
  s_ref[0] = s[0];
  for (int k = 1; k <= N; ++k) s_ref[k] = std::min(goal_s, s[k] + progress_speed(s[k]) * dt_);

  QpProblem p;
  p.stages.resize(N);
  p.slack_weight = w.slack;
  p.slack_quadratic = w.slack_quadratic;

  auto init_stage = [&](StageQp & st, int nc) {
    st.Q = MatrixXd::Zero(kNz, kNz);
    st.q = VectorXd::Zero(kNz);
    st.Cx = MatrixXd::Zero(nc, kNz);
    st.lower = VectorXd::Constant(nc, -kInf);
    st.upper = VectorXd::Constant(nc, kInf);
    st.x_lower = VectorXd::Constant(kNz, -kInf);
    st.x_upper = VectorXd::Constant(kNz, kInf);
  };

  // State cost of stage k (1..N) and the joint limits.
  auto state_terms = [&](StageQp & st, int k) {
    const State & xb = nominal.x[k];
    const Vector2d pos = xb.head<2>();
    const Vector2d t = tangent(route, s[k]);
    const Vector2d n(-t.y(), t.x());
    const double path_heading = std::atan2(t.y(), t.x());
    MatrixXd J = MatrixXd::Zero(1, kNz);
    VectorXd r(1);

    // contouring
    J(0, 0) = n.x();
    J(0, 1) = n.y();
    r(0) = n.dot(pos - route.at(s[k]));
    add_state_residual(st, J, r, w.contour);
    // lag against the progress reference
    J(0, 0) = t.x();
    J(0, 1) = t.y();
    r(0) = t.dot(pos - route.at(s_ref[k]));
    add_state_residual(st, J, r, w.lag);
    // heading: the path tangent, fading out over the last kGoalHeadingM so it cannot hold the base
    // sideways next to the goal; the goal heading on the stages within kGoalHeadingM, once the
    // measured base is near the goal point (align): turning towards it earlier made the
    // forward-only base arc off the route and park next to the goal, facing away from it.
    J.setZero();
    J(0, kYaw) = 1.0;
    const double closeness = std::clamp(1.0 - (pos - goal_point).norm() / kGoalHeadingM, 0.0, 1.0);
    r(0) = wrap(xb(kYaw) - path_heading);
    add_state_residual(st, J, r, w.heading * (1.0 - closeness) * (1.0 - closeness));
    r(0) = wrap(xb(kYaw) - in.goal_yaw);
    add_state_residual(st, J, r, w.goal_heading * closeness * align);

    // panel reference with the largest weight at this stage's progress
    double ref_weight = 0.0;
    const PanelReference * ref = nullptr;
    for (const auto & candidate : in.references) {
      const double weight = reference_weight(candidate, s[k]);
      if (weight > ref_weight) {
        ref_weight = weight;
        ref = &candidate;
      }
    }
    if (ref != nullptr) {
      const PanelError pe = panel_error(xb, ref->panel_in_base);
      MatrixXd Jp = MatrixXd::Zero(3, kNz);
      Jp.leftCols(kNx) = pe.J.topRows<3>();
      add_state_residual(st, Jp, pe.e.head<3>(), ref_weight * w.panel_position);
      const OrientationError oe = orientation_error(xb, ref->panel_in_base.linear());
      MatrixXd Jo = MatrixXd::Zero(3, kNz);
      Jo.leftCols(kNx) = oe.J;
      add_state_residual(st, Jo, oe.e, ref_weight * w.panel_orientation);
    }
    // posture preference
    const double posture = posture_weight * (1.0 - ref_weight);
    if (posture > 0.0) {
      for (int i = 0; i < 6; ++i) {
        st.Q(kQ + i, kQ + i) += posture;
        st.q(kQ + i) += posture * (xb(kQ + i) - posture_target(i));
      }
    }
    // joint position limits
    for (int i = 0; i < 6; ++i) {
      st.x_lower(kQ + i) = limits_.q_min(i) - xb(kQ + i);
      st.x_upper(kQ + i) = limits_.q_max(i) - xb(kQ + i);
    }
  };

  // Collision rows of stage k into rows [first, first + Rc) of st, slacked; with a clearance
  // weight, the clearance cost of the obstacle rows below the target.
  auto collision_terms = [&](StageQp & st, int k, int first, int global_first) {
    for (int i = 0; i < Rc; ++i) {
      const int row = first + i;
      if (i < static_cast<int>(by_stage[k].size())) {
        const DistanceRow & d = *by_stage[k][i];
        st.Cx.block(row, 0, 1, kNx) = d.gradient;
        const double h = d.distance - d.margin;
        st.lower(row) = -h;
        if (w.clearance > 0.0 && d.distance < w.clearance_target_m && d.obstacle != "floor" &&
            d.obstacle.rfind("self/", 0) != 0) {
          // 0.5 w r^2, r = target - (d + g dx): Jacobian -g.
          st.Q.topLeftCorner<kNx, kNx>().noalias() += w.clearance * d.gradient.transpose() * d.gradient;
          st.q.head<kNx>().noalias() -= w.clearance * (w.clearance_target_m - d.distance) * d.gradient.transpose();
        }
      } else {
        st.lower(row) = kPadLower;
      }
      st.upper(row) = kInf;
      p.slack_weights.push_back(i < static_cast<int>(by_stage[k].size()) && w.slack_inside >= 0.0 &&
                                    by_stage[k][i]->distance < by_stage[k][i]->margin
                                  ? w.slack_inside
                                  : w.slack);
      p.slack_rows.push_back(global_first + i);
    }
  };

  int global_row = 0;
  for (int k = 0; k < N; ++k) {
    StageQp & st = p.stages[k];
    const int nc = kNu + (k == 0 ? 0 : Rc);
    init_stage(st, nc);
    const State & xb = nominal.x[k];
    const Control & ub = nominal.u[k];
    const Control & u_prev = k == 0 ? last : nominal.u[k - 1];

    if (k > 0) state_terms(st, k);

    // dynamics in z = (x, u_prev)
    const Step step = discrete_step(xb, ub, dt_);
    st.A = MatrixXd::Zero(kNz, kNz);
    st.A.topLeftCorner(kNx, kNx) = step.A;
    st.B = MatrixXd::Zero(kNz, kNu);
    st.B.topRows(kNx) = step.B;
    st.B.bottomRows(kNu).setIdentity();
    st.c = VectorXd::Zero(kNz);
    st.c.head(kNx) = step.x_next - nominal.x[k + 1];

    // control cost
    st.R = MatrixXd::Zero(kNu, kNu);
    st.r = VectorXd::Zero(kNu);
    st.S = MatrixXd::Zero(kNz, kNu);
    {
      const Vector2d t = tangent(route, s[k]);
      const double heading_error = wrap(xb(kYaw) - std::atan2(t.y(), t.x()));
      const double v_target = progress_speed(s[k]) * std::max(0.0, std::cos(heading_error));
      st.R(0, 0) += w.progress;
      st.r(0) += w.progress * (ub(0) - v_target);
    }
    for (int i = 0; i < kNu; ++i) {
      // the forward speed's magnitude is the progress term's; effort_base is on the yaw rate
      const double effort = i == 0 ? 0.0 : (i == 1 ? w.effort_base : w.effort_arm);
      const double smooth = i < 2 ? w.smooth_base : w.smooth_arm;
      st.R(i, i) += effort + smooth;
      st.r(i) += effort * ub(i);
      // smoothness 0.5 ws (u_k + du_k - u_prev - dprev)^2
      const double change = ub(i) - u_prev(i);
      st.r(i) += smooth * change;
      st.Q(kPrev + i, kPrev + i) += smooth;
      st.q(kPrev + i) -= smooth * change;
      st.S(kPrev + i, i) -= smooth;
    }

    // control boxes
    st.u_lower = u_lo - ub;
    st.u_upper = u_hi - ub;

    // rate limits: rate_lo <= (u_k + du_k) - (u_prev + dprev) <= rate_hi
    st.Cu = MatrixXd::Zero(nc, kNu);
    for (int i = 0; i < kNu; ++i) {
      st.Cx(i, kPrev + i) = -1.0;
      st.Cu(i, i) = 1.0;
      const double change = ub(i) - u_prev(i);
      st.lower(i) = rate_lo(i) - change;
      st.upper(i) = rate_hi(i) - change;
    }
    if (k > 0) collision_terms(st, k, kNu, global_row + kNu);
    global_row += nc;
  }
  StageQp & terminal = p.terminal;
  init_stage(terminal, Rc);
  state_terms(terminal, N);
  collision_terms(terminal, N, 0, global_row);
  p.slack_count = static_cast<int>(p.slack_rows.size());

  p.dx0 = VectorXd::Zero(kNz);
  p.dx0.head(kNx) = in.x0 - nominal.x[0];
  p.dx0(kYaw) = wrap(p.dx0(kYaw));
  return p;
}

Plan Formulation::rollout(const State & x0, const std::vector<Control> & u) const
{
  Plan plan;
  plan.u = u;
  plan.x.reserve(u.size() + 1);
  plan.x.push_back(x0);
  for (const auto & control : u) plan.x.push_back(discrete_step(plan.x.back(), control, dt_).x_next);
  return plan;
}

Plan Formulation::apply(const Plan & nominal, const QpSolution & s) const
{
  Plan plan = nominal;
  apply_in_place(plan, s);
  return plan;
}

void Formulation::apply_in_place(Plan & plan, const QpSolution & s) const
{
  const int N = horizon_;
  if (static_cast<int>(plan.u.size()) != N || static_cast<int>(plan.x.size()) != N + 1 ||
      static_cast<int>(s.du.size()) != N || s.dx.empty())
    throw std::invalid_argument("Formulation::apply: solution does not match the horizon");
  const Control u_lo = lower_control(limits_), u_hi = upper_control(limits_);
  const Control rate_lo = lower_rate(limits_, dt_), rate_hi = upper_rate(limits_, dt_);
  for (int k = 0; k < N; ++k) {
    Control uk = plan.u[k] + s.du[k].head(kNu);
    if (k > 0) uk = uk.cwiseMax(plan.u[k - 1] + rate_lo).cwiseMin(plan.u[k - 1] + rate_hi);
    plan.u[k] = uk.cwiseMax(u_lo).cwiseMin(u_hi);
  }
  plan.x[0] += s.dx[0].head(kNx);
  for (int k = 0; k < N; ++k) plan.x[k + 1] = discrete_step(plan.x[k], plan.u[k], dt_).x_next;
}

Plan Formulation::shift(const Plan & plan, const State & x0) const
{
  Plan out = plan;
  advance(out, x0, 1);
  return out;
}

void Formulation::advance(Plan & plan, const State & x0, int stages) const
{
  const int N = horizon_;
  if (static_cast<int>(plan.u.size()) != N || static_cast<int>(plan.x.size()) != N + 1)
    throw std::invalid_argument("Formulation::advance: plan does not match the horizon");
  const int n = std::clamp(stages, 0, N);
  for (int k = 0; k < N; ++k) plan.u[k] = plan.u[std::min(k + n, N - 1)];  // reads only indices >= k
  plan.x[0] = x0;
  for (int k = 0; k < N; ++k) plan.x[k + 1] = discrete_step(plan.x[k], plan.u[k], dt_).x_next;
}

Plan Formulation::hold(const State & x0) const
{
  Plan plan;
  plan.x.assign(horizon_ + 1, x0);
  plan.u.assign(horizon_, Control::Zero());
  return plan;
}
}  // namespace mobile_manipulator_mpc
