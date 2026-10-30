#include "mobile_manipulator_medium/planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "mobile_manipulator_mpc/model.hpp"

namespace mobile_manipulator_medium
{
namespace
{
using Clock = std::chrono::steady_clock;
double since(Clock::time_point t0) { return std::chrono::duration<double>(Clock::now() - t0).count(); }

constexpr double kFitRidge = 1e-6;
constexpr double kSampleStepS = 0.02;  // the dense time step the arc-length samples are taken from
constexpr double kSeedYawBlendS = 1.0;  // a pose seed turns from the measured yaw to the route's over this

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
// `a` moved by whole turns to within pi of `reference`.
double unwrap_to(double a, double reference) { return reference + wrap(a - reference); }
double smoothstep(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}
}  // namespace

MediumPlanner::MediumPlanner(MediumProblem problem, std::unique_ptr<DenseQpSolver> solver,
                             std::map<std::string, Joints> seed_poses, Eigen::Vector3d panel_half_size)
: problem_(std::move(problem)),
  solver_(std::move(solver)),
  seed_poses_(std::move(seed_poses)),
  panel_half_(panel_half_size),
  cap_s_(problem_.config().solve_cap_s)
{
  if (!solver_) throw std::invalid_argument("MediumPlanner: no QP solver");
  for (const auto & name : problem_.config().seeds)
    if (name != "previous" && !seed_poses_.count(name))
      throw std::invalid_argument("MediumPlanner: seed " + name + " is not a known pose");
  mmm::State comfort = mmm::State::Zero();
  comfort.tail<6>() = problem_.comfort();
  comfort_top_ = panel_top(comfort);
  // Least squares of the sample values in the free parameters (the same for every coordinate).
  const int K = problem_.samples(), nf = problem_.basis().free_parameters();
  Eigen::MatrixXd SM(K, nf);
  for (int k = 0; k < K; ++k) SM.row(k) = problem_.basis().basis(problem_.sample_time(k), 0) * problem_.basis().map();
  const Eigen::MatrixXd normal = SM.transpose() * SM + kFitRidge * Eigen::MatrixXd::Identity(nf, nf);
  fit_ = normal.ldlt().solve(SM.transpose());
}

void MediumPlanner::set_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                              uint64_t route_id)
{
  route_.emplace(route);
  goal_ = goal;
  route_id_ = route_id;
  new_route_ = true;
  s0_hint_ = std::numeric_limits<double>::quiet_NaN();
  last_.reset();
  incumbent_.clear();
}

double MediumPlanner::panel_top(const mmm::State & x) const
{
  const Eigen::Isometry3d panel = mmm::link_pose(mmm::link_index("panel"), x);
  double top = panel.translation().z();
  for (int i = 0; i < 3; ++i) top += std::abs(panel.linear()(2, i)) * panel_half_(i);
  return top;
}

std::string MediumPlanner::homotopy_label(const Trajectory & t) const
{
  double pan_max = 0.0, pan_min = 0.0, lowest = 1e9;
  for (int k = 0; k < problem_.samples(); ++k) {
    const mmm::State x = problem_.state_at(t, k);
    pan_max = std::max(pan_max, x(3));
    pan_min = std::min(pan_min, x(3));
    lowest = std::min(lowest, panel_top(x));
  }
  std::string label;
  if (pan_max > kPanSideRad || pan_min < -kPanSideRad) label = pan_max >= -pan_min ? "left" : "right";
  if (lowest < comfort_top_ - kLoweredM) label += label.empty() ? "lowered" : "+lowered";
  return label.empty() ? "comfort" : label;
}

Trajectory MediumPlanner::fit(const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate,
                              const Eigen::MatrixXd & targets) const
{
  const int C = problem_.coordinates(), K = problem_.samples();
  // The start line (zero parameters), then the parameters that bring each coordinate closest to its targets.
  const Trajectory line = problem_.from_parameters(start, rate, Eigen::MatrixXd::Zero(C, problem_.basis().free_parameters()));
  Eigen::MatrixXd params(C, problem_.basis().free_parameters());
  for (int c = 0; c < C; ++c) {
    Eigen::VectorXd residual(K);
    for (int k = 0; k < K; ++k)
      residual(k) = targets(k, c) - problem_.basis().basis(problem_.sample_time(k), 0).dot(line.control_points.row(c).transpose());
    params.row(c) = (fit_ * residual).transpose();
  }
  return problem_.from_parameters(start, rate, params);
}

Trajectory MediumPlanner::pose_seed(const Joints & pose, const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate,
                                    const RouteTask & task) const
{
  const int C = problem_.coordinates(), K = problem_.samples();
  const double T = problem_.basis().horizon_s();
  const double v = problem_.config().speed_fraction * problem_.limits().v_max;
  Eigen::MatrixXd targets(K, C);
  for (int k = 0; k < K; ++k) {
    const double t = problem_.sample_time(k);
    const double s = std::min(task.goal_s, task.s0 + v * t);
    const Eigen::Vector2d p = task.route->at(s);
    // the base: from the measured position onto the route, at the reference speed, turning to the route's heading
    const double blend = smoothstep(t / kSeedYawBlendS);
    targets(k, 0) = (1.0 - blend) * start(0) + blend * p.x();
    targets(k, 1) = (1.0 - blend) * start(1) + blend * p.y();
    const double route_yaw = unwrap_to(task.route->heading_at(s), start(2));
    targets(k, 2) = start(2) + blend * (route_yaw - start(2));
    // the masked joints: to the pose over the first half of the horizon, then held
    const double u = smoothstep(t / (0.5 * T));
    for (size_t m = 0; m < problem_.joints().size(); ++m) {
      const int j = problem_.joints()[m];
      targets(k, 3 + static_cast<int>(m)) = start(3 + j) + u * (pose(j) - start(3 + j));
    }
  }
  return fit(start, rate, targets);
}

Trajectory MediumPlanner::shifted_previous(double now, const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate) const
{
  const int C = problem_.coordinates(), K = problem_.samples();
  const double T = problem_.basis().horizon_s();
  const double shift = std::max(0.0, now - last_now_);
  // The published curve at t + shift (held at its end beyond the horizon), its yaw unwrapped to the measured yaw.
  const Eigen::VectorXd yaw_then = last_->control_points.row(2) * problem_.basis().basis(std::min(shift, T), 0).transpose();
  const double turns = start(2) - unwrap_to(yaw_then(0), start(2));
  Eigen::MatrixXd targets(K, C);
  for (int k = 0; k < K; ++k) {
    const Eigen::VectorXd values =
      last_->control_points * problem_.basis().basis(std::min(problem_.sample_time(k) + shift, T), 0).transpose();
    targets.row(k) = values.transpose();
    targets(k, 2) += turns;
  }
  return fit(start, rate, targets);
}

std::vector<PlanSample> MediumPlanner::samples_of(const Trajectory & t) const
{
  const double T = problem_.basis().horizon_s();
  const auto & cfg = problem_.config();
  std::vector<PlanSample> out;
  double s = 0.0, s_last = 0.0, yaw_last = 0.0;
  Eigen::Vector2d p_prev;
  auto sample_at = [&](double time, double arc) {
    const Eigen::VectorXd value = t.control_points * problem_.basis().basis(time, 0).transpose();
    const Eigen::VectorXd rate = t.control_points * problem_.basis().basis(time, 1).transpose();
    PlanSample ps;
    ps.s = arc;
    ps.x = value(0);
    ps.y = value(1);
    ps.yaw = value(2);
    ps.speed = rate.head<2>().norm();
    ps.q = problem_.comfort();
    for (size_t m = 0; m < problem_.joints().size(); ++m) ps.q(problem_.joints()[m]) = value(3 + static_cast<int>(m));
    return ps;
  };
  const int steps = static_cast<int>(std::ceil(T / kSampleStepS));
  for (int i = 0; i <= steps; ++i) {
    const double time = std::min(T, i * kSampleStepS);
    const Eigen::Vector2d p = (t.control_points.topRows(2) * problem_.basis().basis(time, 0).transpose());
    if (i > 0) s += (p - p_prev).norm();
    p_prev = p;
    const PlanSample ps = sample_at(time, s);
    if (out.empty() || s - s_last >= cfg.sample_ds || std::abs(ps.yaw - yaw_last) >= cfg.sample_dyaw || i == steps) {
      if (i == steps && !out.empty() && s - s_last < 1e-9 && std::abs(ps.yaw - yaw_last) < 1e-9) break;  // nothing new
      out.push_back(ps);
      s_last = s;
      yaw_last = ps.yaw;
    }
  }
  return out;
}

const PlanOutput & MediumPlanner::plan(double now, const mmm::State & measured, const Eigen::Matrix<double, 9, 1> & rate,
                                       const mmm::World & world)
{
  if (!route_) throw std::logic_error("MediumPlanner::plan: no route");
  const auto t0 = Clock::now();
  const auto deadline = t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cap_s_));
  const auto & cfg = problem_.config();

  RouteTask task;
  task.route = &*route_;
  task.goal_s = route_->length();
  task.goal_yaw = goal_[2];
  task.s0 = std::isnan(s0_hint_) ? route_->project_range(measured.head<2>(), 0.0, route_->length()).s
                                 : route_->project(measured.head<2>(), s0_hint_, 1.0).s;
  s0_hint_ = task.s0;

  const bool full = !last_ || new_route_ || full_requested_ || now - last_full_ >= cfg.full_multistart_period_s;
  // The incumbent first: the published plan carried on, or, with none, the measured joints held.
  std::vector<std::pair<std::string, Trajectory>> seeds;
  if (last_) seeds.emplace_back("previous", shifted_previous(now, measured, rate));
  else seeds.emplace_back("hold", pose_seed(measured.tail<6>(), measured, rate, task));
  if (full)
    for (const auto & name : cfg.seeds)
      if (name != "previous") seeds.emplace_back(name, pose_seed(seed_poses_.at(name), measured, rate, task));

  PlanOutput out;
  out.route_id = route_id_;
  out.plan_id = ++plan_id_;
  out.full_multistart = full;
  std::vector<ScpResult> results;
  std::vector<int> kept;  // indices into results of the distinct solutions
  for (auto & [name, seed] : seeds) {
    if (Clock::now() >= deadline) {
      out.capped = true;
      break;
    }
    ScpResult r = solve_scp(problem_, *solver_, std::move(seed), task, world, now, deadline);
    out.linearize_s += r.linearize_s;
    out.qp_s += r.qp_s;
    SeedStats st;
    st.name = name;
    st.homotopy = homotopy_label(r.traj);
    st.merit = r.merit;
    st.slack = r.violations.soft();
    st.solve_s = r.solve_s;
    st.iterations = r.iterations;
    st.converged = r.converged;
    st.feasible = r.feasible;
    for (const int i : kept)
      if ((results[i].traj.control_points - r.traj.control_points).lpNorm<Eigen::Infinity>() < kMergeTolerance) {
        st.merged = true;
        if (r.merit < results[i].merit) {
          results[i] = r;  // the same solution, found cheaper
          out.seeds[i].merit = st.merit;
        }
        break;
      }
    if (!st.merged) kept.push_back(static_cast<int>(results.size()));
    results.push_back(std::move(r));
    out.seeds.push_back(st);
    if (Clock::now() >= deadline) out.capped = true;
  }

  // A capped cycle that solved nothing (the incumbent unfinished): the last plan again.
  const bool solved = !results.empty() && results[0].iterations > 0;
  if (!solved && !out_.samples.empty()) {
    const auto keep = std::move(out);
    out_.status = "stale_reused";
    out_.plan_id = keep.plan_id;
    out_.capped = true;
    out_.seeds = keep.seeds;
    out_.solve_s = since(t0);
    return out_;
  }

  // The cheapest feasible distinct solution, else the one with the least slack; then the homotopy hysteresis.
  int best = -1;
  for (const int i : kept)
    if (results[i].feasible && (best < 0 || results[i].merit < results[best].merit)) best = i;
  bool any_feasible = best >= 0;
  if (!any_feasible)
    for (const int i : kept)
      if (best < 0 || results[i].violations.soft() < results[best].violations.soft()) best = i;
  std::string label = out.seeds[best].homotopy;
  if (!incumbent_.empty() && label != incumbent_ && any_feasible) {
    int incumbent = -1;
    for (const int i : kept)
      if (results[i].feasible && out.seeds[i].homotopy == incumbent_ &&
          (incumbent < 0 || results[i].merit < results[incumbent].merit))
        incumbent = i;
    if (incumbent >= 0 && results[best].merit > (1.0 - cfg.homotopy_hysteresis) * results[incumbent].merit) {
      best = incumbent;
      label = incumbent_;
    }
  }
  out.homotopy_switched = !incumbent_.empty() && label != incumbent_;
  out.status = any_feasible ? "ok" : "infeasible";
  out.seed = out.seeds[best].name;
  out.homotopy = label;
  out.cost = results[best].merit;
  out.samples = samples_of(results[best].traj);
  const Eigen::Vector2d end(out.samples.back().x, out.samples.back().y);
  out.splice_s = route_->project_range(end, task.s0, route_->length()).s;

  last_ = results[best].traj;
  last_now_ = now;
  incumbent_ = label;
  new_route_ = false;
  full_requested_ = false;
  if (full) last_full_ = now;
  out.solve_s = since(t0);
  out_ = std::move(out);
  return out_;
}
}  // namespace mobile_manipulator_medium
