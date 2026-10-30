#include "mobile_manipulator_medium/planner.hpp"

#include <algorithm>
#include <cmath>
#include <set>
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
// The swing side of a homotopy label ("left", "right" or "").
std::string side_of(const std::string & label)
{
  if (label.rfind("left", 0) == 0) return "left";
  if (label.rfind("right", 0) == 0) return "right";
  return "";
}
double smoothstep(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}
}  // namespace

std::vector<Eigen::Vector3d> paced_route(const mmm::Route & route, double s0, double goal_s, const mmm::State & start,
                                         double v_start, double w_start, const std::function<double(double)> & v_ref,
                                         const mmm::Limits & limits, const std::vector<double> & times)
{
  constexpr double kStep = 0.01;      // integration step (s)
  constexpr double kYawGain = 2.0;    // yaw rate per heading error (1/s)
  constexpr double kScanStep = 0.05;  // corner look-ahead resolution (m)
  constexpr double kLookM = 0.1;      // the yaw steers by the heading this far ahead (so a stop at a corner turns)
  goal_s = std::clamp(goal_s, s0, route.length());
  // from the measured speed and turn rate (forward only); the limits below bring them to the pace
  double s = std::min(s0, goal_s), yaw = start(2), v = std::max(0.0, v_start), w = w_start, t = 0.0;
  // the start's offset from the route, faded out over the first second (the route's speed is kept)
  const Eigen::Vector2d offset = start.head<2>() - route.at(s);
  std::vector<Eigen::Vector3d> out;
  out.reserve(times.size());
  auto pose = [&] {
    const Eigen::Vector2d p = route.at(s) + (1.0 - smoothstep(t / kSeedYawBlendS)) * offset;
    return Eigen::Vector3d(p.x(), p.y(), yaw);
  };
  for (const double target : times) {
    while (t + 0.5 * kStep < target) {
      const double error = wrap(route.heading_at(std::min(goal_s, s + kLookM)) - yaw);
      // yaw: towards the heading, the rate within omega_max and changed within yaw_accel
      const double w_want = std::clamp(kYawGain * error, -limits.omega_max, limits.omega_max);
      w += std::clamp(w_want - w, -limits.yaw_accel * kStep, limits.yaw_accel * kStep);
      // speed: falls with the heading error; brakes for the next corner beyond kTurnInPlaceRad and for the goal
      double v_allowed = v_ref(s) * std::clamp(1.0 - std::abs(error) / kTurnInPlaceRad, 0.0, 1.0);
      // (stopping where the look-ahead point has just passed the corner, so the heading there is the next leg's)
      const double reach = v * v / (2.0 * limits.decel) + kLookM + 2.0 * kScanStep;
      for (double d = kLookM + kScanStep; d <= reach && s + d <= goal_s; d += kScanStep)
        if (std::abs(wrap(route.heading_at(s + d) - yaw)) > kTurnInPlaceRad) {
          v_allowed = std::min(v_allowed, std::sqrt(2.0 * limits.decel * (d - kLookM)));
          break;
        }
      v_allowed = std::min(v_allowed, std::sqrt(2.0 * limits.decel * std::max(0.0, goal_s - s)));
      v += std::clamp(v_allowed - v, -limits.decel * kStep, limits.accel * kStep);
      yaw += w * kStep;
      s = std::min(goal_s, s + v * kStep);
      t += kStep;
    }
    out.push_back(pose());
  }
  return out;
}

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
    if (name != "previous") seed_pose(name);  // throws for a seed that is not a known pose (or one offset)
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
  const bool same_goal = route_ && std::abs(goal[0] - goal_[0]) < 1e-6 && std::abs(goal[1] - goal_[1]) < 1e-6 &&
                         std::abs(wrap(goal[2] - goal_[2])) < 1e-6;
  route_.emplace(route);
  goal_ = goal;
  route_id_ = route_id;
  new_route_ = true;
  s0_hint_ = std::numeric_limits<double>::quiet_NaN();
  if (!same_goal) {
    last_.reset();
    incumbent_.clear();
    out_ = PlanOutput{};
  }
}

Joints MediumPlanner::seed_pose(const std::string & name) const
{
  const size_t plus = name.find('+');
  const auto base = seed_poses_.find(name.substr(0, plus));
  if (base == seed_poses_.end()) throw std::invalid_argument("MediumPlanner: seed " + name + " is not a known pose");
  Joints q = base->second;
  for (size_t at = plus; at != std::string::npos;) {
    const size_t next = name.find('+', at + 1);
    const std::string part = name.substr(at + 1, next == std::string::npos ? std::string::npos : next - at - 1);
    const size_t colon = part.find(':');
    if (colon == std::string::npos) throw std::invalid_argument("MediumPlanner: seed " + name + ": joint:offset expected");
    const std::string joint = part.substr(0, colon), value = part.substr(colon + 1);
    const auto it = std::find(mmm::kArmJoints.begin(), mmm::kArmJoints.end(), joint);
    if (it == mmm::kArmJoints.end()) throw std::invalid_argument("MediumPlanner: seed " + name + ": unknown joint " + joint);
    const int j = static_cast<int>(it - mmm::kArmJoints.begin());
    const auto & planned = problem_.joints();
    if (std::find(planned.begin(), planned.end(), j) == planned.end())
      throw std::invalid_argument("MediumPlanner: seed " + name + ": " + joint + " is not a planned joint");
    size_t used = 0;
    double offset = 0.0;
    try {
      offset = std::stod(value, &used);
    } catch (const std::exception &) {
      used = 0;
    }
    if (used == 0 || used != value.size()) throw std::invalid_argument("MediumPlanner: seed " + name + ": bad offset " + value);
    q(j) += offset;
    at = next;
  }
  return q;
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
  std::vector<double> times(K);
  for (int k = 0; k < K; ++k) times[k] = problem_.sample_time(k);
  // the base: along the route as the skid-steer drives it (paced_route)
  const double v_start = rate(0) * std::cos(start(2)) + rate(1) * std::sin(start(2));
  const auto base = paced_route(*task.route, task.s0, task.goal_s, start, v_start, rate(2),
                                [&](double s) { return v * problem_.speed_scale(task, s); }, problem_.limits(), times);
  Eigen::MatrixXd targets(K, C);
  for (int k = 0; k < K; ++k) {
    targets.row(k).head<3>() = base[static_cast<size_t>(k)].transpose();
    // the masked joints: to the pose over the first half of the horizon, then held
    const double u = smoothstep(times[k] / (0.5 * T));
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
  // whole turns between the published yaw and the measured one (the measured yaw is wrapped to (-pi, pi])
  const double turns = unwrap_to(yaw_then(0), start(2)) - yaw_then(0);
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
  problem_.schedule_speed(task, measured, world, now);

  const bool periodic = cfg.seeds_per_cycle == 0 && now - last_full_ >= cfg.full_multistart_period_s;
  // A route update for the same goal keeps the incumbent (set_route); with the pose seeds in turn it needs no full
  // multi-start (B5 replans often: in the store room 48 updates in 683 cycles, each 0.18 s of full multi-start).
  const bool full = !last_ || (new_route_ && cfg.seeds_per_cycle == 0) || full_requested_ || periodic;
  // The incumbent first: the published plan carried on, or, with none, the measured joints held.
  std::vector<std::pair<std::string, Trajectory>> seeds;
  if (last_) seeds.emplace_back("previous", shifted_previous(now, measured, rate));
  else seeds.emplace_back("hold", pose_seed(measured.tail<6>(), measured, rate, task));
  const bool last_ok = out_.status == "ok";  // the published plan before this cycle's
  std::vector<std::string> poses;
  for (const auto & name : cfg.seeds)
    if (name != "previous") poses.push_back(name);
  if (full) {
    for (const auto & name : poses) seeds.emplace_back(name, pose_seed(seed_pose(name), measured, rate, task));
    rotation_ = 0;
  } else {
    for (int i = 0; i < cfg.seeds_per_cycle && i < static_cast<int>(poses.size()); ++i) {
      const std::string & name = poses[rotation_++ % poses.size()];
      seeds.emplace_back(name, pose_seed(seed_pose(name), measured, rate, task));
    }
  }

  PlanOutput out;
  out.route_id = route_id_;
  out.plan_id = ++plan_id_;
  out.full_multistart = full;
  std::vector<ScpResult> results;
  std::vector<int> kept;  // indices into results of the distinct solutions
  std::set<std::string> completed;  // the seeds whose SCP the cap did not cut
  for (size_t index = 0; index < seeds.size(); ++index) {
    const std::string name = seeds[index].first;
    Trajectory seed = std::move(seeds[index].second);
    if (!results.empty() && Clock::now() >= deadline) {  // the incumbent is always attempted
      out.capped = true;
      break;
    }
    ScpResult r = solve_scp(problem_, *solver_, std::move(seed), task, world, now, deadline);
    out.linearize_s += r.linearize_s;
    out.qp_s += r.qp_s;
    out.merit_s += r.merit_s;
    out.linearize_parts.collision_s += r.linearize_parts.collision_s;
    out.linearize_parts.torque_s += r.linearize_parts.torque_s;
    out.linearize_parts.cost_s += r.linearize_parts.cost_s;
    out.linearize_parts.assemble_s += r.linearize_parts.assemble_s;
    out.linearize_parts.total_s += r.linearize_parts.total_s;
    SeedStats st;
    st.name = name;
    st.homotopy = homotopy_label(r.traj);
    st.merit = r.merit;
    st.slack = r.violations.soft();
    st.solve_s = r.solve_s;
    st.iterations = r.iterations;
    st.converged = r.converged;
    st.feasible = r.feasible;
    const bool usable = !r.interrupted;  // a seed the cap cut short is dropped this cycle (spec 3.5)
    if (usable) completed.insert(name);
    for (const int i : kept)
      if (usable && (results[i].traj.control_points - r.traj.control_points).lpNorm<Eigen::Infinity>() < kMergeTolerance) {
        st.merged = true;
        // the same solution: keep the better one (feasible first, then cheaper), with its own statistics
        if ((r.feasible && !results[i].feasible) || (r.feasible == results[i].feasible && r.merit < results[i].merit)) {
          // the better one takes the slot (its result and statistics); the replaced seed is listed as merged
          std::swap(results[i], r);
          std::swap(out.seeds[i], st);
          out.seeds[i].merged = false;
          st.merged = true;
        }
        break;
      }
    if (!st.merged && usable) kept.push_back(static_cast<int>(results.size()));
    results.push_back(std::move(r));
    out.seeds.push_back(st);
    if (Clock::now() >= deadline) out.capped = true;
    // An incumbent that turns infeasible on a cycle of seeds in turn (the last plan was ok): every other pose seed too
    // (one seed in turn may be the wrong side at a gate), within the cap; while it stays infeasible the seeds go on in
    // turn (a start inside a margin: every cycle would be a full multi-start); the rotation goes on.
    if (index == 0 && !full && last_ok && !results[0].feasible)
      for (const auto & other : poses)
        if (std::none_of(seeds.begin(), seeds.end(), [&](const auto & s) { return s.first == other; }))
          seeds.emplace_back(other, pose_seed(seed_pose(other), measured, rate, task));
  }

  // A cycle whose incumbent the cap cut short: the last valid plan again, with its own planning time; with none, the
  // incumbent seed as it is.
  const bool solved = !results.empty() && results[0].iterations > 0 && !results[0].interrupted;
  if (!solved) {
    if (!out_.samples.empty()) {
      out_.status = "stale_reused";
      out_.route_id = route_id_;
      out_.plan_id = out.plan_id;
      out_.capped = true;
      out_.full_multistart = full;
      out_.seeds = out.seeds;
      out_.linearize_s = out.linearize_s;
      out_.qp_s = out.qp_s;
      out_.homotopy_switched = false;
      out_.solve_s = since(t0);
      return out_;
    }
    out.status = "infeasible";
    out.capped = true;
    out.seed = seeds.front().first;
    const Trajectory & unsolved = results.empty() ? seeds.front().second : results.front().traj;
    out.homotopy = homotopy_label(unsolved);
    out.planned_at = now;
    out.samples = samples_of(unsolved);
    const Eigen::Vector2d end(out.samples.back().x, out.samples.back().y);
    out.splice_s = route_->project_range(end, task.s0, route_->length()).s;
    out.solve_s = since(t0);
    out_ = std::move(out);
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
  out.homotopy_switched = !side_of(incumbent_).empty() && !side_of(label).empty() && side_of(label) != side_of(incumbent_);
  out.status = any_feasible ? "ok" : "infeasible";
  out.seed = out.seeds[best].name;
  out.homotopy = label;
  out.cost = results[best].merit;
  out.samples = samples_of(results[best].traj);
  const Eigen::Vector2d end(out.samples.back().x, out.samples.back().y);
  out.splice_s = route_->project_range(end, task.s0, route_->length()).s;

  out.planned_at = now;
  last_ = results[best].traj;
  last_now_ = now;
  incumbent_ = label;
  if (!out.capped) {
    new_route_ = false;
    full_requested_ = false;
    if (full) last_full_ = now;
  } else if (full && cfg.seeds_per_cycle > 0) {
    // Seeds in turn: a capped full multi-start is not retried (a cap it always meets made every cycle one, Task 11
    // review); the seeds in turn go on from the first pose seed it did not complete. (seeds_per_cycle 0: retried.)
    new_route_ = false;
    full_requested_ = false;
    last_full_ = now;
    rotation_ = 0;
    while (rotation_ < poses.size() && completed.count(poses[rotation_])) ++rotation_;
  }
  out.solve_s = since(t0);
  out_ = std::move(out);
  return out_;
}
}  // namespace mobile_manipulator_medium
