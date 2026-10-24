#include "mobile_manipulator_mpc/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mobile_manipulator_mpc
{
namespace
{
using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// `value` moved towards zero by at most `step`, and zero inside the dead zone.
double towards_zero(double value, double step, double dead)
{
  const double next = value > 0.0 ? std::max(0.0, value - step) : std::min(0.0, value + step);
  return std::abs(next) < dead ? 0.0 : next;
}
}  // namespace

Control breakaway_output(const Control & planned, bool base_at_rest, const Breakaway & b)
{
  Control out = planned;
  if (std::abs(out(0)) < b.dead_linear_mps) out(0) = 0.0;
  if (std::abs(out(1)) < b.dead_yaw_radps) out(1) = 0.0;
  if (!base_at_rest) return out;
  // the larger base component relative to its breakaway
  const double linear = std::abs(out(0)) / b.linear_mps, yaw = std::abs(out(1)) / b.yaw_radps;
  const double ratio = std::max(linear, yaw);
  if (!(ratio > 0.0 && ratio < 1.0)) return out;
  // The dominant component is set to its breakaway exactly (dividing it by the ratio can land one
  // ULP below), the other is scaled by the same factor.
  if (linear >= yaw) {
    out(1) /= ratio;
    out(0) = std::copysign(b.linear_mps, out(0));
  } else {
    out(0) /= ratio;
    out(1) = std::copysign(b.yaw_radps, out(1));
  }
  return out;
}

bool base_at_rest(double linear_mps, double yaw_radps, const Breakaway & b)
{
  return std::abs(linear_mps) < b.rest_linear_mps && std::abs(yaw_radps) < b.rest_yaw_radps;
}

const char * to_string(CycleStatus status)
{
  switch (status) {
    case CycleStatus::Ok:
      return "ok";
    case CycleStatus::Fallback:
      return "fallback";
    case CycleStatus::Braking:
      return "braking";
    case CycleStatus::Failed:
      return "failed";
    case CycleStatus::GoalReached:
      return "goal_reached";
    case CycleStatus::StaleState:
      return "stale_state";
    case CycleStatus::Holding:
      return "holding";
    case CycleStatus::Blocked:
      return "blocked";
  }
  return "unknown";
}

ControllerOptions controller_options(const MpcConfig & config, double period_s)
{
  ControllerOptions o;
  o.qp_iterations = config.qp_iterations;
  o.iter2_budget = config.iter2_budget;
  o.iter2_cost_tolerance = config.iter2_cost_tolerance;
  o.warm_start = config.solver.warm_start;
  o.deadline_cap = config.solver.deadline_cap;
  o.period_s = period_s;
  o.publish_margin_s = config.solver.publish_margin_s;
  o.rti_split = config.rti_split;
  o.rti_max_deviation = config.rti_max_deviation;
  return o;
}

Controller::Controller(Formulation formulation, CollisionModel collision, std::unique_ptr<QpSolver> solver,
                       Breakaway breakaway, int fallback_cycles, double goal_xy_m, double goal_yaw_rad,
                       SlackGate slack_gate, ControllerOptions options)
: formulation_(std::move(formulation)),
  collision_(std::move(collision)),
  solver_(std::move(solver)),
  breakaway_(breakaway),
  fallback_cycles_(fallback_cycles),
  goal_xy_m_(goal_xy_m),
  goal_yaw_rad_(goal_yaw_rad),
  gate_(slack_gate),
  options_(options),
  tightness_(kTightComfortM, collision_.config().margin_m, kTightTauS)
{
  if (!solver_) throw std::invalid_argument("Controller: no QP solver");
  if (formulation_.rows_per_stage() != row_budget(collision_.config()))
    throw std::invalid_argument("Controller: formulation and collision model use different row budgets");
  if (fallback_cycles_ < 0) throw std::invalid_argument("Controller: negative fallback cycle count");
  if (options_.deadline_cap && options_.period_s > 0.0 && !(options_.publish_margin_s < options_.period_s))
    throw std::invalid_argument("Controller: the deadline cap needs a publish margin below the period");
  if (!(goal_xy_m_ > 0.0) || !(goal_yaw_rad_ > 0.0)) throw std::invalid_argument("Controller: bad goal tolerance");
  if (!(gate_.horizon_s > 0.0) || !(gate_.resume_horizon_s > 0.0) || !(gate_.tolerance_m >= 0.0) ||
      !(gate_.resume_dwell_s >= 0.0))
    throw std::invalid_argument("Controller: bad slack gate");
  // Buffers of the horizon's size, reused every cycle.
  nominal_ = formulation_.hold(State::Zero());
  out_.plan = nominal_;
}

long cycles_in(double seconds, double rate_hz)
{
  if (!(seconds > 0.0) || !(rate_hz > 0.0)) throw std::invalid_argument("cycles_in: non-positive time or rate");
  // The tolerance keeps exact products (0.3 s * 50 Hz = 15.000000000000002) from rounding up.
  return static_cast<long>(std::ceil(seconds * rate_hz - 1e-9));
}

int fallback_cycles_for(double seconds, double rate_hz) { return static_cast<int>(cycles_in(seconds, rate_hz)); }

void Controller::start(const Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                       std::vector<PanelReference> refs, PostureTargets posture)
{
  discard_preparation();
  set_goal(route, goal, std::move(refs));
  inputs_.posture = std::move(posture);
  mode_ = Mode::Tracking;
  fresh_plan_ = true;
  failures_ = 0;
  was_holding_ = false;
  resume_since_ = std::numeric_limits<double>::quiet_NaN();
  have_warm_ = false;
}

Eigen::Vector2d brake_base(const Eigen::Vector2d & from, double elapsed_s, const Limits & l, const Breakaway & b)
{
  return {towards_zero(from(0), l.decel * elapsed_s, b.dead_linear_mps),
          towards_zero(from(1), l.yaw_decel * elapsed_s, b.dead_yaw_radps)};
}

ArmHold safe_arm_hold(const CollisionModel & collision, const World & world, double now, const State & x,
                      const Eigen::Matrix<double, 6, 1> & v0, const Limits & l, const Eigen::Vector2d & base_vw)
{
  using Joints = Eigen::Matrix<double, 6, 1>;
  const Joints q0 = x.tail<6>();
  const double full = std::max(kArmHoldMinS, (v0.cwiseAbs().array() / l.qdd_max.array()).maxCoeff());
  // The base braking from base_vw: its pose t seconds on (integrated in 10 ms steps) and when it is at rest.
  constexpr double kStep = 0.01;
  const double base_stop = std::max(std::abs(base_vw(0)) / l.decel, std::abs(base_vw(1)) / l.yaw_decel);
  auto base_at = [&](double t) {
    State b = x;
    Eigen::Vector2d vw = base_vw;
    for (double s = 0.0; s < t - 1e-12 && !vw.isZero(); s += kStep) {
      const double h = std::min(kStep, t - s);
      Control u = Control::Zero();
      u.head<2>() = vw;
      b = discrete_step(b, u, h).x_next;
      vw = brake_base(vw, h, l, l.breakaway);
    }
    return b;
  };
  auto clearance = [&](const State & b, const Joints & q) { return collision.clearance_with_joints({b}, q, 0.1, now, world); };
  // The base's own effect: the arm frozen at q0 along the base's braking path.
  double frozen = clearance(x, q0);
  for (int i = 1; i <= kHoldSamples; ++i) frozen = std::min(frozen, clearance(base_at(base_stop * i / kHoldSamples), q0));
  const double required = std::min(frozen, collision.config().margin_m) - kHoldTolM;
  // A stop over `duration`: joints at fraction s of the time by `position(s)`, ending at `target`.
  auto worst_along = [&](double duration, const auto & position, const Joints & target) {
    double worst = std::numeric_limits<double>::infinity();
    for (int i = 1; i <= kHoldSamples; ++i) {
      const double s = position.uniform ? static_cast<double>(i) / kHoldSamples
                                        : 1.0 - std::sqrt(1.0 - static_cast<double>(i) / kHoldSamples);  // even displacement
      worst = std::min(worst, clearance(base_at(s * duration), position(s)));
    }
    return std::min(worst, clearance(base_at(std::max(duration, base_stop)), target));
  };
  struct Parabola
  {
    const Joints & q0, & v0;
    double duration;
    Joints lo, hi;
    bool uniform = false;
    Joints operator()(double s) const { return (q0 + v0 * duration * (s - 0.5 * s * s)).cwiseMax(lo).cwiseMin(hi); }
  };
  if (v0.isZero()) return {q0, kArmHoldMinS, 1.0, std::min(frozen, clearance(base_at(base_stop), q0))};
  for (const double k : {1.0, kHoldFaster[0], kHoldFaster[1], kHoldFaster[2]}) {
    const double duration = full / k;
    const Joints target = (q0 + 0.5 * duration * v0).cwiseMax(l.q_min).cwiseMin(l.q_max);
    const double worst = worst_along(duration, Parabola{q0, v0, duration, l.q_min, l.q_max}, target);
    if (worst >= required) return {target, duration, k, worst};
  }
  struct Overshoot  // the Hermite from (q0, v0) to (q0, 0) over kHoldEmergencyS
  {
    const Joints & q0, & v0;
    bool uniform = true;
    Joints operator()(double s) const { return q0 + v0 * kHoldEmergencyS * s * (1.0 - s) * (1.0 - s); }
  };
  return {q0, kHoldEmergencyS, std::numeric_limits<double>::infinity(),
          worst_along(kHoldEmergencyS, Overshoot{q0, v0}, q0)};
}

void Controller::reset_command(const Control & sent) { last_command_ = sent; }

void Controller::note_failure()
{
  if (mode_ != Mode::Tracking) return;
  // At most one failure per cycle: a step whose solve already failed is not counted again.
  failures_ = std::max(failures_, failures_at_step_ + 1);
  if (failures_ > fallback_cycles_) mode_ = Mode::Braking;
}

void Controller::update_goal(const Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                             std::vector<PanelReference> refs)
{
  discard_preparation();
  set_goal(route, goal, std::move(refs));
}

void Controller::set_goal(const Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                          std::vector<PanelReference> refs)
{
  route_.emplace(route);
  goal_xy_ = Eigen::Vector2d(goal[0], goal[1]);
  inputs_.route = &*route_;
  inputs_.s0_hint = std::numeric_limits<double>::quiet_NaN();  // the first cycle searches the whole route
  // A FollowPath path ends at its goal: project it onto the route's last metre (a route that
  // passes near its own goal earlier is not cut short).
  inputs_.goal_s = route_->project(goal_xy_, route_->length(), 1.0).s;
  inputs_.goal_yaw = goal[2];
  inputs_.references = std::move(refs);
}

double worst_violation(const std::vector<DistanceRow> & rows, int last_stage)
{
  double worst = 0.0;
  for (const DistanceRow & row : rows)
    if (row.stage >= 1 && row.stage <= last_stage) worst = std::max(worst, row.margin - row.distance);
  return worst;
}

bool accept_second_iterate(const Iter2Measure & first, const Iter2Measure & second, double cost_tolerance)
{
  return second.near <= first.near + kIter2ViolationTolM && second.horizon <= first.horizon + kIter2ViolationTolM &&
         second.cost <= (1.0 + cost_tolerance) * first.cost + 1e-12;
}

double Controller::linearization_error(const std::vector<DistanceRow> & rows, const QpSolution & solution,
                                       const World & world, double now, bool midstage) const
{
  const int last = static_cast<int>(std::ceil(gate_.horizon_s / formulation_.dt() - 1e-9));
  double worst = std::numeric_limits<double>::quiet_NaN();
  for (const DistanceRow & row : rows) {
    if (row.midstage != midstage || row.stage < 1 || row.stage > last ||
        row.stage >= static_cast<int>(solution.dx.size()))
      continue;
    double predicted = row.distance + (row.gradient * solution.dx[row.stage].head<kNx>())(0);
    State at = nominal_.x[row.stage];
    if (midstage) {
      if (row.stage >= static_cast<int>(solution.du.size())) continue;
      predicted += (row.control_gradient * solution.du[row.stage].head<kNu>())(0);
      at = discrete_step(nominal_.x[row.stage], nominal_.u[row.stage], formulation_.dt() / 2).x_next;
    }
    const double actual = collision_.pair_distance(at, row, world, now);
    if (!std::isfinite(actual)) continue;
    const double error = std::abs(predicted - actual);
    worst = std::isnan(worst) ? error : std::max(worst, error);
  }
  return worst;
}

bool Controller::gate_passes(const std::vector<DistanceRow> & rows, const Eigen::VectorXd & slack, const State & x0,
                             const World & world, double now, double horizon_s)
{
  out_.gate_slack = 0.0;
  out_.gate_violation = 0.0;
  const int per_stage = formulation_.rows_per_stage();
  // Stages 1..last lie within the gate's horizon (stage k at k dt after now; stage 0 has no rows).
  const int last = std::min(formulation_.horizon(),
                            static_cast<int>(std::floor(horizon_s / formulation_.dt() + 1e-9)));
  const double tolerance = gate_.tolerance_m;
  // Slack j belongs to stage j / per_stage + 1, to its (j % per_stage)-th row in `rows` (ordered by
  // stage, as Formulation::build numbers them); beyond a stage's rows it is a padded, satisfied row.
  // The mapping needs `rows` in stage order (CollisionModel::rows emits them so): checked, as a row
  // out of order would pair a slack with the wrong pair's violation.
  if (!std::is_sorted(rows.begin(), rows.end(),
                      [](const DistanceRow & a, const DistanceRow & b) { return a.stage < b.stage; }))
    throw std::logic_error("Controller: collision rows are not in stage order (slack gate)");
  // The current distance of a pair is computed once per cycle, and only for a row with slack.
  struct Known
  {
    int primitive, link;
    const std::string * obstacle;
    double violation;
  };
  std::vector<Known> known;
  auto violation_of = [&](const DistanceRow & row) {
    for (const Known & k : known)
      if (k.primitive == row.primitive && k.link == row.link && *k.obstacle == row.obstacle) return k.violation;
    const double v = std::max(0.0, row.margin - collision_.pair_distance(x0, row, world, now));
    known.push_back({row.primitive, row.link, &row.obstacle, v});
    return v;
  };
  // A pair outside the margin (no current violation) may plan up to the tolerance into it; a pair
  // already inside may keep its violation (or plan its way out) but never plan deeper (beyond the
  // solver's numerical noise), so the robot cannot creep in by the tolerance per cycle.
  bool pass = true;
  double largest = 0.0, worst = -std::numeric_limits<double>::infinity();
  size_t first = 0;  // the first row of stage k
  for (int k = 1; k <= last; ++k) {
    while (first < rows.size() && rows[first].stage < k) ++first;
    for (int i = 0; i < per_stage; ++i) {
      const long j = static_cast<long>(k - 1) * per_stage + i;
      if (j >= slack.size()) break;
      const double s = slack(j);
      largest = std::max(largest, s);
      if (!(s > 0.0)) continue;
      const size_t r = first + static_cast<size_t>(i);
      const double violation = r < rows.size() && rows[r].stage == k ? violation_of(rows[r]) : 0.0;
      const double allowed = violation > 0.0 ? violation + kGateInsideAllowanceM : tolerance;
      if (s - allowed > worst) {
        worst = s - allowed;
        out_.gate_slack = s;
        out_.gate_violation = violation;
      }
      if (s > allowed) pass = false;
    }
  }
  if (!(worst > 0.0)) {  // no slack beyond its allowance: the largest slack there
    out_.gate_slack = largest;
    out_.gate_violation = 0.0;
  }
  out_.gate_unsafe = !pass;
  return pass;
}

bool Controller::at_goal(const State & x) const
{
  return (x.head<2>() - goal_xy_).norm() <= goal_xy_m_ && std::abs(wrap(x(2) - inputs_.goal_yaw)) <= goal_yaw_rad_;
}

void Controller::hold(const State & x)
{
  out_.command.setZero();
  std::fill(out_.plan.x.begin(), out_.plan.x.end(), x);
  std::fill(out_.plan.u.begin(), out_.plan.u.end(), Control::Zero());
}

void Controller::brake(double elapsed, const State & x, bool arm_hold)
{
  const Limits & l = formulation_.limits();
  const double dt = formulation_.dt();
  // At the envelope's braking limits from the command sent last.
  Control u = Control::Zero();
  u.head<2>() = brake_base(last_command_.head<2>(), elapsed, l, breakaway_);
  // The arm: zero joint velocities, or (arm_hold) the node's braking hold: from the joint
  // velocities at the hold's start to zero over arm_hold_duration_ at a constant deceleration.
  auto arm = [&](double t) -> Eigen::Matrix<double, 6, 1> {
    return arm_hold_v0_ * std::max(0.0, 1.0 - t / arm_hold_duration_);
  };
  if (arm_hold) {
    arm_hold_t_ += elapsed;
    u.tail<6>() = arm(arm_hold_t_);
  }
  out_.command = u;
  // The plan continues braking over the horizon.
  Plan & plan = out_.plan;
  plan.x[0] = x;
  for (size_t k = 0; k < plan.u.size(); ++k) {
    plan.u[k] = u;
    plan.x[k + 1] = discrete_step(plan.x[k], u, dt).x_next;
    u.head<2>() = brake_base(u.head<2>(), dt, l, breakaway_);
    if (arm_hold) u.tail<6>() = arm(arm_hold_t_ + static_cast<double>(k + 1) * dt);
  }
}

namespace
{
bool close_to(const State & a, const State & b, const RtiDeviation & d)
{
  return (a.head<2>() - b.head<2>()).norm() <= d.base_m && std::abs(wrap(a(2) - b(2))) <= d.yaw_rad &&
         (a.tail<6>() - b.tail<6>()).cwiseAbs().maxCoeff() <= d.joint_rad;
}
}  // namespace

Controller::Snapshot Controller::snapshot() const
{
  return Snapshot{nominal_, plan_time_, fresh_plan_, tightness_, picker_, inputs_};
}

void Controller::restore(const Snapshot & s)
{
  nominal_ = s.nominal;
  plan_time_ = s.plan_time;
  fresh_plan_ = s.fresh_plan;
  tightness_ = s.tightness;
  picker_ = s.picker;
  inputs_ = s.inputs;
}

void Controller::discard_preparation()
{
  if (prepared_) restore(prepared_->before);
  prepared_.reset();
}

void Controller::prepare(double t_next, const World & world)
{
  discard_preparation();
  if (!options_.rti_split || !route_ || mode_ != Mode::Tracking || out_.status != CycleStatus::Ok || external_hold_ ||
      !have_last_time_ || !(t_next > last_time_))
    return;
  const auto t0 = Clock::now();
  Prepared p;
  p.t_next = t_next;
  p.before = snapshot();
  p.x_pred = discrete_step(last_x0_, last_command_, t_next - last_time_).x_next;
  p.last_command = last_command_;
  linearize(t_next, p.x_pred, world, t_next - last_time_, p.lin);
  p.prep_s = seconds_since(t0);
  last_prep_s_ = p.prep_s;
  prepared_ = std::move(p);
}

std::vector<DistanceRow> midstage_rows(const CollisionModel & collision, const Plan & nominal, double dt, double now,
                                       const World & world, double horizon_s, CollisionStats * stats)
{
  const int intervals = static_cast<int>(nominal.u.size());
  const int last = std::min(intervals, static_cast<int>(std::ceil(horizon_s / dt - 1e-9)));  // stages 1..last-1
  if (last < 2) return {};
  // Index k holds the midpoint of the interval k -> k+1 (index 0 a placeholder: stage 0 gets no rows).
  std::vector<State> mid(static_cast<size_t>(last), nominal.x.front());
  std::vector<Step> half(static_cast<size_t>(last));
  for (int k = 1; k < last; ++k) {
    half[static_cast<size_t>(k)] = discrete_step(nominal.x[static_cast<size_t>(k)], nominal.u[static_cast<size_t>(k)], dt / 2);
    mid[static_cast<size_t>(k)] = half[static_cast<size_t>(k)].x_next;
  }
  // Movers where they are half a stage later.
  std::vector<DistanceRow> rows = collision.rows(mid, dt, now + dt / 2, world, stats);
  std::vector<DistanceRow> out;
  out.reserve(rows.size());
  for (DistanceRow & row : rows) {
    if (row.stage < 1 || row.stage >= last) continue;
    const Step & s = half[static_cast<size_t>(row.stage)];
    const Eigen::Matrix<double, 1, kNx> g = row.gradient;
    row.gradient = g * s.A;
    row.control_gradient = g * s.B;
    row.midstage = true;
    out.push_back(std::move(row));
  }
  return out;
}

std::vector<DistanceRow> merge_stage_rows(std::vector<DistanceRow> stage, std::vector<DistanceRow> mid)
{
  std::vector<DistanceRow> out;
  out.reserve(stage.size() + mid.size());
  size_t i = 0, j = 0;
  while (i < stage.size() || j < mid.size()) {
    // A stage's own rows first, then its mid-stage rows.
    if (j == mid.size() || (i < stage.size() && stage[i].stage <= mid[j].stage)) out.push_back(std::move(stage[i++]));
    else out.push_back(std::move(mid[j++]));
  }
  return out;
}

void Controller::linearize(double now, const State & x0, const World & world, double filter_dt, Linearized & L)
{
  const Limits & l = formulation_.limits();
  const double dt = formulation_.dt();
  L = Linearized{};
  // Nominal: the latest plan, advanced by the whole stages elapsed since its stage 0.
  auto t = Clock::now();
  if (fresh_plan_) {
    std::fill(nominal_.x.begin(), nominal_.x.end(), x0);
    std::fill(nominal_.u.begin(), nominal_.u.end(), Control::Zero());
    plan_time_ = now;
    fresh_plan_ = false;
  } else {
    const int stages = std::clamp(static_cast<int>(std::floor((now - plan_time_) / dt + 1e-6)), 0,
                                  formulation_.horizon());
    formulation_.advance(nominal_, x0, stages);
    plan_time_ += stages * dt;
  }
  L.timing.linearize_s = seconds_since(t);

  t = Clock::now();
  L.rows = collision_.rows(nominal_.x, dt, now, world, &L.collision);
  if (collision_.config().midstage_rows) {
    CollisionStats mid_stats;
    L.rows = merge_stage_rows(std::move(L.rows), midstage_rows(collision_, nominal_, dt, now, world,
                                                               collision_.config().midstage_horizon_s, &mid_stats));
    L.collision.active_pairs += mid_stats.active_pairs;
    L.collision.dropped_pairs += mid_stats.dropped_pairs;
    L.collision.self_pairs += mid_stats.self_pairs;
  }
  const double clearance = collision_.clearance_with_joints(nominal_.x, inputs_.posture.comfort, dt, now, world);
  L.tightness = tightness_.update(clearance, filter_dt);
  // The side picker (Phase 4B-A spec Section 3): per-stage probes only in the cycle that chooses.
  probe_comfort_.clear();
  probe_tights_.clear();
  const PostureTargets & posture = inputs_.posture;
  // (No stage can conflict when the comfort pose's whole-horizon clearance is above the comfort distance.)
  if (posture.mode == PostureMode::Blend && picker_.needs_probe(L.tightness) && clearance < kTightComfortM) {
    probe_comfort_ = collision_.clearance_per_stage(nominal_.x, posture.comfort, dt, now, world);
    if (posture.tights.empty()) {
      probe_tights_.push_back(collision_.clearance_per_stage(nominal_.x, posture.tight, dt, now, world));
    } else {
      for (const auto & q : posture.tights)
        probe_tights_.push_back(collision_.clearance_per_stage(nominal_.x, q, dt, now, world));
    }
  }
  // The route probe (Phase 4B-A Task 11), only when the picker needs it (both tight poses eligible): the poses
  // held rigidly on the route from the current progress up to kRouteProbeM ahead, past the horizon, with the
  // obstacles where they are now; clear gaps shorter than the horizon's travel stay in the tight segment.
  auto route_probe = [&]() -> const RouteProbe * {
    const double s_now = std::isnan(inputs_.s0_hint) ? route_->project(x0.head<2>(), 0.0, route_->length()).s
                                                      : inputs_.s0_hint;
    route_samples_.clear();
    for (double s = s_now; s <= std::min(s_now + kRouteProbeM, route_->length()) + 1e-9; s += kRouteProbeStepM) {
      State at = x0;
      at.head<2>() = route_->at(s);
      at(2) = route_->heading_at(s);
      route_samples_.push_back(at);
    }
    probe_route_.comfort = collision_.clearance_per_stage(route_samples_, posture.comfort, 0.0, now, world);
    probe_route_.tights.clear();
    if (posture.tights.empty()) {
      probe_route_.tights.push_back(collision_.clearance_per_stage(route_samples_, posture.tight, 0.0, now, world));
    } else {
      for (const auto & q : posture.tights)
        probe_route_.tights.push_back(collision_.clearance_per_stage(route_samples_, q, 0.0, now, world));
    }
    probe_route_.step_m = kRouteProbeStepM;
    probe_route_.bridge_m = l.v_max * formulation_.horizon() * dt;
    probe_route_.margin_m = collision_.config().margin_m;
    return &probe_route_;
  };
  // The route's turn over the horizon, from the nominal plan's heading (a tie-break only).
  const double turn = wrap(nominal_.x.back()(2) - nominal_.x.front()(2));
  const double turn_sign = std::abs(turn) < kPickTurnDeadbandRad ? 0.0 : (turn > 0.0 ? 1.0 : -1.0);
  // (The callback is built only in a probing cycle: elsewhere the picker cannot need it.)
  inputs_.posture_choice =
    picker_.update(L.tightness, probe_comfort_, probe_tights_, x0(3), turn_sign,
                   probe_comfort_.empty() ? std::function<const RouteProbe *()>() : std::function<const RouteProbe *()>(route_probe));
  L.posture_choice = inputs_.posture_choice;
  L.probe_scores = picker_.scores();
  L.timing.collision_s = seconds_since(t);

  t = Clock::now();
  inputs_.x0 = x0;
  inputs_.last_command = last_command_;
  inputs_.tightness = L.tightness;
  L.qp = formulation_.build(inputs_, nominal_, L.rows);
  L.hint_used = inputs_.s0_hint;  // the second iteration builds with the same hint
  inputs_.s0_hint = formulation_.stage0_progress(inputs_);  // the next cycle searches around it
  L.timing.setup_s = seconds_since(t);
}

const CycleOutput & Controller::step(double now, const State & x, bool state_fresh, const World & world,
                                     bool base_at_rest)
{
  const auto start = Clock::now();
  if (!route_) throw std::logic_error("Controller::step before start");
  failures_at_step_ = failures_;
  const Limits & l = formulation_.limits();
  const double dt = formulation_.dt();
  // Time since the last command (one stage on the first cycle); the rate limits use at most dt.
  const double since_last = have_last_time_ ? std::max(0.0, now - last_time_) : dt;
  const double elapsed = std::min(since_last, dt);

  out_.timing = CycleTiming{};
  out_.collision = CollisionStats{};
  out_.iterations = 0;
  out_.iter2_used = out_.iter2_rejected = false;
  out_.solve_failed = out_.time_limited = false;
  const Clock::time_point budget_start = cycle_start_.value_or(start);
  cycle_start_.reset();
  double diagnostics_s = 0.0;
  out_.lin_error_m = std::numeric_limits<double>::quiet_NaN();
  out_.max_slack = 0.0;
  out_.gate_slack = 0.0;
  out_.gate_violation = 0.0;
  out_.gate_unsafe = false;
  out_.manipulability = manipulability(x);
  out_.tightness = tightness_.value();
  out_.plan_time = now;

  // Braking (stale state or after the fallback cycles) ends in Failed once the command is zero.
  auto braking = [&](CycleStatus status) {
    mode_ = Mode::Braking;
    brake(elapsed, x);
    if (out_.command.head<2>().isZero()) {
      mode_ = Mode::Failed;
      status = CycleStatus::Failed;
    }
    out_.status = status;
  };

  if (!state_fresh && (mode_ == Mode::Tracking || mode_ == Mode::Braking)) {
    braking(CycleStatus::StaleState);
  } else if (mode_ == Mode::Failed) {
    hold(x);
    out_.status = CycleStatus::Failed;
  } else if (mode_ == Mode::GoalReached) {
    hold(x);
    out_.status = CycleStatus::GoalReached;
  } else if (mode_ == Mode::Braking) {
    braking(CycleStatus::Braking);
  } else if (at_goal(x)) {
    mode_ = Mode::GoalReached;
    hold(x);
    out_.status = CycleStatus::GoalReached;
  } else {
    // The QP starts from the measured joints clamped into their bounds: JTC tracking errors can
    // put a joint slightly past a bound, from where the hard joint bounds would be infeasible.
    State x0 = x;
    x0.tail<6>() = x.tail<6>().cwiseMax(l.q_min).cwiseMin(l.q_max);
    last_x0_ = x0;

    // The linearization (nominal, collision rows, tightness, side choice, QP): prepared after the last cycle
    // (rti_split, Phase 4B-B spec Section 4.5) when this cycle is the one it was prepared for and the measured state
    // is close to the prediction, else now (restoring what the preparation advanced).
    const double cycle = options_.period_s > 0.0 ? std::min(dt, options_.period_s) : dt;
    const bool use_prepared = prepared_ && std::abs(now - prepared_->t_next) <= 0.5 * cycle &&
                              close_to(x0, prepared_->x_pred, options_.rti_max_deviation);
    if (prepared_ && !use_prepared) restore(prepared_->before);
    Linearized & L = linearized_;
    if (use_prepared) {
      L = std::move(prepared_->lin);
      // Feedback: the measured state and the command sent last, as deviations from what the QP was built for.
      L.qp.dx0.head<kNx>() = x0 - prepared_->x_pred;
      L.qp.dx0(2) = wrap(L.qp.dx0(2));  // yaw
      L.qp.dx0.tail<kNu>() = last_command_.cwiseMax(lower_control(l)).cwiseMin(upper_control(l)) -
                             prepared_->last_command.cwiseMax(lower_control(l)).cwiseMin(upper_control(l));
      inputs_.x0 = x0;
      inputs_.last_command = last_command_;
      out_.timing.prep_s = prepared_->prep_s;
    } else {
      linearize(now, x0, world, have_last_time_ ? since_last : dt, L);
    }
    prepared_.reset();
    out_.timing.linearize_s = L.timing.linearize_s;
    out_.timing.collision_s = L.timing.collision_s;
    out_.timing.setup_s = L.timing.setup_s;
    out_.collision = L.collision;
    out_.tightness = L.tightness;
    out_.posture_choice = L.posture_choice;
    out_.probe_scores = L.probe_scores;
    const QpProblem & qp = L.qp;
    std::vector<DistanceRow> & rows = L.rows;
    const double hint_used = L.hint_used;

    if (!solver_ready_) {
      solver_->setup(qp);  // the first real linearization fixes the pattern and the scaling
      solver_ready_ = true;
    }
    SolveResult result;
    // The deadline cap (Phase 4B-B): the solve gets what is left of the period; nothing left is a solver failure.
    // With the split the cycle also runs the next preparation after the command: its last time is reserved.
    const double reserve = options_.rti_split ? last_prep_s_ : 0.0;
    const double left = options_.period_s - options_.publish_margin_s - reserve - seconds_since(budget_start);
    if (options_.deadline_cap && options_.period_s > 0.0 && left <= 0.0) {
      result.status = "time_limit";
    } else {
      if (options_.deadline_cap && options_.period_s > 0.0) solver_->set_time_limit(left);
      result = solver_->solve(qp, options_.warm_start && have_warm_ ? &warm_ : nullptr);
    }
    out_.timing.solve_s = result.solve_s;
    out_.solve_failed = !result.ok;
    out_.time_limited = result.status == "time_limit";
    out_.iterations = result.iterations;

    // The second QP iteration (Phase 4B-B spec Section 4.4): re-linearize the model and the collision rows around the
    // first solution's plan and solve again, warm-started from the first solution. Its iterate is kept only if its
    // true worst margin violation is not larger on the next kIter2NearStages stages and within the slack gate's
    // horizon and its cost (Formulation::objective, anchored at the first plan) not higher by more than
    // iter2_cost_tolerance (accept_second_iterate); the slack gate then runs on the kept iterate's rows. Skipped when
    // the first solve used more than iter2_budget of the period, or when the cap leaves no time.
    if (options_.qp_iterations >= 2 && result.ok &&
        !(options_.period_s > 0.0 && seconds_since(budget_start) > options_.iter2_budget * options_.period_s)) {
      auto t2 = Clock::now();
      Plan plan1 = formulation_.apply(nominal_, result.solution);
      CollisionStats stats2;
      std::vector<DistanceRow> rows2 = collision_.rows(plan1.x, dt, now, world, &stats2);
      const double sweep_s = seconds_since(t2);  // reserved for the step check's sweep after the solve
      out_.timing.collision_s += sweep_s;
      const double hint_next = inputs_.s0_hint;
      inputs_.s0_hint = hint_used;  // build2 and the step check's objective use the first build's hint
      t2 = Clock::now();
      const QpProblem qp2 = formulation_.build(inputs_, plan1, rows2);
      out_.timing.setup_s += seconds_since(t2);
      // The cap: the time left now, less the step check's sweep.
      const double left2 =
        options_.period_s - options_.publish_margin_s - reserve - seconds_since(budget_start) - sweep_s;
      const bool capped = options_.deadline_cap && options_.period_s > 0.0;
      if (!(capped && left2 <= 0.0)) {
        QpSolution warm2 = result.solution;
        warm2.primal.setZero();  // the second QP is in deviations from plan1
        if (capped) solver_->set_time_limit(left2);
        SolveResult result2 = solver_->solve(qp2, options_.warm_start ? &warm2 : nullptr);
        out_.timing.solve_s += result2.solve_s;
        out_.iterations += result2.iterations;
        if (result2.ok) {
          t2 = Clock::now();
          const Plan plan2 = formulation_.apply(plan1, result2.solution);
          const std::vector<DistanceRow> rows_at_2 = collision_.rows(plan2.x, dt, now, world, nullptr);
          const int last_stage = static_cast<int>(std::ceil(gate_.horizon_s / dt - 1e-9));
          // rows2's distances are plan1's.
          const Iter2Measure first{worst_violation(rows2, kIter2NearStages), worst_violation(rows2, last_stage),
                                   formulation_.objective(inputs_, plan1, &plan1)};
          const Iter2Measure second{worst_violation(rows_at_2, kIter2NearStages), worst_violation(rows_at_2, last_stage),
                                    formulation_.objective(inputs_, plan2, &plan1)};
          out_.timing.collision_s += seconds_since(t2);
          if (accept_second_iterate(first, second, options_.iter2_cost_tolerance)) {
            nominal_ = std::move(plan1);
            rows = std::move(rows2);
            out_.collision = stats2;
            result = std::move(result2);
            out_.iter2_used = true;
          } else {
            out_.iter2_rejected = true;
          }
        }
      }
      inputs_.s0_hint = hint_next;
    }

    bool holding = false;
    if (result.ok) {
      out_.max_slack = result.solution.slack.size() ? result.solution.slack.maxCoeff() : 0.0;
      // The slack gate: a plan that pushes deeper into an obstacle than the robot already is within
      // the gate's horizon is not executed. While holding, plans must pass over resume_horizon_s
      // for resume_dwell_s before tracking resumes (one-iteration linearizations alternate between
      // a clean plan and one that pushes in). With the gate off nothing of it runs (no pair
      // distances: the cycle of the recorded B5; gate_slack, gate_violation and gate_unsafe stay 0).
      if (gate_.enabled) {
        const double horizon_s = was_holding_ ? gate_.resume_horizon_s : gate_.horizon_s;
        const bool pass = gate_passes(rows, result.solution.slack, x0, world, now, horizon_s);
        if (!pass || !was_holding_) {
          resume_since_ = std::numeric_limits<double>::quiet_NaN();
        } else if (std::isnan(resume_since_)) {
          resume_since_ = now;
        }
        const bool dwelling = was_holding_ && pass && now - resume_since_ < gate_.resume_dwell_s - 1e-9;
        holding = !pass || dwelling;
      }
      holding = holding || external_hold_;
      // A held plan still becomes the next nominal: the next cycle linearizes where this plan went
      // (into the obstacle), which a nominal at rest would not show. It is never executed: a solver
      // failure in the hold keeps holding (below), and a hold ends only with a plan that passes.
      formulation_.apply_in_place(nominal_, result.solution);
      if (options_.diagnostics) {
        const auto diagnostics_start = Clock::now();
        out_.lin_error_m = linearization_error(rows, result.solution, world, now);
        out_.midstage_lin_error_m = linearization_error(rows, result.solution, world, now, true);
        diagnostics_s = seconds_since(diagnostics_start);
      }
      plan_time_ = now;
      warm_ = std::move(result.solution);
      warm_.primal.setZero();  // the next nominal is this plan (or the hold): its deviations start at zero
      have_warm_ = true;
      failures_ = 0;  // a held plan is no solver failure
      out_.status = external_hold_ ? CycleStatus::Blocked : holding ? CycleStatus::Holding : CycleStatus::Ok;
    } else if (was_holding_ || external_hold_) {
      // A solver failure while holding keeps holding: the nominal is the held plan, which the gate
      // rejected, so it never stands in for a solution. The failure counts towards the fallback
      // budget (a persistent failure still brakes and fails) and the resume dwell starts again.
      ++failures_;
      resume_since_ = std::numeric_limits<double>::quiet_NaN();
      holding = true;
      out_.status = external_hold_ ? CycleStatus::Blocked : CycleStatus::Holding;
    } else {
      ++failures_;  // the nominal (the shifted previous plan) stands in for the solution
      out_.status = CycleStatus::Fallback;
    }

    if (failures_ > fallback_cycles_) {
      braking(CycleStatus::Braking);
    } else if (holding) {
      // Base at the braking limits from the command sent last, the arm's braking hold from the
      // joint velocities sent last (started once per hold). From rest the plan that passes the
      // gate starts through the breakaway output stage, as any start.
      if (!was_holding_) {
        arm_hold_v0_ = last_command_.tail<6>();
        arm_hold_duration_ =
          std::max(kArmHoldMinS, (arm_hold_v0_.cwiseAbs().array() / l.qdd_max.array()).maxCoeff());
        arm_hold_t_ = 0.0;
      }
      brake(elapsed, x, true);
    } else {
      out_.plan = nominal_;
      // The command sent: the plan's first control within the boxes and the rate band around the
      // command sent last (the QP holds stage 0 to it only within its tolerance, and over the
      // time actually elapsed), then the breakaway output stage.
      const Control u = nominal_.u[0]
                          .cwiseMax(last_command_ + lower_rate(l, elapsed))
                          .cwiseMin(last_command_ + upper_rate(l, elapsed))
                          .cwiseMax(lower_control(l))
                          .cwiseMin(upper_control(l));
      out_.command = breakaway_output(u, base_at_rest, breakaway_);
    }
  }

  was_holding_ = out_.status == CycleStatus::Holding || out_.status == CycleStatus::Blocked;
  last_command_ = out_.command;
  last_time_ = now;
  have_last_time_ = true;
  out_.timing.diagnostics_s = diagnostics_s;
  out_.timing.total_s = seconds_since(start) - diagnostics_s;
  out_.timing.feedback_s = out_.timing.prep_s > 0.0 ? out_.timing.total_s : 0.0;
  return out_;
}
}  // namespace mobile_manipulator_mpc
