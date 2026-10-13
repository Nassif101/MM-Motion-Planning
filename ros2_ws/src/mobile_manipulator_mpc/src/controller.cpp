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
  }
  return "unknown";
}

Controller::Controller(Formulation formulation, CollisionModel collision, std::unique_ptr<QpSolver> solver,
                       Breakaway breakaway, int fallback_cycles, double goal_xy_m, double goal_yaw_rad)
: formulation_(std::move(formulation)),
  collision_(std::move(collision)),
  solver_(std::move(solver)),
  breakaway_(breakaway),
  fallback_cycles_(fallback_cycles),
  goal_xy_m_(goal_xy_m),
  goal_yaw_rad_(goal_yaw_rad),
  tightness_(kTightComfortM, collision_.config().margin_m, kTightTauS)
{
  if (!solver_) throw std::invalid_argument("Controller: no QP solver");
  if (formulation_.rows_per_stage() != collision_.config().pairs_per_stage)
    throw std::invalid_argument("Controller: formulation and collision model use different row budgets");
  if (fallback_cycles_ < 0) throw std::invalid_argument("Controller: negative fallback cycle count");
  if (!(goal_xy_m_ > 0.0) || !(goal_yaw_rad_ > 0.0)) throw std::invalid_argument("Controller: bad goal tolerance");
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
  set_goal(route, goal, std::move(refs));
  inputs_.posture = std::move(posture);
  mode_ = Mode::Tracking;
  fresh_plan_ = true;
  failures_ = 0;
  have_warm_ = false;
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

void Controller::brake(double elapsed, const State & x)
{
  const Limits & l = formulation_.limits();
  const double dt = formulation_.dt();
  // At the envelope's braking limits from the command sent last; the arm holds.
  Control u = Control::Zero();
  u(0) = towards_zero(last_command_(0), l.decel * elapsed, breakaway_.dead_linear_mps);
  u(1) = towards_zero(last_command_(1), l.yaw_decel * elapsed, breakaway_.dead_yaw_radps);
  out_.command = u;
  // The plan continues braking over the horizon.
  Plan & plan = out_.plan;
  plan.x[0] = x;
  for (size_t k = 0; k < plan.u.size(); ++k) {
    plan.u[k] = u;
    plan.x[k + 1] = discrete_step(plan.x[k], u, dt).x_next;
    u(0) = towards_zero(u(0), l.decel * dt, breakaway_.dead_linear_mps);
    u(1) = towards_zero(u(1), l.yaw_decel * dt, breakaway_.dead_yaw_radps);
  }
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
  out_.max_slack = 0.0;
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
    out_.timing.linearize_s = seconds_since(t);

    t = Clock::now();
    const std::vector<DistanceRow> rows = collision_.rows(nominal_.x, dt, now, world, &out_.collision);
    const double clearance = collision_.clearance_with_joints(nominal_.x, inputs_.posture.home, dt, now, world);
    out_.tightness = tightness_.update(clearance, have_last_time_ ? since_last : dt);
    out_.timing.collision_s = seconds_since(t);

    t = Clock::now();
    inputs_.x0 = x0;
    inputs_.last_command = last_command_;
    inputs_.tightness = out_.tightness;
    const QpProblem qp = formulation_.build(inputs_, nominal_, rows);
    inputs_.s0_hint = formulation_.stage0_progress(inputs_);  // the next cycle searches around it
    out_.timing.setup_s = seconds_since(t);

    if (!solver_ready_) {
      solver_->setup(qp);  // the first real linearization fixes the pattern and the scaling
      solver_ready_ = true;
    }
    SolveResult result = solver_->solve(qp, have_warm_ ? &warm_ : nullptr);
    out_.timing.solve_s = result.solve_s;
    out_.iterations = result.iterations;

    if (result.ok) {
      formulation_.apply_in_place(nominal_, result.solution);
      plan_time_ = now;
      out_.max_slack = result.solution.slack.size() ? result.solution.slack.maxCoeff() : 0.0;
      warm_ = std::move(result.solution);
      warm_.primal.setZero();  // the next nominal is this plan: its deviations start at zero
      have_warm_ = true;
      failures_ = 0;
      out_.status = CycleStatus::Ok;
    } else {
      ++failures_;  // the nominal (the shifted previous plan) stands in for the solution
      out_.status = CycleStatus::Fallback;
    }

    if (failures_ > fallback_cycles_) {
      braking(CycleStatus::Braking);
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

  last_command_ = out_.command;
  last_time_ = now;
  have_last_time_ = true;
  out_.timing.total_s = seconds_since(start);
  return out_;
}
}  // namespace mobile_manipulator_mpc
