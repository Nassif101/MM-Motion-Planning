#include "mobile_manipulator_medium/closed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <set>
#include <sstream>
#include <stdexcept>

#include "mobile_manipulator_mpc/kinematic_plant.hpp"
#include "mobile_manipulator_mpc/medium_switch.hpp"
#include "mobile_manipulator_mpc/statics.hpp"

namespace mobile_manipulator_medium
{
namespace
{
// a plan's feasibility for B5's medium switch: unknown for stale_reused (the last plan again)
std::optional<bool> plan_feasibility(const std::string & status)
{
  if (status == "stale_reused") return std::nullopt;
  return status != "infeasible";
}
}  // namespace
namespace
{
constexpr double kExecutedSlackM = 1e-3;  // as the medium layer's feasibility threshold (slack_feasible_m)
constexpr double kReplanMinLeftM = 0.05;  // a replan needs this much route left beyond the robot

std::string side_of(const std::string & homotopy)
{
  if (homotopy.rfind("left", 0) == 0) return "left";
  if (homotopy.rfind("right", 0) == 0) return "right";
  return "";
}
}  // namespace

void SideTracker::observe(const std::string & homotopy)
{
  const std::string side = side_of(homotopy);
  if (side.empty() || side == last) return;
  if (first.empty()) first = side;
  else ++switches;
  last = side;
  sequence.push_back(side);
}

std::string ClosedLoopResult::fallback_summary() const
{
  std::ostringstream out;
  for (const auto & [reason, count] : fallbacks) out << reason << " " << count << "; ";
  return out.str();
}

ClosedLoopResult run_closed_loop(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                 const mmm::PostureTargets & posture, const mmm::Route & route,
                                 const mobile_manipulator_navigation::Pose2 & goal, const mmm::State & start,
                                 const mmm::World & world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                 const ClosedLoopOptions & o)
{
  mmm::KinematicClosedLoop plant(start, medium.problem().limits().breakaway);
  return run_closed_loop(b5, medium, b5_config, posture, route, goal, plant, world, primitives, o);
}

ClosedLoopStepper::ClosedLoopStepper(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                     const mmm::PostureTargets & posture, const mmm::Route & route,
                                     const mobile_manipulator_navigation::Pose2 & goal, mmm::ClosedLoopPlant & plant,
                                     mmm::World world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                     const ClosedLoopOptions & o)
: b5_(b5), medium_(medium), b5_config_(b5_config), posture_(posture), goal_(goal), plant_(plant), world_(std::move(world)),
  o_(o), dt_(1.0 / o.b5_rate_hz), medium_period_(1.0 / medium.problem().config().rate_hz),
  torque_limits_(medium.problem().config().torque_limits_nm), clearance_(primitives, mmm::CollisionConfig{}), sw_(b5_config),
  active_(route), active_goal_s_(route.project(Eigen::Vector2d(goal[0], goal[1]), route.length(), 1.0).s),
  next_medium_(o.start_time_s), next_update_(o.start_time_s + o.route_update_period_s), goal_started_at_(o.start_time_s)
{
  b5_.start(route, goal, {}, posture_);
  if (o_.medium) medium_.set_route(active_, goal, route_id_);
  r_.end_status = "timeout";
  last_command_ = o_.initial_command;
}

bool ClosedLoopStepper::step()
{
  if (done_) return false;
  const double t = time();
  // the recorded mode's rule as it was (t - start > timeout); a goal manager's deadline is absolute
  const bool expired = std::isfinite(absolute_deadline_) ? t > absolute_deadline_ + 1e-9
                                                         : t - goal_started_at_ > o_.timeout_s;
  if (expired) {
    timed_out_ = true;
    finish();
    return false;
  }
  plant_.update_world(world_, t);  // the plant's movers, for this cycle's medium plan and B5
  if (o_.route_update_period_s > 0.0 && t >= next_update_ - 1e-9) {
    next_update_ += o_.route_update_period_s;
    // the replanned path starts where the robot is and follows the rest of the route
    const double s_robot = active_.project_range(plant_.state().head<2>(), 0.0, active_.length()).s;
    if (s_robot < active_.length() - kReplanMinLeftM) {
      std::vector<Eigen::Vector2d> points{plant_.state().head<2>()};
      double s = 0.0;
      for (size_t i = 0; i < active_.points().size(); ++i) {
        if (i > 0) s += (active_.points()[i] - active_.points()[i - 1]).norm();
        if (s > s_robot + kReplanMinLeftM) points.push_back(active_.points()[i]);
      }
      if (points.size() < 2) points.push_back(active_.points().back());
      active_ = mmm::Route(std::move(points));
      active_goal_s_ = active_.project(Eigen::Vector2d(goal_[0], goal_[1]), active_.length(), 1.0).s;
      b5_.update_goal(active_, goal_, {});
      ++route_id_;
      ++r_.route_updates;
      if (o_.medium) {
        sw_.route_changed(b5_);
        // the published plan carried on under the new route_id: the node publishes it once a cycle in flight has
        // published (its executor runs the route callback between cycles), B5 sees it the next cycle (no solve)
        if (medium_.set_route(active_, goal_, route_id_)) {
          const PlanOutput & carried = medium_.last_plan();
          const double at = std::max(t + dt_, pending_.empty() ? 0.0 : pending_.back().at);
          Delivery d{at, {}, carried.splice_s, carried.planned_at, carried.plan_id, carried.route_id,
                     plan_feasibility(carried.status)};
          for (const auto & smp : carried.samples) d.samples.push_back({smp.s, smp.x, smp.y, smp.yaw, smp.speed, smp.q});
          infeasible_[carried.plan_id] = carried.status == "infeasible";
          pending_.push_back(std::move(d));
        }
      }
    }
  }
  if (o_.medium && t >= next_medium_ - 1e-9) {
    next_medium_ += medium_period_;
    Eigen::Matrix<double, 9, 1> rate;
    rate << plant_.executed()(0) * std::cos(plant_.state()(2)), plant_.executed()(0) * std::sin(plant_.state()(2)),
      plant_.executed()(1), plant_.executed().tail<6>();
    if (o_.before_plan) o_.before_plan(t, *this);
    const PlanOutput & out = medium_.plan(t, plant_.state(), rate, world_);
    if (o_.on_plan) o_.on_plan(t, plant_.state(), out);
    ++r_.medium_cycles;
    r_.medium_solve_s.push_back(out.solve_s);
    r_.medium_capped += out.capped;
    r_.medium_infeasible += out.status == "infeasible";
    if (out.status == "ok") sides_.observe(out.homotopy);  // a plan published infeasible is no side choice
    infeasible_[out.plan_id] = out.status == "infeasible";
    if (out.samples.size() >= 2) {
      const double latency = std::isnan(o_.fixed_latency_s) ? out.solve_s : o_.fixed_latency_s;
      Delivery d{t + std::ceil(latency / dt_ - 1e-9) * dt_, {}, out.splice_s, out.planned_at, out.plan_id, out.route_id,
                 plan_feasibility(out.status)};
      for (const auto & smp : out.samples) d.samples.push_back({smp.s, smp.x, smp.y, smp.yaw, smp.speed, smp.q});
      pending_.push_back(std::move(d));
    }
  }
  while (!pending_.empty() && pending_.front().at <= t + 1e-9) {
    Delivery & d = pending_.front();
    // against the active route, as the node converts it (a plan for an older route may not fit it: rejected there)
    try {
      sw_.set_plan(mmm::MediumReference(std::move(d.samples), active_, d.splice_s, posture_.comfort), d.plan_id,
                   d.route_id, d.stamp, d.feasible);
    } catch (const std::invalid_argument &) {
    }
    pending_.pop_front();
  }
  last_fallback_.clear();
  if (o_.medium) {
    const mmm::MediumFallback reason = sw_.update(b5_, t, plant_.state().head<2>(), route_id_, active_, active_goal_s_);
    if (sw_.following()) {
      ++r_.following_cycles;
      if (infeasible_[sw_.plan_id()]) {
        ++r_.followed_infeasible_cycles;
        followed_infeasible_.insert(sw_.plan_id());
      }
    } else {
      last_fallback_ = mmm::to_string(reason);
      ++r_.fallbacks[last_fallback_];
    }
  }
  if (const auto v = plant_.previous_arm_velocity()) {  // the node's reset_command with the reference velocity
    mmm::Control previous = b5_.last_command();
    previous.tail<6>() = *v;
    b5_.reset_command(previous);
  }
  const mmm::CycleOutput & out = b5_.step(t, plant_.state(), true, world_, plant_.at_rest());
  ++r_.b5_cycles;
  ++b5_cycles_stepped_;
  last_b5_timing_ = out.timing;
  last_status_ = out.status;
  last_row_trace_ = out.row_trace;
  last_row_trace_x_ = out.row_trace_x;
  last_row_trace_u_ = out.row_trace_u;
  last_gate_ = {out.gate_slack,   out.gate_violation,      out.gate_unsafe, out.gate_dwelling,
                out.gate_depth_m, out.gate_depth_stage1_m, out.lin_error_m, out.guard_tripped, out.guard_depth_m,
                out.gate_pair,    out.gate_gradient,       out.gate_gradient_kink,
                out.probe_link,   out.probe_obstacle,      out.probe_distance, out.probe_in_stage1, out.probe_in_rows,
                out.probe_dropped,
                out.tracking};
  r_.time_s = t;
  if (out.status == mmm::CycleStatus::GoalReached || out.status == mmm::CycleStatus::Failed)
    r_.end_status = mmm::to_string(out.status);
  if (out.status == mmm::CycleStatus::GoalReached) {
    r_.reached = true;
    finish();
    return false;
  }
  if (out.status == mmm::CycleStatus::Failed) {
    finish();
    return false;
  }
  last_model_ = out.tracking_model;
  last_planned_ = out.plan.u.empty() ? mmm::Control::Zero() : out.plan.u.front();
  // Spec 2 Task 7: the command-boundary hook. `out` aliases B5's output: a refusal sends a separate hold (never a
  // zeroed base under an Ok/Fallback status, whose plan the realistic arm path would still stream).
  bool allowed = true;
  if (o_.command_guard) {
    try {
      allowed = o_.command_guard(t, plant_.state(), out);
    } catch (...) {
      allowed = false;
    }
  }
  // Execution evidence describes what was sent (Task 7 review I4): a refused proposal's status and slack are not
  // executed; the result's end status is the sent output's.
  if (!allowed) {
    const mmm::CycleOutput held = holding_output(mmm::CycleStatus::Blocked, out.tracking_model);
    last_status_ = held.status;
    last_command_ = held.command;
    r_.end_status = mmm::to_string(held.status);
    plant_.step(held.command, held, t, dt_);
    b5_.record_external_command(t, held.command);  // same-time correction: B5's history has what was sent
  } else {
    holding_ = false;
    last_command_ = out.command;
    r_.end_status = mmm::to_string(out.status);
    if (out.status == mmm::CycleStatus::Ok && out.max_slack > kExecutedSlackM) {
      ++r_.executed_slack_plans;
      r_.max_executed_slack_m = std::max(r_.max_executed_slack_m, out.max_slack);
    }
    plant_.step(out.command, out, t, dt_);
  }
  if (o_.on_cycle) o_.on_cycle(t, plant_.state());
  account_safety(t, true);
  const Joints tau = statics_.torque(plant_.state().tail<6>());
  for (int j = 0; j < 6; ++j)
    if (std::isfinite(torque_limits_(j))) r_.max_torque_ratio = std::max(r_.max_torque_ratio, std::abs(tau(j)) / torque_limits_(j));
  ++k_;
  r_.final_state = plant_.state();
  return true;
}

mmm::CycleOutput ClosedLoopStepper::holding_output(const mmm::CycleStatus status, const mmm::TrackingModel & model)
{
  // B5's braking hold (Controller::brake with arm_hold), without a solve: base at the envelope's braking limits from
  // the command sent last, arm velocities from those sent when the hold began to zero at a constant deceleration
  const mmm::Limits & l = medium_.problem().limits();
  if (!holding_) {
    holding_ = true;
    hold_v0_ = last_command_.tail<6>();
    // Task 7 review I1: the arm stop checked against the obstacles (mmm::safe_arm_hold, as the realistic arm path and
    // the node's hold_arm choose it), so the ideal plant, which executes these velocities, stops the same way: the
    // constant deceleration over the checked duration (made faster near obstacles), or the emergency stop
    const mmm::ArmHold checked = mmm::safe_arm_hold(b5_.collision(), world_, time(), plant_.state(), hold_v0_, l,
                                                    last_command_.head<2>(), model);
    hold_duration_s_ = checked.duration_s;
    hold_emergency_ = !std::isfinite(checked.decel_factor);
    hold_q0_ = plant_.state().tail<6>();
    hold_elapsed_s_ = 0.0;
  }
  const double s0 = std::min(1.0, hold_elapsed_s_ / hold_duration_s_);
  hold_elapsed_s_ += dt_;
  const double s1 = std::min(1.0, hold_elapsed_s_ / hold_duration_s_);
  mmm::CycleOutput out;
  out.status = status;
  out.command.head<2>() = mmm::brake_base(last_command_.head<2>(), dt_, l, l.breakaway);
  // Task 8 review I1: each tick sends the interval's average velocity of the checked position profile, so the executed
  // joints are on the checked stop at every tick: the parabola q0 + v0 T (s - s^2 / 2) within the joint limits, or the
  // emergency Hermite q0 + v0 T s (1 - s)^2 (safe_arm_hold's curves)
  const auto position = [&](double f) -> Joints {
    if (hold_emergency_) return hold_q0_ + hold_v0_ * (hold_duration_s_ * f * (1.0 - f) * (1.0 - f));
    return (hold_q0_ + hold_v0_ * (hold_duration_s_ * (f - 0.5 * f * f))).cwiseMax(l.q_min).cwiseMin(l.q_max);
  };
  out.command.tail<6>() = (position(s1) - position(s0)) / dt_;
  out.tracking_model = model;
  out.tracking = Eigen::Vector3d(1.0 + model.turn_delta_inf, 1.0 + model.turn_delta_zero, model.sideslip);
  out.x0_planned = plant_.state();
  return out;
}

void ClosedLoopStepper::clear_cycle_diagnostics()
{
  last_fallback_.clear();
  last_gate_ = Gate{};
  last_planned_ = mmm::Control::Zero();
  last_row_trace_.clear();
  last_row_trace_x_.clear();
  last_row_trace_u_.clear();
}

bool ClosedLoopStepper::step_holding()
{
  const double t = time();
  plant_.update_world(world_, t);  // the movers, and the world the realistic arm path checks its hold against
  clear_cycle_diagnostics();
  // Task 7 review I2: as in step(), the previous interval's arm part becomes the velocity the plant actually streamed
  // (the realistic arm path sends its own checked stop), before this tick appends it to B5's delay history
  if (const auto v = plant_.previous_arm_velocity()) {
    mmm::Control previous = b5_.last_command();
    previous.tail<6>() = *v;
    b5_.reset_command(previous);
  }
  const mmm::CycleOutput held = holding_output(mmm::CycleStatus::Holding, last_model_);
  last_status_ = held.status;
  last_command_ = held.command;
  plant_.step(held.command, held, t, dt_);
  b5_.record_external_command(t, held.command);
  if (o_.on_cycle) o_.on_cycle(t, plant_.state());
  account_safety(t, false);  // no goal is tracked: run-level evidence only
  ++k_;
  return true;
}

void ClosedLoopStepper::account_safety(double t, bool goal_local)
{
  const auto truth = plant_.true_min_clearance();
  const bool contact = plant_.contacts() > 0;
  const auto nearest = clearance_.nearest(plant_.state(), world_.at(0.0, t));
  ++safety_.ticks;
  safety_.plant_contact_ticks += contact;
  if (truth) safety_.plant_min_clearance_m = std::min(safety_.plant_min_clearance_m, *truth);
  if (nearest) safety_.min_clearance_m = std::min(safety_.min_clearance_m, nearest->distance);
  if (!goal_local) return;
  if (truth) r_.plant_min_clearance_m = std::min(r_.plant_min_clearance_m, *truth);
  r_.plant_contacts += contact;
  if (nearest) r_.min_clearance_m = std::min(r_.min_clearance_m, nearest->distance);
}

void ClosedLoopStepper::adopt_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                                    uint64_t route_id, bool new_goal, double absolute_deadline)
{
  if (!new_goal && (done_ || route_id <= route_id_))
    throw std::invalid_argument("ClosedLoopStepper::adopt_route: a same-goal route needs a running goal and a newer route_id");
  const double t = time();
  active_ = route;
  goal_ = goal;
  active_goal_s_ = active_.project(Eigen::Vector2d(goal_[0], goal_[1]), active_.length(), 1.0).s;
  route_id_ = route_id;
  if (!new_goal) {
    b5_.update_goal(active_, goal_, {});
    ++r_.route_updates;
    if (o_.medium) {
      sw_.route_changed(b5_);
      if (medium_.set_route(active_, goal_, route_id_)) {  // the incumbent carries on under the new route_id
        const PlanOutput & carried = medium_.last_plan();
        const double at = std::max(t + dt_, pending_.empty() ? 0.0 : pending_.back().at);
        Delivery d{at, {}, carried.splice_s, carried.planned_at, carried.plan_id, carried.route_id,
                   plan_feasibility(carried.status)};
        for (const auto & smp : carried.samples) d.samples.push_back({smp.s, smp.x, smp.y, smp.yaw, smp.speed, smp.q});
        infeasible_[carried.plan_id] = carried.status == "infeasible";
        pending_.push_back(std::move(d));
      }
    }
    return;
  }
  b5_.start(active_, goal_, {}, posture_);  // clears B5's pointer into the switch's plan before the switch is replaced
  sw_ = mmm::MediumSwitch(b5_config_);
  pending_.clear();
  infeasible_.clear();
  followed_infeasible_.clear();
  sides_ = SideTracker{};
  if (o_.medium) medium_.set_route(active_, goal_, route_id_, true);  // fresh F2 planning, even at the same coordinates
  next_medium_ = t;
  next_update_ = t + o_.route_update_period_s;
  goal_started_at_ = t;
  absolute_deadline_ = absolute_deadline;
  r_ = ClosedLoopResult{};
  r_.end_status = "timeout";
  done_ = false;
  timed_out_ = false;
}

ClosedLoopResult ClosedLoopStepper::snapshot() const
{
  ClosedLoopResult r = r_;
  if (done_) return r;  // finish() materialized them; a later goal's trackers start empty
  r.final_state = plant_.state();
  r.first_side = sides_.first;
  r.homotopy_switches_after_first = sides_.switches;
  r.side_sequence = sides_.sequence;
  r.followed_infeasible_plans = static_cast<int>(followed_infeasible_.size());
  return r;
}

void ClosedLoopStepper::continue_with(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                                      uint64_t route_id)
{
  if (!done_ || !r_.reached || timed_out_)
    throw std::invalid_argument("ClosedLoopStepper::continue_with: the current leg must have reached its goal");
  const double t = time();
  active_ = route;
  goal_ = goal;
  active_goal_s_ = active_.project(Eigen::Vector2d(goal_[0], goal_[1]), active_.length(), 1.0).s;
  route_id_ = route_id;
  b5_.start(active_, goal_, {}, posture_);
  b5_.record_external_command(t, last_command_);  // the command in effect, not the leg's unsent goal-reached output
  sw_ = mmm::MediumSwitch(b5_config_);
  pending_.clear();
  if (o_.medium) medium_.set_route(active_, goal_, route_id_, true);
  next_medium_ = t;
  next_update_ = t + o_.route_update_period_s;
  r_.reached = false;
  r_.end_status = "timeout";
  done_ = false;
}

void ClosedLoopStepper::finish()
{
  done_ = true;
  r_.final_state = plant_.state();
  r_.first_side = sides_.first;
  r_.homotopy_switches_after_first = sides_.switches;
  r_.side_sequence = sides_.sequence;
  r_.followed_infeasible_plans = static_cast<int>(followed_infeasible_.size());
}

ClosedLoopResult run_closed_loop(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                 const mmm::PostureTargets & posture, const mmm::Route & route,
                                 const mobile_manipulator_navigation::Pose2 & goal, mmm::ClosedLoopPlant & plant,
                                 mmm::World world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                 const ClosedLoopOptions & o)
{
  ClosedLoopStepper stepper(b5, medium, b5_config, posture, route, goal, plant, std::move(world), primitives, o);
  while (stepper.step()) {
  }
  return stepper.result();
}
}  // namespace mobile_manipulator_medium
