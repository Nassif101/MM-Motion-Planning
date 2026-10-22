#include "mobile_manipulator_mpc/replay.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/solver_factory.hpp"

namespace mobile_manipulator_mpc
{
namespace
{
namespace mmg = mobile_manipulator_geometry;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kStage1BaseM = 1e-3, kStage1AngleRad = 1e-3;
// A rate this close to the recording's (relative) replays it as recorded; a wall gap longer than kGoalGapS between
// cycles is a goal that ended (host stalls measured 0.3-0.4 s; the behaviour tree's recovery takes longer).
constexpr double kRecordedRateTolerance = 0.01, kGoalGapS = 0.75;

bool same_stage(const State & a, const State & b)
{
  return (a.head<2>() - b.head<2>()).cwiseAbs().maxCoeff() <= kStage1BaseM &&
         (a.tail<7>() - b.tail<7>()).cwiseAbs().maxCoeff() <= kStage1AngleRad;
}

double stage_error(const State & a, const State & b) { return (a - b).cwiseAbs().maxCoeff(); }

bool iterations_close(int replayed, int recorded)
{
  return std::abs(replayed - recorded) <= std::max(1.0, 0.1 * recorded);
}
}  // namespace

std::unique_ptr<Controller> build_controller(const MpcConfig & config, const Limits & limits, const std::string & urdf,
                                             const mobile_manipulator_navigation::Json & payload, double rate_hz,
                                             bool diagnostics)
{
  if (!(rate_hz > 0.0)) throw std::invalid_argument("build_controller: rate_hz must be positive");
  ControllerOptions controller = controller_options(config, 1.0 / rate_hz);
  controller.diagnostics = diagnostics;
  return std::make_unique<Controller>(
    Formulation(config.horizon, config.dt, limits, config.weights, config.collision),
    CollisionModel(mmg::load_primitives(urdf, mmg::payload_from_json(payload)), config.collision),
    make_solver(config), limits.breakaway, fallback_cycles_for(config.fallback_s, rate_hz), kGoalXyM,
    kGoalYawRad, config.slack_gate, controller);
}

ReplayResult replay(Controller & controller, const Recording & recording, const ReplaySetup & setup,
                    const ReplayOptions & options)
{
  ReplayResult result;
  if (recording.cycles.empty()) return result;
  // The node's timer is a wall timer: rate and goal gaps on the recording's clock (bag_time); the cycle stamps are
  // sim time quantized by Unity's 50 Hz clock (repeats and 0.04 s steps) and only the controller's `now`.
  const double wall_period = recording.wall_period_s > 0.0 ? recording.wall_period_s : recording.period_s;
  const bool recorded_rate =
    !(options.rate_hz > 0.0) || std::abs(options.rate_hz * wall_period - 1.0) <= kRecordedRateTolerance;
  const double period = recorded_rate ? wall_period : 1.0 / options.rate_hz;
  const double dt = controller.formulation().dt();
  const int horizon_stages = std::min(options.clearance_stages, controller.formulation().horizon());
  World world = setup.world;
  std::optional<Route> route;
  size_t next_goal = 0, next_mover = 0;
  double due = recording.cycles.front().bag_time;
  const RecordedCycle * previous = nullptr;
  const RecordedGoal * last_goal = nullptr;
  bool replay_failed = false;
  Fidelity fidelity;
  int status_same = 0, stage1_same = 0, iterations_compared = 0, iterations_same = 0, cycles = 0;

  for (size_t index = 0; index < recording.cycles.size(); ++index) {
    const RecordedCycle & c = recording.cycles[index];
    if (!recorded_rate) {
      if (c.bag_time < due - 1e-9) continue;
      while (due <= c.bag_time + 1e-9) due += period;
    }
    for (; next_mover < recording.movers.size() && recording.movers[next_mover].bag_time <= c.bag_time; ++next_mover) {
      const MoverSample & m = recording.movers[next_mover];
      for (size_t i = 0; i < m.positions.size() && i < setup.movers.size(); ++i)
        world.update_mover(setup.movers[i].name, m.positions[i], setup.movers[i].size, m.stamp);
    }
    // The latest goal received by this cycle's state time (the node takes it at the cycle's start).
    const RecordedGoal * goal = nullptr;
    for (; next_goal < recording.goals.size() && recording.goals[next_goal].bag_time <= c.bag_time; ++next_goal)
      goal = &recording.goals[next_goal];
    const bool ended = previous != nullptr && (c.bag_time - previous->bag_time > kGoalGapS ||
                                               previous->status == "goal_reached" || previous->status == "failed");
    // A goal that ended and resumed without a new /plan: the behaviour tree resent FollowPath with the same path. The
    // replay's own failure while the recording tracks is resent the same way (counted apart).
    const bool own_restart = replay_failed && !ended && goal == nullptr && last_goal != nullptr;
    if (goal == nullptr && (ended || own_restart) && last_goal != nullptr) goal = last_goal;
    if (goal != nullptr) {
      last_goal = goal;
      std::vector<Eigen::Vector2d> points = goal->points;
      if (std::all_of(points.begin(), points.end(), [&](const Eigen::Vector2d & p) { return (p - points[0]).norm() < 1e-6; }))
        points.insert(points.begin(), c.x.head<2>());
      route.emplace(std::move(points));
      std::vector<PanelReference> refs = setup.references ? setup.references(*route) : std::vector<PanelReference>{};
      if (previous == nullptr || ended || own_restart) {
        controller.start(*route, goal->goal, std::move(refs), setup.posture);
        ++(own_restart ? result.replay_restarts : result.goal_starts);
      } else {
        controller.update_goal(*route, goal->goal, std::move(refs));
        ++result.goal_updates;
      }
    }
    previous = &c;
    if (!route) continue;  // cycles before the first goal

    Control last = controller.last_command();
    last.head<2>() = c.executed;
    last.tail<6>() = c.arm_v.cwiseMax(-setup.limits.qd_max).cwiseMin(setup.limits.qd_max);
    controller.reset_command(last);
    controller.set_external_hold(c.status == "blocked");
    const CycleOutput & out = controller.step(c.stamp, c.x, c.status != "stale_state",
                                              world, base_at_rest(c.odom_v, c.odom_omega, setup.limits.breakaway));
    if (out.solve_failed) ++result.solve_failures;
    replay_failed = out.status == CycleStatus::Failed;
    // The preparation/feedback split (no effect without it) runs in the same cycle, as in the node's callback.
    const auto prepare_start = std::chrono::steady_clock::now();
    controller.prepare(c.stamp + period, world);
    const double prepare_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - prepare_start).count();
    const double total_s = out.timing.total_s + prepare_s;
    const bool missed = options.recorded_misses ? c.deadline_missed : total_s > period;
    if (missed) {
      controller.note_failure();
      ++result.misses;
    }
    result.prepare_run_s.push_back(prepare_s);

    result.recorded_index.push_back(static_cast<int>(index));
    result.stamp.push_back(c.stamp);
    result.total_s.push_back(total_s);
    result.solve_s.push_back(out.timing.solve_s);
    result.collision_s.push_back(out.timing.collision_s);
    result.linearize_s.push_back(out.timing.linearize_s);
    result.setup_s.push_back(out.timing.setup_s);
    result.prep_s.push_back(out.timing.prep_s);
    result.feedback_s.push_back(out.timing.feedback_s);
    result.diagnostics_s.push_back(out.timing.diagnostics_s);
    result.max_slack_m.push_back(out.max_slack);
    result.iterations.push_back(out.iterations);
    result.status.push_back(to_string(out.status));
    result.posture_choice.push_back(to_string(out.posture_choice));
    result.lin_error_m.push_back(out.lin_error_m);
    // Plan quality over the slack gate's horizon (stages 1..horizon_stages), the same look-ahead whatever N and dt.
    const bool executed = out.status == CycleStatus::Ok || out.status == CycleStatus::Fallback;
    const int last_stage = std::min(horizon_stages, static_cast<int>(out.plan.x.size()) - 1);
    double clearance = kNaN, progress = kNaN, contour = kNaN;
    if (executed && last_stage >= 1) {
      clearance = std::numeric_limits<double>::infinity();
      double sum = 0.0;
      for (int k = 1; k <= last_stage; ++k) {
        const State & x = out.plan.x[static_cast<size_t>(k)];
        const auto nearest = controller.collision().nearest(x, world.at(k * dt, c.stamp));
        if (nearest) clearance = std::min(clearance, nearest->distance);
        const auto self = controller.collision().nearest_self(x);  // the arm links, with self links configured
        if (self) clearance = std::min(clearance, self->distance);
        sum += std::abs(route->project_range(x.head<2>(), 0.0, route->length()).lateral);
      }
      const auto first = route->project_range(out.plan.x.front().head<2>(), 0.0, route->length());
      const auto end = route->project_range(out.plan.x[static_cast<size_t>(last_stage)].head<2>(), 0.0, route->length());
      progress = (end.s - first.s) / (last_stage * dt);
      contour = sum / last_stage;
    }
    result.true_clearance_m.push_back(clearance);
    result.progress_mps.push_back(progress);
    result.contour_m.push_back(contour);

    const bool both_planned = c.plan.size() > 1 && out.plan.x.size() > 1;
    result.stage1_error_m.push_back(recorded_rate && both_planned ? stage_error(out.plan.x[1], c.plan[1]) : kNaN);
    if (recorded_rate) {
      ++cycles;
      const bool status_same_now = to_string(out.status) == c.status;
      if (status_same_now) ++status_same;
      bool diverged = !status_same_now;
      if (c.status == "ok") {
        ++fidelity.compared;
        if (out.status == CycleStatus::Ok && both_planned && same_stage(out.plan.x[1], c.plan[1])) {
          ++stage1_same;
        } else {
          diverged = true;
        }
      }
      if (diverged && std::isnan(fidelity.first_divergence_stamp)) fidelity.first_divergence_stamp = c.stamp;
      if (c.iterations > 0) {
        ++iterations_compared;
        if (iterations_close(out.iterations, c.iterations)) ++iterations_same;
      }
    }
  }
  if (recorded_rate) {
    fidelity.status_match = cycles ? static_cast<double>(status_same) / cycles : 1.0;
    fidelity.stage1_match = fidelity.compared ? static_cast<double>(stage1_same) / fidelity.compared : 1.0;
    fidelity.iterations_match = iterations_compared ? static_cast<double>(iterations_same) / iterations_compared : 1.0;
    result.fidelity = fidelity;
  }
  return result;
}
}  // namespace mobile_manipulator_mpc
