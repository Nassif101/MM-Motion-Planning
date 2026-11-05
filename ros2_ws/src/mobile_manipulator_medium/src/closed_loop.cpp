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

ClosedLoopResult run_closed_loop(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                 const mmm::PostureTargets & posture, const mmm::Route & route,
                                 const mobile_manipulator_navigation::Pose2 & goal, mmm::ClosedLoopPlant & plant,
                                 mmm::World world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                 const ClosedLoopOptions & o)
{
  const double dt = 1.0 / o.b5_rate_hz;
  const double medium_period = 1.0 / medium.problem().config().rate_hz;
  const Joints torque_limits = medium.problem().config().torque_limits_nm;
  const mmm::CollisionModel clearance(primitives, mmm::CollisionConfig{});
  const mmm::GeneratedStaticTorque statics;
  mmm::MediumSwitch sw(b5_config);
  const double goal_s = route.project(Eigen::Vector2d(goal[0], goal[1]), route.length(), 1.0).s;
  struct Delivery
  {
    double at;
    std::vector<mmm::MediumSample> samples;
    double splice_s, stamp;
    uint64_t plan_id, route_id;
  };
  std::deque<Delivery> pending;
  std::map<uint64_t, bool> infeasible;  // plan_id -> published infeasible
  std::set<uint64_t> followed_infeasible;
  SideTracker sides;

  b5.start(route, goal, {}, posture);
  mmm::Route active = route;  // B5's Nav2 route: the given one, then each replan's
  double active_goal_s = goal_s;
  uint64_t route_id = 1;
  if (o.medium) medium.set_route(active, goal, route_id);
  ClosedLoopResult r;
  r.end_status = "timeout";
  double next_medium = 0.0, next_update = o.route_update_period_s;
  for (long k = 0;; ++k) {
    const double t = k * dt;
    if (t > o.timeout_s) break;
    if (o.route_update_period_s > 0.0 && t >= next_update - 1e-9) {
      next_update += o.route_update_period_s;
      // the replanned path starts where the robot is and follows the rest of the route
      const double s_robot = active.project_range(plant.state().head<2>(), 0.0, active.length()).s;
      if (s_robot < active.length() - kReplanMinLeftM) {
        std::vector<Eigen::Vector2d> points{plant.state().head<2>()};
        double s = 0.0;
        for (size_t i = 0; i < active.points().size(); ++i) {
          if (i > 0) s += (active.points()[i] - active.points()[i - 1]).norm();
          if (s > s_robot + kReplanMinLeftM) points.push_back(active.points()[i]);
        }
        if (points.size() < 2) points.push_back(active.points().back());
        active = mmm::Route(std::move(points));
        active_goal_s = active.project(Eigen::Vector2d(goal[0], goal[1]), active.length(), 1.0).s;
        b5.update_goal(active, goal, {});
        ++route_id;
        ++r.route_updates;
        if (o.medium) {
          sw.route_changed(b5);
          // the published plan carried on under the new route_id: the node publishes it once a cycle in flight has
          // published (its executor runs the route callback between cycles), B5 sees it the next cycle (no solve)
          if (medium.set_route(active, goal, route_id)) {
            const PlanOutput & carried = medium.last_plan();
            const double at = std::max(t + dt, pending.empty() ? 0.0 : pending.back().at);
            Delivery d{at, {}, carried.splice_s, carried.planned_at, carried.plan_id, carried.route_id};
            for (const auto & s : carried.samples) d.samples.push_back({s.s, s.x, s.y, s.yaw, s.speed, s.q});
            infeasible[carried.plan_id] = carried.status == "infeasible";
            pending.push_back(std::move(d));
          }
        }
      }
    }
    if (o.medium && t >= next_medium - 1e-9) {
      next_medium += medium_period;
      Eigen::Matrix<double, 9, 1> rate;
      rate << plant.executed()(0) * std::cos(plant.state()(2)), plant.executed()(0) * std::sin(plant.state()(2)),
        plant.executed()(1), plant.executed().tail<6>();
      const PlanOutput & out = medium.plan(t, plant.state(), rate, world);
      if (o.on_plan) o.on_plan(t, plant.state(), out);
      ++r.medium_cycles;
      r.medium_solve_s.push_back(out.solve_s);
      r.medium_capped += out.capped;
      r.medium_infeasible += out.status == "infeasible";
      if (out.status == "ok") sides.observe(out.homotopy);  // a plan published infeasible is no side choice
      infeasible[out.plan_id] = out.status == "infeasible";
      if (out.samples.size() >= 2) {
        const double latency = std::isnan(o.fixed_latency_s) ? out.solve_s : o.fixed_latency_s;
        Delivery d{t + std::ceil(latency / dt - 1e-9) * dt, {}, out.splice_s, out.planned_at, out.plan_id, out.route_id};
        for (const auto & s : out.samples) d.samples.push_back({s.s, s.x, s.y, s.yaw, s.speed, s.q});
        pending.push_back(std::move(d));
      }
    }
    while (!pending.empty() && pending.front().at <= t + 1e-9) {
      Delivery & d = pending.front();
      // against the active route, as the node converts it (a plan for an older route may not fit it: rejected there)
      try {
        sw.set_plan(mmm::MediumReference(std::move(d.samples), active, d.splice_s, posture.comfort), d.plan_id,
                    d.route_id, d.stamp);
      } catch (const std::invalid_argument &) {
      }
      pending.pop_front();
    }
    if (o.medium) {
      const mmm::MediumFallback reason = sw.update(b5, t, plant.state().head<2>(), route_id, active, active_goal_s);
      if (sw.following()) {
        ++r.following_cycles;
        if (infeasible[sw.plan_id()]) {
          ++r.followed_infeasible_cycles;
          followed_infeasible.insert(sw.plan_id());
        }
      } else {
        ++r.fallbacks[mmm::to_string(reason)];
      }
    }
    plant.update_world(world, t);
    const mmm::CycleOutput & out = b5.step(t, plant.state(), true, world, plant.at_rest());
    ++r.b5_cycles;
    if (out.status == mmm::CycleStatus::Ok && out.max_slack > kExecutedSlackM) {
      ++r.executed_slack_plans;
      r.max_executed_slack_m = std::max(r.max_executed_slack_m, out.max_slack);
    }
    r.end_status = mmm::to_string(out.status);
    r.time_s = t;
    if (out.status == mmm::CycleStatus::GoalReached) {
      r.reached = true;
      break;
    }
    if (out.status == mmm::CycleStatus::Failed) break;
    plant.step(out.command, out, t, dt);
    if (o.on_cycle) o.on_cycle(t, plant.state());
    if (const auto truth = plant.true_min_clearance()) r.plant_min_clearance_m = std::min(r.plant_min_clearance_m, *truth);
    r.plant_contacts += plant.contacts() > 0;
    const auto nearest = clearance.nearest(plant.state(), world.at(0.0, t));
    if (nearest) r.min_clearance_m = std::min(r.min_clearance_m, nearest->distance);
    const Joints tau = statics.torque(plant.state().tail<6>());
    for (int j = 0; j < 6; ++j)
      if (std::isfinite(torque_limits(j))) r.max_torque_ratio = std::max(r.max_torque_ratio, std::abs(tau(j)) / torque_limits(j));
  }
  r.final_state = plant.state();
  r.first_side = sides.first;
  r.homotopy_switches_after_first = sides.switches;
  r.side_sequence = sides.sequence;
  r.followed_infeasible_plans = static_cast<int>(followed_infeasible.size());
  return r;
}
}  // namespace mobile_manipulator_medium
