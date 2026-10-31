#include "mobile_manipulator_medium/closed_loop.hpp"

#include <cmath>
#include <deque>
#include <set>
#include <sstream>

#include "mobile_manipulator_mpc/kinematic_plant.hpp"
#include "mobile_manipulator_mpc/medium_switch.hpp"
#include "mobile_manipulator_mpc/statics.hpp"

namespace mobile_manipulator_medium
{
namespace
{
constexpr double kExecutedSlackM = 1e-3;  // as the medium layer's feasibility threshold (slack_feasible_m)
constexpr uint64_t kRouteId = 1;

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
  const auto & limits = medium.problem().limits();
  const double dt = 1.0 / o.b5_rate_hz;
  const double medium_period = 1.0 / medium.problem().config().rate_hz;
  const Joints torque_limits = medium.problem().config().torque_limits_nm;
  mmm::KinematicPlant plant{start, mmm::Control::Zero(), limits.breakaway};
  const mmm::CollisionModel clearance(primitives, mmm::CollisionConfig{});
  const mmm::GeneratedStaticTorque statics;
  mmm::MediumSwitch sw(b5_config);
  const double goal_s = route.project(Eigen::Vector2d(goal[0], goal[1]), route.length(), 1.0).s;
  struct Delivery
  {
    double at;
    std::vector<mmm::MediumSample> samples;
    double splice_s, stamp;
    uint64_t plan_id;
  };
  std::deque<Delivery> pending;
  std::map<uint64_t, bool> infeasible;  // plan_id -> published infeasible
  std::set<uint64_t> followed_infeasible;
  SideTracker sides;

  b5.start(route, goal, {}, posture);
  if (o.medium) medium.set_route(route, goal, kRouteId);
  ClosedLoopResult r;
  r.end_status = "timeout";
  double next_medium = 0.0;
  for (long k = 0;; ++k) {
    const double t = k * dt;
    if (t > o.timeout_s) break;
    if (o.medium && t >= next_medium - 1e-9) {
      next_medium += medium_period;
      Eigen::Matrix<double, 9, 1> rate;
      rate << plant.executed(0) * std::cos(plant.x(2)), plant.executed(0) * std::sin(plant.x(2)), plant.executed(1),
        plant.executed.tail<6>();
      const PlanOutput & out = medium.plan(t, plant.x, rate, world);
      ++r.medium_cycles;
      r.medium_solve_s.push_back(out.solve_s);
      r.medium_capped += out.capped;
      r.medium_infeasible += out.status == "infeasible";
      sides.observe(out.homotopy);
      infeasible[out.plan_id] = out.status == "infeasible";
      if (out.samples.size() >= 2) {
        const double latency = std::isnan(o.fixed_latency_s) ? out.solve_s : o.fixed_latency_s;
        Delivery d{t + std::ceil(latency / dt - 1e-9) * dt, {}, out.splice_s, out.planned_at, out.plan_id};
        for (const auto & s : out.samples) d.samples.push_back({s.s, s.x, s.y, s.yaw, s.speed, s.q});
        pending.push_back(std::move(d));
      }
    }
    while (!pending.empty() && pending.front().at <= t + 1e-9) {
      Delivery & d = pending.front();
      sw.set_plan(mmm::MediumReference(std::move(d.samples), route, d.splice_s, posture.comfort), d.plan_id, kRouteId,
                  d.stamp);
      pending.pop_front();
    }
    if (o.medium) {
      const mmm::MediumFallback reason = sw.update(b5, t, plant.x.head<2>(), kRouteId, route, goal_s);
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
    const mmm::CycleOutput & out = b5.step(t, plant.x, true, world, plant.at_rest());
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
    plant.step(out.command, dt);
    const auto nearest = clearance.nearest(plant.x, world.at(0.0, t));
    if (nearest) r.min_clearance_m = std::min(r.min_clearance_m, nearest->distance);
    const Joints tau = statics.torque(plant.x.tail<6>());
    for (int j = 0; j < 6; ++j)
      if (std::isfinite(torque_limits(j))) r.max_torque_ratio = std::max(r.max_torque_ratio, std::abs(tau(j)) / torque_limits(j));
  }
  r.final_state = plant.x;
  r.first_side = sides.first;
  r.homotopy_switches_after_first = sides.switches;
  r.side_sequence = sides.sequence;
  r.followed_infeasible_plans = static_cast<int>(followed_infeasible.size());
  return r;
}
}  // namespace mobile_manipulator_medium
