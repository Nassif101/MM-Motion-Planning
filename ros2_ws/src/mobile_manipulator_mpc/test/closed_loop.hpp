#pragma once
// Closed loops of the B5 controller against a kinematic plant with the base's static friction (test_controller,
// test_golden): the plant, the map world, the true 3D clearance and the gate loop.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>

#include "mobile_manipulator_mpc/controller.hpp"
#include "test_support.hpp"

namespace mpc_test
{
namespace mmn = mobile_manipulator_navigation;

inline Breakaway breakaway() { return limits().breakaway; }

// The generated model with the base's static friction: from rest the base does not move while
// |v| < breakaway linear and |omega| < breakaway yaw (the arm still moves); it counts as at rest
// again once the executed speeds fall below the rest thresholds.
struct Plant
{
  State x;
  Control executed = Control::Zero();

  bool at_rest() const { return base_at_rest(executed(0), executed(1), breakaway()); }

  void step(const Control & command, double dt)
  {
    const Breakaway b = breakaway();
    const bool from_rest = at_rest();
    executed = command;
    if (from_rest && std::abs(command(0)) < b.linear_mps && std::abs(command(1)) < b.yaw_radps) {
      executed(0) = 0.0;
      executed(1) = 0.0;
    }
    x = discrete_step(x, executed, dt).x_next;
  }
};

constexpr double kGateX = -7.225;  // ManipulationRequiredGate_1p05m: post centres, 0.4 m deep
constexpr double kGateY = -7.725;  // opening centre

inline World map_world()
{
  return world_of(mmn::known_boxes(share("mobile_manipulator_navigation") + "/maps/construction_site.boxes.yaml", nullptr));
}

// The map boxes plus scenario-like extra boxes.
inline World world_of_map_plus(std::vector<mmn::Box> extra)
{
  auto boxes = mmn::known_boxes(share("mobile_manipulator_navigation") + "/maps/construction_site.boxes.yaml", nullptr);
  boxes.insert(boxes.end(), extra.begin(), extra.end());
  return world_of(boxes);
}

// The true 3D distance of the robot to the obstacles (every primitive, every box) where they are now.
inline double obstacle_clearance(const State & x, const World & world, double now)
{
  static const CollisionModel model(primitives(), CollisionConfig{});
  const auto nearest = model.nearest(x, world.at(0.0, now));
  return nearest ? nearest->distance : std::numeric_limits<double>::infinity();
}

// A closed loop of the controller with the plant (static friction) at `period`.
struct GateLoop
{
  std::vector<CycleStatus> statuses;
  std::vector<State> states;
  std::vector<double> clearance;  // at the start of each cycle
  std::vector<double> gate_slack, gate_violation;
  std::vector<bool> gate_unsafe;
  std::vector<PostureChoice> choices;
  int holding = 0, holding_events = 0;
  bool reached = false;
  double max_collision_ms = 0.0;  // the cycle's collision step (rows, tightness, side probes)
  double closest = std::numeric_limits<double>::infinity();

  int count(CycleStatus s) const { return static_cast<int>(std::count(statuses.begin(), statuses.end(), s)); }
  void print(const char * name) const
  {
    double worst_slack = 0.0;
    for (double s : gate_slack) worst_slack = std::max(worst_slack, s);
    const State & x = states.back();
    std::printf("[%s] %zu cycles: ok %d, holding %d (%d events), fallback %d, braking %d, failed %d, goal %d; "
                "closest %.4f m, largest gated slack %.4f m; final (%.3f, %.3f, %.3f); collision step max %.1f ms\n",
                name, statuses.size(), count(CycleStatus::Ok), holding, holding_events, count(CycleStatus::Fallback),
                count(CycleStatus::Braking), count(CycleStatus::Failed), reached, closest, worst_slack, x(0), x(1),
                x(2), max_collision_ms);
  }
};

// `before` runs before every cycle (movers): it gets the cycle's time and the plant's state.
inline GateLoop run_gate_loop(Controller & c, Plant & plant, World & world, double period, double seconds,
                       const std::function<bool(const State &)> & done = nullptr,
                       const std::function<void(double, const State &)> & before = nullptr)
{
  GateLoop run;
  for (int i = 0; i * period < seconds; ++i) {
    const double now = i * period;
    if (before) before(now, plant.x);
    const double clearance = obstacle_clearance(plant.x, world, now);
    const CycleOutput & out = c.step(now, plant.x, true, world, plant.at_rest());
    run.statuses.push_back(out.status);
    run.states.push_back(plant.x);
    run.clearance.push_back(clearance);
    run.closest = std::min(run.closest, clearance);
    run.gate_slack.push_back(out.gate_slack);
    run.gate_violation.push_back(out.gate_violation);
    run.gate_unsafe.push_back(out.gate_unsafe);
    run.choices.push_back(out.posture_choice);
    run.max_collision_ms = std::max(run.max_collision_ms, 1e3 * out.timing.collision_s);
    if (out.status == CycleStatus::Holding) {
      if (run.statuses.size() < 2 || run.statuses[run.statuses.size() - 2] != CycleStatus::Holding) ++run.holding_events;
      ++run.holding;
    }
    if (out.status == CycleStatus::GoalReached) {
      run.reached = true;
      break;
    }
    if (out.status == CycleStatus::Failed) break;
    plant.step(out.command, period);
    if (done && done(plant.x)) break;
  }
  return run;
}
}  // namespace mpc_test
