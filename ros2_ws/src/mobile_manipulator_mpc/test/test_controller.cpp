// The B5 controller loop (no ROS): the breakaway output stage, the in-place turn from rest
// against a plant with static friction, and the failure handling (fallback, braking, stale
// state, goal).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/controller.hpp"
#include "test_support.hpp"

using namespace mobile_manipulator_mpc;
using namespace mpc_test;
using Eigen::Vector2d;

namespace mobile_manipulator_mpc
{
void PrintTo(CycleStatus status, std::ostream * os) { *os << to_string(status); }
}  // namespace mobile_manipulator_mpc

namespace
{
Breakaway breakaway() { return limits().breakaway; }

Control base_command(double v, double omega)
{
  Control u = Control::Zero();
  u(0) = v;
  u(1) = omega;
  return u;
}

// OSQP, but the calls for which `fail` returns true report a failure without solving.
class ScriptedSolver : public QpSolver
{
public:
  explicit ScriptedSolver(std::function<bool(int)> fail) : fail_(std::move(fail)), inner_(solver_options()) {}
  void setup(const QpProblem & shape) override { inner_.setup(shape); }
  SolveResult solve(const QpProblem & problem, const QpSolution * warm) override
  {
    if (fail_(calls++)) {
      SolveResult result;
      result.status = "scripted failure";
      return result;
    }
    return inner_.solve(problem, warm);
  }
  int calls = 0;

private:
  std::function<bool(int)> fail_;
  OsqpSolver inner_;
};

// The forward speed one braking cycle after `v`: down by decel * dt, zero inside the dead zone.
double braked(double v)
{
  const double next = std::max(0.0, v - limits().decel * config().dt);
  return next < breakaway().dead_linear_mps ? 0.0 : next;
}

std::unique_ptr<Controller> make_controller(std::unique_ptr<QpSolver> solver, int fallback_cycles = 3)
{
  return std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::move(solver),
                                      breakaway(), fallback_cycles);
}

std::unique_ptr<Controller> make_controller() { return make_controller(std::make_unique<OsqpSolver>(solver_options())); }

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

struct Timings
{
  std::vector<double> total, solve, linearize, collision, setup;
  std::vector<int> iterations;
  void add(const CycleOutput & out)
  {
    if (out.status != CycleStatus::Ok) return;
    total.push_back(out.timing.total_s * 1e3);
    solve.push_back(out.timing.solve_s * 1e3);
    linearize.push_back(out.timing.linearize_s * 1e3);
    collision.push_back(out.timing.collision_s * 1e3);
    setup.push_back(out.timing.setup_s * 1e3);
    iterations.push_back(out.iterations);
  }
  static double median(std::vector<double> v)
  {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  }
  void print(const char * name)
  {
    if (total.empty()) return;
    std::vector<double> t = total, s = solve;
    std::sort(t.begin(), t.end());
    std::sort(s.begin(), s.end());
    std::sort(iterations.begin(), iterations.end());
    const size_t n = t.size();
    std::printf("[%s] %zu Ok cycles: total ms median %.2f p90 %.2f max %.2f; solve ms median %.2f max %.2f; "
                "iterations median %d max %d; medians ms linearize %.3f collision %.3f setup %.3f\n",
                name, n, t[n / 2], t[n * 9 / 10], t.back(), s[n / 2], s.back(), iterations[n / 2], iterations.back(),
                median(linearize), median(collision), median(setup));
  }
};

// A straight 10 m route along x from the origin; drives `cycles` cycles from rest.
struct Drive
{
  Route route{{Vector2d(0, 0), Vector2d(10, 0)}};
  World world;
  Plant plant{state(0, 0, 0, posture().comfort)};
  double dt = config().dt;
  int cycle = 0;
  Timings timings;

  explicit Drive(Controller & c) { c.start(route, {10.0, 0.0, 0.0}, {}, posture()); }

  const CycleOutput & step(Controller & c, bool fresh = true)
  {
    const CycleOutput & out = c.step(cycle * dt, plant.x, fresh, world, plant.at_rest());
    timings.add(out);
    plant.step(out.command, dt);
    ++cycle;
    return out;
  }
};
}  // namespace

// ---- output stage ----

TEST(BreakawayOutput, LiftsSubBreakawayCommandAtRest)
{
  const Breakaway b = breakaway();
  Control planned = base_command(0.0, 0.03);
  planned(4) = 0.2;  // arm velocities pass unchanged
  Control out = breakaway_output(planned, true, b);
  EXPECT_DOUBLE_EQ(out(0), 0.0);
  EXPECT_DOUBLE_EQ(out(1), 0.07);
  EXPECT_DOUBLE_EQ(out(4), 0.2);
  EXPECT_DOUBLE_EQ(breakaway_output(base_command(0.0, -0.03), true, b)(1), -0.07);
  EXPECT_DOUBLE_EQ(breakaway_output(base_command(0.02, 0.0), true, b)(0), 0.0375);
  // Both below: scaled together until the larger one (relative to its breakaway) reaches it,
  // keeping the curvature.
  out = breakaway_output(base_command(0.02, 0.03), true, b);
  EXPECT_DOUBLE_EQ(out(0), 0.0375);
  EXPECT_NEAR(out(1), 0.03 * 0.0375 / 0.02, 1e-12);
  // One component already starts the base: unchanged.
  out = breakaway_output(base_command(0.05, 0.03), true, b);
  EXPECT_DOUBLE_EQ(out(0), 0.05);
  EXPECT_DOUBLE_EQ(out(1), 0.03);
}

TEST(BreakawayOutput, LeavesMovingBaseUnchanged)
{
  const Control out = breakaway_output(base_command(0.02, 0.03), false, breakaway());
  EXPECT_DOUBLE_EQ(out(0), 0.02);
  EXPECT_DOUBLE_EQ(out(1), 0.03);
  EXPECT_TRUE(base_at_rest(0.005, 0.01, breakaway()));
  EXPECT_FALSE(base_at_rest(0.02, 0.0, breakaway()));
  EXPECT_FALSE(base_at_rest(0.0, 0.03, breakaway()));
}

TEST(BreakawayOutput, ZeroBelowDeadZone)
{
  for (bool at_rest : {true, false}) {
    const Control out = breakaway_output(base_command(0.004, -0.009), at_rest, breakaway());
    EXPECT_DOUBLE_EQ(out(0), 0.0);
    EXPECT_DOUBLE_EQ(out(1), 0.0);
  }
  // Only the component inside its dead zone is zeroed; the other is lifted from rest.
  const Control out = breakaway_output(base_command(0.004, 0.03), true, breakaway());
  EXPECT_DOUBLE_EQ(out(0), 0.0);
  EXPECT_DOUBLE_EQ(out(1), 0.07);
}

TEST(BreakawayOutput, LiftsExactlyToTheBreakaway)
{
  // The dominant component lands exactly on its breakaway: dividing by the ratio can land one ULP
  // below it, which a plant with a strict threshold would not start from.
  const Breakaway b = breakaway();
  const int n = 2000;
  for (int i = 1; i < n; ++i) {
    const double v = b.dead_linear_mps + i * (b.linear_mps - b.dead_linear_mps) / n;
    const double w = b.dead_yaw_radps + i * (b.yaw_radps - b.dead_yaw_radps) / n;
    ASSERT_EQ(breakaway_output(base_command(v, 0.0), true, b)(0), b.linear_mps) << "v " << v;
    ASSERT_EQ(breakaway_output(base_command(0.0, w), true, b)(1), b.yaw_radps) << "omega " << w;
    ASSERT_EQ(breakaway_output(base_command(0.0, -w), true, b)(1), -b.yaw_radps) << "omega " << -w;
    // Both components: the dominant one exact, the other scaled by the same factor (omega kept
    // outside its dead zone).
    const double small_w = std::max(0.5 * v / b.linear_mps * b.yaw_radps, 2.0 * b.dead_yaw_radps);
    if (small_w / b.yaw_radps >= v / b.linear_mps) continue;
    const Control both = breakaway_output(base_command(v, small_w), true, b);
    ASSERT_EQ(both(0), b.linear_mps) << "v " << v;
    ASSERT_NEAR(both(1), small_w * b.linear_mps / v, 1e-12);
  }
}

// ---- controller ----

TEST(Controller, TurnsInPlaceFromRestToGoalBehind)
{
  // The 2026-10-06 probe: at rest at (12, 0) facing -pi, goal (15, 0) facing 0, 3 m behind, sent
  // right after the previous goal (at (12, 0)) was reached, as a mission's next goal. At the 20 Hz
  // bring-up rate the rate band from the zero command (0.8 rad/s^2 * 0.05 s = 0.04 rad/s) is below
  // the yaw breakaway, so the turn only starts through the output stage.
  const double period = 0.05;
  auto c = make_controller();
  const World world;
  Plant plant{state(12, 0, -M_PI, posture().comfort)};
  int i = 0;
  c->start(Route({Vector2d(13, 0), Vector2d(12, 0)}), {12.0, 0.0, -M_PI}, {}, posture());
  for (; i < 20; ++i) {
    const CycleOutput & out = c->step(i * period, plant.x, true, world, plant.at_rest());
    ASSERT_EQ(out.status, CycleStatus::GoalReached);
    plant.step(out.command, period);
  }

  c->start(Route({Vector2d(12, 0), Vector2d(15, 0)}), {15.0, 0.0, 0.0}, {}, posture());
  const int first = i;
  const Limits & l = limits();
  const Control band_lo = lower_rate(l, period), band_hi = upper_rate(l, period);
  Timings timings;
  bool reached = false;
  int stuck = 0, longest_stuck = 0, lifted = 0;
  Control last = Control::Zero();
  for (; (i - first) * period < 30.0; ++i) {
    const bool at_rest = plant.at_rest();
    const CycleOutput & out = c->step(i * period, plant.x, true, world, at_rest);
    timings.add(out);
    if (out.status == CycleStatus::GoalReached) {
      reached = true;
      break;
    }
    ASSERT_TRUE(out.status == CycleStatus::Ok || out.status == CycleStatus::Fallback) << to_string(out.status);
    ASSERT_GE(out.command(0), 0.0) << "reversing at cycle " << i;
    // lifted: the command differs from the plan's first control within the rate band
    const Control banded = out.plan.u[0].cwiseMax(last + band_lo).cwiseMin(last + band_hi);
    if (at_rest && (out.command.head<2>() - banded.head<2>()).cwiseAbs().maxCoeff() > 1e-9) ++lifted;
    plant.step(out.command, period);
    last = out.command;
    // a non-zero base command the plant did not execute
    stuck = (out.command.head<2>().norm() > 0.0 && plant.executed.head<2>().norm() == 0.0) ? stuck + 1 : 0;
    longest_stuck = std::max(longest_stuck, stuck);
  }
  timings.print("goal behind, 20 Hz");
  const State & x = plant.x;
  std::printf("goal behind: reached %d after %.2f s at (%.3f, %.3f, %.3f); output lifted %d times; longest stall %d "
              "cycles\n",
              reached, (i - first) * period, x(0), x(1), x(2), lifted, longest_stuck);
  EXPECT_TRUE(reached) << "final pose " << x(0) << " " << x(1) << " " << x(2);
  EXPECT_LT((x.head<2>() - Vector2d(15, 0)).norm(), kGoalXyM);
  EXPECT_LT(std::abs(std::remainder(x(2), 2 * M_PI)), kGoalYawRad);
  EXPECT_GE(lifted, 1) << "the start from rest never needed the output stage";
  EXPECT_EQ(longest_stuck, 0) << "the base stalled on a sub-breakaway command";
}

TEST(Controller, ApproachTapersToTheBreakawayWithinTheBrakingLimit)
{
  // Ruling R25: the final approach slows along the progress taper, so the base arrives near its
  // breakaway speed (GoalReached then zeroes the command: a step no larger than that) and the
  // commanded deceleration stays within the envelope's braking limit on the way in. 20 Hz, from
  // rest on a straight 4 m route, the plant with static friction.
  const double period = 0.05;
  auto c = make_controller();
  const World world;
  Plant plant{state(0, 0, 0, posture().comfort)};
  const Vector2d goal(4.0, 0.0);
  c->start(Route({Vector2d(0, 0), goal}), {goal(0), goal(1), 0.0}, {}, posture());
  const Limits & l = limits();
  double last_v = 0.0, worst_decel = 0.0, arrival = -1.0, cruise = 0.0;
  bool reached = false;
  for (int i = 0; i * period < 40.0; ++i) {
    const CycleOutput & out = c->step(i * period, plant.x, true, world, plant.at_rest());
    if (out.status == CycleStatus::GoalReached) {
      reached = true;
      arrival = last_v;  // the command sent while the base entered the goal tolerance
      break;
    }
    ASSERT_TRUE(out.status == CycleStatus::Ok || out.status == CycleStatus::Fallback) << to_string(out.status);
    const double v = out.command(0);
    worst_decel = std::max(worst_decel, (last_v - v) / period);
    cruise = std::max(cruise, v);
    if ((plant.x.head<2>() - goal).norm() <= kGoalXyM) arrival = std::max(arrival, v);
    plant.step(out.command, period);
    last_v = v;
  }
  std::printf("approach: reached %d, cruise %.3f m/s, arrival command %.3f m/s, largest commanded decel %.3f m/s^2\n",
              reached, cruise, arrival, worst_decel);
  ASSERT_TRUE(reached);
  EXPECT_GT(cruise, 0.25) << "never cruised";
  EXPECT_LE(arrival, 0.06) << "arrived too fast for a breakaway-sized stop";
  EXPECT_LE(worst_decel, l.decel + 1e-9);
}

TEST(Controller, FallbackUsesShiftedPlan)
{
  const int fail_at = 40;  // cruising: no limit is active
  auto solver = std::make_unique<ScriptedSolver>([&](int call) { return call == fail_at; });
  auto c = make_controller(std::move(solver));
  Drive drive(*c);
  Plan previous;
  for (int i = 0; i < fail_at; ++i) {
    const CycleOutput & out = drive.step(*c);
    ASSERT_EQ(out.status, CycleStatus::Ok) << "cycle " << i;
    previous = out.plan;
  }
  ASSERT_GT(previous.u[0](0), 0.25) << "not cruising";
  const State measured = drive.plant.x;
  const CycleOutput & fallback = drive.step(*c);
  ASSERT_EQ(fallback.status, CycleStatus::Fallback);
  for (int i = 0; i < kNu; ++i) EXPECT_NEAR(fallback.command(i), previous.u[1](i), 1e-12) << "control " << i;
  for (int k = 0; k + 1 < config().horizon; ++k)
    EXPECT_TRUE(fallback.plan.u[k].isApprox(previous.u[k + 1], 1e-12)) << "stage " << k;
  EXPECT_EQ(fallback.iterations, 0);
  // The shifted plan's states are rolled out from the state measured now: its time origin is now.
  EXPECT_DOUBLE_EQ(fallback.plan_time, (drive.cycle - 1) * drive.dt);
  EXPECT_TRUE(fallback.plan.x[0].isApprox(measured, 1e-12)) << "stage 0 is not the measured state";
  // The next solve succeeds: back to Ok, its plan starting now.
  const CycleOutput & ok = drive.step(*c);
  EXPECT_EQ(ok.status, CycleStatus::Ok);
  EXPECT_DOUBLE_EQ(ok.plan_time, (drive.cycle - 1) * drive.dt);
}

TEST(Controller, FallbackBudgetFollowsTheRate)
{
  // Ruling R27: the fallback budget is time (wholebody_mpc.yaml fallback_s), K = ceil(s * rate).
  EXPECT_DOUBLE_EQ(config().fallback_s, 0.3);
  EXPECT_EQ(fallback_cycles_for(config().fallback_s, 20.0), 6);
  EXPECT_EQ(fallback_cycles_for(config().fallback_s, 50.0), 15);
  EXPECT_EQ(fallback_cycles_for(0.3, 10.0), 3);
  EXPECT_EQ(fallback_cycles_for(0.3, 30.0), 9);   // 9.000000000000002 must not round up
  EXPECT_EQ(fallback_cycles_for(0.3, 33.0), 10);  // 9.9 cycles cover at least 0.3 s
  EXPECT_THROW(fallback_cycles_for(0.0, 20.0), std::invalid_argument);
  EXPECT_THROW(fallback_cycles_for(0.3, 0.0), std::invalid_argument);
  // The node's statistics reserve (R33-M8): an hour of cycles at its rate.
  EXPECT_EQ(cycles_in(3600.0, 20.0), 72000);
  EXPECT_EQ(cycles_in(3600.0, 50.0), 180000);
  EXPECT_THROW(cycles_in(3600.0, 0.0), std::invalid_argument);
  // The controller rides out exactly K failures in a row at each rate, then brakes.
  for (const double rate : {20.0, 50.0}) {
    const int k = fallback_cycles_for(config().fallback_s, rate);
    const int first_failure = 40;
    auto c = make_controller(std::make_unique<ScriptedSolver>([&](int call) { return call >= first_failure; }), k);
    Drive drive(*c);
    for (int i = 0; i < first_failure; ++i) drive.step(*c);
    for (int i = 0; i < k; ++i) ASSERT_EQ(drive.step(*c).status, CycleStatus::Fallback) << rate << " Hz, failure " << i + 1;
    EXPECT_EQ(drive.step(*c).status, CycleStatus::Braking) << rate << " Hz, after K = " << k;
  }
}

TEST(Controller, BrakesAfterKFailedCycles)
{
  const int first_failure = 40;
  auto owned = std::make_unique<ScriptedSolver>([&](int call) { return call >= first_failure; });
  ScriptedSolver * solver = owned.get();
  auto c = make_controller(std::move(owned), 3);
  Drive drive(*c);
  Control last = Control::Zero();
  for (int i = 0; i < first_failure; ++i) last = drive.step(*c).command;
  ASSERT_GT(last(0), 0.25);
  for (int i = 0; i < 3; ++i) {
    const CycleOutput & out = drive.step(*c);
    ASSERT_EQ(out.status, CycleStatus::Fallback) << "failure " << i + 1;
    last = out.command;
  }
  const Limits & l = limits();
  const double dt = config().dt;
  bool failed = false;
  int braking = 0;
  for (int i = 0; i < 100 && !failed; ++i) {
    const CycleOutput & out = drive.step(*c);
    if (i == 0) ASSERT_EQ(out.status, CycleStatus::Braking) << "after K = 3 failures";
    ASSERT_TRUE(out.status == CycleStatus::Braking || out.status == CycleStatus::Failed) << to_string(out.status);
    const Control & u = out.command;
    // at the braking limit (or to zero)
    EXPECT_NEAR(u(0), braked(last(0)), 1e-12) << "cycle " << i;
    EXPECT_LE(std::abs(u(1) - last(1)), l.yaw_decel * dt + 1e-12);
    EXPECT_LE(std::abs(u(1)), std::abs(last(1)));
    EXPECT_TRUE(u.tail<6>().isZero()) << "arm velocities";
    for (const auto & stage : out.plan.u) EXPECT_TRUE(stage.tail<6>().isZero());
    for (const auto & x : out.plan.x) EXPECT_TRUE(x.tail<6>().isApprox(out.plan.x[0].tail<6>()));
    failed = out.status == CycleStatus::Failed;
    if (failed)
      EXPECT_TRUE(u.head<2>().isZero());
    else
      ++braking;
    last = u;
  }
  std::printf("braking: %d cycles, solver calls %d\n", braking, solver->calls);
  EXPECT_TRUE(failed);
  EXPECT_GE(braking, 5);  // 0.3 m/s at 0.5 m/s^2 is 0.6 s
  const int calls = solver->calls;
  const CycleOutput & after = drive.step(*c);
  EXPECT_EQ(after.status, CycleStatus::Failed) << "latched";
  EXPECT_TRUE(after.command.isZero());
  EXPECT_EQ(solver->calls, calls) << "no solve once braking";
  EXPECT_EQ(calls, first_failure + 4);
}

TEST(Controller, BrakesWhenStateIsStale)
{
  auto owned = std::make_unique<ScriptedSolver>([](int) { return false; });
  ScriptedSolver * solver = owned.get();
  auto c = make_controller(std::move(owned));
  Drive drive(*c);
  Control last = Control::Zero();
  for (int i = 0; i < 40; ++i) last = drive.step(*c).command;
  drive.timings.print("straight");
  ASSERT_GT(last(0), 0.25);
  const int calls = solver->calls;
  const CycleOutput & first = drive.step(*c, false);
  EXPECT_EQ(first.status, CycleStatus::StaleState);
  EXPECT_NEAR(first.command(0), last(0) - limits().decel * config().dt, 1e-12) << "brakes at once, at the limit";
  EXPECT_TRUE(first.command.tail<6>().isZero());
  last = first.command;
  bool failed = false;
  for (int i = 0; i < 20 && !failed; ++i) {
    const CycleOutput & out = drive.step(*c, false);
    EXPECT_NEAR(out.command(0), braked(last(0)), 1e-12);
    EXPECT_TRUE(out.command.tail<6>().isZero());
    failed = out.status == CycleStatus::Failed;
    if (!failed) EXPECT_EQ(out.status, CycleStatus::StaleState);
    last = out.command;
  }
  EXPECT_TRUE(failed) << "stopped but not failed";
  EXPECT_EQ(solver->calls, calls) << "solved from a stale state";
  // Fresh again: still failed until a new goal.
  EXPECT_EQ(drive.step(*c).status, CycleStatus::Failed);
  drive.plant.executed.setZero();
  c->start(drive.route, {10.0, 0.0, 0.0}, {}, posture());
  EXPECT_EQ(drive.step(*c).status, CycleStatus::Ok);
}

TEST(Controller, ReportsGoalReachedWithinTolerance)
{
  auto c = make_controller();
  const Route route({Vector2d(0, 0), Vector2d(3, 0)});
  const World world;
  struct Case
  {
    State x;
    bool reached;
  };
  const std::vector<Case> cases = {
    {state(2.9, 0.05, 0.1, posture().comfort), true},  // 0.112 m, 0.1 rad
    {state(3.0, 0.14, -0.14, posture().comfort), true},
    {state(2.8, 0.0, 0.0, posture().comfort), false},  // 0.2 m
    {state(3.0, 0.0, 0.2, posture().comfort), false},  // 0.2 rad
    {state(3.0, 0.0, 2 * M_PI - 0.1, posture().comfort), true},  // wrapped
  };
  for (const auto & tc : cases) {
    c->start(route, {3.0, 0.0, 0.0}, {}, posture());
    const CycleOutput & out = c->step(0.0, tc.x, true, world, true);
    EXPECT_EQ(out.status == CycleStatus::GoalReached, tc.reached)
      << to_string(out.status) << " at " << tc.x(0) << " " << tc.x(1) << " " << tc.x(2);
    if (tc.reached) {
      EXPECT_TRUE(out.command.isZero());
      // latched: holds while the goal stays active
      EXPECT_EQ(c->step(0.1, tc.x, true, world, true).status, CycleStatus::GoalReached);
    }
  }
}

TEST(Controller, ClampsMeasuredJointsIntoBounds)
{
  // JTC tracking errors reach 0.09 rad: a measurement past a joint bound must not make the QP
  // infeasible (the joint can leave at most qdd * dt^2 per stage from rest).
  auto c = make_controller();
  const Limits & l = limits();
  Joints q = posture().comfort;
  q(4) = l.q_max(4) + 0.05;  // wrist_2
  q(0) = l.q_min(0) - 0.05;  // shoulder_pan
  const Route route({Vector2d(0, 0), Vector2d(5, 0)});
  c->start(route, {5.0, 0.0, 0.0}, {}, posture());
  const CycleOutput & out = c->step(0.0, state(0, 0, 0, q), true, World(), true);
  EXPECT_EQ(out.status, CycleStatus::Ok) << to_string(out.status);
  // the plan starts at the clamped joints (within the solver's tolerance on the fixed x0)
  EXPECT_NEAR(out.plan.x[0](3 + 4), l.q_max(4), 1e-5);
  EXPECT_NEAR(out.plan.x[0](3 + 0), l.q_min(0), 1e-5);
  for (const auto & x : out.plan.x)
    for (int j = 0; j < 6; ++j) {
      EXPECT_GE(x(3 + j), l.q_min(j) - 2e-3);
      EXPECT_LE(x(3 + j), l.q_max(j) + 2e-3);
    }
}

TEST(Controller, SentCommandRespectsRateLimitRelativeToLastCommand)
{
  // Every command sent changes from the one before by at most the rate band (the breakaway lift
  // from rest aside), also on the stage the QP only enforces within its tolerance.
  auto c = make_controller();
  const Route route({Vector2d(0, 0), Vector2d(5, 0), Vector2d(5, 5)});
  c->start(route, {5.0, 5.0, M_PI / 2}, {}, posture());
  Plant plant{state(0, 0, 0, posture().comfort)};
  const World world;
  const Limits & l = limits();
  const double dt = config().dt;
  Control last = Control::Zero();
  for (int i = 0; i < 450; ++i) {
    const bool at_rest = plant.at_rest();
    const CycleOutput & out = c->step(i * dt, plant.x, true, world, at_rest);
    if (out.status == CycleStatus::GoalReached) break;
    const Control & u = out.command;
    EXPECT_EQ(c->last_command(), u);
    if (!at_rest) {
      EXPECT_GE(u(0) - last(0), -l.decel * dt - 1e-12) << "cycle " << i;
      EXPECT_LE(u(0) - last(0), l.accel * dt + 1e-12) << "cycle " << i;
      EXPECT_LE(std::abs(u(1) - last(1)), l.yaw_accel * dt + 1e-12) << "cycle " << i;
    }
    for (int j = 0; j < 6; ++j) EXPECT_LE(std::abs(u(2 + j) - last(2 + j)), l.qdd_max(j) * dt + 1e-12);
    plant.step(u, dt);
    last = u;
  }
  EXPECT_GT(plant.x(1), 2.0) << "did not drive into the second leg";
}


TEST(Controller, ResetCommandRestartsTheRateBandFromTheSentCommand)
{
  // The node publishes a zero command itself (cancel, goal end): the next goal's rate band starts
  // from what was really sent, not from the cruise command the controller sent last.
  auto c = make_controller();
  Drive drive(*c);
  Control last = Control::Zero();
  for (int i = 0; i < 40; ++i) last = drive.step(*c).command;
  ASSERT_GT(last(0), 0.25);
  c->reset_command();
  EXPECT_TRUE(c->last_command().isZero());
  c->start(drive.route, {10.0, 0.0, 0.0}, {}, posture());
  drive.plant.executed.setZero();  // the zero command stopped the base
  const double dt = config().dt;
  const CycleOutput & from_zero = drive.step(*c);
  ASSERT_EQ(from_zero.status, CycleStatus::Ok);
  EXPECT_LE(from_zero.command(0), limits().accel * dt + 1e-12);
  for (int j = 0; j < 6; ++j) EXPECT_LE(std::abs(from_zero.command(2 + j)), limits().qdd_max(j) * dt + 1e-12);

  // A non-zero command sent by the node becomes the band's centre.
  c->reset_command(base_command(0.1, 0.0));
  EXPECT_DOUBLE_EQ(c->last_command()(0), 0.1);
  const CycleOutput & from_sent = drive.step(*c);
  EXPECT_GE(from_sent.command(0), 0.1 - limits().decel * dt - 1e-12);
  EXPECT_LE(from_sent.command(0), 0.1 + limits().accel * dt + 1e-12);
}

TEST(Controller, MissedDeadlinesCountAsFailures)
{
  // A cycle that overran its period still sends its result but counts as a failure: more than K
  // failures in a row (late cycles or solver failures, one per cycle) brake from the next cycle.
  const int fail_at = 44;  // solver calls count from 0: cycles 0..39, 40, 41..43, then this one
  auto c = make_controller(std::make_unique<ScriptedSolver>([&](int call) { return call == fail_at; }), 3);
  Drive drive(*c);
  for (int i = 0; i < 40; ++i) ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok) << "cycle " << i;
  // One late cycle between on-time ones: no braking.
  ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok);
  c->note_failure();
  for (int i = 0; i < 3; ++i) ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok) << "after one miss";
  // Solver call 44 fails and that cycle is also late: one failure, not two.
  ASSERT_EQ(drive.step(*c).status, CycleStatus::Fallback);
  c->note_failure();
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok) << "late cycle " << i + 2;
    c->note_failure();
  }
  ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok) << "three failures in a row are within K = 3";
  c->note_failure();  // the fourth
  const CycleOutput & braking = drive.step(*c);
  EXPECT_EQ(braking.status, CycleStatus::Braking);
  EXPECT_TRUE(braking.command.tail<6>().isZero());
}

TEST(Controller, NoteFailureOutsideTrackingIsIgnored)
{
  auto c = make_controller();
  const Route route({Vector2d(0, 0), Vector2d(3, 0)});
  c->start(route, {3.0, 0.0, 0.0}, {}, posture());
  const State x = state(3.0, 0.0, 0.0, posture().comfort);
  ASSERT_EQ(c->step(0.0, x, true, World(), true).status, CycleStatus::GoalReached);
  for (int i = 0; i < 5; ++i) c->note_failure();
  EXPECT_EQ(c->step(0.1, x, true, World(), true).status, CycleStatus::GoalReached);
}

TEST(Controller, UpdateGoalKeepsFailureCount)
{
  // A preempting FollowPath (a replanned route) keeps the failures in a row: three fallbacks, the
  // route update, then one more failure brakes (start() would have reset the count).
  const int first_failure = 40;
  auto c = make_controller(std::make_unique<ScriptedSolver>([&](int call) { return call >= first_failure; }), 3);
  Drive drive(*c);
  for (int i = 0; i < first_failure; ++i) ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok);
  for (int i = 0; i < 3; ++i) ASSERT_EQ(drive.step(*c).status, CycleStatus::Fallback);
  const double x = drive.plant.x(0);
  c->update_goal(Route({Vector2d(x, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {});
  EXPECT_EQ(drive.step(*c).status, CycleStatus::Braking);
}

TEST(Controller, UpdateGoalKeepsThePlanAndMovesTheGoal)
{
  auto c = make_controller();
  Drive drive(*c);
  for (int i = 0; i < 30; ++i) ASSERT_EQ(drive.step(*c).status, CycleStatus::Ok);
  const Control before = c->last_command();
  ASSERT_GT(before(0), 0.25);
  // A shorter route from the robot: the goal moves, the plan stays warm (no cold restart: the
  // first plan after the update still cruises, its controls continue the previous ones).
  const double x = drive.plant.x(0);
  const double goal = x + 1.5;
  c->update_goal(Route({Vector2d(x, 0), Vector2d(goal, 0)}), {goal, 0.0, 0.0}, {});
  const CycleOutput & first = drive.step(*c);
  ASSERT_EQ(first.status, CycleStatus::Ok);
  EXPECT_GT(first.plan.u[1](0), 0.2) << "the plan restarted from rest";
  bool reached = false;
  for (int i = 0; i < 300 && !reached; ++i) reached = drive.step(*c).status == CycleStatus::GoalReached;
  EXPECT_TRUE(reached);
  EXPECT_LT(std::abs(drive.plant.x(0) - goal), kGoalXyM);
}

TEST(Controller, ProgressStaysOnTheCurrentBranchOfAURoute)
{
  // Review R31: a U-route whose return leg runs 0.24 m beside the outgoing one. Cruising on the
  // outgoing leg, a lateral error of 0.14 m puts the measured base nearer the return leg; its
  // route progress must stay on the outgoing branch (projected around the previous cycle's), so
  // the base keeps driving out along +x. A global projection of stage 0 snapped to the return
  // leg, about 1 m before the goal, and the plan turned the base back towards the goal.
  const double gap = 0.24, leg = 4.0;
  auto c = make_controller();
  const World world;
  const double dt = config().dt;
  Plant plant{state(0, 0, 0, posture().comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(leg, 0), Vector2d(leg, gap), Vector2d(0, gap)}), {0.0, gap, M_PI}, {},
           posture());
  int i = 0;
  for (; i < 30; ++i) {
    const CycleOutput & out = c->step(i * dt, plant.x, true, world, plant.at_rest());
    ASSERT_EQ(out.status, CycleStatus::Ok) << "cycle " << i;
    plant.step(out.command, dt);
  }
  ASSERT_GT(plant.x(0), 0.3);
  ASSERT_LT(plant.x(0), 1.5);
  plant.x(1) += 0.14;  // nearer the return leg (0.10 m) than the outgoing one (0.14 m)
  const double x_disturbed = plant.x(0);
  double min_v = std::numeric_limits<double>::infinity();
  for (const int end = i + 20; i < end; ++i) {
    const CycleOutput & out = c->step(i * dt, plant.x, true, world, plant.at_rest());
    ASSERT_EQ(out.status, CycleStatus::Ok) << "cycle " << i;
    min_v = std::min(min_v, out.command(0));
    plant.step(out.command, dt);
  }
  EXPECT_GT(min_v, 0.15) << "the base slowed down as if it were on the return leg";
  EXPECT_GT(plant.x(0), x_disturbed + 0.3) << "the base stopped driving out along the outgoing leg";
  EXPECT_LT(std::abs(std::remainder(plant.x(2), 2 * M_PI)), M_PI / 4) << "the base turned back";
}

TEST(Controller, FallbackPlanStartsAtTheMeasuredStateNowAt20Hz)
{
  // At 20 Hz a fallback half a stage after the last solve advances the plan by no whole stage, but
  // its states are rolled out from the state measured now: the plan's time origin is now (the
  // node streams stage k at k dt from now).
  const double period = 0.05;
  const int fail_at = 41;
  auto c = make_controller(std::make_unique<ScriptedSolver>([&](int call) { return call == fail_at; }));
  const World world;
  Plant plant{state(0, 0, 0, posture().comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(10, 0)}), {10.0, 0.0, 0.0}, {}, posture());
  for (int i = 0; i < fail_at; ++i) {
    const CycleOutput & out = c->step(i * period, plant.x, true, world, plant.at_rest());
    ASSERT_EQ(out.status, CycleStatus::Ok) << "cycle " << i;
    EXPECT_DOUBLE_EQ(out.plan_time, i * period);
    plant.step(out.command, period);
  }
  const State measured = plant.x;
  const CycleOutput & fallback = c->step(fail_at * period, measured, true, world, plant.at_rest());
  ASSERT_EQ(fallback.status, CycleStatus::Fallback);
  EXPECT_DOUBLE_EQ(fallback.plan_time, fail_at * period);
  EXPECT_TRUE(fallback.plan.x[0].isApprox(measured, 1e-12));
}

// ---- slack gate (spec Section 3, amended 2026-10-13) ----

namespace
{
namespace mmn = mobile_manipulator_navigation;
using Eigen::Vector3d;

constexpr double kGateX = -7.225;  // ManipulationRequiredGate_1p05m: post centres, 0.4 m deep
constexpr double kGateY = -7.725;  // opening centre

World map_world()
{
  return world_of(mmn::known_boxes(share("mobile_manipulator_navigation") + "/maps/construction_site.boxes.yaml", nullptr));
}

const SlackGate & gate() { return config().slack_gate; }

std::unique_ptr<Controller> gated_controller(bool enabled = true, std::unique_ptr<QpSolver> solver = nullptr)
{
  SlackGate g = gate();
  g.enabled = enabled;
  if (!solver) solver = std::make_unique<OsqpSolver>(solver_options());
  return std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::move(solver),
                                      breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g);
}

// The true 3D distance of the robot to the obstacles (every primitive, every box) where they are now.
double obstacle_clearance(const State & x, const World & world, double now)
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
  int holding = 0, holding_events = 0;
  bool reached = false;
  double closest = std::numeric_limits<double>::infinity();

  int count(CycleStatus s) const { return static_cast<int>(std::count(statuses.begin(), statuses.end(), s)); }
  void print(const char * name) const
  {
    double worst_slack = 0.0;
    for (double s : gate_slack) worst_slack = std::max(worst_slack, s);
    const State & x = states.back();
    std::printf("[%s] %zu cycles: ok %d, holding %d (%d events), fallback %d, braking %d, failed %d, goal %d; "
                "closest %.4f m, largest gated slack %.4f m; final (%.3f, %.3f, %.3f)\n",
                name, statuses.size(), count(CycleStatus::Ok), holding, holding_events, count(CycleStatus::Fallback),
                count(CycleStatus::Braking), count(CycleStatus::Failed), reached, closest, worst_slack, x(0), x(1),
                x(2));
  }
};

// `before` runs before every cycle (movers): it gets the cycle's time and the plant's state.
GateLoop run_gate_loop(Controller & c, Plant & plant, World & world, double period, double seconds,
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
}  // namespace

TEST(SlackGate, DefaultsFromTheConfiguration)
{
  // wholebody_mpc.yaml: on, the first second of the plan, 5 mm.
  EXPECT_TRUE(gate().enabled);
  EXPECT_DOUBLE_EQ(gate().horizon_s, 1.0);
  EXPECT_DOUBLE_EQ(gate().tolerance_m, 0.005);
  // While holding, plans must pass over the whole 2 s plan for 0.2 s before tracking resumes.
  EXPECT_DOUBLE_EQ(gate().resume_horizon_s, 2.0);
  EXPECT_DOUBLE_EQ(gate().resume_dwell_s, 0.2);
  SlackGate bad;
  bad.horizon_s = 0.0;
  EXPECT_THROW(Controller(formulation(), CollisionModel(primitives(), CollisionConfig{}),
                          std::make_unique<OsqpSolver>(solver_options()), breakaway(), 3, kGoalXyM, kGoalYawRad, bad),
               std::invalid_argument);
  EXPECT_STREQ(to_string(CycleStatus::Holding), "holding");
}

TEST(SlackGate, ComfortOnlyHoldsAtTheGatePosts)
{
  // The 2026-10-13 follow-up runs: posture comfort_only (no vertical-carry attractor) cannot pass the
  // 1.05 m gate, and the slacked rows let the plan into the posts (slack up to 0.51 m, contact; this
  // loop without the gate: -0.145 m, SlackGate.DisabledExecutesTheSlackedPlan). With the gate the MPC
  // drives up to the posts at the margin (pitching the panel), then holds where the plans start to
  // push in: it never gets deeper than the margin less the tolerance, and the hold persists (the
  // node's no-progress check then ends the goal).
  auto c = gated_controller();
  const PostureTargets targets = posture_targets(payload(), "home", PostureMode::ComfortOnly);
  World world = map_world();
  const double period = 0.02;  // the node's 50 Hz
  Plant plant{state(-2.2, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, period, 60.0, [](const State & x) { return x(0) < -12.4; });
  run.print("comfort_only gate");
  const double margin = CollisionConfig{}.margin_m;
  EXPECT_GE(run.closest, margin - gate().tolerance_m) << "pushed into the posts";
  ASSERT_GT(run.holding, 0) << "never held";
  const size_t first = static_cast<size_t>(
    std::find(run.statuses.begin(), run.statuses.end(), CycleStatus::Holding) - run.statuses.begin());
  std::printf("comfort_only gate: first hold at %.2f s, x %.3f; final x %.3f\n", first * period, run.states[first](0),
              plant.x(0));
  EXPECT_LT(std::abs(plant.x(0) - run.states[first](0)), 0.25) << "drove on after the first hold";
  EXPECT_GT(plant.x(0), -9.0) << "passed the gate in comfort_only";
  // Holding persists: the last 10 s are all holding, at rest.
  const size_t last = static_cast<size_t>(10.0 / period);
  ASSERT_GE(run.statuses.size(), last);
  for (size_t i = run.statuses.size() - last; i < run.statuses.size(); ++i)
    ASSERT_EQ(run.statuses[i], CycleStatus::Holding) << "cycle " << i;
  EXPECT_TRUE(c->last_command().head<2>().isZero()) << "the base is not at rest while holding";
  // Each hold starts with a gated row: slack beyond its pair's current violation plus the tolerance.
  for (size_t i = 0; i < run.statuses.size(); ++i)
    if (run.statuses[i] == CycleStatus::Holding && (i == 0 || run.statuses[i - 1] != CycleStatus::Holding))
      ASSERT_GT(run.gate_slack[i] - run.gate_violation[i], gate().tolerance_m) << "cycle " << i;
}

namespace
{
// worker_late_crossing_nav in the open: a 0.5 x 0.5 x 1.8 m worker waits at (x_cross, 2.5) until the
// robot's centre comes within 2.0 m of (x_cross, 0), then walks to (x_cross, -2.5) at 0.5 m/s; it
// waits while the robot is within 0.1 m of its next position (Unity's wait rule). The MPC sees it as
// a mover sampled every cycle (constant-velocity prediction).
struct LateWorker
{
  double x_cross = 5.0;
  Vector2d position{5.0, 2.5};
  bool walking = false;
  double closest = std::numeric_limits<double>::infinity();  // robot to worker over the run
  double speed = 0.5, trigger = 2.0;
  static constexpr double kWait = 0.1;
  Vector3d size{0.5, 0.5, 1.8};

  ObstacleBox box(const Vector2d & p) const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Vector3d(p.x(), p.y(), size.z() / 2.0);
    return {"late_worker", pose, size};
  }

  void step(double now, double period, const State & robot, World & world)
  {
    static const CollisionModel model(primitives(), CollisionConfig{});
    const auto distance = [&](const Vector2d & p) { return model.nearest(robot, {box(p)})->distance; };
    closest = std::min(closest, distance(position));
    if (!walking && (robot.head<2>() - Vector2d(x_cross, 0.0)).norm() <= trigger) walking = true;
    if (walking && position.y() > -2.5) {
      const Vector2d next(x_cross, std::max(-2.5, position.y() - speed * period));
      if (distance(next) > kWait) position = next;
    }
    world.update_mover("late_worker", position, size, now);
  }
};
}  // namespace

TEST(SlackGate, HoldsWhileAPredictedMoverCrossesThenReachesTheGoal)
{
  // The recorded worker_late_crossing_nav deadlock: B5 crept into the worker's path (sustained slack
  // 0.11-0.14 m) and the worker's wait rule then blocked both. With the gate the MPC holds while the
  // worker crosses, then resumes from rest and reaches the goal.
  auto c = gated_controller();
  World world;
  LateWorker worker;
  worker.speed = 0.8;
  worker.trigger = 1.0;
  const double period = 0.02;  // the node's 50 Hz
  Plant plant{state(0, 0, 0, posture().comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(10, 0)}), {10.0, 0.0, 0.0}, {}, posture());
  const GateLoop run = run_gate_loop(*c, plant, world, period, 80.0, nullptr,
                                     [&](double now, const State & x) { worker.step(now, period, x, world); });
  run.print("late worker");
  std::printf("late worker: closest robot-worker distance %.4f m, worker at (%.2f, %.2f)\n", worker.closest,
              worker.position.x(), worker.position.y());
  EXPECT_GT(run.holding, 0) << "never held";
  EXPECT_TRUE(run.reached) << "did not reach the goal";
  EXPECT_LE(worker.position.y(), -2.5 + 1e-9) << "the worker never got across";
  EXPECT_GE(worker.closest, CollisionConfig{}.margin_m - gate().tolerance_m - 2e-3) << "into the worker";
  // It resumed after holding: some Ok cycle follows the last holding one.
  const auto last_hold = std::find(run.statuses.rbegin(), run.statuses.rend(), CycleStatus::Holding);
  ASSERT_NE(last_hold, run.statuses.rend());
  EXPECT_NE(std::find(run.statuses.rbegin(), last_hold, CycleStatus::Ok), last_hold) << "never resumed";
}

TEST(SlackGate, StartingInsideTheMarginIsNotGated)
{
  // A crate 0.02 m beside the front right wheel (inside the 0.05 m margin): the plan needs slack up to
  // the current violation (0.03 m) to start, which the gate allows; the robot moves away.
  auto c = gated_controller();
  const double probe_y = -1.0;
  auto crate_at = [](double y) {
    return world_of({mmn::Box{"crate", Vector3d(0.5, y, 0.25), Vector3d(0.6, 0.2, 0.5), 0.0}});
  };
  const State x0 = state(0, 0, 0, posture().comfort);
  const double extent = -probe_y - 0.1 - obstacle_clearance(x0, crate_at(probe_y), 0.0);
  World world = crate_at(-(extent + 0.02 + 0.1));
  ASSERT_NEAR(obstacle_clearance(x0, world, 0.0), 0.02, 1e-6);
  Plant plant{x0};
  c->start(Route({Vector2d(0, 0), Vector2d(5, 0)}), {5.0, 0.0, 0.0}, {}, posture());
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 5.0);
  run.print("inside the margin");
  double largest = 0.0;
  for (double s : run.gate_slack) largest = std::max(largest, s);
  EXPECT_GT(largest, 1e-3) << "the plan needed no slack: the case does not test the allowance";
  EXPECT_EQ(run.holding, 0);
  EXPECT_EQ(run.count(CycleStatus::Ok), static_cast<int>(run.statuses.size()));
  EXPECT_GT(obstacle_clearance(plant.x, world, 0.0), CollisionConfig{}.margin_m) << "did not move away";
  EXPECT_GT(plant.x(0), 0.5) << "did not drive";
}

TEST(SlackGate, BlendPassesTheGateWithoutHolding)
{
  // The default posture preference through the 1.05 m gate (the recorded B5 drive): the gate never
  // triggers, the robot passes as without it.
  auto c = gated_controller();
  World world = map_world();
  Plant plant{state(-2.2, kGateY, M_PI, posture().comfort)};
  c->start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, posture());
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 120.0, [](const State & x) { return x(0) < -12.4; });
  run.print("blend gate");
  EXPECT_EQ(run.holding, 0);
  EXPECT_LT(plant.x(0), -12.4) << "did not pass the gate";
  EXPECT_GE(run.closest, CollisionConfig{}.margin_m - 0.01);
}

TEST(SlackGate, DisabledExecutesTheSlackedPlan)
{
  // slack_gate false (the recorded B5): comfort_only drives the plan into the posts, never holding.
  auto c = gated_controller(false);
  const PostureTargets targets = posture_targets(payload(), "home", PostureMode::ComfortOnly);
  World world = map_world();
  Plant plant{state(-2.2, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 40.0, [](const State & x) { return x(0) < -12.4; });
  run.print("comfort_only gate, no slack gate");
  EXPECT_EQ(run.holding, 0);
  EXPECT_LT(run.closest, CollisionConfig{}.margin_m - gate().tolerance_m) << "the recorded penetration did not occur";
}

TEST(SlackGate, SolverFailureWhileHoldingKeepsHolding)
{
  // The recorded comfort_only gated run (cycle 1557): a held plan became the nominal, the next solve
  // failed (max_iter) and the cycle fell back to that rejected plan (commanded and streamed) and
  // dropped the resume hysteresis. A failure while holding must keep holding: braking as the hold
  // does, counting towards the fallback budget, never commanding the rejected plan.
  bool fail = false;
  auto owned = std::make_unique<ScriptedSolver>([&](int) { return fail; });
  ScriptedSolver * solver = owned.get();
  auto c = gated_controller(true, std::move(owned));
  const PostureTargets targets = posture_targets(payload(), "home", PostureMode::ComfortOnly);
  World world = map_world();
  const double period = 0.02;  // the node's 50 Hz
  const Limits & l = limits();
  Plant plant{state(-2.2, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  int i = 0;
  auto step = [&]() -> const CycleOutput & {
    const CycleOutput & out = c->step(i * period, plant.x, true, world, plant.at_rest());
    plant.step(out.command, period);
    ++i;
    return out;
  };
  // Drive until the second cycle of a hold: the held plan of the first is now the nominal.
  int held = 0;
  while (held < 2 && i * period < 60.0) held = step().status == CycleStatus::Holding ? held + 1 : 0;
  ASSERT_EQ(held, 2) << "never held";
  std::printf("solver failure while holding: hold at %.2f s, x %.3f\n", i * period, plant.x(0));
  // The solves fail from now on: K cycles keep holding (braking), then the budget ends in Braking.
  fail = true;
  const int calls = solver->calls;
  for (int k = 0; k < kDefaultFallbackCycles; ++k) {
    const Control last = c->last_command();
    const CycleOutput & out = step();
    ASSERT_EQ(out.status, CycleStatus::Holding) << "failure " << k + 1 << ": the rejected plan was executed";
    // The hold's braking: the base at the braking limits from the command sent last, the arm
    // decelerating (each joint towards zero, never beyond the command sent last), the plan the same.
    EXPECT_LE(std::abs(out.command(0)), std::max(0.0, std::abs(last(0)) - l.decel * period) + 1e-12);
    EXPECT_LE(std::abs(out.command(1)), std::max(0.0, std::abs(last(1)) - l.yaw_decel * period) + 1e-12);
    for (int j = 2; j < kNu; ++j) {
      EXPECT_LE(std::abs(out.command(j)), std::abs(last(j)) + 1e-12) << "joint " << j - 2;
      EXPECT_GE(out.command(j) * last(j), 0.0) << "joint " << j - 2;
    }
    EXPECT_TRUE(out.plan.u[0].isApprox(out.command, 1e-12) || out.plan.u[0].isZero()) << "the plan is not the hold";
    EXPECT_DOUBLE_EQ(out.max_slack, 0.0);
  }
  EXPECT_EQ(solver->calls, calls + kDefaultFallbackCycles);
  const CycleOutput & braking = step();
  EXPECT_TRUE(braking.status == CycleStatus::Braking || braking.status == CycleStatus::Failed) << to_string(braking.status);
}

TEST(SlackGate, SolverFailureWhileHoldingRestartsTheResumeDwell)
{
  // The late worker: the hold ends after resume_dwell_s of passing plans. A solver failure within
  // that dwell keeps holding and restarts the dwell: tracking resumes resume_dwell_s after the
  // first passing plan following the failure, not earlier.
  const double period = 0.02;  // the node's 50 Hz
  auto run = [&](int fail_at) {
    auto c = gated_controller(true, std::make_unique<ScriptedSolver>([&](int call) { return call == fail_at; }));
    World world;
    LateWorker worker;
    worker.speed = 0.8;
    worker.trigger = 1.0;
    Plant plant{state(0, 0, 0, posture().comfort)};
    c->start(Route({Vector2d(0, 0), Vector2d(10, 0)}), {10.0, 0.0, 0.0}, {}, posture());
    return run_gate_loop(*c, plant, world, period, 80.0, nullptr,
                         [&](double now, const State & x) { worker.step(now, period, x, world); });
  };
  // Without a failure: the cycle whose plan resumes tracking after the last hold.
  const GateLoop clean = run(-1);
  const auto last_hold = std::find(clean.statuses.rbegin(), clean.statuses.rend(), CycleStatus::Holding);
  ASSERT_NE(last_hold, clean.statuses.rend()) << "never held";
  const int resume = static_cast<int>(clean.statuses.rend() - last_hold);  // the cycle after the last hold
  ASSERT_EQ(clean.statuses[static_cast<size_t>(resume)], CycleStatus::Ok);
  const int dwell = static_cast<int>(std::lround(gate().resume_dwell_s / period));
  for (int k = resume - dwell; k < resume; ++k)
    ASSERT_EQ(clean.statuses[static_cast<size_t>(k)], CycleStatus::Holding) << "cycle " << k << " is not in the dwell";
  // The same run with the solve failing in the middle of that dwell (solver call = cycle: every
  // cycle before it tracks or holds, so each calls the solver once).
  const int fail_at = resume - dwell / 2;
  const GateLoop failed = run(fail_at);
  for (int k = 0; k < fail_at; ++k) ASSERT_EQ(failed.statuses[static_cast<size_t>(k)], clean.statuses[static_cast<size_t>(k)]);
  EXPECT_EQ(failed.statuses[static_cast<size_t>(fail_at)], CycleStatus::Holding) << "the rejected plan was executed";
  // Passing plans after it: holding for another full dwell, then tracking.
  for (int k = fail_at + 1; k <= fail_at + dwell; ++k)
    EXPECT_EQ(failed.statuses[static_cast<size_t>(k)], CycleStatus::Holding) << "cycle " << k << ": resumed early";
  EXPECT_TRUE(failed.reached) << "did not reach the goal";
}
