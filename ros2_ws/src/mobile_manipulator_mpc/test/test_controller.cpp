// The B5 controller loop (no ROS): the breakaway output stage, the in-place turn from rest
// against a plant with static friction, and the failure handling (fallback, braking, stale
// state, goal).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
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
  Plant plant{state(0, 0, 0, posture().home)};
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
  Plant plant{state(12, 0, -M_PI, posture().home)};
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
  const CycleOutput & fallback = drive.step(*c);
  ASSERT_EQ(fallback.status, CycleStatus::Fallback);
  for (int i = 0; i < kNu; ++i) EXPECT_NEAR(fallback.command(i), previous.u[1](i), 1e-12) << "control " << i;
  for (int k = 0; k + 1 < config().horizon; ++k)
    EXPECT_TRUE(fallback.plan.u[k].isApprox(previous.u[k + 1], 1e-12)) << "stage " << k;
  EXPECT_EQ(fallback.iterations, 0);
  // The next solve succeeds: back to Ok.
  EXPECT_EQ(drive.step(*c).status, CycleStatus::Ok);
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
    {state(2.9, 0.05, 0.1, posture().home), true},  // 0.112 m, 0.1 rad
    {state(3.0, 0.14, -0.14, posture().home), true},
    {state(2.8, 0.0, 0.0, posture().home), false},  // 0.2 m
    {state(3.0, 0.0, 0.2, posture().home), false},  // 0.2 rad
    {state(3.0, 0.0, 2 * M_PI - 0.1, posture().home), true},  // wrapped
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
  Joints q = posture().home;
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
  Plant plant{state(0, 0, 0, posture().home)};
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

