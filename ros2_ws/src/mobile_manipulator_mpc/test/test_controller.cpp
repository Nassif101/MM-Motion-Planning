// The B5 controller loop (no ROS): the breakaway output stage, the in-place turn from rest
// against a plant with static friction, and the failure handling (fallback, braking, stale
// state, goal).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/controller.hpp"
#include "closed_loop.hpp"
#include "mobile_manipulator_mpc/hpipm_solver.hpp"
#include "mobile_manipulator_mpc/medium_switch.hpp"
#include "mobile_manipulator_mpc/solver_factory.hpp"
#include "test_support.hpp"

#include <unistd.h>

using namespace mobile_manipulator_mpc;
using namespace mpc_test;
using Eigen::Vector2d;

namespace mobile_manipulator_mpc
{
void PrintTo(CycleStatus status, std::ostream * os) { *os << to_string(status); }
}  // namespace mobile_manipulator_mpc

namespace
{
Control base_command(double v, double omega)
{
  Control u = Control::Zero();
  u(0) = v;
  u(1) = omega;
  return u;
}


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

TEST(BreakawayOutput, FollowPlanNeverBoostsAgainstThePlan)
{
  // E2 Task 4b (spec Amendment 1): the boost from rest goes only where the plan asks to go. The latch E2 measured: the
  // rate band left +0.054 (the last kick decaying), the plan asks -0.026, the base reads at rest.
  const Breakaway b = breakaway();
  const auto yaw = [&](double clamped, double planned, bool at_rest, bool follow) {
    return breakaway_output(base_command(0.0, clamped), base_command(0.0, planned), at_rest, b, follow)(1);
  };
  EXPECT_DOUBLE_EQ(yaw(0.054, -0.026, true, false), 0.07);   // frozen: the old sign is kicked again
  EXPECT_DOUBLE_EQ(yaw(0.054, -0.026, true, true), 0.054);   // follow-plan: sent as clamped, decaying
  EXPECT_DOUBLE_EQ(yaw(-0.016, -0.026, true, true), -0.07);  // the plan's own direction: boosted
  EXPECT_DOUBLE_EQ(yaw(-0.016, -0.026, true, false), -0.07);
  EXPECT_DOUBLE_EQ(yaw(0.03, 0.004, true, true), 0.03);      // the plan inside the dead zone: no boost
  EXPECT_DOUBLE_EQ(yaw(0.03, 0.004, true, false), 0.07);
  for (bool follow : {false, true}) EXPECT_DOUBLE_EQ(yaw(0.054, -0.026, false, follow), 0.054);  // moving: unchanged
  // the linear component the same way
  const auto lin = [&](double clamped, double planned, bool follow) {
    return breakaway_output(base_command(clamped, 0.0), base_command(planned, 0.0), true, b, follow)(0);
  };
  EXPECT_DOUBLE_EQ(lin(0.02, -0.01, true), 0.02);
  EXPECT_DOUBLE_EQ(lin(0.02, -0.01, false), 0.0375);
  EXPECT_DOUBLE_EQ(lin(0.02, 0.03, true), 0.0375);
  // the frozen two-argument form is unchanged
  EXPECT_DOUBLE_EQ(breakaway_output(base_command(0.0, 0.054), true, b)(1), 0.07);
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

const SlackGate & gate() { return config().slack_gate; }

std::unique_ptr<Controller> gated_controller(bool enabled = true, std::unique_ptr<QpSolver> solver = nullptr)
{
  SlackGate g = gate();
  g.enabled = enabled;
  if (!solver) solver = std::make_unique<OsqpSolver>(solver_options());
  return std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::move(solver),
                                      breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g);
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
  // Each hold starts with a gated row: slack beyond its allowance (the tolerance outside the margin,
  // the pair's current violation inside it).
  for (size_t i = 0; i < run.statuses.size(); ++i)
    if (run.statuses[i] == CycleStatus::Holding && (i == 0 || run.statuses[i - 1] != CycleStatus::Holding)) {
      ASSERT_TRUE(run.gate_unsafe[i]) << "cycle " << i;
      const double allowance = run.gate_violation[i] > 0.0 ? kGateInsideAllowanceM : gate().tolerance_m;
      ASSERT_GT(run.gate_slack[i] - run.gate_violation[i], allowance) << "cycle " << i;
    }
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

TEST(SlackGate, InsideTheMarginNeverGetsDeeper)
{
  // A funnel of two low walls along the route, the robot starting 0.02-0.025 m inside the 0.05 m
  // margin of each and the opening narrowing by 1 cm per metre per side ahead, with a tenth of the
  // configured slack weights (progress outweighs slack, as where a posture or reference pulls the
  // plan into an obstacle). Without the gate the MPC drives in on slack into contact. With 5 mm of
  // slack allowed above a pair's current violation the gate passed every plan (each a few mm deeper
  // than the robot was) and the robot crept into contact as without it. A pair inside the margin
  // may keep its violation but never plan deeper: the closed loop never gets closer than it started.
  const double inside = 0.03, slope = 0.01, length = 8.0, thick = 0.2, height = 0.5;
  const State x0 = state(0, 0, 0, posture().comfort);
  // The robot's lateral extents at x0 (right, left), probed with straight walls 1 m away.
  auto extent = [&](double side) {
    const World probe = world_of({mmn::Box{"probe", Vector3d(0.0, side * (1.0 + thick / 2.0), height / 2.0),
                                           Vector3d(length, thick, height), 0.0}});
    return 1.0 - obstacle_clearance(x0, probe, 0.0);
  };
  const double right = extent(-1.0), left = extent(1.0);
  // Each wall's inner face passes `inside` beside the robot at x = 0 and closes in by `slope` per metre.
  const double yaw = std::atan(slope), cx = length / 2.0 - 1.0, half = thick / 2.0 / std::cos(yaw);
  World world = world_of(
    {mmn::Box{"right", Vector3d(cx, -(right + inside) + slope * cx - half, height / 2.0), Vector3d(length, thick, height), yaw},
     mmn::Box{"left", Vector3d(cx, (left + inside) - slope * cx + half, height / 2.0), Vector3d(length, thick, height), -yaw}});
  // The front corners are slightly closer (the walls close in ahead of x = 0).
  const double start = obstacle_clearance(x0, world, 0.0);
  std::printf("funnel: robot extents right %.4f m, left %.4f m; start clearance %.4f m\n", right, left, start);
  ASSERT_GT(start, inside - 0.01) << "the funnel is misplaced";
  ASSERT_LE(start, inside + 1e-6) << "the funnel is misplaced";
  Weights weights = config().weights;
  weights.slack /= 10.0;
  weights.slack_quadratic /= 10.0;
  weights.slack_inside = -1.0;  // the same tenth inside the margin (the gate is this test's subject)
  for (const bool enabled : {false, true}) {
    SlackGate g = gate();
    g.enabled = enabled;
    Controller c(Formulation(config().horizon, config().dt, limits(), weights),
                 CollisionModel(primitives(), CollisionConfig{}), std::make_unique<OsqpSolver>(solver_options()),
                 breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g);
    Plant plant{x0};
    c.start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, posture());
    const double period = 0.02;  // the node's 50 Hz
    const GateLoop run = run_gate_loop(c, plant, world, period, 15.0);
    run.print(enabled ? "funnel, gate" : "funnel, no gate");
    std::printf("funnel (%s): start %.4f m, closest %.4f m, final x %.3f\n", enabled ? "gate" : "no gate", start,
                run.closest, plant.x(0));
    if (!enabled) {
      // The case tests the gate: without it the MPC drives into the funnel, into contact.
      ASSERT_LT(run.closest, 0.0) << "the MPC did not push in without the gate";
      continue;
    }
    // The solver's constraint tolerance (eps 1 mm) allows a millimetre.
    EXPECT_GE(run.closest, start - 1e-3) << "got deeper into the margin";
    EXPECT_GT(run.holding, 0) << "never held";
  }
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
  // With the gate off no pair distance is computed (the recorded B5's cycle): no gate diagnostics.
  for (size_t i = 0; i < run.statuses.size(); ++i) {
    ASSERT_EQ(run.gate_slack[i], 0.0) << "cycle " << i;
    ASSERT_EQ(run.gate_violation[i], 0.0) << "cycle " << i;
    ASSERT_FALSE(run.gate_unsafe[i]) << "cycle " << i;
  }
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


TEST(SlackGate, ReportsTheResumeDwell)
{
  // E1's decision trace: gate_dwelling marks the held cycles whose plan passes but waits out the resume dwell (the late
  // worker: the hold ends after resume_dwell_s of passing plans), never a cycle that tracks.
  const double period = 0.02;
  auto c = gated_controller(true);
  World world;
  LateWorker worker;
  worker.speed = 0.8;
  worker.trigger = 1.0;
  Plant plant{state(0, 0, 0, posture().comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(10, 0)}), {10.0, 0.0, 0.0}, {}, posture());
  const GateLoop run = run_gate_loop(*c, plant, world, period, 80.0, nullptr,
                                     [&](double now, const State & x) { worker.step(now, period, x, world); });
  ASSERT_EQ(run.gate_dwelling.size(), run.statuses.size());
  const auto last_hold = std::find(run.statuses.rbegin(), run.statuses.rend(), CycleStatus::Holding);
  ASSERT_NE(last_hold, run.statuses.rend()) << "never held";
  const int resume = static_cast<int>(run.statuses.rend() - last_hold);
  const int dwell = static_cast<int>(std::lround(gate().resume_dwell_s / period));
  // the dwell's cycles before the resume (the first passing plan starts it: dwell - 1 of them at least)
  int dwelling = 0;
  for (int k = resume - dwell; k < resume; ++k) dwelling += run.gate_dwelling[static_cast<size_t>(k)] ? 1 : 0;
  EXPECT_GE(dwelling, dwell - 1);
  for (size_t k = 0; k < run.statuses.size(); ++k) {
    if (!run.gate_dwelling[k]) continue;
    EXPECT_EQ(run.statuses[k], CycleStatus::Holding) << "cycle " << k;
    EXPECT_FALSE(run.gate_unsafe[k]) << "cycle " << k << ": dwelling on a plan that failed the gate";
  }
}

// ---- E1: the measured-depth guard (CP3, user decision 2026-11-08) ----

TEST(DepthGuard, OffByDefault)
{
  EXPECT_LT(gate().depth_guard_m, 0.0);  // the shipped yaml: off (the frozen B5)
}

TEST(DepthGuard, ReferencesLatchFollowDownAndTrip)
{
  // the guard's reference per pair: first seen within T -> T; first seen deeper -> its depth (latched); it follows the
  // pair down, never up; a pair deeper than its reference + the tolerance trips; a pair outside is forgotten
  std::vector<DepthGuardRef> refs;
  auto pair = [](double depth) { return std::vector<DepthGuardPair>{{0, -1, "wall", depth, 0.005}}; };
  EXPECT_FALSE(depth_guard_step(refs, pair(0.002), 0.002));  // within T: reference T (5 mm)
  EXPECT_DOUBLE_EQ(refs.at(0).ref, 0.005);
  EXPECT_FALSE(depth_guard_step(refs, pair(0.0069), 0.002));  // 6.9 mm: within T + 2 mm
  EXPECT_TRUE(depth_guard_step(refs, pair(0.0071), 0.002));   // 7.1 mm: trips
  EXPECT_DOUBLE_EQ(refs.at(0).ref, 0.005) << "a trip does not raise the reference";
  refs.clear();
  EXPECT_FALSE(depth_guard_step(refs, pair(0.030), 0.002));  // first seen 30 mm deep: latched there
  EXPECT_DOUBLE_EQ(refs.at(0).ref, 0.030);
  EXPECT_FALSE(depth_guard_step(refs, pair(0.020), 0.002));  // it leaves: the reference follows down
  EXPECT_DOUBLE_EQ(refs.at(0).ref, 0.020);
  EXPECT_TRUE(depth_guard_step(refs, pair(0.0225), 0.002));  // back in by more than 2 mm: trips
  EXPECT_FALSE(depth_guard_step(refs, pair(0.0), 0.002));    // measured outside its margin: forgotten
  EXPECT_TRUE(refs.empty());
}

TEST(DepthGuard, APairMissingFromTheRowsKeepsItsReference)
{
  // Task 9b review: rows come from the nominal (the closest pairs per stage), so a pair can miss a cycle's rows; its
  // reference is kept for kDepthGuardGraceCycles, and a return deeper than it trips instead of latching anew
  std::vector<DepthGuardRef> refs;
  auto pair = [](double depth) { return std::vector<DepthGuardPair>{{0, -1, "jamb", depth, 0.005}}; };
  EXPECT_FALSE(depth_guard_step(refs, pair(0.003), 0.002));  // reference 5 mm
  for (int i = 0; i < 10; ++i) EXPECT_FALSE(depth_guard_step(refs, {}, 0.002));  // missing for 10 cycles
  ASSERT_EQ(refs.size(), 1u);
  EXPECT_TRUE(depth_guard_step(refs, pair(0.009), 0.002)) << "re-latched at 9 mm instead of tripping";
  refs.clear();
  EXPECT_FALSE(depth_guard_step(refs, pair(0.003), 0.002));
  for (int i = 0; i <= kDepthGuardGraceCycles; ++i) depth_guard_step(refs, {}, 0.002);
  EXPECT_TRUE(refs.empty()) << "kept beyond the grace";
}

TEST(DepthGuard, HoldsWhereThePlanCheckPassesButTheRobotGoesDeeper)
{
  // A crate 0.02 m beside the front right wheel (30 mm inside the 0.05 m margin, beyond T: the plan may stay or leave).
  // The next state is measured 3 mm deeper (the plant's drift): the plan check alone passes it (its allowance follows
  // the current depth); the guard holds (deeper than the latched 30 mm + 2 mm).
  const double probe_y = -1.0;
  auto crate_at = [](double y) { return world_of({mmn::Box{"crate", Vector3d(0.5, y, 0.25), Vector3d(0.6, 0.2, 0.5), 0.0}}); };
  const State x0 = state(0, 0, 0, posture().comfort);
  const double extent = -probe_y - 0.1 - obstacle_clearance(x0, crate_at(probe_y), 0.0);
  World world = crate_at(-(extent + 0.02 + 0.1));
  auto second_cycle = [&](double guard_m) {
    SlackGate g = gate();
    g.depth_guard_m = guard_m;
    Controller c(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::make_unique<OsqpSolver>(solver_options()),
                 breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g);
    c.start(Route({Vector2d(0, 0), Vector2d(5, 0)}), {5.0, 0.0, 0.0}, {}, posture());
    EXPECT_EQ(c.step(0.0, x0, true, world, true).status, CycleStatus::Ok);
    State x1 = x0;
    x1(1) -= 0.003;  // 3 mm toward the crate
    const CycleOutput & o = c.step(0.02, x1, true, world, true);
    return std::make_pair(o.status, o.guard_tripped);
  };
  const auto off = second_cycle(-1.0), on = second_cycle(0.002);
  EXPECT_EQ(off.first, CycleStatus::Ok) << "the plan check alone should pass it (the ratchet)";
  EXPECT_FALSE(off.second);
  EXPECT_EQ(on.first, CycleStatus::Holding);
  EXPECT_TRUE(on.second);
}

TEST(DepthGuard, AStartInsideTheMarginIsNotHeld)
{
  // a pair already deeper than the bound when it is first seen latches its depth (StartingInsideTheMarginIsNotGated
  // with the guard on: the robot moves away without holding)
  SlackGate g = gate();
  g.depth_guard_m = 0.002;
  auto c = std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}),
                                        std::make_unique<OsqpSolver>(solver_options()), breakaway(),
                                        kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g);
  const double probe_y = -1.0;
  auto crate_at = [](double y) { return world_of({mmn::Box{"crate", Vector3d(0.5, y, 0.25), Vector3d(0.6, 0.2, 0.5), 0.0}}); };
  const State x0 = state(0, 0, 0, posture().comfort);
  const double extent = -probe_y - 0.1 - obstacle_clearance(x0, crate_at(probe_y), 0.0);
  World world = crate_at(-(extent + 0.02 + 0.1));
  Plant plant{x0};
  c->start(Route({Vector2d(0, 0), Vector2d(5, 0)}), {5.0, 0.0, 0.0}, {}, posture());
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 5.0);
  EXPECT_EQ(run.holding, 0);
  EXPECT_GT(plant.x(0), 0.5) << "did not drive";
}

// ---- base margin (Phase 4B-A spec Section 2) ----

namespace
{
// A low wall (under the arm and panel) along the route, its face `gap` beside the base hull when the
// base follows the route (y = 0).
World low_wall(double gap)
{
  constexpr double kHalfWidth = 0.365;  // the URDF wheels' outer faces (the MPC's base rows measure from them)
  return world_of({mmn::Box{"low_wall", Vector3d(3.0, -(kHalfWidth + gap + 0.1), 0.2), Vector3d(4.0, 0.2, 0.4), 0.0}});
}

std::unique_ptr<Controller> base_margin_controller(double base_margin)
{
  CollisionConfig cfg;
  cfg.base_margin_m = base_margin;
  return std::make_unique<Controller>(Formulation(config().horizon, config().dt, limits(), config().weights, cfg),
                                      CollisionModel(primitives(), cfg), std::make_unique<OsqpSolver>(solver_options()),
                                      breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate());
}
}  // namespace

TEST(BaseMargin, BaseStopsAtTheBaseMargin)
{
  // The route passes 0.06 m from a low wall: with a 0.09 m base margin the base keeps at least that
  // far (the arm and panel are above the wall); with the plain 0.05 m it follows the route.
  auto drive = [](double base_margin) {
    auto c = base_margin_controller(base_margin);
    World world = low_wall(0.06);
    Plant plant{state(0, 0, 0, posture().comfort)};
    c->start(Route({Vector2d(0, 0), Vector2d(7, 0)}), {7.0, 0.0, 0.0}, {}, posture());
    return run_gate_loop(*c, plant, world, 0.1, 30.0, [](const State & x) { return x(0) > 5.5; });
  };
  const GateLoop wide = drive(0.09), plain = drive(0.05);
  wide.print("base margin 0.09");
  plain.print("base margin 0.05");
  EXPECT_GT(wide.states.back()(0), 5.0) << "did not pass the wall";
  EXPECT_GE(wide.closest, 0.085);
  EXPECT_LT(plain.closest, 0.075);
}

TEST(SlackGate, UsesTheRowMargin)
{
  // A crate 0.06 m beside the front right wheel, inside the 0.09 m base margin (a 0.03 m violation): the plan
  // needs slack up to that violation to drive away, which the gate allows for a pair inside its own margin.
  // Judged against the plain 0.05 m margin the pair would count as outside it (allowance 5 mm) and hold.
  // (A long wall beside the whole base is not used: a forward-only base inside the margin there holds with
  // either margin, as before Phase 4B-A.)
  auto c = base_margin_controller(0.09);
  const double probe_y = -1.0;
  auto crate_at = [](double y) {
    return world_of({mmn::Box{"crate", Vector3d(0.5, y, 0.2), Vector3d(0.6, 0.2, 0.4), 0.0}});
  };
  const State x0 = state(0, 0, 0, posture().comfort);
  const double extent = -probe_y - 0.1 - obstacle_clearance(x0, crate_at(probe_y), 0.0);
  World world = crate_at(-(extent + 0.06 + 0.1));
  ASSERT_NEAR(obstacle_clearance(x0, world, 0.0), 0.06, 1e-6);
  Plant plant{x0};
  c->start(Route({Vector2d(0, 0), Vector2d(5, 0)}), {5.0, 0.0, 0.0}, {}, posture());
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 5.0);
  run.print("inside the base margin");
  double largest = 0.0;
  for (double s : run.gate_slack) largest = std::max(largest, s);
  EXPECT_GT(largest, 0.006) << "the plan needed no more slack than the tolerance: the case does not test the margin";
  EXPECT_EQ(run.holding, 0);
  EXPECT_GT(plant.x(0), 0.5) << "did not drive";
}

// ---- external hold (Phase 4B-A spec Section 2: blocked by the collision monitor) ----

TEST(Controller, ExternalHoldHoldsLikeTheGate)
{
  auto c = make_controller();
  Drive d(*c);
  for (int i = 0; i < 40; ++i) d.step(*c);  // up to speed
  const double v0 = c->last_command()(0);
  ASSERT_GT(v0, 0.1);
  c->set_external_hold(true);
  const CycleOutput & held = d.step(*c);
  EXPECT_EQ(held.status, CycleStatus::Blocked);
  EXPECT_STREQ(to_string(CycleStatus::Blocked), "blocked");
  EXPECT_NEAR(held.command(0), braked(v0), 1e-9) << "the base brakes at the envelope's limit";
  for (int i = 0; i < 30; ++i) {
    const CycleOutput & out = d.step(*c);
    EXPECT_EQ(out.status, CycleStatus::Blocked);
    EXPECT_FALSE(out.gate_dwelling) << i << ": an external hold is no gate dwell (E1's trace: dwell only while holding)";
  }
  EXPECT_DOUBLE_EQ(c->last_command()(0), 0.0) << "stopped while blocked";
  c->set_external_hold(false);
  bool resumed = false;
  for (int i = 0; i < 20 && !resumed; ++i) resumed = d.step(*c).status == CycleStatus::Ok;
  EXPECT_TRUE(resumed);
}

// ---- side picker in closed loop (Phase 4B-A spec Section 3) ----

namespace
{
PostureTargets both_sides()
{
  return posture_targets(payload(), config().comfort_pose, PostureMode::Blend, {"vertical_carry", "vertical_carry_right"});
}

// The shoulder pan of the first state at or past `x` (driving towards -x), NaN if never reached.
double pan_at(const GateLoop & run, double x)
{
  for (const State & s : run.states)
    if (s(0) <= x) return s(3);
  return std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

TEST(RightSwing, ShelfLeftOfTheApproachSwingsRight)
{
  // The 1.05 m gate with a shelf (0.75-2.2 m high, above the base) along the left of the approach, its face
  // 0.32 m from the route: the left-swung panel (its face 0.245 m out) would pass 0.075 m from it, the right one
  // 0.565 m: B5 swings right and passes. Driving towards -x, the robot's left is -y.
  auto c = gated_controller();
  World world = world_of_map_plus({mmn::Box{"shelf", Vector3d(-5.0, kGateY - 0.32 - 0.5, 1.475),
                                            Vector3d(4.0, 1.0, 1.45), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(-1.0, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-1.0, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 120.0, [](const State & x) { return x(0) < -12.4; });
  run.print("shelf left of the gate approach");
  EXPECT_NE(std::find(run.choices.begin(), run.choices.end(), PostureChoice::Tight1), run.choices.end())
    << "never chose the right swing";
  EXPECT_LT(pan_at(run, -7.225), -1.2) << "not swung right at the gate";
  EXPECT_LT(plant.x(0), -12.4) << "did not pass the gate";
  EXPECT_GE(run.closest, CollisionConfig{}.margin_m - 0.001);
}

TEST(RightSwing, SymmetricGateStillSwingsLeft)
{
  auto c = gated_controller();
  World world = map_world();
  const PostureTargets targets = both_sides();
  Plant plant{state(-2.2, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 120.0, [](const State & x) { return x(0) < -12.4; });
  run.print("symmetric gate, both sides");
  EXPECT_GT(pan_at(run, -7.225), 1.2) << "not swung left at the symmetric gate";
  EXPECT_LT(plant.x(0), -12.4);
}

TEST(Guard, LowBeamKeepsComfortAndLowers)
{
  // A beam 1.34 m high across an open route (carry's top 1.32 m plus the 0.05 m margin needs the panel about
  // 0.03 m lower), nothing beside it: no vertical pose beats carry, so the target stays carry and the collision
  // rows lower the panel under the beam. (A beam near the ~1.20 m fold limit is the low-pass sweep's case.)
  auto c = gated_controller();
  World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 60.0, [](const State & x) { return x(0) > 6.0; });
  run.print("low beam");
  EXPECT_NE(std::find(run.choices.begin(), run.choices.end(), PostureChoice::Comfort), run.choices.end());
  EXPECT_EQ(std::find(run.choices.begin(), run.choices.end(), PostureChoice::Tight0), run.choices.end());
  EXPECT_EQ(std::find(run.choices.begin(), run.choices.end(), PostureChoice::Tight1), run.choices.end());
  EXPECT_GT(plant.x(0), 6.0) << "did not pass under the beam";
  // Lowered and kept clear: an unlowered carry (top 1.32 m) would pass the 1.34 m beam at 0.02 m; before the
  // inside-margin slack weight (Phase 4B-A Task 10) the closest approach was 0.036 m, now 0.049-0.051 m. Pinned at
  // the exit criterion.
  EXPECT_GE(run.closest, 0.045);
}

TEST(Guard, GateThenBeamStillPicksVerticalForTheGate)
{
  // The 1.05 m gate with a 1.70 m beam 1.5 m after it: a vertical pose is chosen for the gate. (The beam is not
  // yet within the horizon when the choice is made; where the route probe's segment reaches it, neither vertical
  // pose keeps the margin under it and carry does better, so those samples are left to the collision rows:
  // SidePicker.ABeamBothPassUnderDoesNotEraseTheGateDifference.)
  auto c = gated_controller();
  World world = world_of_map_plus({mmn::Box{"beam", Vector3d(-7.225 - 1.5, kGateY, 1.70 + 0.1), Vector3d(0.3, 4.0, 0.2), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(-2.2, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 120.0, [](const State & x) { return x(0) < -7.5; });
  run.print("gate then beam");
  const auto first = std::find_if(run.choices.begin(), run.choices.end(),
                                  [](PostureChoice p) { return p != PostureChoice::None; });
  ASSERT_NE(first, run.choices.end());
  EXPECT_TRUE(*first == PostureChoice::Tight0 || *first == PostureChoice::Tight1) << to_string(*first);
  EXPECT_GT(std::abs(pan_at(run, -7.225)), 1.2) << "not vertical at the gate";
}

// ---- low passes (Phase 4B-A spec Section 10A: after the freeze, no B5 change; run with
// --gtest_also_run_disabled_tests) ----

// The same sweep with the posture term pulling towards carry only (`comfort_only`: no tight attractor, no side
// swing): how far the collision rows alone lower the panel. Per beam also the arm at the closest approach.
TEST(LowPass, DISABLED_SweepComfortOnly)
{
  CollisionConfig cfg;
  cfg.base_margin_m = config().collision.base_margin_m;
  const PostureTargets targets =
    posture_targets(payload(), config().comfort_pose, PostureMode::ComfortOnly, {"vertical_carry", "vertical_carry_right"});
  for (int cm = 132; cm >= 118; --cm) {
    const double bottom = cm / 100.0;
    Controller c(Formulation(config().horizon, config().dt, limits(), config().weights, cfg),
                 CollisionModel(primitives(), cfg), std::make_unique<OsqpSolver>(solver_options()), breakaway(),
                 kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate());
    World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, bottom + 0.1), Vector3d(0.2, 3.0, 0.2), 0.0}});
    Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
    c.start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
    const GateLoop run = run_gate_loop(c, plant, world, 0.1, 90.0, [](const State & x) { return x(0) > 6.0; });
    size_t at = 0;
    for (size_t i = 0; i < run.clearance.size(); ++i)
      if (run.clearance[i] < run.clearance[at]) at = i;
    const State & q = run.states[at];
    const bool passed = plant.x(0) > 6.0 && run.count(CycleStatus::Failed) == 0;
    std::printf("COMFORTONLY beam %.2f m: %s, closest %.4f m at x %.2f, holds %d, final x %.2f, "
                "q [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
                bottom, passed ? "passed" : "STOPPED", run.closest, q(0), run.holding, plant.x(0), q(3), q(4), q(5),
                q(6), q(7), q(8));
  }
}

TEST(LowPass, DISABLED_Sweep)
{
  // A 3 m wide, 0.2 m deep beam across an 8 m straight route at x 4, its bottom from 1.30 m down to 0.80 m in
  // 0.01 m steps; B5 with the frozen defaults (both tight poses, the configured base margin), starting in carry.
  // Per beam: passed (beyond x 6 without failing), the closest approach to the beam, and the arm under it.
  CollisionConfig cfg;
  cfg.base_margin_m = config().collision.base_margin_m;
  const PostureTargets targets = both_sides();
  for (int cm = 130; cm >= 80; --cm) {
    const double bottom = cm / 100.0;
    Controller c(Formulation(config().horizon, config().dt, limits(), config().weights, cfg),
                 CollisionModel(primitives(), cfg), std::make_unique<OsqpSolver>(solver_options()), breakaway(),
                 kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate());
    World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, bottom + 0.1), Vector3d(0.2, 3.0, 0.2), 0.0}});
    Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
    c.start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
    const GateLoop run = run_gate_loop(c, plant, world, 0.1, 90.0, [](const State & x) { return x(0) > 6.0; });
    State under = run.states.front();
    for (const State & s : run.states)
      if (std::abs(s(0) - 4.0) < std::abs(under(0) - 4.0)) under = s;
    const bool passed = plant.x(0) > 6.0 && run.count(CycleStatus::Failed) == 0;
    std::printf("LOWPASS beam %.2f m: %s, closest %.4f m, holds %d, x %.2f, q under [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
                bottom, passed ? "passed" : "STOPPED", run.closest, run.holding, plant.x(0), under(3), under(4),
                under(5), under(6), under(7), under(8));
  }
}

// ---- margin shortfall diagnosis (Phase 4B-A Task 10, step 2b; run with --gtest_also_run_disabled_tests) ----

TEST(MarginDiagnosis, DISABLED_LowBeam)
{
  // Per cycle near the beam: the plant's true clearance, the planned stage 1's true clearance, the smallest true
  // clearance over the planned stages, the linearized prediction of stage 1's closest row (d + g dx), the largest
  // slack and the dropped pairs.
  auto c = gated_controller();
  World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
  const CollisionModel model(primitives(), CollisionConfig{});
  const double dt = 0.1;
  for (int i = 0; i < 200 && plant.x(0) < 6.0; ++i) {
    const double now = i * dt;
    const CycleOutput & out = c->step(now, plant.x, true, world, plant.at_rest());
    const auto boxes = world.at(0.0, now);
    const double here = model.nearest(plant.x, boxes)->distance;
    const double stage1 = model.nearest(out.plan.x[1], boxes)->distance;
    double plan_min = 1e9;
    int at = -1;
    for (size_t k = 1; k < out.plan.x.size(); ++k) {
      const double d = model.nearest(out.plan.x[k], boxes)->distance;
      if (d < plan_min) plan_min = d, at = static_cast<int>(k);
    }
    // Between stages 0 and 1 (the plant's motion this cycle), at ten points.
    double between = 1e9;
    for (int j = 1; j < 10; ++j) {
      const State xi = plant.x + (out.plan.x[1] - out.plan.x[0]) * (j / 10.0);
      between = std::min(between, model.nearest(xi, boxes)->distance);
    }
    if (here < 0.08 || plan_min < 0.06)
      std::printf("t %.1f x %.3f status %s here %.4f stage1 %.4f between %.4f plan_min %.4f@%d slack %.4f dropped %d "
                  "rows %d choice %s\n", now, plant.x(0), to_string(out.status), here, stage1, between, plan_min, at,
                  out.max_slack, out.collision.dropped_pairs, out.collision.active_pairs, to_string(out.posture_choice));
    plant.step(out.command, dt);
  }
}

TEST(MarginDiagnosis, DISABLED_LowBeamRemedies)
{
  // Closest approach and time under the 1.34 m beam (and the narrow gate) for candidate remedies.
  struct Variant
  {
    const char * name;
    double slack, slack_quadratic, tolerance, extra_margin, slack_inside = -1.0;
  };
  const Variant variants[] = {{"as is", 100, 1000, 0.005, 0.0},
                              {"inside x10", 100, 1000, 0.005, 0.0, 1000.0},
                              {"inside x30", 100, 1000, 0.005, 0.0, 3000.0},
                              {"inside x100", 100, 1000, 0.005, 0.0, 10000.0}};
  for (const auto & v : variants) {
    Weights w = config().weights;
    w.slack = v.slack;
    w.slack_quadratic = v.slack_quadratic;
    w.slack_inside = v.slack_inside;
    SlackGate g = gate();
    g.tolerance_m = v.tolerance;
    CollisionConfig cfg;
    cfg.margin_m += v.extra_margin;
    cfg.base_margin_m = cfg.margin_m;
    for (const bool beam : {true, false}) {
      Controller c(Formulation(config().horizon, config().dt, limits(), w, cfg), CollisionModel(primitives(), cfg),
                   std::make_unique<OsqpSolver>(solver_options()), breakaway(), kDefaultFallbackCycles, kGoalXyM,
                   kGoalYawRad, g);
      const PostureTargets targets = both_sides();
      World world = beam ? world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}})
                         : map_world();
      Plant plant{beam ? state(0.0, 0.0, 0.0, targets.comfort) : state(-2.2, kGateY, M_PI, targets.comfort)};
      if (beam) c.start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
      else c.start(Route({Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
      const GateLoop run = run_gate_loop(c, plant, world, 0.1, 90.0, [beam](const State & x) {
        return beam ? x(0) > 6.0 : x(0) < -12.4;
      });
      std::printf("%-20s %-5s closest %.4f passed %d cycles %zu holding %d\n", v.name, beam ? "beam" : "gate",
                  run.closest, beam ? plant.x(0) > 6.0 : plant.x(0) < -12.4, run.statuses.size(), run.holding);
    }
  }
}

// ---- V2 map ----

namespace
{
World v2_world()
{
  return world_of(mmn::known_boxes(share("mobile_manipulator_navigation") + "/maps/construction_site_v2.boxes.yaml", nullptr));
}
}  // namespace

// ---- side choice along the route (Phase 4B-A Task 11: v2_room_r4_nav picked right at the hallway's right turn
// and kept it through the left turn into R4) ----

namespace
{
// The global path of the failed run (20261018T194619Z), from the spine north of door A: right into the hallway,
// east, left through the 1.05 m R4 door (north partition, y 13.75) into R4.
Route r4_route()
{
  return Route({Vector2d(5.28, 9.5), Vector2d(5.28, 11.35), Vector2d(5.30, 11.45), Vector2d(5.34, 11.6),
                Vector2d(5.40, 11.75), Vector2d(5.48, 11.89), Vector2d(5.57, 12.02), Vector2d(5.67, 12.14),
                Vector2d(5.78, 12.25), Vector2d(5.89, 12.36), Vector2d(6.01, 12.45), Vector2d(6.14, 12.51),
                Vector2d(6.28, 12.54), Vector2d(6.43, 12.55), Vector2d(7.08, 12.55), Vector2d(7.19, 12.56),
                Vector2d(7.30, 12.58), Vector2d(7.42, 12.62), Vector2d(7.54, 12.69), Vector2d(7.63, 12.78),
                Vector2d(7.71, 12.89), Vector2d(7.76, 13.01), Vector2d(7.78, 13.15), Vector2d(7.80, 13.29),
                Vector2d(7.80, 13.50), Vector2d(7.81, 14.0), Vector2d(7.82, 14.5), Vector2d(7.83, 15.2)});
}
}  // namespace

TEST(RouteProbe, R4ChoosesTheSideThatClearsTheWholeSegment)
{
  // Held rigidly on the route, the right pose passes the hallway corner 0.53 m from the partitions, the left 0.62 m,
  // and both 0.18 m in the R4 door: left (the window alone tied, 0.655 / 0.641, and the right turn took it right).
  auto c = gated_controller();
  World world = v2_world();
  const PostureTargets targets = both_sides();
  Plant plant{state(5.28, 9.5, M_PI / 2, targets.comfort)};
  c->start(r4_route(), {7.83, 15.2, M_PI / 2}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 120.0, [](const State & x) { return x(1) > 14.4; });
  run.print("R4 along the route");
  const auto first = std::find_if(run.choices.begin(), run.choices.end(), [](PostureChoice p) {
    return p == PostureChoice::Tight0 || p == PostureChoice::Tight1;
  });
  ASSERT_NE(first, run.choices.end()) << "never chose a tight pose";
  EXPECT_EQ(*first, PostureChoice::Tight0) << "the right pose was chosen";
  EXPECT_GT(plant.x(1), 14.4) << "did not enter R4";
  EXPECT_GE(run.closest, CollisionConfig{}.margin_m - 0.005);
}

// ---- collision-checked arm braking hold (Phase 4B-A Task 10, user decision 2026-10-18) ----

namespace
{
// Smallest arm/panel clearance along a constant-deceleration stop from (q0, v0) over `duration` to `target`.
double along_stop(const CollisionModel & model, const World & world, const State & x, const Joints & v0, double duration,
                  const Joints & target)
{
  double worst = 1e9;
  for (int i = 0; i <= 20; ++i) {
    const double s = i / 20.0;
    Joints q = x.tail<6>() + v0 * duration * (s - 0.5 * s * s);
    if (i == 20) q = target;
    worst = std::min(worst, model.clearance_with_joints({x}, q, 0.1, 0.0, world));
  }
  return worst;
}
}  // namespace

namespace
{
// A box `gap` beside the carry panel (its edge side), and the robot state.
World box_beside_the_panel(const CollisionModel & model, const State & x, double gap)
{
  const double probe = 1.0;  // within the 0.8 m activation distance of the panel's edge (0.6 m)
  auto box_at = [](double y) { return world_of({mmn::Box{"box", Vector3d(0.0, y, 1.3), Vector3d(2.0, 0.2, 0.6), 0.0}}); };
  const double at_probe = model.clearance_with_joints({x}, x.tail<6>(), 0.1, 0.0, box_at(probe));
  return box_at(probe - (at_probe - gap));
}
}  // namespace

TEST(ArmHold, ShortensAStopThatWouldReachAnObstacle)
{
  // A box 0.10 m beside the carry panel; the arm swings (pan, 0.5 rad/s) either way, which swings a panel corner
  // towards it: the full stop (0.82 s, 0.2 rad) would reach it; a faster stop keeps the margin.
  const CollisionModel model(primitives(), CollisionConfig{});
  const State x = state(0.0, 0.0, 0.0, posture().comfort);
  const World world = box_beside_the_panel(model, x, 0.10);
  ASSERT_NEAR(model.clearance_with_joints({x}, x.tail<6>(), 0.1, 0.0, world), 0.10, 1e-6);
  const double margin = CollisionConfig{}.margin_m;
  for (const double sign : {1.0, -1.0}) {
    Joints v0 = Joints::Zero();
    v0(0) = 0.5 * sign;
    const double t_full = std::max(kArmHoldMinS, 0.5 / limits().qdd_max(0));
    const double full = along_stop(model, world, x, v0, t_full, x.tail<6>() + 0.5 * t_full * v0);
    ASSERT_LT(full, margin - 0.001) << "the unchecked stop must reach the box for this test";
    const ArmHold hold = safe_arm_hold(model, world, 0.0, x, v0, limits());
    EXPECT_GT(hold.decel_factor, 1.0);
    EXPECT_LT(hold.decel_factor, std::numeric_limits<double>::infinity()) << "a faster stop suffices with 0.10 m";
    EXPECT_GE(along_stop(model, world, x, v0, hold.duration_s, hold.target), margin - 0.0011);
    EXPECT_NEAR(hold.duration_s, t_full / hold.decel_factor, 1e-12);
  }
}

TEST(ArmHold, NoFasterStopKeepsTheMarginStopsAtTheReference)
{
  // 0.055 m from the box: even an eightfold faster stop takes the panel inside the margin, so the arm stops at the
  // reference within kHoldEmergencyS.
  const CollisionModel model(primitives(), CollisionConfig{});
  const State x = state(0.0, 0.0, 0.0, posture().comfort);
  const World world = box_beside_the_panel(model, x, 0.055);
  Joints v0 = Joints::Zero();
  v0(0) = 0.5;
  const ArmHold hold = safe_arm_hold(model, world, 0.0, x, v0, limits());
  EXPECT_EQ(hold.decel_factor, std::numeric_limits<double>::infinity());
  EXPECT_LT((hold.target - x.tail<6>()).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(hold.duration_s, kHoldEmergencyS);
}

TEST(ArmHold, AStopAwayFromObstaclesIsUnchanged)
{
  const CollisionModel model(primitives(), CollisionConfig{});
  const State x = state(0.0, 0.0, 0.0, posture().comfort);
  Joints v0 = Joints::Zero();
  v0(0) = 0.5;
  const ArmHold hold = safe_arm_hold(model, World{}, 0.0, x, v0, limits());
  const double t_full = std::max(kArmHoldMinS, 0.5 / limits().qdd_max(0));
  EXPECT_DOUBLE_EQ(hold.decel_factor, 1.0);
  EXPECT_NEAR(hold.duration_s, t_full, 1e-12);
  EXPECT_LT((hold.target - (x.tail<6>() + 0.5 * t_full * v0)).norm(), 1e-12);
}

TEST(ArmHold, AtRestIsTheHoldItself)
{
  const CollisionModel model(primitives(), CollisionConfig{});
  const State x = state(0.0, 0.0, 0.0, posture().comfort);
  const ArmHold hold = safe_arm_hold(model, World{}, 0.0, x, Joints::Zero(), limits());
  EXPECT_LT((hold.target - x.tail<6>()).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(hold.duration_s, kArmHoldMinS);
}

TEST(ArmHold, TheBasesBrakingTravelCounts)
{
  // Task 10 review: the base keeps braking while the arm stops. A box 0.10 m ahead of the carry panel, the base at
  // 0.25 m/s towards it, the arm still: the base alone travels 0.0625 m (0.25^2 / 2 / 0.5) and brings the panel inside
  // the margin; the arm's stop cannot help, so the reported clearance along the stop shows it, and an arm stop that
  // moves away from the box is not made faster for the base's sake.
  const CollisionModel model(primitives(), CollisionConfig{});
  const State x = state(0.0, 0.0, 0.0, posture().comfort);
  const double probe = 1.2;
  auto box_at = [](double ahead) { return world_of({mmn::Box{"box", Vector3d(ahead, 0.0, 1.3), Vector3d(0.2, 2.0, 0.6), 0.0}}); };
  const double at_probe = model.clearance_with_joints({x}, x.tail<6>(), 0.1, 0.0, box_at(probe));
  const World world = box_at(probe - (at_probe - 0.10));
  ASSERT_NEAR(model.clearance_with_joints({x}, x.tail<6>(), 0.1, 0.0, world), 0.10, 1e-6);
  const ArmHold still = safe_arm_hold(model, world, 0.0, x, Joints::Zero(), limits(), Eigen::Vector2d(0.25, 0.0));
  EXPECT_LT(still.clearance_m, 0.045) << "the base's braking travel must show in the clearance along the stop";
  EXPECT_DOUBLE_EQ(still.decel_factor, 1.0);
}

TEST(ArmHold, TheEmergencyStopsOvershootIsChecked)
{
  // The stop at the reference within kHoldEmergencyS is a Hermite from (q0, v0) to (q0, 0): it overshoots by about
  // 4/27 v0 T towards the motion; the reported clearance includes it.
  const CollisionModel model(primitives(), CollisionConfig{});
  const State x = state(0.0, 0.0, 0.0, posture().comfort);
  const World world = box_beside_the_panel(model, x, 0.055);
  Joints v0 = Joints::Zero();
  v0(0) = 0.5;
  const ArmHold hold = safe_arm_hold(model, world, 0.0, x, v0, limits());
  ASSERT_EQ(hold.decel_factor, std::numeric_limits<double>::infinity());
  EXPECT_LT(hold.clearance_m, 0.055) << "the overshoot towards the box is part of the emergency stop";
}

// ---- Phase 4B-B: warm-start switch and deadline-aware solve cap (spec Sections 4.3, 4.7) ----

namespace
{
// OSQP that records how it was called.
class SpySolver : public QpSolver
{
public:
  SpySolver() : inner_(solver_options()) {}
  void setup(const QpProblem & shape) override { inner_.setup(shape); }
  SolveResult solve(const QpProblem & problem, const QpSolution * warm) override
  {
    ++solves;
    warm_given.push_back(warm != nullptr);
    return inner_.solve(problem, warm);
  }
  void set_time_limit(double seconds) override
  {
    limits.push_back(seconds);
    inner_.set_time_limit(seconds);
  }
  int solves = 0;
  std::vector<bool> warm_given;
  std::vector<double> limits;

private:
  OsqpSolver inner_;
};

std::unique_ptr<Controller> optioned_controller(std::unique_ptr<QpSolver> solver, const ControllerOptions & options)
{
  return std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::move(solver),
                                      breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, SlackGate{}, options);
}
}  // namespace

TEST(Controller, WarmStartOffPassesNoWarmSolution)
{
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.warm_start = false;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  for (int i = 0; i < 10; ++i) EXPECT_EQ(d.step(*c).status, CycleStatus::Ok);
  ASSERT_EQ(s->solves, 10);
  for (bool given : s->warm_given) EXPECT_FALSE(given);

  auto on = std::make_unique<SpySolver>();
  SpySolver * w = on.get();
  auto warm = optioned_controller(std::move(on), ControllerOptions{});
  Drive dw(*warm);
  for (int i = 0; i < 3; ++i) dw.step(*warm);
  EXPECT_FALSE(w->warm_given[0]);
  EXPECT_TRUE(w->warm_given[1]) << "the frozen default warm-starts";
}

TEST(Controller, DeadlineCapAlreadyExceededFallsBack)
{
  // A period of 1 us: every cycle is past its deadline before the solve, which is skipped as a solver failure.
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.deadline_cap = true;
  o.period_s = 1e-6;
  o.publish_margin_s = 0.0;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  const CycleOutput & out = d.step(*c);
  EXPECT_EQ(out.status, CycleStatus::Fallback);
  EXPECT_TRUE(out.time_limited);
  EXPECT_TRUE(out.solve_failed);
  EXPECT_EQ(s->solves, 0);
}

TEST(Controller, DeadlineCapSetsTheTimeLeft)
{
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.deadline_cap = true;
  o.period_s = 1.0;
  o.publish_margin_s = 0.002;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  EXPECT_EQ(d.step(*c).status, CycleStatus::Ok);
  ASSERT_EQ(s->limits.size(), 1u);
  EXPECT_GT(s->limits[0], 0.9);
  EXPECT_LT(s->limits[0], 1.0 - 0.002) << "the cycle's time so far is taken off";
}

TEST(Controller, DeadlineCapCountsFromTheCycleStart)
{
  // The node's cycle starts before step() (goal handling, state, TF): the budget counts from there.
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.deadline_cap = true;
  o.period_s = 1.0;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  c->set_cycle_start(std::chrono::steady_clock::now() - std::chrono::milliseconds(500));
  EXPECT_EQ(d.step(*c).status, CycleStatus::Ok);
  ASSERT_EQ(s->limits.size(), 1u);
  EXPECT_LT(s->limits[0], 0.5);
  EXPECT_GT(s->limits[0], 0.4);
  d.step(*c);
  ASSERT_EQ(s->limits.size(), 2u);
  EXPECT_GT(s->limits[1], 0.9) << "the cycle start is used once";
}

TEST(Controller, DeadlineCapNeedsAMarginBelowThePeriod)
{
  ControllerOptions o;
  o.deadline_cap = true;
  o.period_s = 0.002;
  o.publish_margin_s = 0.002;
  EXPECT_THROW(optioned_controller(std::make_unique<OsqpSolver>(solver_options()), o), std::invalid_argument);
  o.deadline_cap = false;
  EXPECT_NO_THROW(optioned_controller(std::make_unique<OsqpSolver>(solver_options()), o));
}

TEST(Controller, DeadlineCapOffNeverSetsALimit)
{
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.period_s = 0.02;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  for (int i = 0; i < 5; ++i) d.step(*c);
  EXPECT_TRUE(s->limits.empty());
}

TEST(SolverFactory, BuildsTheConfiguredBackend)
{
  MpcConfig c = config();
  c.solver.backend = "osqp";
  EXPECT_NE(dynamic_cast<OsqpSolver *>(make_solver(c).get()), nullptr);
  c.solver.backend = "hpipm";
  EXPECT_NE(dynamic_cast<HpipmSolver *>(make_solver(c).get()), nullptr);
  c.solver.backend = "qpoases";
  EXPECT_THROW(make_solver(c), std::invalid_argument);
}

TEST(Controller, DrivesWithTheHpipmBackend)
{
  MpcConfig c = config();
  c.solver.backend = "hpipm";
  auto controller = make_controller(make_solver(c));
  Drive d(*controller);
  for (int i = 0; i < 30; ++i) ASSERT_EQ(d.step(*controller).status, CycleStatus::Ok) << "cycle " << i;
  EXPECT_GT(d.plant.x(0), 0.3) << "the base moved along the route";
}

TEST(Controller, ReportsWhetherTheSolveFailed)
{
  auto c = make_controller(std::make_unique<ScriptedSolver>([](int call) { return call == 1; }));
  Drive d(*c);
  EXPECT_FALSE(d.step(*c).solve_failed);
  const CycleOutput & failed = d.step(*c);
  EXPECT_EQ(failed.status, CycleStatus::Fallback);
  EXPECT_TRUE(failed.solve_failed);
  EXPECT_FALSE(d.step(*c).solve_failed);
}

TEST(Controller, DiagnosticsAreTimedApartFromTheCycle)
{
  ControllerOptions o;
  o.diagnostics = true;
  auto c = optioned_controller(std::make_unique<OsqpSolver>(solver_options()), o);
  World world = world_of({mmn::Box{"beam", Vector3d(2.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  Plant plant{state(0.0, 0.0, 0.0, posture().comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, posture());
  bool measured = false;
  for (int i = 0; i < 60 && !measured; ++i) {
    const auto before = std::chrono::steady_clock::now();
    const CycleOutput & out = c->step(i * 0.1, plant.x, true, world, plant.at_rest());
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - before).count();
    if (std::isfinite(out.lin_error_m)) {
      measured = true;
      EXPECT_GT(out.timing.diagnostics_s, 0.0);
      EXPECT_LE(out.timing.total_s + out.timing.diagnostics_s, wall + 1e-6) << "total_s leaves the diagnostics out";
    }
    plant.step(out.command, 0.1);
  }
  EXPECT_TRUE(measured);
}

// ---- Phase 4B-B: the second QP iteration with a step check (spec Section 4.4) ----

namespace
{
// OSQP whose every second call (the second iteration of a cycle) is altered by `second`.
class SecondSolveSolver : public QpSolver
{
public:
  explicit SecondSolveSolver(std::function<void(SolveResult &)> second) : second_(std::move(second)), inner_(solver_options()) {}
  void setup(const QpProblem & shape) override { inner_.setup(shape); }
  SolveResult solve(const QpProblem & problem, const QpSolution * warm) override
  {
    SolveResult r = inner_.solve(problem, warm);
    if (calls++ % 2 == 1) second_(r);
    return r;
  }
  int calls = 0;

private:
  std::function<void(SolveResult &)> second_;
  OsqpSolver inner_;
};

std::unique_ptr<Controller> iterated_controller(int iterations, std::unique_ptr<QpSolver> solver = nullptr,
                                                double period_s = 0.0)
{
  ControllerOptions o;
  o.qp_iterations = iterations;
  o.period_s = period_s;
  if (!solver) solver = std::make_unique<OsqpSolver>(solver_options());
  return std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::move(solver),
                                      breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate(), o);
}

GateLoop beam_run(int iterations)
{
  auto c = iterated_controller(iterations);
  World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
  return run_gate_loop(*c, plant, world, 0.1, 60.0, [](const State & x) { return x(0) > 6.0; });
}

GateLoop shelf_run(int iterations)
{
  auto c = iterated_controller(iterations);
  World world = world_of_map_plus({mmn::Box{"shelf", Vector3d(-5.0, kGateY - 0.32 - 0.5, 1.475),
                                            Vector3d(4.0, 1.0, 1.45), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(-1.0, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-1.0, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  return run_gate_loop(*c, plant, world, 0.1, 120.0, [](const State & x) { return x(0) < -12.4; });
}
}  // namespace

TEST(Controller, Iter2RejectsAWorseIterate)
{
  // The second solve's controls are bent by 0.3 rad/s on every arm joint: dearer, so the first iterate is kept.
  auto bent = std::make_unique<SecondSolveSolver>([](SolveResult & r) {
    for (auto & du : r.solution.du) du.tail<6>().array() += 0.3;
  });
  auto c = iterated_controller(2, std::move(bent));
  auto one = iterated_controller(1);
  Drive d(*c), d1(*one);
  const CycleOutput & out = d.step(*c);
  EXPECT_TRUE(out.iter2_rejected);
  EXPECT_FALSE(out.iter2_used);
  const Plan kept = out.plan;
  const Plan & first = d1.step(*one).plan;
  for (size_t k = 0; k < kept.u.size(); ++k) EXPECT_TRUE(kept.u[k].isApprox(first.u[k], 1e-12)) << "stage " << k;
}

TEST(Controller, Iter2BudgetSkipsTheSecondSolve)
{
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  auto c = iterated_controller(2, std::move(spy), 1e-6);
  Drive d(*c);
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(d.step(*c).iter2_used);
  EXPECT_EQ(s->solves, 5) << "one solve per cycle";
}

TEST(Controller, Iter2FailureKeepsFirstIterate)
{
  auto failing = std::make_unique<SecondSolveSolver>([](SolveResult & r) {
    r = SolveResult{};
    r.status = "scripted failure";
  });
  auto c = iterated_controller(2, std::move(failing));
  auto one = iterated_controller(1);
  Drive d(*c), d1(*one);
  // The first cycle's plan is the one-iteration plan; later cycles differ in OSQP's rho history (the failed second
  // solve still adapted it), not in what is kept.
  for (int i = 0; i < 5; ++i) {
    const CycleOutput & out = d.step(*c);
    EXPECT_EQ(out.status, CycleStatus::Ok);
    EXPECT_FALSE(out.solve_failed) << "the first solve succeeded";
    EXPECT_FALSE(out.iter2_used);
    EXPECT_FALSE(out.iter2_rejected) << "a failed second solve is not a rejected iterate";
    if (i == 0) {
      const Plan kept = out.plan;
      const Plan & first = d1.step(*one).plan;
      for (size_t k = 0; k < kept.u.size(); ++k) EXPECT_TRUE(kept.u[k].isApprox(first.u[k], 1e-12)) << "stage " << k;
    }
  }
}

// The plan expected the second iteration to raise the clearance in these loops; it does not (2026-10-22: beam 0.0492 m
// against 0.0509 m with one iteration, shelf 0.0891 against 0.0941 m; with the horizon-only step check 0.0421 m under
// the beam). The tests keep what must hold, the pass and the 0.045 m criterion; the sweep decides the switch.
TEST(Controller, Iter2KeepsTheCriterionUnderTheBeam)
{
  const GateLoop one = beam_run(1), two = beam_run(2);
  one.print("low beam, one iteration");
  two.print("low beam, two iterations");
  EXPECT_GT(two.states.back()(0), 5.9) << "did not pass under the beam";
  EXPECT_GE(two.closest, 0.045);
}

TEST(Controller, Iter2KeepsTheCriterionBesideTheShelf)
{
  const GateLoop one = shelf_run(1), two = shelf_run(2);
  one.print("shelf, one iteration");
  two.print("shelf, two iterations");
  EXPECT_LT(two.states.back()(0), -12.3) << "did not pass the gate";
  EXPECT_GE(two.closest, 0.045);
}

TEST(Iter2StepCheck, KeepsOnlyAnIterateThatIsNoWorse)
{
  // near: worst violation on stages 1..kIter2NearStages; horizon: on the gate's horizon; cost: the objective.
  EXPECT_TRUE(accept_second_iterate({0.01, 0.02, 1.0}, {0.005, 0.01, 1.005}, 0.01));
  EXPECT_FALSE(accept_second_iterate({0.0, 0.02, 1.0}, {0.003, 0.0, 0.9}, 0.01)) << "the near term got worse";
  EXPECT_FALSE(accept_second_iterate({0.0, 0.0, 1.0}, {0.0, 0.004, 0.9}, 0.01)) << "the horizon got worse";
  EXPECT_FALSE(accept_second_iterate({0.0, 0.0, 1.0}, {0.0, 0.0, 1.02}, 0.01)) << "dearer beyond the tolerance";
  EXPECT_TRUE(accept_second_iterate({0.0, 0.0, 1.0}, {1e-7, 0.0, 1.0}, 0.01)) << "solver noise";
}
TEST(Controller, Iter2KeptPlanIsARollout)
{
  // In open space the second iterate is kept on some cycles; the executed plan is then the second step, rolled out
  // from the measured state.
  auto c = iterated_controller(2);
  Drive d(*c);
  int used = 0;
  for (int i = 0; i < 20; ++i) {
    const State measured = d.plant.x;
    const CycleOutput & out = d.step(*c);
    ASSERT_EQ(out.status, CycleStatus::Ok);
    if (!out.iter2_used) continue;
    ++used;
    const Plan rolled = formulation().rollout(out.plan.x[0], out.plan.u);
    for (size_t k = 0; k < rolled.x.size(); ++k) EXPECT_TRUE(rolled.x[k].isApprox(out.plan.x[k], 1e-9)) << k;
    EXPECT_LT((out.plan.x[0] - measured).cwiseAbs().maxCoeff(), 1e-2);
  }
  EXPECT_GT(used, 0) << "no second iterate was kept";
}

TEST(Controller, Iter2SkippedWhenNoTimeIsLeft)
{
  // The cap leaves the second solve no time (period 0.5 s, margin just below it): one solve per cycle.
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.qp_iterations = 2;
  o.deadline_cap = true;
  o.period_s = 0.5;
  o.publish_margin_s = 0.5 - 1e-9;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  for (int i = 0; i < 3; ++i) EXPECT_FALSE(d.step(*c).iter2_used);
  EXPECT_LE(s->solves, 3);
}

// ---- Phase 4B-B: panel-against-arm-link rows in closed loop (spec Section 4.6) ----

TEST(Controller, SelfRowsKeepThePanelOffTheArmUnderAPostureIntoIt)
{
  // A comfort pose with the panel inside the upper arm (found by sampling the joint ranges): the posture term pulls the
  // arm into itself. Without the rows the arm goes inside the margin; with them it holds the margin.
  CollisionConfig cc;
  cc.self_pairs_per_stage = 2;
  cc.self_links = {"forearm_link", "shoulder_pan_link", "upper_arm_link", "wrist_1_link"};
  const CollisionModel probe(primitives(), cc);
  PostureTargets targets = posture();
  targets.mode = PostureMode::ComfortOnly;
  targets.comfort << 1.00649, 0.0389859, 1.72881, 2.80244, -1.8021, 4.22147;
  ASSERT_LT(probe.nearest_self(state(0, 0, 0, targets.comfort))->distance, 0.0) << "the target is self-colliding";
  for (bool on : {false, true}) {
    CollisionConfig used = cc;
    used.self_rows = on;
    auto c = std::make_unique<Controller>(Formulation(config().horizon, config().dt, limits(), config().weights, used),
                                          CollisionModel(primitives(), used),
                                          std::make_unique<OsqpSolver>(solver_options()), breakaway(),
                                          kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate());
    Plant plant{state(0, 0, 0, Joints::Zero())};
    c->start(Route({Vector2d(0, 0), Vector2d(20, 0)}), {20.0, 0.0, 0.0}, {}, targets);
    const World world;
    double least = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 300; ++i) {
      const CycleOutput & out = c->step(i * 0.1, plant.x, true, world, plant.at_rest());
      plant.step(out.command, 0.1);
      least = std::min(least, probe.nearest_self(plant.x)->distance);
    }
    std::printf("arm rows %s: closest panel to arm link %.4f m\n", on ? "on" : "off", least);
    if (on) {
      EXPECT_GE(least, CollisionConfig{}.margin_m - 2e-3);
    } else {
      EXPECT_LT(least, CollisionConfig{}.margin_m - 0.01) << "the case must fail without the rows";
    }
  }
}

TEST(Controller, SelfRowsKeepTheRightSwingPlansOffTheArm)
{
  // The right swing past the shelf (vertical_carry_right reached through the arm's own workspace): with the rows on,
  // no planned stage brings the panel within the margin of an arm link.
  CollisionConfig cc;
  cc.self_rows = true;
  cc.self_pairs_per_stage = 2;
  cc.self_links = {"forearm_link", "shoulder_pan_link", "upper_arm_link", "wrist_1_link"};
  for (bool on : {false, true}) {
    CollisionConfig used = cc;
    used.self_rows = on;
    auto c = std::make_unique<Controller>(Formulation(config().horizon, config().dt, limits(), config().weights, used),
                                          CollisionModel(primitives(), used),
                                          std::make_unique<OsqpSolver>(solver_options()), breakaway(),
                                          kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate());
    const CollisionModel probe(primitives(), cc);
    World world = world_of_map_plus({mmn::Box{"shelf", Vector3d(-5.0, kGateY - 0.32 - 0.5, 1.475),
                                              Vector3d(4.0, 1.0, 1.45), 0.0}});
    const PostureTargets targets = both_sides();
    Plant plant{state(-1.0, kGateY, M_PI, targets.comfort)};
    c->start(Route({Vector2d(-1.0, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
    double least = std::numeric_limits<double>::infinity();
    int self_rows = 0;
    for (int i = 0; i < 1200 && plant.x(0) > -12.4; ++i) {
      const CycleOutput & out = c->step(i * 0.1, plant.x, true, world, plant.at_rest());
      self_rows += out.collision.self_pairs;
      if (out.status == CycleStatus::Ok)
        for (const State & x : out.plan.x) least = std::min(least, probe.nearest_self(x)->distance);
      plant.step(out.command, 0.1);
    }
    std::printf("arm rows %s: closest planned panel to arm link %.4f m, %d arm rows, final x %.2f\n", on ? "on" : "off",
                least, self_rows, plant.x(0));
    if (on) {
      EXPECT_LT(plant.x(0), -12.4) << "did not pass the gate";
      EXPECT_GE(least, CollisionConfig{}.margin_m - 1e-3);
    }
  }
}

// ---- Phase 4B-B: the preparation/feedback split (spec Section 4.5) ----

namespace
{
// `tight`: an OSQP tolerance far below the configured 1e-3, for comparisons of equivalent QPs written differently.
std::unique_ptr<Controller> split_controller(bool split, bool tight = false)
{
  ControllerOptions o;
  o.rti_split = split;
  o.period_s = 0.05;
  OsqpOptions solver = solver_options();
  if (tight) {
    solver.eps_abs = solver.eps_rel = 1e-9;
    solver.max_iter = 50000;
  }
  return std::make_unique<Controller>(formulation(), CollisionModel(primitives(), CollisionConfig{}),
                                      std::make_unique<OsqpSolver>(solver), breakaway(),
                                      kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate(), o);
}

// The state the split predicts for t_next: one model step with the command just sent.
State predicted(const State & x, const Control & sent, double elapsed) { return discrete_step(x, sent, elapsed).x_next; }
}  // namespace

TEST(Controller, RtiFeedbackAtPredictedStateEqualsFullSolve)
{
  auto split = split_controller(true), full = split_controller(false);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  State x = state(0, 0, 0, posture().comfort);
  split->start(route, {10.0, 0.0, 0.0}, {}, posture());
  full->start(route, {10.0, 0.0, 0.0}, {}, posture());
  const World world;
  for (int i = 0; i < 15; ++i) {
    const double now = i * 0.05;
    const CycleOutput & a = split->step(now, x, true, world, i == 0);
    const Control command = a.command;
    const CycleOutput & b = full->step(now, x, true, world, i == 0);
    ASSERT_EQ(a.status, CycleStatus::Ok);
    EXPECT_LT((command - b.command).cwiseAbs().maxCoeff(), 1e-6) << "cycle " << i;
    if (i > 0) EXPECT_GT(a.timing.prep_s, 0.0) << "cycle " << i << " used the preparation";
    split->prepare(now + 0.05, world);
    x = predicted(x, command, (now + 0.05) - now);  // the controller's elapsed time, bit for bit
  }
}

TEST(Controller, RtiLargeDeviationRunsFullCycle)
{
  auto split = split_controller(true), full = split_controller(false);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  State x = state(0, 0, 0, posture().comfort);
  split->start(route, {10.0, 0.0, 0.0}, {}, posture());
  full->start(route, {10.0, 0.0, 0.0}, {}, posture());
  const World world;
  const Control command = split->step(0.0, x, true, world, true).command;
  full->step(0.0, x, true, world, true);
  split->prepare(0.05, world);
  State off = predicted(x, command, 0.05);
  off(1) += 0.06;  // 6 cm sideways: beyond rti_max_deviation's 0.05 m
  const CycleOutput & a = split->step(0.05, off, true, world, false);
  const Control ca = a.command;
  EXPECT_DOUBLE_EQ(a.timing.prep_s, 0.0) << "prepared again in-cycle";
  const CycleOutput & b = full->step(0.05, off, true, world, false);
  EXPECT_LT((ca - b.command).cwiseAbs().maxCoeff(), 1e-9) << "the same full cycle";
}

TEST(Controller, RtiWithoutPreparationRunsFullCycle)
{
  auto split = split_controller(true), full = split_controller(false);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  const State x = state(0, 0, 0, posture().comfort);
  split->start(route, {10.0, 0.0, 0.0}, {}, posture());
  full->start(route, {10.0, 0.0, 0.0}, {}, posture());
  const World world;
  const CycleOutput & a = split->step(0.0, x, true, world, true);  // nothing prepared yet
  EXPECT_DOUBLE_EQ(a.timing.prep_s, 0.0);
  const Control ca = a.command;
  EXPECT_LT((ca - full->step(0.0, x, true, world, true).command).cwiseAbs().maxCoeff(), 1e-12);
  // A preparation for another time is not used either.
  split->prepare(1.0, world);
  EXPECT_DOUBLE_EQ(split->step(0.05, x, true, world, false).timing.prep_s, 0.0);
  // Nor after a hold.
  split->prepare(0.1, world);
  split->set_external_hold(true);
  EXPECT_EQ(split->step(0.1, x, true, world, false).status, CycleStatus::Blocked);
  split->set_external_hold(false);
  split->prepare(0.15, world);
  EXPECT_DOUBLE_EQ(split->step(0.15, x, true, world, false).timing.prep_s, 0.0) << "no preparation after a hold";
}

TEST(Controller, RtiReportsPrepAndFeedbackTimes)
{
  auto split = split_controller(true);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  State x = state(0, 0, 0, posture().comfort);
  split->start(route, {10.0, 0.0, 0.0}, {}, posture());
  const World world;
  const Control command = split->step(0.0, x, true, world, true).command;
  split->prepare(0.05, world);
  const CycleOutput & out = split->step(0.05, predicted(x, command, 0.05), true, world, false);
  EXPECT_GT(out.timing.prep_s, 0.0);
  EXPECT_GT(out.timing.feedback_s, 0.0);
  EXPECT_LE(out.timing.feedback_s, out.timing.total_s + 1e-9);
}

TEST(Controller, RejectsFormulationAndCollisionWithDifferentRowBudgets)
{
  CollisionConfig with;
  with.self_rows = true;
  with.self_links = {"upper_arm_link"};
  EXPECT_THROW(Controller(formulation(), CollisionModel(primitives(), with), std::make_unique<OsqpSolver>(solver_options()),
                          breakaway()),
               std::invalid_argument);
}

TEST(Controller, RtiPreemptionKeepsTheSplitEqualToTheFullCycle)
{
  // The behaviour tree resends the path in place between cycles: a preparation it drops must be undone (the tightness
  // filter and the side picker carry over goals). Under the beam, where tightness is not zero.
  auto split = split_controller(true), full = split_controller(false);
  const World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  const Route route({Vector2d(2.5, 0), Vector2d(8, 0)});
  const PostureTargets targets = both_sides();
  State x = state(2.5, 0, 0, targets.comfort);  // the beam within the horizon: the tightness rises
  split->start(route, {8.0, 0.0, 0.0}, {}, targets);
  full->start(route, {8.0, 0.0, 0.0}, {}, targets);
  double highest = 0.0;
  for (int i = 0; i < 60; ++i) {
    const double now = i * 0.05;
    const CycleOutput & a = split->step(now, x, true, world, i == 0);
    const Control command = a.command;
    const double tightness = a.tightness;
    const CycleOutput & b = full->step(now, x, true, world, i == 0);
    EXPECT_LT((command - b.command).cwiseAbs().maxCoeff(), 1e-6) << "cycle " << i;
    EXPECT_NEAR(tightness, b.tightness, 1e-9) << "cycle " << i;
    highest = std::max(highest, tightness);
    split->prepare(now + 0.05, world);
    if (i % 3 == 0) {  // a replan with the same path
      split->update_goal(route, {8.0, 0.0, 0.0}, {});
      full->update_goal(route, {8.0, 0.0, 0.0}, {});
    }
    x = predicted(x, command, (now + 0.05) - now);  // the controller's elapsed time, bit for bit
  }
  EXPECT_GT(highest, 0.05) << "the tightness filter took part";
}

TEST(Controller, RtiFeedbackFollowsAResetCommand)
{
  // The node passes the executed command (reset_command) after the preparation: the feedback's u_prev deviation. The
  // feedback QP is the full QP with u_prev shifted linearly: equal answers at a tight solver tolerance.
  auto split = split_controller(true, true), full = split_controller(false, true);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  State x = state(0, 0, 0, posture().comfort);
  split->start(route, {10.0, 0.0, 0.0}, {}, posture());
  full->start(route, {10.0, 0.0, 0.0}, {}, posture());
  const World world;
  Control command = Control::Zero();
  for (int i = 0; i < 6; ++i) {
    command = split->step(i * 0.05, x, true, world, i == 0).command;
    full->step(i * 0.05, x, true, world, i == 0);
    split->prepare(i * 0.05 + 0.05, world);
    x = predicted(x, command, (i * 0.05 + 0.05) - i * 0.05);  // the controller's elapsed time, bit for bit
  }
  Control executed = command;
  executed(0) *= 0.5;  // the collision monitor scaled the base command
  executed(3) += 0.05;
  split->reset_command(executed);
  full->reset_command(executed);
  const CycleOutput & a = split->step(0.3, x, true, world, false);
  EXPECT_GT(a.timing.prep_s, 0.0) << "the preparation was used";
  const Control ca = a.command;
  EXPECT_LT((ca - full->step(0.3, x, true, world, false).command).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(Controller, RtiFeedbackWithinToleranceIsCloseToTheFullSolve)
{
  auto split = split_controller(true), full = split_controller(false);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  State x = state(0, 0, 0, posture().comfort);
  split->start(route, {10.0, 0.0, 0.0}, {}, posture());
  full->start(route, {10.0, 0.0, 0.0}, {}, posture());
  const World world;
  Control command = Control::Zero();
  for (int i = 0; i < 6; ++i) {
    command = split->step(i * 0.05, x, true, world, i == 0).command;
    full->step(i * 0.05, x, true, world, i == 0);
    split->prepare(i * 0.05 + 0.05, world);
    x = predicted(x, command, (i * 0.05 + 0.05) - i * 0.05);  // the controller's elapsed time, bit for bit
  }
  State off = x;
  off(1) += 0.02;  // 2 cm left of the route, within rti_max_deviation
  off(2) += 0.02;
  const CycleOutput & a = split->step(0.3, off, true, world, false);
  EXPECT_GT(a.timing.prep_s, 0.0);
  const Control ca = a.command;
  const Control cb = full->step(0.3, off, true, world, false).command;
  EXPECT_LT((ca - cb).cwiseAbs().maxCoeff(), 0.02) << "a linear correction close to the full re-linearization";
  EXPECT_LT(ca(1), 0.0) << "turns back towards the route";
}

TEST(Controller, RtiCapReservesThePreparation)
{
  auto spy = std::make_unique<SpySolver>();
  SpySolver * s = spy.get();
  ControllerOptions o;
  o.rti_split = true;
  o.deadline_cap = true;
  o.period_s = 0.5;
  o.publish_margin_s = 0.002;
  auto c = optioned_controller(std::move(spy), o);
  Drive d(*c);
  d.step(*c);
  c->prepare(d.cycle * d.dt, d.world);
  const CycleOutput & out = d.step(*c);
  ASSERT_GT(out.timing.prep_s, 0.0);
  ASSERT_EQ(s->limits.size(), 2u);
  EXPECT_LE(s->limits[1], 0.5 - 0.002 - out.timing.prep_s + 1e-9) << "the next preparation's time is reserved";
}

// ---- Phase 4B-C: mid-stage collision rows ----
namespace
{
CollisionConfig midstage_config(bool on)
{
  CollisionConfig cfg;
  cfg.midstage_rows = on;
  return cfg;
}

std::unique_ptr<Controller> midstage_controller(bool on)
{
  const CollisionConfig cfg = midstage_config(on);
  ControllerOptions options;
  options.diagnostics = true;
  // Phase 4B-C's measurements were made under the frozen gate rule (E1 made absolute the default): pinned to it
  SlackGate g = gate();
  g.inside_rule = GateInsideRule::Increment;
  return std::make_unique<Controller>(Formulation(config().horizon, config().dt, limits(), config().weights, cfg),
                                      CollisionModel(primitives(), cfg), std::make_unique<OsqpSolver>(solver_options()),
                                      breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g, options);
}

// A plan driving at 0.3 m/s past a box beside the panel, the arm moving: its intervals bring a pair into activation.
Plan moving_plan(int n, double dt)
{
  Plan p;
  Control u = Control::Zero();
  u(0) = 0.3;
  u.tail<6>().setConstant(0.2);
  State x = state(0.0, 0.0, 0.0, both_sides().comfort);
  for (int k = 0; k <= n; ++k) {
    p.x.push_back(x);
    if (k < n) {
      p.u.push_back(u);
      x = discrete_step(x, u, dt).x_next;
    }
  }
  return p;
}

// The smallest clearance of a plan between its stages 1..last (5 sub-steps per interval, the MPC's model).
double between_stages(const CollisionModel & model, const Plan & plan, const World & world, double now, double dt,
                      int last)
{
  double worst = std::numeric_limits<double>::infinity();
  for (int k = 1; k < last && k < static_cast<int>(plan.u.size()); ++k) {
    State xs = plan.x[static_cast<size_t>(k)];
    for (int j = 1; j <= 5; ++j) {
      xs = discrete_step(xs, plan.u[static_cast<size_t>(k)], dt / 5).x_next;
      const auto nearest = model.nearest(xs, world.at(k * dt + j * dt / 5, now));
      if (nearest) worst = std::min(worst, nearest->distance);
    }
  }
  return worst;
}
}  // namespace

TEST(Midstage, RowGradientsMatchFiniteDifferences)
{
  const double dt = config().dt;
  const CollisionModel model(primitives(), midstage_config(true));
  const World world = world_of({mmn::Box{"crate", Vector3d(0.9, 0.55, 1.0), Vector3d(0.4, 0.3, 2.0), 0.0}});
  const Plan plan = moving_plan(config().horizon, dt);
  const std::vector<DistanceRow> rows = midstage_rows(model, plan, dt, 0.0, world, 1.0);
  ASSERT_FALSE(rows.empty()) << "no pair within activation at any midpoint";
  const double h = 1e-5;
  for (const DistanceRow & row : rows) {
    ASSERT_TRUE(row.midstage);
    const State & x = plan.x[static_cast<size_t>(row.stage)];
    const Control & u = plan.u[static_cast<size_t>(row.stage)];
    for (int i = 0; i < kNx; ++i) {
      State dx = State::Zero();
      dx(i) = h;
      const double actual = model.pair_distance(discrete_step(x + dx, u, dt / 2).x_next, row, world, 0.0);
      EXPECT_NEAR(actual, row.distance + (row.gradient * dx)(0), 1e-7) << "stage " << row.stage << " x" << i;
    }
    for (int i = 0; i < kNu; ++i) {
      Control du = Control::Zero();
      du(i) = h;
      const double actual = model.pair_distance(discrete_step(x, u + du, dt / 2).x_next, row, world, 0.0);
      EXPECT_NEAR(actual, row.distance + (row.control_gradient * du)(0), 1e-7) << "stage " << row.stage << " u" << i;
    }
  }
}

TEST(Midstage, RowsStayInStageOrderWithAUniformBudget)
{
  const double dt = config().dt;
  const CollisionConfig cfg = midstage_config(true);
  const CollisionModel model(primitives(), cfg);
  const World world = world_of({mmn::Box{"crate", Vector3d(0.9, 0.55, 1.0), Vector3d(0.4, 0.3, 2.0), 0.0}});
  const Plan plan = moving_plan(config().horizon, dt);
  const auto merged = merge_stage_rows(model.rows(plan.x, dt, 0.0, world, nullptr),
                                       midstage_rows(model, plan, dt, 0.0, world, 1.0));
  EXPECT_TRUE(std::is_sorted(merged.begin(), merged.end(),
                             [](const DistanceRow & a, const DistanceRow & b) { return a.stage < b.stage; }));
  const int last = static_cast<int>(std::ceil(1.0 / dt - 1e-9));
  std::map<int, int> per_stage;
  for (const auto & row : merged) {
    ++per_stage[row.stage];
    if (row.midstage) {
      EXPECT_GE(row.stage, 1);
      EXPECT_LT(row.stage, last);
    }
  }
  for (const auto & [stage, n] : per_stage) EXPECT_LE(n, row_budget(cfg)) << "stage " << stage;
  // Within a stage its own rows come before its mid-stage rows (the slack gate's slack-to-row mapping).
  for (size_t i = 1; i < merged.size(); ++i)
    if (merged[i].stage == merged[i - 1].stage) EXPECT_FALSE(merged[i - 1].midstage && !merged[i].midstage) << i;
  EXPECT_EQ(row_budget(cfg), 2 * row_budget(midstage_config(false)));
}

// The near-term (intervals 1..3) between-stage clearance of the plans and the closest measured approach, driving
// past a panel-height edge (a shelf or door frame the base passes beside) at `y` beside the route.
std::pair<double, double> drive_past_edge(bool midstage, double y)
{
  auto c = midstage_controller(midstage);
  World world = world_of({mmn::Box{"edge", Vector3d(3.0, y, 1.5), Vector3d(0.2, 0.2, 0.6), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, targets);
  double between = std::numeric_limits<double>::infinity(), measured = between;
  for (int i = 0; i * 0.1 < 40.0 && plant.x(0) < 5.0; ++i) {
    const double now = i * 0.1;
    measured = std::min(measured, obstacle_clearance(plant.x, world, now));
    const CycleOutput & out = c->step(now, plant.x, true, world, plant.at_rest());
    if (out.status == CycleStatus::Ok)
      between = std::min(between, between_stages(c->collision(), out.plan, world, now, config().dt, 4));
    plant.step(out.command, 0.1);
  }
  EXPECT_GT(plant.x(0), 5.0) << "did not pass the edge";
  return {between, measured};
}

TEST(Midstage, KeepsTheIntervalAtTheMarginPastAnEdge)
{
  // The frozen plans hold the margin at their stage points and dip between them while the panel sweeps past the
  // edge (Phase 4B-C: the store S / R4 door-frame case, small linearization error); mid-stage rows keep the interval.
  const auto [off, off_measured] = drive_past_edge(false, 0.30);
  const auto [on, on_measured] = drive_past_edge(true, 0.30);
  std::printf("[edge 0.30 m] near between-stage: frozen %.4f, mid-stage %.4f; measured %.4f / %.4f\n", off, on,
              off_measured, on_measured);
  EXPECT_LT(off, 0.047) << "the frozen plans did not dip between stages: the loop does not discriminate";
  EXPECT_GE(on, 0.0485);
  EXPECT_GE(on, off + 0.003);
  EXPECT_GE(on_measured, off_measured - 1e-3);
}

TEST(Midstage, LowBeamStillPasses)
{
  // Non-regression in a linearization-error-dominated loop (the 1.34 m beam: stage rows mispredict by 1-24 cm while
  // the panel lowers): mid-stage rows cannot fix that error, and the closest approach was 0.0435 m against the
  // frozen 0.053 m (Phase 4B-C finding, a known limitation); the robot still passes under the beam clear of it.
  auto c = midstage_controller(true);
  World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
  const GateLoop run = run_gate_loop(*c, plant, world, 0.1, 60.0, [](const State & x) { return x(0) > 6.0; });
  run.print("low beam, mid-stage rows");
  EXPECT_GT(plant.x(0), 6.0) << "did not pass under the beam";
  EXPECT_GT(run.closest, 0.03);
}

// Exploration (MIDSTAGE_POST_SCAN): near-term between-stage clearance driving past a post, frozen against mid-stage.
TEST(Midstage, DISABLED_PostScan)
{
  const char * zs = std::getenv("MIDSTAGE_POST_Z");
  const double z = zs ? std::atof(zs) : 1.0, height = zs ? 0.6 : 2.0;
  for (double y : {0.60, 0.50, 0.40, 0.30, 0.20}) {
    for (bool on : {false, true}) {
      auto c = midstage_controller(on);
      World world = world_of({mmn::Box{"post", Vector3d(3.0, y, z), Vector3d(0.2, 0.2, height), 0.0}});
      const PostureTargets targets = both_sides();
      Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
      c->start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, targets);
      double between = 1e9, measured = 1e9, lin = 0.0;
      for (int i = 0; i * 0.1 < 40.0 && plant.x(0) < 5.0; ++i) {
        const double now = i * 0.1;
        measured = std::min(measured, obstacle_clearance(plant.x, world, now));
        const CycleOutput & out = c->step(now, plant.x, true, world, plant.at_rest());
        if (out.status == CycleStatus::Ok) {
          between = std::min(between, between_stages(c->collision(), out.plan, world, now, config().dt, 4));
          if (std::isfinite(out.lin_error_m)) lin = std::max(lin, out.lin_error_m);
        }
        plant.step(out.command, 0.1);
      }
      std::printf("post y %.2f midstage %d: near between %.4f measured %.4f lin max %.4f final x %.2f\n", y, on,
                  between, measured, lin, plant.x(0));
    }
  }
}

TEST(Midstage, DiagnosticsResetWithoutASolve)
{
  // Past the edge the mid-stage rows are measured; a cycle without a solve (stale state: braking) reports NaN.
  auto c = midstage_controller(true);
  World world = world_of({mmn::Box{"edge", Vector3d(3.0, 0.30, 1.5), Vector3d(0.2, 0.2, 0.6), 0.0}});
  const PostureTargets targets = both_sides();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, targets);
  bool measured = false;
  int i = 0;
  for (; i * 0.1 < 40.0 && !measured; ++i) {
    const CycleOutput & out = c->step(i * 0.1, plant.x, true, world, plant.at_rest());
    measured = std::isfinite(out.midstage_lin_error_m);
    plant.step(out.command, 0.1);
  }
  ASSERT_TRUE(measured) << "no mid-stage row ever measured";
  const CycleOutput & stale = c->step(i * 0.1, plant.x, false, world, plant.at_rest());
  EXPECT_TRUE(std::isnan(stale.midstage_lin_error_m));
  EXPECT_TRUE(std::isnan(stale.lin_error_m));
}

// ---- Phase 4B-C: collision over references ----
TEST(SlackGate, ArmRowsEnterOnlyByTheArmTolerance)
{
  SlackGate g = gate();
  // Frozen: every pair outside its margin may plan the shared 5 mm into it.
  EXPECT_DOUBLE_EQ(gate_allowance(0.0, false, g), 0.005);
  EXPECT_DOUBLE_EQ(gate_allowance(0.0, true, g), 0.005);
  g.arm_tolerance_m = 0.0001;
  EXPECT_DOUBLE_EQ(gate_allowance(0.0, false, g), 0.0001);  // arm and panel rows: solver noise only
  EXPECT_DOUBLE_EQ(gate_allowance(0.0, true, g), 0.005);    // base rows keep the shared tolerance
  // Inside the margin: its current violation (plus solver noise), whatever the row.
  EXPECT_DOUBLE_EQ(gate_allowance(0.01, false, g), 0.01 + kGateInsideAllowanceM);
}

// ---- E1: the gate's inside-margin rule (spec 2026-11-08 Section 3.1) ----
TEST(SlackGate, AbsoluteRuleBoundsTheDepthNotTheStep)
{
  SlackGate g = gate();
  g.inside_rule = GateInsideRule::Absolute;
  EXPECT_DOUBLE_EQ(gate_allowance(0.0, true, g), 0.005);                          // outside: T, as before
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, true, g), 0.005);                        // inside by < T: up to T deep
  EXPECT_DOUBLE_EQ(gate_allowance(0.02, true, g), 0.02 + kGateInsideAllowanceM);  // deeper than T: stay or get out
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, false, g), 0.005);  // arm rows share T without the arm switch
  g.arm_tolerance_m = 0.0001;
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, false, g), 0.001 + kGateInsideAllowanceM);  // arm T 0.1 mm
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, true, g), 0.005);                           // base rows keep T
  g.inside_rule = GateInsideRule::Increment;
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, true, g), 0.001 + kGateInsideAllowanceM);  // the frozen rule
}

TEST(SlackGate, InsideRuleFromTheConfiguration)
{
  EXPECT_EQ(gate().inside_rule, GateInsideRule::Absolute);  // the shipped yaml (E1, user decision 2026-11-09)
  EXPECT_EQ(parse_gate_inside_rule("absolute"), GateInsideRule::Absolute);
  EXPECT_EQ(parse_gate_inside_rule("increment"), GateInsideRule::Increment);
  EXPECT_THROW(parse_gate_inside_rule("deep"), std::invalid_argument);
  EXPECT_STREQ(to_string(GateInsideRule::Absolute).c_str(), "absolute");
  EXPECT_STREQ(to_string(GateInsideRule::Increment).c_str(), "increment");
  // Copies of the shipped yaml: without the key (increment), absolute, and a bad value (an error naming the key).
  const std::string shipped = share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml";
  std::ifstream in(shipped);
  std::stringstream text;
  text << in.rdbuf();
  std::string without;
  std::istringstream lines(text.str());
  for (std::string line; std::getline(lines, line);)
    if (line.rfind("slack_gate_inside:", 0) != 0) without += line + "\n";
  const auto dir = std::filesystem::temp_directory_path() / ("gate_rule_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  auto write = [&](const std::string & name, const std::string & extra) {
    const std::string path = (dir / name).string();
    std::ofstream(path) << without << extra;
    return path;
  };
  EXPECT_EQ(load_mpc_config(write("without.yaml", "")).slack_gate.inside_rule, GateInsideRule::Increment);
  EXPECT_EQ(load_mpc_config(write("absolute.yaml", "slack_gate_inside: absolute\n")).slack_gate.inside_rule,
            GateInsideRule::Absolute);
  try {
    load_mpc_config(write("bad.yaml", "slack_gate_inside: deep\n"));
    ADD_FAILURE() << "a bad slack_gate_inside loaded";
  } catch (const std::exception & e) {
    EXPECT_NE(std::string(e.what()).find("slack_gate_inside"), std::string::npos) << e.what();
  }
  std::filesystem::remove_all(dir);
}

TEST(SlackGate, ReportsTheDeepestPairAndItsStageOnePrediction)
{
  // E1 Task 9 (the depth ratchet): each gated cycle reports the deepest pair among the rows with slack (its current
  // violation) and that pair's stage-1 slack (the plan's linearized violation one stage ahead). The lock funnel: the
  // robot starts about 1 mm inside the 0.05 m margin.
  const double inside = 0.0534, slope = 0.01, length = 8.0, thick = 0.2, height = 0.5;
  const State x0 = state(0, 0, 0, posture().comfort);
  auto extent = [&](double side) {
    const World probe = world_of({mmn::Box{"probe", Vector3d(0.0, side * (1.0 + thick / 2.0), height / 2.0),
                                           Vector3d(length, thick, height), 0.0}});
    return 1.0 - obstacle_clearance(x0, probe, 0.0);
  };
  const double right = extent(-1.0), left = extent(1.0);
  const double yaw = std::atan(slope), cx = length / 2.0 - 1.0, half = thick / 2.0 / std::cos(yaw);
  World world = world_of(
    {mmn::Box{"right", Vector3d(cx, -(right + inside) + slope * cx - half, height / 2.0), Vector3d(length, thick, height), yaw},
     mmn::Box{"left", Vector3d(cx, (left + inside) - slope * cx + half, height / 2.0), Vector3d(length, thick, height), -yaw}});
  const double start = obstacle_clearance(x0, world, 0.0);
  ASSERT_LT(start, 0.05);
  Weights weights = config().weights;
  weights.slack /= 10.0;
  weights.slack_quadratic /= 10.0;
  weights.slack_inside = -1.0;
  Controller c(Formulation(config().horizon, config().dt, limits(), weights), CollisionModel(primitives(), CollisionConfig{}),
               std::make_unique<OsqpSolver>(solver_options()), breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad,
               gate());
  c.start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, posture());
  const CycleOutput & out = c.step(0.0, x0, true, world, true);
  EXPECT_NEAR(out.gate_depth_m, 0.05 - start, 1e-6) << "the deepest pair's current violation";
  std::printf("deepest pair: depth %.5f m, stage-1 slack %.5f m\n", out.gate_depth_m, out.gate_depth_stage1_m);
  // the pair is inside its margin at stage 0 and the funnel narrows: its stage-1 row carries slack
  EXPECT_GT(out.gate_depth_stage1_m, 0.0) << "the deepest pair's stage-1 slack was not found";
  EXPECT_LT(out.gate_depth_stage1_m, 0.05);
  // a cycle without any slacked row reports zero
  Controller free(Formulation(config().horizon, config().dt, limits(), config().weights),
                  CollisionModel(primitives(), CollisionConfig{}), std::make_unique<OsqpSolver>(solver_options()),
                  breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate());
  free.start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, posture());
  const CycleOutput & open = free.step(0.0, x0, true, World{}, true);
  EXPECT_EQ(open.gate_depth_m, 0.0);
  EXPECT_EQ(open.gate_depth_stage1_m, 0.0);
}

TEST(SlackGateDiagnostics, NamesTheDeepestPairAndItsGradient)
{
  // E2 Task 1: with diagnostics the gate also names the deepest pair and its distance gradient at the measured state
  // (for the rollout's projection of the measured motion onto the pair). The funnel of the test above.
  const double inside = 0.0534, slope = 0.01, length = 8.0, thick = 0.2, height = 0.5;
  const State x0 = state(0, 0, 0, posture().comfort);
  auto extent = [&](double side) {
    const World probe = world_of({mmn::Box{"probe", Vector3d(0.0, side * (1.0 + thick / 2.0), height / 2.0),
                                           Vector3d(length, thick, height), 0.0}});
    return 1.0 - obstacle_clearance(x0, probe, 0.0);
  };
  const double right = extent(-1.0), left = extent(1.0);
  const double yaw = std::atan(slope), cx = length / 2.0 - 1.0, half = thick / 2.0 / std::cos(yaw);
  const mmn::Box right_box{"right", Vector3d(cx, -(right + inside) + slope * cx - half, height / 2.0),
                           Vector3d(length, thick, height), yaw};
  const mmn::Box left_box{"left", Vector3d(cx, (left + inside) - slope * cx + half, height / 2.0),
                          Vector3d(length, thick, height), -yaw};
  World world = world_of({right_box, left_box});
  Weights weights = config().weights;
  weights.slack /= 10.0;
  weights.slack_quadratic /= 10.0;
  weights.slack_inside = -1.0;
  auto run = [&](bool diagnostics, Controller * & keep) {
    ControllerOptions options;
    options.diagnostics = diagnostics;
    keep = new Controller(Formulation(config().horizon, config().dt, limits(), weights),
                          CollisionModel(primitives(), CollisionConfig{}), std::make_unique<OsqpSolver>(solver_options()),
                          breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, gate(), options);
    keep->start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, posture());
    return keep->step(0.0, x0, true, world, true);
  };
  Controller * with = nullptr, * without = nullptr;
  const CycleOutput out = run(true, with);
  std::unique_ptr<Controller> own_with(with);
  ASSERT_GT(out.gate_depth_m, 0.0);
  ASSERT_FALSE(out.gate_pair.empty());
  const bool names_a_wall = out.gate_pair.find("|right") != std::string::npos ||
                            out.gate_pair.find("|left") != std::string::npos;
  EXPECT_TRUE(names_a_wall) << out.gate_pair;
  // the gradient is that wall's clearance gradient, by central differences against the named wall alone (the funnel is
  // symmetric, so the clearance to both walls has a kink at x0)
  const bool right_wall = out.gate_pair.find("|right") != std::string::npos;
  const World named = world_of({right_wall ? right_box : left_box});
  const double h = 1e-5;
  for (int i : {0, 1, 2}) {
    State plus = x0, minus = x0;
    plus(i) += h;
    minus(i) -= h;
    const double fd = (obstacle_clearance(plus, named, 0.0) - obstacle_clearance(minus, named, 0.0)) / (2.0 * h);
    EXPECT_NEAR(out.gate_gradient(i), fd, 1e-4) << "state index " << i;
  }
  EXPECT_GT(out.gate_gradient.head<3>().norm(), 0.5) << "the gradient of a wall pair is not zero";
  // review F3: on this smooth pair the one-sided slopes agree (the kink, forward minus backward, is ~0)
  EXPECT_LT(out.gate_gradient_kink.cwiseAbs().maxCoeff(), 1e-3) << out.gate_gradient_kink.transpose();
  // review: the primitive and link indices are the model's (the primitive's link is the named link)
  const size_t bar1 = out.gate_pair.find('|'), bar2 = out.gate_pair.find('|', bar1 + 1);
  const int primitive = std::stoi(out.gate_pair.substr(0, bar1));
  const int link = std::stoi(out.gate_pair.substr(bar1 + 1, bar2 - bar1 - 1));
  ASSERT_GE(primitive, 0);
  ASSERT_LT(primitive, static_cast<int>(primitives().size()));
  // the payload is the last primitive and its rows carry link_index("panel") (Collision.PayloadIsTheLastPrimitive)
  const bool payload = primitive + 1 == static_cast<int>(primitives().size());
  EXPECT_EQ(link, link_index(payload ? std::string("panel") : primitives()[static_cast<size_t>(primitive)].link))
    << out.gate_pair;
  // review: an active cycle's pair does not leak into the next cycle without a pair
  const CycleOutput clear = with->step(0.02, x0, true, World{}, true);
  EXPECT_TRUE(clear.gate_pair.empty());
  EXPECT_EQ(clear.gate_gradient.norm(), 0.0);
  EXPECT_EQ(clear.gate_gradient_kink.norm(), 0.0);
  // E2 Task 4b, the store_room probe: the true nearest robot-obstacle pair at the measured state, whether the cycle's
  // rows carry it (stage 1, any stage) and the rows the per-stage budget dropped
  EXPECT_TRUE(out.probe_obstacle == "right" || out.probe_obstacle == "left") << out.probe_obstacle;
  EXPECT_FALSE(out.probe_link.empty());
  EXPECT_NEAR(out.probe_distance, obstacle_clearance(x0, world, 0.0), 1e-9);
  EXPECT_TRUE(out.probe_in_stage1);
  EXPECT_TRUE(out.probe_in_rows);
  EXPECT_EQ(out.probe_dropped, out.collision.dropped_pairs);
  // without diagnostics: neither
  const CycleOutput plain = run(false, without);
  EXPECT_TRUE(plain.probe_link.empty() && plain.probe_obstacle.empty());
  std::unique_ptr<Controller> own_without(without);
  EXPECT_GT(plain.gate_depth_m, 0.0);
  EXPECT_TRUE(plain.gate_pair.empty());
  EXPECT_EQ(plain.gate_gradient.norm(), 0.0);
  EXPECT_EQ(plain.gate_gradient_kink.norm(), 0.0);
}

TEST(BreakawayOutput, FollowPlanEndsTheLatchMeasuredOnThePlant)
{
  // E2 Task 4b: the output path cycle by cycle with the latch's measured inputs (e2.md, door_105 s20): the plan asks
  // -0.026 rad/s every cycle, the last command was a +0.07 kick, the rate band is 0.8 rad/s^2 x 0.02 s, and the base
  // achieves half of a turn below 0.1 rad/s and reads "at rest" below 0.02 rad/s. Frozen: the command re-kicks in the
  // old sign for good. Follow-plan: it decays at the rate limit, never grows against the plan, and turns the plan's way
  // within the decay time.
  const Breakaway b = breakaway();
  const double band = 0.8 * 0.02, planned = -0.026;
  auto run = [&](bool follow, int & grew_against, int & first_with_plan) {
    double last = 0.07, executed = 0.035;
    grew_against = 0;
    first_with_plan = -1;
    for (int k = 0; k < 50; ++k) {
      const double clamped = std::clamp(planned, last - band, last + band);
      const bool at_rest = base_at_rest(0.0, executed, b);
      const double sent = breakaway_output(base_command(0.0, clamped), base_command(0.0, planned), at_rest, b, follow)(1);
      if (sent * planned < 0.0 && std::abs(sent) > std::abs(last) + 1e-12) ++grew_against;
      if (first_with_plan < 0 && sent * planned > 0.0) first_with_plan = k;
      executed = std::abs(sent) < 0.1 ? 0.5 * sent : sent;
      last = sent;
    }
  };
  int grew_frozen = 0, first_frozen = 0, grew_follow = 0, first_follow = 0;
  run(false, grew_frozen, first_frozen);
  run(true, grew_follow, first_follow);
  EXPECT_GT(grew_frozen, 0) << "the frozen stage should re-kick against the plan (the measured latch)";
  EXPECT_EQ(first_frozen, -1) << "the frozen stage never turns the plan's way";
  EXPECT_EQ(grew_follow, 0);
  EXPECT_GE(first_follow, 0);
  EXPECT_LE(first_follow, 7) << "0.07 decays to the dead zone in 4-5 cycles, then the boost goes the plan's way";
}

TEST(SlackGate, AbsoluteRuleReleasesTheLockAtAMargin)
{
  // InsideTheMarginNeverGetsDeeper's funnel with the robot 1 mm inside the 0.05 m margin: every plan that makes
  // progress asks a little deeper. Under the frozen rule (the violation + 0.1 mm) the loop holds where it starts;
  // under absolute (up to the 5 mm tolerance deep) it progresses, then holds at the depth bound, never deeper than the
  // tolerance (plus the solver's 1 mm).
  // the walls' inner faces `inside` beside the robot's widest point; its front corners are 4.4 mm closer (the funnel
  // closes in ahead), so it starts about 1 mm inside the margin
  const double inside = 0.0534, slope = 0.01, length = 8.0, thick = 0.2, height = 0.5;
  const State x0 = state(0, 0, 0, posture().comfort);
  auto extent = [&](double side) {
    const World probe = world_of({mmn::Box{"probe", Vector3d(0.0, side * (1.0 + thick / 2.0), height / 2.0),
                                           Vector3d(length, thick, height), 0.0}});
    return 1.0 - obstacle_clearance(x0, probe, 0.0);
  };
  const double right = extent(-1.0), left = extent(1.0);
  const double yaw = std::atan(slope), cx = length / 2.0 - 1.0, half = thick / 2.0 / std::cos(yaw);
  World world = world_of(
    {mmn::Box{"right", Vector3d(cx, -(right + inside) + slope * cx - half, height / 2.0), Vector3d(length, thick, height), yaw},
     mmn::Box{"left", Vector3d(cx, (left + inside) - slope * cx + half, height / 2.0), Vector3d(length, thick, height), -yaw}});
  const double start = obstacle_clearance(x0, world, 0.0);
  ASSERT_GT(start, 0.05 - 0.002) << "the funnel is misplaced: not just inside the margin";
  ASSERT_LT(start, 0.05) << "the funnel is misplaced: outside the margin";
  Weights weights = config().weights;
  weights.slack /= 10.0;
  weights.slack_quadratic /= 10.0;
  weights.slack_inside = -1.0;
  double x_increment = 0.0;
  for (const GateInsideRule rule : {GateInsideRule::Increment, GateInsideRule::Absolute}) {
    SlackGate g = gate();
    g.inside_rule = rule;
    Controller c(Formulation(config().horizon, config().dt, limits(), weights),
                 CollisionModel(primitives(), CollisionConfig{}), std::make_unique<OsqpSolver>(solver_options()),
                 breakaway(), kDefaultFallbackCycles, kGoalXyM, kGoalYawRad, g);
    Plant plant{x0};
    c.start(Route({Vector2d(0, 0), Vector2d(6, 0)}), {6.0, 0.0, 0.0}, {}, posture());
    const GateLoop run = run_gate_loop(c, plant, world, 0.02, 15.0);
    const char * name = rule == GateInsideRule::Absolute ? "absolute" : "increment";
    run.print(name);
    std::printf("lock funnel (%s): start %.4f m, closest %.4f m, final x %.3f, holding %d\n", name, start, run.closest,
                plant.x(0), run.holding);
    if (rule == GateInsideRule::Increment) {
      EXPECT_GT(run.holding, 0) << "the frozen rule did not hold";
      EXPECT_LT(plant.x(0), 0.5) << "the frozen rule did not lock";
      x_increment = plant.x(0);
      continue;
    }
    // The funnel narrows by 1 cm per metre per side: the robot may go on until a plan within the gate's horizon would
    // take it past T deep (about 0.4 m less the look-ahead), then holds there.
    EXPECT_GT(plant.x(0), x_increment + 0.1) << "absolute did not release the lock";
    EXPECT_LT(run.closest, start - 1e-3) << "absolute never went deeper than the start";
    EXPECT_GE(run.closest, 0.05 - 0.005 - 1e-3) << "absolute let the robot deeper than the tolerance";
    ASSERT_FALSE(run.statuses.empty());
    EXPECT_EQ(run.statuses.back(), CycleStatus::Holding) << "absolute did not end holding at the depth bound";
  }
}

// ---- reference_source: medium (Phase 5A Task 10) ----

namespace
{
// A medium plan from (x0, 0) along a path bending 0.5 m left over 2 m, at `speed`, the pan at +0.3 rad off comfort.
std::vector<MediumSample> bending_plan(double x0, double speed)
{
  std::vector<MediumSample> out;
  double s = 0.0;
  Vector2d last(x0, 0.0);
  for (int i = 0; i <= 40; ++i) {
    const double u = 0.05 * i;  // 0 .. 2 m ahead
    const Vector2d p(x0 + u, 0.25 * (1.0 - std::cos(M_PI * std::min(u, 2.0) / 2.0)));
    s += (p - last).norm();
    last = p;
    const double slope = 0.25 * M_PI / 2.0 * std::sin(M_PI * u / 2.0);
    Eigen::Matrix<double, 6, 1> q = posture().comfort;
    q(0) += 0.3;
    out.push_back({s, p.x(), p.y(), std::atan(slope), speed, q});
  }
  return out;
}

MpcConfig medium_config()
{
  MpcConfig c = config();
  c.reference_source = "medium";
  return c;
}
}  // namespace

TEST(MediumMode, FollowsMediumYawSpeedAndPosture)
{
  // The node re-sends the plan every cycle (fresh): B5 slows to the plan's 0.12 m/s, turns along its bend (0.12 rad
  // where the base is by 2 s) and moves the pan towards the plan's +0.3 rad within 2 s; the same controller on the
  // Nav2 route alone does none of it.
  auto medium = make_controller();
  auto nav2 = make_controller();
  Drive dm(*medium), dn(*nav2);
  MediumSwitch sw(medium_config());
  double top_speed_m = 0.0, top_speed_n = 0.0;
  for (int i = 0; i * dm.dt < 2.0; ++i) {
    const double now = i * dm.dt;
    // the same plan in space, re-sent fresh (as the node's plans are, each from its own start)
    sw.set_plan(MediumReference(bending_plan(0.0, 0.12), dm.route, 2.0, posture().comfort), static_cast<uint64_t>(i + 1),
                1, now);
    ASSERT_EQ(sw.update(*medium, now, dm.plant.x.head<2>(), 1, dm.route, 10.0), MediumFallback::None) << i;
    dm.step(*medium);
    dn.step(*nav2);
    if (now > 1.0) {
      top_speed_m = std::max(top_speed_m, dm.plant.executed(0));
      top_speed_n = std::max(top_speed_n, dn.plant.executed(0));
    }
  }
  EXPECT_LT(top_speed_m, 0.16);
  EXPECT_GT(top_speed_m, 0.06);
  EXPECT_GT(top_speed_n, 0.2);  // B5 alone accelerates on to its own reference
  EXPECT_GT(dm.plant.x(2), 0.05);  // turned along the bend
  EXPECT_LT(std::abs(dn.plant.x(2)), 0.02);
  EXPECT_GT(dm.plant.x(3) - posture().comfort(0), 0.1);  // the pan moving towards +0.3
  EXPECT_LT(std::abs(dn.plant.x(3) - posture().comfort(0)), 0.02);
}

TEST(MediumMode, PickerAndTightnessOff)
{
  // A post beside the route within the comfort pose's tightness distance: in nav2 mode the tightness rises; following
  // the medium plan the posture comes from the plan, tightness 0 and no side choice.
  const World world = world_of({mobile_manipulator_navigation::Box{"post", Eigen::Vector3d(1.2, 0.75, 1.0),
                                                                    Eigen::Vector3d(0.3, 0.3, 2.0), 0.0}});
  auto medium = make_controller();
  auto nav2 = make_controller();
  Drive dm(*medium), dn(*nav2);
  dm.world = world;
  dn.world = world;
  MediumSwitch sw(medium_config());
  double tight_n = 0.0;
  for (int i = 0; i < 15; ++i) {
    const double now = i * dm.dt;
    sw.set_plan(MediumReference(bending_plan(dm.plant.x(0), 0.2), dm.route, std::min(dm.plant.x(0) + 2.0, 10.0),
                                posture().comfort),
                static_cast<uint64_t>(i + 1), 1, now);
    sw.update(*medium, now, dm.plant.x.head<2>(), 1, dm.route, 10.0);
    const CycleOutput & om = dm.step(*medium);
    EXPECT_DOUBLE_EQ(om.tightness, 0.0) << i;
    EXPECT_EQ(om.posture_choice, PostureChoice::None) << i;
    tight_n = std::max(tight_n, dn.step(*nav2).tightness);
  }
  EXPECT_GT(tight_n, 0.0);  // the post does matter to the frozen B5
}

TEST(MediumMode, PlanForOldRouteFallsBack)
{
  // B5 announced route 2; a plan for route 1 is not followed (route_id) until a plan for route 2 arrives.
  auto c = make_controller();
  Drive d(*c);
  MediumSwitch sw(medium_config());
  EXPECT_EQ(sw.update(*c, 0.0, d.plant.x.head<2>(), 2, d.route, 10.0), MediumFallback::NoPlan);
  sw.set_plan(MediumReference(bending_plan(0.0, 0.2), d.route, 2.0, posture().comfort), 1, 1, 0.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(sw.update(*c, i * d.dt, d.plant.x.head<2>(), 2, d.route, 10.0), MediumFallback::RouteId);
    EXPECT_FALSE(sw.following());
    d.step(*c);
  }
  sw.set_plan(MediumReference(bending_plan(d.plant.x(0), 0.2), d.route, 2.0, posture().comfort), 2, 2, 3 * d.dt);
  EXPECT_EQ(sw.update(*c, 3 * d.dt, d.plant.x.head<2>(), 2, d.route, 10.0), MediumFallback::None);
  EXPECT_TRUE(sw.following());
}

TEST(MediumMode, InfeasiblePlansFallBackAfterTheirTime)
{
  // E1 CP3: B5 followed medium plans that had been infeasible for 7-27 s. With medium_max_infeasible_s 1.0 a run of
  // infeasible plans is followed for 1 s from the first one's stamp, then falls back to nav2 (infeasible) until a
  // feasible plan arrives; off (-1, the default) it is followed throughout.
  auto c = make_controller();
  Drive d(*c);
  MpcConfig cfg = medium_config();
  EXPECT_DOUBLE_EQ(cfg.medium_max_infeasible_s, 2.0) << "the shipped yaml (E1, user decision 2026-11-09)";
  cfg.medium_max_infeasible_s = 1.0;
  MpcConfig frozen_mode = medium_config();
  frozen_mode.medium_max_infeasible_s = -1.0;  // the frozen 5A behaviour
  MediumSwitch on(cfg), off(frozen_mode);
  auto plan_at = [&](double t) { return MediumReference(bending_plan(d.plant.x(0), 0.1), d.route, 2.0, posture().comfort); };
  uint64_t id = 0;
  for (double t : {0.0, 0.4, 0.8, 1.2}) {  // infeasible plans every 0.4 s (fresh: below medium_stale_s)
    on.set_plan(plan_at(t), ++id, 1, t, false);
    off.set_plan(plan_at(t), id, 1, t, false);
    const MediumFallback reason = on.update(*c, t + 0.02, d.plant.x.head<2>(), 1, d.route, 10.0);
    EXPECT_EQ(reason, t + 0.02 > 1.0 ? MediumFallback::Infeasible : MediumFallback::None) << t;
    EXPECT_EQ(off.update(*c, t + 0.02, d.plant.x.head<2>(), 1, d.route, 10.0), MediumFallback::None) << t;
  }
  EXPECT_STREQ(to_string(MediumFallback::Infeasible), "infeasible");
  // Task 9c review: a stale_reused plan (feasibility unknown: the last plan again) does not end the run
  on.set_plan(plan_at(1.4), ++id, 1, 1.4, std::nullopt);
  EXPECT_EQ(on.update(*c, 1.42, d.plant.x.head<2>(), 1, d.route, 10.0), MediumFallback::Infeasible);
  on.set_plan(plan_at(1.6), ++id, 1, 1.6, true);  // a feasible plan ends the run
  EXPECT_EQ(on.update(*c, 1.62, d.plant.x.head<2>(), 1, d.route, 10.0), MediumFallback::None);
  EXPECT_TRUE(on.following());
  // a route change ends a run: the new route's first infeasible plan starts its own
  on.set_plan(plan_at(2.0), ++id, 1, 2.0, false);
  on.route_changed(*c);
  on.set_plan(plan_at(4.0), ++id, 1, 4.0, false);
  EXPECT_EQ(on.update(*c, 4.02, d.plant.x.head<2>(), 1, d.route, 10.0), MediumFallback::None);
}

TEST(MediumMode, StalePlanFallsBackToFrozen)
{
  // A plan older than medium_stale_s from the start: every cycle falls back, and the controller's outputs equal a
  // nav2-mode controller's (the fallback is the frozen B5, not an approximation of it).
  auto medium = make_controller();
  auto nav2 = make_controller();
  Drive dm(*medium), dn(*nav2);
  MediumSwitch sw(medium_config());
  sw.set_plan(MediumReference(bending_plan(0.0, 0.1), dm.route, 2.0, posture().comfort), 1, 1, -0.6);
  for (int i = 0; i < 30; ++i) {
    EXPECT_EQ(sw.update(*medium, i * dm.dt, dm.plant.x.head<2>(), 1, dm.route, 10.0), MediumFallback::Stale) << i;
    const CycleOutput & om = dm.step(*medium);
    const CycleOutput & on = dn.step(*nav2);
    ASSERT_EQ(om.status, on.status) << i;
    EXPECT_LT((om.command - on.command).cwiseAbs().maxCoeff(), 1e-9) << i;
    EXPECT_LT((dm.plant.x - dn.plant.x).cwiseAbs().maxCoeff(), 1e-9) << i;
  }
}

TEST(MediumMode, RouteChangeDropsTheFollowedPlan)
{
  // A replanned goal (update_goal: the Nav2 route again) while following: the switch drops the plan it followed (the
  // controller builds without it) and follows again only after the next plan's update.
  auto c = make_controller();
  Drive d(*c);
  MediumSwitch sw(medium_config());
  sw.set_plan(MediumReference(bending_plan(0.0, 0.2), d.route, 2.0, posture().comfort), 1, 1, 0.0);
  ASSERT_EQ(sw.update(*c, 0.0, d.plant.x.head<2>(), 1, d.route, 10.0), MediumFallback::None);
  ASSERT_TRUE(sw.following());
  EXPECT_NE(c->medium(), nullptr);
  d.step(*c);
  c->update_goal(d.route, {10.0, 0.0, 0.0}, {});
  sw.route_changed(*c);
  EXPECT_FALSE(sw.following());
  EXPECT_EQ(c->medium(), nullptr);
  d.step(*c);
  sw.set_plan(MediumReference(bending_plan(0.0, 0.2), d.route, 2.0, posture().comfort), 2, 2, 2 * d.dt);
  EXPECT_EQ(sw.update(*c, 2 * d.dt, d.plant.x.head<2>(), 2, d.route, 10.0), MediumFallback::None);
  EXPECT_TRUE(sw.following());
  EXPECT_NE(c->medium(), nullptr);
}

TEST(MediumMode, FallingBackAfterFollowingIsTheFrozenB5)
{
  // Following, then a plan going stale: the controller drops the medium reference (also with the preparation/feedback
  // split, whose prepared inputs captured it: Task 10 review) and is back on the Nav2 route, where the frozen B5's
  // tightness works again (a post beside the route raises it).
  for (const bool rti : {false, true}) {
    ControllerOptions o;
    o.rti_split = rti;
    auto c = optioned_controller(std::make_unique<OsqpSolver>(solver_options()), o);
    Drive d(*c);
    d.world = world_of({mobile_manipulator_navigation::Box{"post", Eigen::Vector3d(1.2, 0.75, 1.0),
                                                            Eigen::Vector3d(0.3, 0.3, 2.0), 0.0}});
    MediumSwitch sw(medium_config());
    for (int i = 0; i < 5; ++i) {  // following, fresh plans
      sw.set_plan(MediumReference(bending_plan(0.0, 0.2), d.route, 2.0, posture().comfort),
                  static_cast<uint64_t>(i + 1), 1, i * d.dt);
      ASSERT_EQ(sw.update(*c, i * d.dt, d.plant.x.head<2>(), 1, d.route, 10.0), MediumFallback::None);
      d.step(*c);
      c->prepare(d.cycle * d.dt, d.world);
    }
    double tightness = 0.0;
    for (int i = 5; i < 25; ++i) {  // the last plan goes stale after medium_stale_s
      const MediumFallback reason = sw.update(*c, i * d.dt, d.plant.x.head<2>(), 1, d.route, 10.0);
      if (reason == MediumFallback::None) {
        EXPECT_NE(c->medium(), nullptr);
      } else {
        EXPECT_EQ(reason, MediumFallback::Stale) << i;
        EXPECT_EQ(c->medium(), nullptr) << "rti " << rti << " cycle " << i;
      }
      tightness = std::max(tightness, d.step(*c).tightness);
      c->prepare(d.cycle * d.dt, d.world);
    }
    EXPECT_FALSE(sw.following());
    EXPECT_EQ(c->medium(), nullptr) << "rti " << rti;
    EXPECT_GT(tightness, 0.0) << "rti " << rti << ": the frozen B5's tightness after the fallback";
  }
}

// ---- E2 Task 7: the tracking model and the online correction in B5 ----

namespace
{
// A base that executes 60 % of every commanded yaw rate (a slippery-free understeering ground), else the model.
struct HalfTurnBase
{
  State x;
  Control executed = Control::Zero();
  bool at_rest() const { return base_at_rest(executed(0), executed(1), breakaway()); }
  void step(const Control & command, double dt)
  {
    executed = command;
    executed(1) *= 0.6;
    x = discrete_step(x, executed, dt).x_next;
  }
};

std::unique_ptr<Controller> tracking_controller(const TrackingModel & m, bool adaptation)
{
  ControllerOptions options;
  options.tracking_adaptation = adaptation;
  Formulation f = formulation();
  f.set_tracking(m);
  return std::make_unique<Controller>(std::move(f), CollisionModel(primitives(), CollisionConfig{}),
                                      std::make_unique<OsqpSolver>(solver_options()), breakaway(), kDefaultFallbackCycles,
                                      kGoalXyM, kGoalYawRad, gate(), options);
}

TrackingModel sideslip_model()
{
  TrackingModel m;
  m.turn_delta_inf = -0.02;
  m.turn_delta_zero = -0.97;
  m.turn_scale = 0.0225;
  m.speed_turn = 0.2;
  m.sideslip = 0.05;
  return m;
}
}  // namespace

TEST(TrackingModelInController, PlanStatesFollowTheModel)
{
  // a solved plan in a turn rolls out with the tracking model, not the unicycle
  const TrackingModel m = sideslip_model();
  auto c = tracking_controller(m, false);
  c->start(Route({Vector2d(0, 0), Vector2d(2, 0), Vector2d(2, 2)}), {2.0, 2.0, M_PI / 2}, {}, posture());
  HalfTurnBase base{state(0, 0, 0.5, posture().comfort)};  // 0.5 rad off the route: the plans turn at once
  bool differs = false;
  for (int k = 0; k < 150; ++k) {
    const CycleOutput & out = c->step(0.02 * k, base.x, true, World{}, base.at_rest());
    if (out.status == CycleStatus::Ok && out.plan.u.size() > 1) {
      for (size_t s = 0; s + 1 < out.plan.x.size(); ++s) {
        ASSERT_LT((out.plan.x[s + 1] - discrete_step(out.plan.x[s], out.plan.u[s], c->formulation().dt(), m).x_next)
                    .cwiseAbs().maxCoeff(), 1e-9) << "cycle " << k << " stage " << s;
        differs = differs || (out.plan.x[s + 1] - discrete_step(out.plan.x[s], out.plan.u[s], c->formulation().dt()).x_next)
                               .cwiseAbs().maxCoeff() > 1e-6;
      }
    }
    base.step(out.command, 0.02);
  }
  EXPECT_TRUE(differs) << "the plans never turned: the model was not exercised";
}

TEST(TrackingModelInController, AdaptationOffKeepsTheFixedModel)
{
  const TrackingModel m = sideslip_model();
  auto c = tracking_controller(m, false);
  c->start(Route({Vector2d(0, 0), Vector2d(2, 0), Vector2d(2, 2)}), {2.0, 2.0, M_PI / 2}, {}, posture());
  HalfTurnBase base{state(0, 0, 0, posture().comfort)};
  for (int k = 0; k < 300; ++k) {
    const CycleOutput & out = c->step(0.02 * k, base.x, true, World{}, base.at_rest());
    EXPECT_EQ(out.tracking(0), 1.0 + m.turn_delta_inf);
    EXPECT_EQ(out.tracking(2), m.sideslip);
    base.step(out.command, 0.02);
  }
  EXPECT_EQ(c->formulation().tracking().turn_delta_inf, m.turn_delta_inf);
}

TEST(TrackingModelInController, AdaptationLearnsTheBaseItDrives)
{
  // the controller observes each transition (the measured base pose before and after, the command it sent before) and
  // plans with the estimate: on a base turning 60 % of the command, g_inf moves from the prior towards it, and the
  // formulation's model is the estimator's
  const TrackingModel m = sideslip_model();
  auto c = tracking_controller(m, true);
  // a square route: turns at speed, held long enough to be learned (the estimator's gates)
  c->start(Route({Vector2d(0, 0), Vector2d(3, 0), Vector2d(3, 3), Vector2d(0, 3), Vector2d(0, 0.5)}), {0.0, 0.5, -M_PI / 2}, {},
           posture());
  HalfTurnBase base{state(0, 0, 0, posture().comfort)};
  double g_inf = 1.0 + m.turn_delta_inf;
  for (int k = 0; k < 2500; ++k) {
    const CycleOutput & out = c->step(0.02 * k, base.x, true, World{}, base.at_rest());
    g_inf = out.tracking(0);
    EXPECT_EQ(1.0 + c->formulation().tracking().turn_delta_inf, out.tracking(0));
    EXPECT_EQ(out.tracking(1), 1.0 + m.turn_delta_zero) << "g_0 is not estimated";
    if (out.status == CycleStatus::GoalReached) break;
    base.step(out.command, 0.02);
  }
  EXPECT_LT(g_inf, 0.8) << "the estimate did not move towards the base's 0.6";
  EXPECT_GT(g_inf, 0.5);
}

// ---- E2 Task 7 review: one model per cycle (C1), the measurement's own interval (I1), discontinuities (M1),
// the command/transition pairing (M3) ----

namespace
{
std::unique_ptr<Controller> rti_tracking_controller(const TrackingModel & m)
{
  ControllerOptions options;
  options.tracking_adaptation = true;
  options.rti_split = true;
  options.period_s = 0.02;
  Formulation f = formulation();
  f.set_tracking(m);
  return std::make_unique<Controller>(std::move(f), CollisionModel(primitives(), CollisionConfig{}),
                                      std::make_unique<OsqpSolver>(solver_options()), breakaway(), kDefaultFallbackCycles,
                                      kGoalXyM, kGoalYawRad, gate(), options);
}

Route square_route() { return Route({Vector2d(0, 0), Vector2d(3, 0), Vector2d(3, 3), Vector2d(0, 3), Vector2d(0, 0.5)}); }

// The largest gap between a plan's states and the model's rollout of its controls.
double rollout_gap(const Plan & plan, const TrackingModel & m, double dt)
{
  double gap = 0.0;
  for (size_t s = 0; s + 1 < plan.x.size(); ++s)
    gap = std::max(gap, (plan.x[s + 1] - discrete_step(plan.x[s], plan.u[s], dt, m).x_next).cwiseAbs().maxCoeff());
  return gap;
}

// A steady command on a base that turns 60 % of it (the estimate moves every window once steady): the cycles at
// which the controller's estimate changed, with an optional discontinuity at cycle `at` (a time rewind or a stale cycle)
std::vector<int> estimate_changes(bool rewind, bool stale, int at, int cycles)
{
  auto c = tracking_controller(sideslip_model(), true);
  c->start(Route({Vector2d(0, 0), Vector2d(80, 0)}), {80.0, 0.0, 0.0}, {}, posture());
  Control u = Control::Zero();
  u(0) = 0.3;
  u(1) = 0.2;
  HalfTurnBase base{state(0, 0, 0, posture().comfort)};
  std::vector<int> changes;
  double g = c->formulation().tracking().turn_delta_inf, t = 0.0;
  for (int k = 0; k < cycles; ++k) {
    if (k == at && rewind) t -= 1.0;
    c->reset_command(u);
    c->step(t, base.x, !(k == at && stale), World{}, false);
    const double now_g = c->formulation().tracking().turn_delta_inf;
    if (now_g != g) changes.push_back(k);
    g = now_g;
    base.step(u, 0.02);
    t += 0.02;
  }
  return changes;
}
}  // namespace

TEST(TrackingModelInController, AdaptationWithRtiPlansEachCycleWithOneModel)
{
  // Review C1: the QP prepared after a cycle is built with that cycle's estimate; the next cycle must not swap the
  // model under it. Each cycle's plan rolls out with the model the cycle reports (tracking_model: the QP's).
  const TrackingModel m = sideslip_model();
  auto c = rti_tracking_controller(m);
  c->start(square_route(), {0.0, 0.5, -M_PI / 2}, {}, posture());
  HalfTurnBase base{state(0, 0, 0, posture().comfort)};
  int prepared = 0, estimates = 0;
  double g = 1.0 + m.turn_delta_inf;
  for (int k = 0; k < 1500; ++k) {
    const double now = 0.02 * k;
    const CycleOutput & out = c->step(now, base.x, true, World{}, base.at_rest());
    ASSERT_EQ(out.tracking(0), 1.0 + out.tracking_model.turn_delta_inf) << "cycle " << k;
    if (out.status == CycleStatus::Ok && out.plan.u.size() > 1) {
      ASSERT_LT(rollout_gap(out.plan, out.tracking_model, c->formulation().dt()), 1e-9) << "cycle " << k;
      prepared += out.timing.prep_s > 0.0;
    }
    estimates += out.tracking(0) != g;
    g = out.tracking(0);
    if (out.status == CycleStatus::GoalReached) break;
    base.step(out.command, 0.02);
    c->prepare(now + 0.02, World{});
  }
  EXPECT_GT(prepared, 100) << "the preparations were not used: RTI not exercised";
  EXPECT_GT(estimates, 5) << "the estimate never changed: adaptation not exercised";
}

TEST(TrackingModelInController, ObservesEachBaseMeasurementOnceOverItsOwnInterval)
{
  // Review I1: the node's base pose comes from TF, which may refresh slower than the loop. With the measurement's
  // stamp, a repeated pose is not a fresh zero-motion transition and a jump after it is not a 20 ms one: the
  // estimate stays at the true (prior) map. Every pose stamped `now` (the campaign) learns the same.
  const TrackingModel m = sideslip_model();
  auto run = [&](int refresh) {
    auto c = tracking_controller(m, true);
    c->start(Route({Vector2d(0, 0), Vector2d(80, 0)}), {80.0, 0.0, 0.0}, {}, posture());
    Control u = Control::Zero();
    u(0) = 0.3;
    u(1) = 0.2;
    State truth = state(0, 0, 0, posture().comfort), seen = truth;
    double seen_at = 0.0;
    for (int k = 0; k < 500; ++k) {
      const double t = 0.02 * k;
      if (k % refresh == 0) {
        seen = truth;
        seen_at = t;
      }
      c->reset_command(u);
      c->step(t, seen, true, World{}, false, seen_at);
      truth = discrete_step(truth, u, 0.02, m).x_next;
    }
    return 1.0 + c->formulation().tracking().turn_delta_inf;
  };
  EXPECT_NEAR(run(1), 1.0 + m.turn_delta_inf, 0.01);
  EXPECT_NEAR(run(7), 1.0 + m.turn_delta_inf, 0.01) << "a 140 ms TF refresh was learned as a changed ground";
}

TEST(TrackingModelInController, ADiscontinuityRestartsTheWindowButKeepsTheEstimate)
{
  // Review M1: after a time rewind or a stale cycle the observations before it are not mixed with those after it:
  // the window and the steady-command wait restart (no update for the steady hold, 0.3 s = 15 cycles), the learned
  // estimate stays
  const auto plain = estimate_changes(false, false, -1, 200);
  ASSERT_GE(plain.size(), 10u) << "the steady run must update the estimate regularly for this test";
  const int at = plain[6] + 3;  // inside a window: observations pending
  for (const bool rewind : {true, false}) {
    const auto changes = estimate_changes(rewind, !rewind, at, 200);
    for (const int k : changes) {
      if (k < at) continue;
      EXPECT_GE(k, at + 15) << (rewind ? "rewind" : "stale") << ": updated at cycle " << k << " from data before it";
      break;
    }
  }
}

TEST(TrackingModelInController, ObservesTheTransitionUnderTheCommandThatDroveIt)
{
  // Review M3: the transition into cycle j was driven by the command sent at cycle j-1. A reference estimator fed
  // exactly that pairing must agree with the controller's at every cycle (a one-cycle-older command does not).
  const TrackingModel m = sideslip_model();
  auto c = tracking_controller(m, true);
  c->start(square_route(), {0.0, 0.5, -M_PI / 2}, {}, posture());
  TrackingEstimator reference(m);
  HalfTurnBase base{state(0, 0, 0, posture().comfort)};
  Eigen::Vector3d before = base.x.head<3>();
  Control sent = Control::Zero();
  int updates = 0;
  double g = reference.estimate()(0);
  std::vector<double> times(2500);  // stored: the controller's elapsed time is now minus the last now, bit for bit
  for (size_t k = 0; k < times.size(); ++k) times[k] = 0.02 * static_cast<double>(k);
  for (int k = 0; k < 2500; ++k) {
    if (k > 0) reference.observe(before, base.x.head<3>(), sent, times[k] - times[k - 1]);
    updates += reference.estimate()(0) != g;
    g = reference.estimate()(0);
    const CycleOutput & out = c->step(times[k], base.x, true, World{}, base.at_rest());
    ASSERT_EQ(out.tracking(0), reference.estimate()(0)) << "cycle " << k;
    ASSERT_EQ(out.tracking(2), reference.estimate()(2)) << "cycle " << k;
    if (out.status == CycleStatus::GoalReached) break;
    before = base.x.head<3>();
    sent = out.command;
    base.step(out.command, 0.02);
  }
  EXPECT_GT(updates, 5) << "the estimate never moved: the pairing was not exercised";
}

// ---- E2 Task 8: delay compensation ----

TEST(Delay, PlansFromTheAdvancedState)
{
  // the QP starts from the state at actuation: the measured state advanced through the commands sent within the
  // delays (each held until the next, the last until now); with zero delays, the measured state bit for bit
  for (const double delay : {0.04, 0.0}) {
    TrackingModel m = sideslip_model();
    m.base_delay_s = delay;
    m.arm_delay_s = delay;
    auto c = tracking_controller(m, false);
    c->start(Route({Vector2d(0, 0), Vector2d(10, 0)}), {10.0, 0.0, 0.0}, {}, posture());
    Control u = Control::Zero();
    u(0) = 0.3;
    u(1) = 0.1;
    u(2) = 0.2;
    std::vector<SentCommand> sent;
    State x = state(0, 0, 0, posture().comfort);
    std::vector<double> times(4);
    for (size_t k = 0; k < times.size(); ++k) times[k] = 0.02 * static_cast<double>(k);
    for (size_t k = 0; k < times.size(); ++k) {
      if (k > 0) {
        c->reset_command(u);
        sent.push_back({times[k - 1], u});
      }
      const CycleOutput & out = c->step(times[k], x, true, World{}, k == 0);
      const State expected = delay > 0.0 ? advance_to_actuation(x, sent, times[k], m) : x;
      EXPECT_EQ(out.x0_planned, expected) << "delay " << delay << " cycle " << k;
      if (delay > 0.0 && k >= 2) EXPECT_GT((out.x0_planned - x).cwiseAbs().maxCoeff(), 1e-4) << "nothing advanced";
      x = discrete_step(x, u, 0.02).x_next;
    }
  }
}

TEST(Delay, TheCorrectionPairsATransitionWithTheCommandActingThen)
{
  // Task 6b review M6: with a base delay d the transition from t to t + 0.02 was driven by the command in effect at
  // t - d: here 0.04 s, the command sent two cycles before the transition's start
  TrackingModel m = sideslip_model();
  m.base_delay_s = 0.04;
  auto c = tracking_controller(m, true);
  c->start(square_route(), {0.0, 0.5, -M_PI / 2}, {}, posture());
  TrackingEstimator reference(m);
  HalfTurnBase base{state(0, 0, 0, posture().comfort)};
  std::vector<Control> sent;  // sent[k]: the command of cycle k
  std::vector<Eigen::Vector3d> poses;
  std::vector<double> times(2500);
  for (size_t k = 0; k < times.size(); ++k) times[k] = 0.02 * static_cast<double>(k);
  int updates = 0;
  double g = reference.estimate()(0);
  for (int k = 0; k < 2500; ++k) {
    poses.push_back(base.x.head<3>());
    if (k > 0) {
      const Control acting = k >= 3 ? sent[static_cast<size_t>(k - 3)] : Control::Zero();
      reference.observe(poses[static_cast<size_t>(k - 1)], poses[static_cast<size_t>(k)], acting, times[k] - times[k - 1]);
    }
    updates += reference.estimate()(0) != g;
    g = reference.estimate()(0);
    const CycleOutput & out = c->step(times[k], base.x, true, World{}, base.at_rest());
    ASSERT_EQ(out.tracking(0), reference.estimate()(0)) << "cycle " << k;
    ASSERT_EQ(out.tracking(2), reference.estimate()(2)) << "cycle " << k;
    if (out.status == CycleStatus::GoalReached) break;
    sent.push_back(out.command);
    base.step(out.command, 0.02);
  }
  EXPECT_GT(updates, 5) << "the estimate never moved: the pairing was not exercised";
}
