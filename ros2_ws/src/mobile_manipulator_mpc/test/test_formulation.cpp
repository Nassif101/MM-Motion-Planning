// The B5 formulation in closed loop with the generated model as the plant (no ROS): every step
// shifts the previous plan as the nominal, recomputes the collision rows, builds and solves the
// QP and applies the first control with discrete_step.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/osqp_solver.hpp"
#include "mobile_manipulator_navigation/known_world.hpp"
#include "test_support.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;
using namespace mobile_manipulator_mpc;
using namespace mpc_test;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace
{
const std::string kGate = "Environment/NavigationObstacles/ManipulationChallenge/ManipulationRequiredGate_1p05m/";
constexpr double kGateX = -7.225;               // post centres (0.4 m deep: x in [-7.425, -7.025])
constexpr double kGateY = -7.725;               // opening centre (posts at y -7.0 and -8.45, 0.4 m wide)
constexpr double kPostHalfDepth = 0.2;

World map_world()
{
  return world_of(mmn::known_boxes(share("mobile_manipulator_navigation") + "/maps/construction_site.boxes.yaml", nullptr));
}

// base_footprint -> panel centre with the arm in `q`.
Eigen::Isometry3d panel_in_base(const Joints & q) { return link_pose(link_index("panel"), state(0, 0, 0, q)); }

double rotation_angle(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}

// Smallest distance of any robot primitive to an obstacle box (not the floor or the base), capped
// at the activation distance.
double obstacle_distance(const State & x, const World & world)
{
  static const CollisionModel exact = [] {
    CollisionConfig c;
    c.pairs_per_stage = 1000;
    return CollisionModel(primitives(), c);
  }();
  double best = CollisionConfig{}.activation_m;
  for (const auto & row : exact.rows({x}, 0.1, 0.0, world, nullptr)) {
    if (row.obstacle == "floor" || row.obstacle.rfind("self/", 0) == 0) continue;
    best = std::min(best, row.distance);
  }
  return best;
}

struct Scenario
{
  std::vector<Vector2d> route;
  double goal_yaw = 0.0;
  State x0;
  Control last_command = Control::Zero();
  World world;
  std::vector<PanelReference> references;  // given in arc length
  PostureTargets posture = mpc_test::posture();  // comfort pose, tight pose and mode
  Weights weights = config().weights;            // the formulation's weights (wholebody_mpc.yaml)
  int max_steps = 300;
  std::function<bool(const State &)> done;  // stops the loop when true
};

struct StepLog
{
  State x;          // state at the start of the step
  Control command;  // first control applied
  Control last;     // command of the previous step
  Plan plan;
  double alpha = 0.0;
  double distance = 0.0;  // exact obstacle distance of x
  int iterations = 0;
  double solve_s = 0.0;
  double build_s = 0.0;  // collision rows, tightness and QP assembly
  double max_slack = 0.0;
  bool ok = false;
};

struct LoopRun
{
  std::vector<StepLog> steps;
  State final_state;
  int failures = 0;
};

void print_stats(const char * name, const LoopRun & run)
{
  std::vector<int> iterations;
  std::vector<double> times, builds;
  for (const auto & s : run.steps) {
    iterations.push_back(s.iterations);
    times.push_back(s.solve_s * 1e3);
    builds.push_back(s.build_s * 1e3);
  }
  if (iterations.empty()) return;
  std::sort(iterations.begin(), iterations.end());
  std::sort(times.begin(), times.end());
  std::sort(builds.begin(), builds.end());
  const size_t n = iterations.size();
  std::printf("[%s] %zu steps, %d failed solves; iterations median %d p90 %d max %d; solve ms median %.2f p90 %.2f "
              "max %.2f; rows+build ms median %.2f max %.2f\n",
              name, n, run.failures, iterations[n / 2], iterations[n * 9 / 10], iterations.back(), times[n / 2],
              times[n * 9 / 10], times.back(), builds[n / 2], builds.back());
}

LoopRun closed_loop(const Scenario & sc)
{
  const Formulation f(config().horizon, config().dt, limits(), sc.weights);
  const CollisionModel collision(primitives(), CollisionConfig{});
  const Route route(sc.route);
  const double dt = f.dt();
  OsqpSolver solver(solver_options());
  TightnessFilter tightness;
  LoopRun run;
  State x = sc.x0;
  Control last = sc.last_command;
  Plan plan = f.hold(x);
  QpSolution warm;
  bool have_warm = false;
  for (int i = 0; i < sc.max_steps; ++i) {
    const double now = i * dt;
    if (i > 0) plan = f.shift(plan, x);
    const auto start = std::chrono::steady_clock::now();
    CollisionStats stats;
    const auto rows = collision.rows(plan.x, dt, now, sc.world, &stats);
    const double alpha =
      tightness.update(collision.clearance_with_joints(plan.x, sc.posture.comfort, dt, now, sc.world), dt);
    const Inputs in{x, last, &route, route.length(), sc.goal_yaw, sc.references, sc.posture, alpha};
    const QpProblem qp = f.build(in, plan, rows);
    const double build_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (i == 0) solver.setup(qp);  // the first real linearization fixes the pattern and the scaling
    const SolveResult result = solver.solve(qp, have_warm ? &warm : nullptr);
    StepLog log;
    log.x = x;
    log.last = last;
    log.alpha = alpha;
    log.distance = obstacle_distance(x, sc.world);
    log.iterations = result.iterations;
    log.solve_s = result.solve_s;
    log.build_s = build_s;
    log.ok = result.ok;
    if (result.ok) {
      plan = f.apply(plan, result.solution);
      log.max_slack = result.solution.slack.size() ? result.solution.slack.maxCoeff() : 0.0;
      warm = result.solution;
      warm.primal.setZero();  // the next nominal is this plan shifted: deviations start at zero
      have_warm = true;
    } else {
      ++run.failures;  // keep the shifted plan, as the controller's fallback
    }
    log.plan = plan;
    log.command = plan.u[0];
    run.steps.push_back(log);
    last = plan.u[0];
    x = discrete_step(x, last, dt).x_next;
    if (sc.done && sc.done(x)) break;
  }
  run.final_state = x;
  return run;
}

Scenario straight(double length, const Joints & q)
{
  Scenario sc;
  sc.route = {Vector2d(0, 0), Vector2d(length, 0)};
  sc.x0 = state(0, 0, 0, q);
  return sc;
}

// Effective bounds on control i of stage 0 from the QP: the control box intersected with every
// stage-0 inequality row that involves only that control.
std::pair<double, double> stage0_control_range(const QpProblem & qp, const Plan & nominal, int i)
{
  const StageQp & st = qp.stages[0];
  double lo = st.u_lower(i), hi = st.u_upper(i);
  for (int row = 0; row < st.lower.size(); ++row) {
    const double c = st.Cu(row, i);
    if (c == 0.0) continue;
    bool alone = true;
    for (int j = 0; j < st.Cu.cols(); ++j) alone = alone && (j == i || st.Cu(row, j) == 0.0);
    if (!alone) continue;
    const double offset = st.Cx.row(row).dot(qp.dx0);
    double a = (st.lower(row) - offset) / c, b = (st.upper(row) - offset) / c;
    if (c < 0) std::swap(a, b);
    lo = std::max(lo, a);
    hi = std::min(hi, b);
  }
  return {nominal.u[0](i) + lo, nominal.u[0](i) + hi};
}

// The narrow-gate drive (unscripted, home start), shared by the gate tests.
const LoopRun & gate_run()
{
  static const LoopRun run = [] {
    Scenario sc;
    sc.route = {Vector2d(-2.2, kGateY), Vector2d(-13.0, kGateY)};
    sc.goal_yaw = M_PI;
    sc.x0 = state(-2.2, kGateY, M_PI, posture().comfort);
    sc.world = map_world();
    sc.max_steps = 1200;
    sc.done = [](const State & x) { return x(0) < -12.4; };
    LoopRun r = closed_loop(sc);
    print_stats("gate", r);
    return r;
  }();
  return run;
}
}  // namespace

// ---- limits and configuration ----

TEST(Formulation, LoadsLimitsFromEnvelopeAndUrdf)
{
  const Limits & l = limits();
  EXPECT_DOUBLE_EQ(l.v_max, 0.3);
  EXPECT_DOUBLE_EQ(l.omega_max, 0.4);
  EXPECT_DOUBLE_EQ(l.accel, 0.45);
  EXPECT_DOUBLE_EQ(l.decel, 0.5);
  EXPECT_DOUBLE_EQ(l.yaw_accel, 0.8);
  EXPECT_DOUBLE_EQ(l.yaw_decel, 1.0);
  EXPECT_DOUBLE_EQ(l.breakaway.linear_mps, 0.0375);
  EXPECT_DOUBLE_EQ(l.breakaway.yaw_radps, 0.07);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(l.qd_max(i), 0.60);
    EXPECT_DOUBLE_EQ(l.qdd_max(i), 0.61);
  }
  EXPECT_NEAR(l.q_max(4), 1.77, 1e-12);   // wrist_2: the envelope's joint bound
  EXPECT_NEAR(l.q_min(4), -2.0943, 1e-12);  // the bound's -2.0943 is inside the URDF's -2.09439...
  EXPECT_NEAR(l.q_max(0), 2.9670, 1e-12);  // shoulder_pan: bound 2.9670 below the URDF 2.96705...
}

// ---- tightness ----

TEST(Tightness, ZeroWhenComfortable)
{
  TightnessFilter filter;
  double alpha = 0.0;
  for (int i = 0; i < 100; ++i) alpha = filter.update(0.5, 0.1);
  EXPECT_DOUBLE_EQ(alpha, 0.0);
}

TEST(Tightness, OneAtMargin)
{
  TightnessFilter filter;
  double alpha = 0.0;
  for (int i = 0; i < 200; ++i) alpha = filter.update(0.05, 0.1);
  EXPECT_NEAR(alpha, 1.0, 1e-6);
  TightnessFilter between;
  for (int i = 0; i < 200; ++i) alpha = between.update(0.175, 0.1);  // halfway between margin and comfort
  EXPECT_NEAR(alpha, 0.5, 1e-6);
  TightnessFilter inside;
  for (int i = 0; i < 200; ++i) alpha = inside.update(-0.1, 0.1);  // penetrating: still 1
  EXPECT_NEAR(alpha, 1.0, 1e-6);
}

TEST(Tightness, LowPassedWithOneSecond)
{
  TightnessFilter filter;
  double alpha = 0.0;
  for (int i = 0; i < 10; ++i) alpha = filter.update(0.0, 0.1);
  EXPECT_NEAR(alpha, 1.0 - std::exp(-1.0), 1e-3);
  EXPECT_NEAR(alpha, 0.63, 0.01);
}

// ---- formulation ----

TEST(Formulation, RateLimitRelativeToLastCommand)
{
  const Formulation f = formulation();
  const Route route({Vector2d(0, 0), Vector2d(-5, 0)});
  const State x0 = state(0, 0, 0, posture().comfort);  // at rest
  Control last = Control::Zero();
  last(1) = 0.04;
  const Inputs in{x0, last, &route, route.length(), M_PI, {}, posture(), 0.0};

  // The nominal's own stage-0 control (a stale plan turning at 0.3 rad/s) must not matter.
  for (double nominal_omega : {0.0, 0.3, -0.3}) {
    Plan nominal = f.hold(x0);
    for (auto & u : nominal.u) u(1) = nominal_omega;
    nominal = f.rollout(x0, nominal.u);
    const QpProblem qp = f.build(in, nominal, {});
    const auto [lo, hi] = stage0_control_range(qp, nominal, 1);
    EXPECT_NEAR(lo, 0.04 - 0.8 * f.dt(), 1e-9) << "nominal omega " << nominal_omega;
    EXPECT_NEAR(hi, 0.04 + 0.8 * f.dt(), 1e-9) << "nominal omega " << nominal_omega;
    const auto [v_lo, v_hi] = stage0_control_range(qp, nominal, 0);
    EXPECT_NEAR(v_lo, 0.0, 1e-9);
    EXPECT_NEAR(v_hi, 0.45 * f.dt(), 1e-9);

    // The goal is behind: the solution turns as fast as the bound allows.
    OsqpSolver solver(solver_options());
    solver.setup(qp);
    const SolveResult result = solver.solve(qp, nullptr);
    ASSERT_TRUE(result.ok) << result.status;
    const Plan plan = f.apply(nominal, result.solution);
    EXPECT_GT(std::abs(plan.u[0](1) - 0.04), 0.8 * f.dt() - 2e-3) << "omega " << plan.u[0](1);
    EXPECT_LE(std::abs(plan.u[0](1) - 0.04), 0.8 * f.dt() + 1e-3);
  }
}

TEST(Formulation, TracksStraightRouteAtProgressSpeed)
{
  Scenario sc = straight(10.0, posture().comfort);
  sc.done = [](const State & x) { return x(0) > 9.0; };
  sc.max_steps = 400;
  const LoopRun run = closed_loop(sc);
  print_stats("straight", run);
  double v_max = 0.0, lateral = 0.0;
  for (const auto & s : run.steps) {
    v_max = std::max(v_max, s.command(0));
    lateral = std::max(lateral, std::abs(s.x(1)));
  }
  EXPECT_GT(run.final_state(0), 9.0) << "did not arrive within the step budget";
  EXPECT_GT(v_max, 0.3 - 1e-3);
  EXPECT_LT(lateral, 0.02);
  EXPECT_EQ(run.failures, 0);
}

TEST(Formulation, TurnsTowardGoalBehindWithoutReversing)
{
  Scenario sc;
  sc.route = {Vector2d(0, 0), Vector2d(-5, 0)};
  sc.goal_yaw = M_PI;
  sc.x0 = state(0, 0, 0, posture().comfort);
  sc.max_steps = 150;
  const LoopRun run = closed_loop(sc);
  print_stats("behind", run);
  double peak_omega = 0.0;
  int sign = 0;
  bool consistent = true;
  for (size_t i = 0; i < run.steps.size(); ++i) {
    const auto & s = run.steps[i];
    for (const auto & u : s.plan.u) ASSERT_GE(u(0), -1e-9) << "reversing at step " << i;
    if (std::cos(s.x(2) - M_PI) < 0.9) {  // still turning towards the route
      const int now = (s.command(1) > 1e-6) - (s.command(1) < -1e-6);
      if (sign == 0) sign = now;
      consistent = consistent && (now == 0 || now == sign);
    }
    if (i < 30) peak_omega = std::max(peak_omega, std::abs(s.command(1)));
  }
  EXPECT_TRUE(consistent) << "the turn changed direction";
  EXPECT_GT(peak_omega, limits().omega_max - 1e-3) << "omega did not saturate within 3 s";
  const double heading_error = std::abs(std::remainder(run.final_state(2) - M_PI, 2 * M_PI));
  EXPECT_LT(heading_error, 0.1);
  EXPECT_LT(run.final_state(0), -1.0) << "did not drive along the route after the turn";
  EXPECT_LT(std::abs(run.final_state(1)), 0.3);
}

TEST(Formulation, PanelReferenceRampsInBeforeAnchor)
{
  const double anchor = 3.0;
  Scenario sc = straight(10.0, posture().comfort);
  sc.references = {PanelReference{panel_in_base(posture().tight), anchor, 10.0, kDefaultRampM}};
  sc.done = [&](const State & x) { return x(0) >= anchor; };
  const LoopRun run = closed_loop(sc);
  print_stats("reference", run);
  double first_move_s = std::numeric_limits<double>::infinity();
  for (const auto & s : run.steps)
    if ((s.x.tail<6>() - posture().comfort).cwiseAbs().maxCoeff() > 0.01) {
      first_move_s = s.x(0);
      break;
    }
  ASSERT_GE(run.final_state(0), anchor) << "did not arrive";
  const Eigen::Isometry3d actual = panel_in_base(run.final_state.tail<6>());
  const Eigen::Isometry3d wanted = panel_in_base(posture().tight);
  const double position = (actual.translation() - wanted.translation()).norm();
  const double angle = rotation_angle(actual.linear(), wanted.linear());
  std::printf("arm starts at s %.2f m; at the anchor: panel error %.4f m, %.4f rad\n", first_move_s, position, angle);
  EXPECT_LT(first_move_s, anchor);
  EXPECT_LT(position, 0.05);
  EXPECT_LT(angle, 0.05);
  EXPECT_EQ(run.failures, 0);
}

TEST(Formulation, AvoidsBoxOnRoute)
{
  Scenario sc = straight(10.0, posture().comfort);
  // A 0.4 m crate 0.5 m high (under the panel) slightly left of the route at 4 m.
  sc.world = world_of({mmn::Box{"crate", Vector3d(4.0, 0.1, 0.25), Vector3d(0.4, 0.4, 0.5), 0.0}});
  sc.max_steps = 450;
  sc.done = [](const State & x) { return x(0) > 8.0; };
  const LoopRun run = closed_loop(sc);
  print_stats("box", run);
  double closest = std::numeric_limits<double>::infinity();
  for (const auto & s : run.steps) closest = std::min(closest, s.distance);
  std::printf("closest %.4f m, final x %.2f\n", closest, run.final_state(0));
  EXPECT_GE(closest, CollisionConfig{}.margin_m - 0.01);
  // The route runs through the crate: B5 stops at the margin (the global route goes around
  // known boxes). It must still have driven up to it.
  EXPECT_GT(run.final_state(0), 3.0);
  EXPECT_EQ(run.failures, 0);
}

TEST(Formulation, SlackKeepsQpFeasibleInsideMargin)
{
  const Formulation f = formulation();
  const CollisionModel collision(primitives(), CollisionConfig{});
  const Route route({Vector2d(0, 0), Vector2d(5, 0)});
  const State x0 = state(0, 0, 0, posture().comfort);
  // A 0.6 m long crate beside the base, 0.02 m from it (inside the 0.05 m margin).
  // The crate is beside the front right wheel (0.5 m ahead of the base centre). Centred beside
  // the base it faces both right wheels, whose rows then have opposite yaw gradients: the
  // linearized escape cancels and the L1 problem becomes degenerate (see the report).
  auto crate_at = [](double y) {
    return world_of({mmn::Box{"crate", Vector3d(0.5, y, 0.25), Vector3d(0.6, 0.2, 0.5), 0.0}});
  };
  const double probe_y = -1.0;
  const double extent = -probe_y - 0.1 - obstacle_distance(x0, crate_at(probe_y));
  const World world = crate_at(-(extent + 0.02 + 0.1));
  const double start = obstacle_distance(x0, world);
  ASSERT_NEAR(start, 0.02, 1e-6);

  const Plan nominal = f.hold(x0);
  const auto rows = collision.rows(nominal.x, f.dt(), 0.0, world, nullptr);
  const Inputs in{x0, Control::Zero(), &route, route.length(), 0.0, {}, posture(), 0.0};
  const QpProblem qp = f.build(in, nominal, rows);
  OsqpSolver solver(solver_options());
  solver.setup(qp);
  const SolveResult result = solver.solve(qp, nullptr);
  ASSERT_TRUE(result.ok) << result.status;
  std::printf("%s, iterations %d, max slack %.4f\n", result.status.c_str(), result.iterations,
              result.solution.slack.maxCoeff());
  EXPECT_GT(result.solution.slack.maxCoeff(), 1e-3);
  const Plan plan = f.apply(nominal, result.solution);
  const double end = obstacle_distance(plan.x.back(), world);
  std::printf("distance: start %.4f m, end of the plan %.4f m\n", start, end);
  EXPECT_GT(end, start + 0.01);
  for (const auto & x : plan.x) EXPECT_GT(obstacle_distance(x, world), 0.0);
}

TEST(Formulation, Stage0ProgressSearchesAroundTheHint)
{
  // Review R31: a U-route whose return leg runs 0.24 m beside the outgoing one; the base at x 1.0,
  // 0.14 m off the outgoing leg, is nearer the return leg. Without a hint (a fresh goal) stage 0
  // takes the nearest point of the whole route (on the return leg); with the previous cycle's
  // progress as the hint it stays on the outgoing branch.
  const Formulation f = formulation();
  const Route route({Vector2d(0, 0), Vector2d(4, 0), Vector2d(4, 0.24), Vector2d(0, 0.24)});
  Inputs in{state(1.0, 0.14, 0.0, posture().comfort), Control::Zero(), &route, route.length(), M_PI, {}, posture(), 0.0};
  EXPECT_NEAR(f.stage0_progress(in), 4.24 + 3.0, 1e-9);
  in.s0_hint = 0.95;
  EXPECT_NEAR(f.stage0_progress(in), 1.0, 1e-9);
  EXPECT_GE(f.progress_window(), 1.0);
  in.route = nullptr;
  EXPECT_THROW(f.stage0_progress(in), std::invalid_argument);
}

TEST(Formulation, RespectsJointAndRateLimits)
{
  Scenario sc;
  sc.route = {Vector2d(0, 0), Vector2d(5, 0), Vector2d(5, 5)};
  sc.goal_yaw = M_PI / 2;
  sc.x0 = state(0, 0, 0, posture().comfort);
  sc.references = {PanelReference{panel_in_base(posture().tight), 2.0, 5.0, kDefaultRampM},
                   PanelReference{panel_in_base(posture().comfort), 7.0, 10.0, kDefaultRampM}};
  sc.max_steps = 300;
  const LoopRun run = closed_loop(sc);
  print_stats("limits", run);
  const Limits & l = limits();
  // Stage 0 and the states are the solver's (eps 1e-3 absolute plus relative); later controls are
  // projected onto the limits by apply().
  const double dt = config().dt, tol = 1e-6, solver_tol = 3e-3;
  double joint_motion = 0.0;
  for (size_t i = 0; i < run.steps.size(); ++i) {
    const auto & s = run.steps[i];
    for (size_t k = 0; k < s.plan.u.size(); ++k) {
      const Control & u = s.plan.u[k];
      const Control & prev = k == 0 ? s.last : s.plan.u[k - 1];
      const double t = k == 0 ? solver_tol : tol;
      ASSERT_GE(u(0), -tol);
      ASSERT_LE(u(0), l.v_max + tol);
      ASSERT_LE(std::abs(u(1)), l.omega_max + tol);
      ASSERT_GE(u(0) - prev(0), -l.decel * dt - t) << "step " << i << " stage " << k;
      ASSERT_LE(u(0) - prev(0), l.accel * dt + t) << "step " << i << " stage " << k;
      ASSERT_LE(std::abs(u(1) - prev(1)), l.yaw_accel * dt + t) << "step " << i << " stage " << k;
      for (int j = 0; j < 6; ++j) {
        ASSERT_LE(std::abs(u(2 + j)), l.qd_max(j) + tol);
        ASSERT_LE(std::abs(u(2 + j) - prev(2 + j)), l.qdd_max(j) * dt + t) << "step " << i << " stage " << k;
      }
    }
    for (const auto & x : s.plan.x)
      for (int j = 0; j < 6; ++j) {
        ASSERT_GE(x(3 + j), l.q_min(j) - solver_tol) << "step " << i;
        ASSERT_LE(x(3 + j), l.q_max(j) + solver_tol) << "step " << i;
      }
    joint_motion = std::max(joint_motion, (s.x.tail<6>() - posture().comfort).cwiseAbs().maxCoeff());
  }
  EXPECT_GT(joint_motion, 1.0) << "the references did not move the arm";
  EXPECT_EQ(run.failures, 0);
}

TEST(Formulation, GoalHeadingAtEnd)
{
  Scenario sc = straight(3.0, posture().comfort);
  sc.goal_yaw = M_PI / 2;
  sc.max_steps = 400;
  const LoopRun run = closed_loop(sc);
  print_stats("goal heading", run);
  const State & x = run.final_state;
  std::printf("final pose %.3f %.3f %.3f\n", x(0), x(1), x(2));
  EXPECT_LT((x.head<2>() - Vector2d(3.0, 0.0)).norm(), 0.15);
  EXPECT_LT(std::abs(std::remainder(x(2) - M_PI / 2, 2 * M_PI)), 0.15);
}

TEST(Formulation, PrefersVerticalCarryInNarrowGate)
{
  const LoopRun & run = gate_run();
  double closest = std::numeric_limits<double>::infinity();
  double tilt_in_gate = 0.0;
  bool passed = false;
  double alpha_max = 0.0;
  for (const auto & s : run.steps) {
    closest = std::min(closest, s.distance);
    alpha_max = std::max(alpha_max, s.alpha);
    const Eigen::Isometry3d panel = link_pose(link_index("panel"), s.x);
    if (std::abs(panel.translation().x() - kGateX) <= kPostHalfDepth) {
      passed = true;
      // vertical: the plate's thin axis (panel z) horizontal
      tilt_in_gate = std::max(tilt_in_gate, std::abs(panel.linear().col(2).z()));
    }
  }
  std::printf("gate: closest %.4f m, passed %d, max |thin axis z| in the gate %.3f, alpha max %.2f, final x %.2f\n",
              closest, passed, tilt_in_gate, alpha_max, run.final_state(0));
  EXPECT_GT(closest, 0.0) << "contact";
  if (passed) EXPECT_LT(tilt_in_gate, std::sin(10.0 * M_PI / 180.0)) << "the panel was not vertical in the gate";
}

TEST(Formulation, ReturnsHomeAfterTightSection)
{
  const LoopRun & run = gate_run();
  const StepLog * after = nullptr;
  for (const auto & s : run.steps)
    if (s.x(0) <= kGateX - 5.0) {
      after = &s;
      break;
    }
  ASSERT_NE(after, nullptr) << "did not get 5 m past the gate (final x " << run.final_state(0) << ")";
  const double error = (after->x.tail<6>() - posture().comfort).cwiseAbs().maxCoeff();
  std::printf("5 m after the gate: max joint offset from home %.4f rad, alpha %.3f\n", error, after->alpha);
  EXPECT_LT(error, 0.1);
}

// ---- posture modes (spec Section 3, amended 2026-10-13) ----

namespace
{
PostureTargets posture_in(PostureMode mode)
{
  PostureTargets targets = posture();
  targets.mode = mode;
  return targets;
}

// The QP of one cycle on a straight route from rest at x0 (no collision rows), with the given
// posture targets and tightness.
QpProblem straight_qp(const Formulation & f, const State & x0, const PostureTargets & targets, double alpha)
{
  static const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  const Inputs in{x0, Control::Zero(), &route, route.length(), 0.0, {}, targets, alpha};
  return f.build(in, f.hold(x0), {});
}

// The cost of every stage (Q and q of the states, R and r of the controls) equal.
void expect_same_cost(const QpProblem & a, const QpProblem & b, const std::string & what)
{
  ASSERT_EQ(a.stages.size(), b.stages.size());
  for (size_t k = 0; k <= a.stages.size(); ++k) {
    const StageQp & sa = k < a.stages.size() ? a.stages[k] : a.terminal;
    const StageQp & sb = k < b.stages.size() ? b.stages[k] : b.terminal;
    EXPECT_EQ(sa.Q, sb.Q) << what << ", stage " << k;
    EXPECT_EQ(sa.q, sb.q) << what << ", stage " << k;
    if (k < a.stages.size()) {
      EXPECT_EQ(sa.R, sb.R) << what << ", stage " << k;
      EXPECT_EQ(sa.r, sb.r) << what << ", stage " << k;
    }
  }
}

// Largest |q| entry over the joint states of every stage after 0: the posture term's pull at a
// nominal that holds the arm still.
double joint_pull(const QpProblem & qp)
{
  double pull = 0.0;
  for (size_t k = 1; k <= qp.stages.size(); ++k) {
    const StageQp & st = k < qp.stages.size() ? qp.stages[k] : qp.terminal;
    pull = std::max(pull, st.q.segment(3, 6).cwiseAbs().maxCoeff());
  }
  return pull;
}
}  // namespace

TEST(PostureModes, ComfortOnlyNeverPullsTowardsVerticalCarry)
{
  const Formulation f = formulation();
  const State at_comfort = state(0, 0, 0, posture().comfort);
  // Blend at full tightness pulls the arm at the comfort pose towards vertical_carry ...
  const QpProblem blend = straight_qp(f, at_comfort, posture_in(PostureMode::Blend), 1.0);
  EXPECT_GT(joint_pull(blend), 1.0);
  // ... comfort_only does not, whatever the tightness: its cost is blend's at tightness 0.
  const QpProblem relaxed = straight_qp(f, at_comfort, posture_in(PostureMode::Blend), 0.0);
  for (double alpha : {0.0, 0.5, 1.0}) {
    const QpProblem comfort = straight_qp(f, at_comfort, posture_in(PostureMode::ComfortOnly), alpha);
    EXPECT_EQ(joint_pull(comfort), 0.0) << "alpha " << alpha;
    expect_same_cost(comfort, relaxed, "comfort_only at alpha " + std::to_string(alpha));
  }
  // Away from the comfort pose it pulls back to it, never towards the tight pose.
  const Joints between = 0.5 * (posture().comfort + posture().tight);
  const QpProblem away = straight_qp(f, state(0, 0, 0, between), posture_in(PostureMode::ComfortOnly), 1.0);
  const Joints gradient = away.stages[1].q.segment(3, 6);
  const Joints towards_tight = posture().tight - between;
  EXPECT_LT((-gradient).dot(towards_tight), 0.0) << "the descent direction must point away from the tight pose";
  EXPECT_TRUE(gradient.isApprox(config().weights.posture * (between - posture().comfort)));
}

TEST(PostureModes, NoneAddsNoPostureTerm)
{
  const Formulation f = formulation();
  Weights no_posture = config().weights;
  no_posture.posture = 0.0;
  const Formulation without(config().horizon, config().dt, limits(), no_posture);
  for (const Joints & q : {posture().comfort, posture().tight, Joints(0.5 * (posture().comfort + posture().tight))}) {
    const State x0 = state(0, 0, 0, q);
    for (double alpha : {0.0, 1.0}) {
      expect_same_cost(straight_qp(f, x0, posture_in(PostureMode::None), alpha),
                       straight_qp(without, x0, posture_in(PostureMode::Blend), alpha), "none");
    }
  }
}

TEST(PostureModes, NoneStaysWellPosedAtTheStraightHome)
{
  // Without a posture term the joint states carry no cost; the joint effort and smoothness weights
  // keep every stage's control Hessian positive definite (effort_arm + smooth_arm on each joint
  // velocity), so the QP stays well posed at the singular straight home. Nothing then moves the
  // arm while the base drives.
  const Formulation f = formulation();
  const State home = state(0, 0, 0, posture().comfort);
  const QpProblem qp = straight_qp(f, home, posture_in(PostureMode::None), 0.0);
  for (const auto & st : qp.stages) {
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(st.R);
    EXPECT_GE(eig.eigenvalues().minCoeff(), config().weights.effort_base + config().weights.smooth_base - 1e-9);
  }
  OsqpSolver solver(solver_options());
  solver.setup(qp);
  const SolveResult result = solver.solve(qp, nullptr);
  ASSERT_TRUE(result.ok) << result.status;
  std::printf("posture none at the straight home: %s in %d iterations\n", result.status.c_str(), result.iterations);
  for (const auto & du : result.solution.du) EXPECT_LT(du.tail<6>().cwiseAbs().maxCoeff(), 1e-3);

  Scenario sc = straight(10.0, posture().comfort);
  sc.posture = posture_in(PostureMode::None);
  sc.done = [](const State & x) { return x(0) > 9.0; };
  sc.max_steps = 400;
  const LoopRun run = closed_loop(sc);
  print_stats("posture none", run);
  double arm = 0.0;
  for (const auto & step : run.steps) arm = std::max(arm, (step.x.tail<6>() - posture().comfort).cwiseAbs().maxCoeff());
  EXPECT_GT(run.final_state(0), 9.0);
  EXPECT_EQ(run.failures, 0);
  EXPECT_LT(arm, 0.01) << "the arm moved without a posture term or an obstacle";
}

// ---- clearance cost (spec Section 3, amended 2026-10-13) ----

namespace
{
Formulation with_clearance(double weight, double target_m = 0.25)
{
  Weights w = config().weights;
  w.clearance = weight;
  w.clearance_target_m = target_m;
  return Formulation(config().horizon, config().dt, limits(), w);
}

// Every matrix and vector of two QPs equal.
void expect_same_qp(const QpProblem & a, const QpProblem & b)
{
  ASSERT_EQ(a.stages.size(), b.stages.size());
  for (size_t k = 0; k <= a.stages.size(); ++k) {
    const StageQp & sa = k < a.stages.size() ? a.stages[k] : a.terminal;
    const StageQp & sb = k < b.stages.size() ? b.stages[k] : b.terminal;
    EXPECT_EQ(sa.Q, sb.Q) << k;
    EXPECT_EQ(sa.q, sb.q) << k;
    EXPECT_EQ(sa.R, sb.R) << k;
    EXPECT_EQ(sa.r, sb.r) << k;
    EXPECT_EQ(sa.S, sb.S) << k;
    EXPECT_EQ(sa.A, sb.A) << k;
    EXPECT_EQ(sa.B, sb.B) << k;
    EXPECT_EQ(sa.c, sb.c) << k;
    EXPECT_EQ(sa.Cx, sb.Cx) << k;
    EXPECT_EQ(sa.Cu, sb.Cu) << k;
    EXPECT_EQ(sa.lower, sb.lower) << k;
    EXPECT_EQ(sa.upper, sb.upper) << k;
    EXPECT_EQ(sa.x_lower, sb.x_lower) << k;
    EXPECT_EQ(sa.x_upper, sb.x_upper) << k;
    EXPECT_EQ(sa.u_lower, sb.u_lower) << k;
    EXPECT_EQ(sa.u_upper, sb.u_upper) << k;
  }
  EXPECT_EQ(a.dx0, b.dx0);
  EXPECT_EQ(a.slack_rows, b.slack_rows);
}

// A tall box (2 m, above the panel) beside a route along y = 0, left of it at x = 4 m: its nearest
// robot primitive is `gap` from the robot at (4, 0, 0) in the comfort pose.
World box_beside(double gap)
{
  const auto box = [](double y) {
    return world_of({mmn::Box{"pallet_rack", Vector3d(4.0, y, 1.0), Vector3d(0.4, 0.4, 2.0), 0.0}});
  };
  const State probe = state(4.0, 0.0, 0.0, posture().comfort);
  const double far_y = 1.2;  // within the activation distance (distances are capped there)
  const double reach = far_y - 0.2 - obstacle_distance(probe, box(far_y));  // the robot's extent towards +y
  return box(reach + gap + 0.2);
}

struct ClearanceDrive
{
  double closest = std::numeric_limits<double>::infinity(), lateral = 0.0, arm = 0.0, time_s = 0.0;
  int failures = 0;
  bool arrived = false;
};

ClearanceDrive drive_past_box(double weight, double gap)
{
  Scenario sc = straight(10.0, posture().comfort);
  sc.world = box_beside(gap);
  sc.weights.clearance = weight;
  sc.max_steps = 450;
  sc.done = [](const State & x) { return x(0) > 8.0; };
  const LoopRun run = closed_loop(sc);
  ClearanceDrive out;
  for (const auto & step : run.steps) {
    out.closest = std::min(out.closest, step.distance);
    out.lateral = std::max(out.lateral, std::abs(step.x(1)));
    out.arm = std::max(out.arm, (step.x.tail<6>() - posture().comfort).cwiseAbs().maxCoeff());
  }
  out.time_s = static_cast<double>(run.steps.size()) * config().dt;
  out.failures = run.failures;
  out.arrived = run.final_state(0) > 8.0;
  return out;
}
}  // namespace

TEST(Clearance, WeightZeroReproducesTheQp)
{
  // wholebody_mpc.yaml: off by default (the recorded B5).
  EXPECT_EQ(config().weights.clearance, 0.0);
  EXPECT_EQ(config().weights.clearance_target_m, 0.25);
  const Formulation f = formulation();
  const CollisionModel collision(primitives(), CollisionConfig{});
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  const State x0 = state(3.6, 0.0, 0.0, posture().comfort);
  const World world = box_beside(0.10);
  Plan nominal = f.hold(x0);
  for (auto & u : nominal.u) u(0) = 0.3;
  nominal = f.rollout(x0, nominal.u);
  const auto rows = collision.rows(nominal.x, f.dt(), 0.0, world, nullptr);
  ASSERT_TRUE(std::any_of(rows.begin(), rows.end(), [](const DistanceRow & r) {
    return r.stage > 0 && r.obstacle == "pallet_rack" && r.distance < 0.25;
  })) << "no obstacle row below the target";
  const Inputs in{x0, Control::Zero(), &route, route.length(), 0.0, {}, posture(), 0.0};
  const QpProblem qp = f.build(in, nominal, rows);
  // Weight 0: the rows constrain but leave the cost as without them, and the target does not matter.
  expect_same_cost(qp, f.build(in, nominal, {}), "weight 0");
  expect_same_qp(qp, with_clearance(0.0, 1.0).build(in, nominal, rows));
  // A weight changes only Q and q (the pattern and the constraints stay).
  const QpProblem weighted = with_clearance(20.0).build(in, nominal, rows);
  bool changed = false;
  for (size_t k = 0; k <= qp.stages.size(); ++k) {
    const StageQp & a = k < qp.stages.size() ? qp.stages[k] : qp.terminal;
    const StageQp & b = k < qp.stages.size() ? weighted.stages[k] : weighted.terminal;
    changed = changed || a.Q != b.Q;
    EXPECT_EQ(a.Cx, b.Cx);
    EXPECT_EQ(a.lower, b.lower);
    EXPECT_EQ(a.R, b.R);
  }
  EXPECT_TRUE(changed);
}

TEST(Clearance, GaussNewtonTermOfObstacleRowsBelowTheTarget)
{
  const double weight = 10.0, target = 0.25;
  const Formulation f = formulation(), fw = with_clearance(weight, target);
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  const State x0 = state(0, 0, 0, posture().comfort);
  const Plan nominal = f.hold(x0);
  Eigen::Matrix<double, 1, kNx> g;
  g << 0.0, -1.0, 0.3, 0.2, -0.1, 0.0, 0.4, 0.0, 0.1;
  const int panel = link_index("panel");
  // Stage 2: one obstacle row below the target, one above it, a floor row and a self row below it.
  const std::vector<DistanceRow> rows = {
    {2, panel, "crate", 0.10, g}, {2, panel, "crate", 0.30, g}, {2, panel, "floor", 0.10, g},
    {2, panel, "self/base_link", 0.10, g}};
  const Inputs in{x0, Control::Zero(), &route, route.length(), 0.0, {}, posture(), 0.0};
  const QpProblem a = f.build(in, nominal, rows), b = fw.build(in, nominal, rows);
  for (size_t k = 0; k <= a.stages.size(); ++k) {
    const StageQp & sa = k < a.stages.size() ? a.stages[k] : a.terminal;
    const StageQp & sb = k < b.stages.size() ? b.stages[k] : b.terminal;
    if (k != 2) {
      EXPECT_EQ(sa.Q, sb.Q) << k;
      EXPECT_EQ(sa.q, sb.q) << k;
      continue;
    }
    // 0.5 w (target - d - g dx)^2 of the one obstacle row below the target.
    Eigen::MatrixXd dQ = Eigen::MatrixXd::Zero(Formulation::kNz, Formulation::kNz);
    dQ.topLeftCorner<kNx, kNx>() = weight * g.transpose() * g;
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(Formulation::kNz);
    dq.head<kNx>() = -weight * (target - 0.10) * g.transpose();
    EXPECT_TRUE((sb.Q - sa.Q).isApprox(dQ, 1e-12));
    EXPECT_TRUE((sb.q - sa.q).isApprox(dq, 1e-12));
  }
}

TEST(SlackInside, BaseRowsKeepThePlainWeight)
{
  // Phase 4B-A Task 11 (user decision 2026-10-19): the inside-margin slack weight applies to the arm and panel rows
  // only; a base row inside its (0.12 m) base margin keeps the plain weight (with it the base stalled crooked in the
  // store S door: v2_store_room_nav 1 of 3, 3 of 3 without the weight).
  Weights w = config().weights;
  w.slack = 100.0;
  w.slack_inside = 3000.0;
  const Formulation f(config().horizon, config().dt, limits(), w, CollisionConfig{});
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  const State x0 = state(0, 0, 0, posture().comfort);
  const Plan nominal = f.hold(x0);
  Eigen::Matrix<double, 1, kNx> g;
  g << 0.0, -1.0, 0.3, 0.2, -0.1, 0.0, 0.4, 0.0, 0.1;
  DistanceRow panel_inside{2, link_index("panel"), "crate", 0.03, g, -1, 0.05};
  DistanceRow panel_outside{2, link_index("panel"), "crate", 0.08, g, -1, 0.05};
  DistanceRow base_inside{2, link_index("base_link"), "wall", 0.10, g, -1, 0.12};
  base_inside.base = true;
  const Inputs in{x0, Control::Zero(), &route, route.length(), 0.0, {}, posture(), 0.0};
  const QpProblem qp = f.build(in, nominal, {panel_inside, panel_outside, base_inside});
  ASSERT_EQ(qp.slack_weights.size(), static_cast<size_t>(qp.slack_count));
  EXPECT_EQ(std::count(qp.slack_weights.begin(), qp.slack_weights.end(), 3000.0), 1)
    << "only the panel row inside its margin gets the inside weight";
}

TEST(Clearance, KeepsFartherFromABoxBesideTheRoute)
{
  // A tall box 0.12 m beside the robot's straight route. Without the clearance cost (the recorded
  // B5) the tightness pulls the arm towards vertical carry, which swings the panel towards the box,
  // and the constraints hold it at the 0.05 m margin. With the cost the closed loop keeps more
  // distance, mostly with the arm (the base stays within a few centimetres of the route), and still
  // reaches the end of the route in the same time. The sweep is printed for the choice of weight.
  const double gap = 0.12;
  const ClearanceDrive off = drive_past_box(0.0, gap);
  const auto print = [](double weight, const ClearanceDrive & d) {
    std::printf("clearance weight %5.1f: closest %.4f m, max |y| %.4f m, max arm offset %.3f rad, %.1f s, "
                "%d failed solves\n", weight, d.closest, d.lateral, d.arm, d.time_s, d.failures);
  };
  print(0.0, off);
  ClearanceDrive chosen;
  for (const double weight : {5.0, 10.0, 20.0, 50.0, 100.0}) {
    const ClearanceDrive on = drive_past_box(weight, gap);
    print(weight, on);
    if (weight == 20.0) chosen = on;
  }
  ASSERT_TRUE(off.arrived);
  EXPECT_EQ(off.failures, 0);
  EXPECT_GE(off.closest, CollisionConfig{}.margin_m - 0.01);
  // weight 20 (the contour weight): noticeable, and it does not take over the route tracking
  EXPECT_TRUE(chosen.arrived);
  EXPECT_EQ(chosen.failures, 0);
  EXPECT_GT(chosen.closest, off.closest + 0.05);
  EXPECT_LT(chosen.closest, 0.25);  // below the target: the cost trades distance against the other terms
  EXPECT_LT(chosen.lateral, 0.03);
  EXPECT_LE(chosen.time_s, off.time_s + 0.5);
}

// ---- side picker (Phase 4B-A spec Section 3) ----

namespace
{
// Per-stage clearances: `value` everywhere, `at` from stage `from` on (a conflict ahead; none by default).
std::vector<double> profile(double value, int from = 21, double at = 0.0, int n = 21)
{
  std::vector<double> v(static_cast<size_t>(n), value);
  for (int k = from; k < n; ++k) v[static_cast<size_t>(k)] = at;
  return v;
}
}  // namespace

TEST(SidePicker, NoProbeBelowAlphaOn)
{
  SidePicker p;
  EXPECT_FALSE(p.needs_probe(0.04));
  EXPECT_TRUE(p.needs_probe(0.05));
  EXPECT_EQ(p.update(0.04, profile(0.8, 5, 0.1), {profile(0.8), profile(0.8)}, 0.0, 0.0), PostureChoice::None);
}

TEST(SidePicker, LargerScoreWins)
{
  SidePicker p;
  // Comfort conflicts from stage 5 (0.05 m); tight scores 0.20 and 0.12 there.
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.12)}, 0.0, 0.0),
            PostureChoice::Tight0);
  SidePicker q;
  EXPECT_EQ(q.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.12), profile(0.8, 5, 0.20)}, 0.0, 0.0),
            PostureChoice::Tight1);
}

TEST(SidePicker, GuardKeepsComfortUnderABeam)
{
  SidePicker p;  // both vertical poses are taller: worse than the comfort pose under a beam
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.06), profile(0.8, 5, 0.06)}, 0.0, 0.0),
            PostureChoice::Comfort);
  SidePicker q;  // better, but not by more than 0.02 m
  EXPECT_EQ(q.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.115), profile(0.8, 5, 0.11)}, 0.0, 0.0),
            PostureChoice::Comfort);
}

TEST(SidePicker, ScoresTheFirstConflictWindowOnly)
{
  // The comfort pose gets tight at stage 5; Tight0 is better there (0.25 vs 0.15) but meets a beam at stage 15,
  // Tight1 does not: the window k*-1..k*+1 decides, the later conflict is left to the collision rows.
  std::vector<double> t0 = profile(0.8, 5, 0.25), t1 = profile(0.8, 5, 0.15);
  for (int k = 15; k < 21; ++k) t0[static_cast<size_t>(k)] = 0.01;
  SidePicker p;
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {t0, t1}, 0.0, 0.0), PostureChoice::Tight0);
}

TEST(SidePicker, TieKeepsTheCurrentPanSide)
{
  SidePicker p;
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.21)}, -1.2, 1.0),
            PostureChoice::Tight1);
  SidePicker q;
  EXPECT_EQ(q.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.21), profile(0.8, 5, 0.20)}, 1.2, -1.0),
            PostureChoice::Tight0);
}

TEST(SidePicker, TieFollowsTheRouteTurn)
{
  SidePicker p;
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.20)}, 0.0, -1.0),
            PostureChoice::Tight1);
  SidePicker q;
  EXPECT_EQ(q.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.20)}, 0.0, 1.0),
            PostureChoice::Tight0);
}

TEST(SidePicker, TieDefaultsLeft)
{
  SidePicker p;
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.20)}, 0.0, 0.0),
            PostureChoice::Tight0);
}

TEST(SidePicker, LatchesUntilAlphaOff)
{
  SidePicker p;
  ASSERT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.12)}, 0.0, 0.0),
            PostureChoice::Tight0);
  EXPECT_FALSE(p.needs_probe(0.5));  // latched: no probes
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.10), profile(0.8, 5, 0.30)}, 0.0, 0.0),
            PostureChoice::Tight0);
  EXPECT_EQ(p.update(0.03, profile(0.8), {}, 0.0, 0.0), PostureChoice::Tight0);  // between alpha_off and alpha_on
  EXPECT_EQ(p.update(0.019, profile(0.8), {}, 0.0, 0.0), PostureChoice::None);
  EXPECT_TRUE(p.needs_probe(0.5));
}

TEST(SidePicker, OneTightPoseIsTheLeftOnlyBlend)
{
  SidePicker p;  // tight_poses [vertical_carry]: the guard still applies
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.05), {profile(0.8, 5, 0.20)}, 0.0, 0.0), PostureChoice::Tight0);
  SidePicker q;
  EXPECT_EQ(q.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.06)}, 0.0, 0.0), PostureChoice::Comfort);
}

TEST(SidePicker, NoConflictStageKeepsNone)
{
  SidePicker p;  // alpha still decaying, the comfort pose already clear everywhere
  EXPECT_EQ(p.update(0.3, profile(0.8), {profile(0.8), profile(0.8)}, 0.0, 0.0), PostureChoice::None);
}

TEST(SidePicker, ComfortCanStillTurnTight)
{
  // Task 8 review: a low beam (Comfort) followed by a gate before alpha falls back: the comfort choice stays
  // open to a tight pose (no side is crossed); left and right stay latched (LatchesUntilAlphaOff).
  SidePicker p;
  ASSERT_EQ(p.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.06), profile(0.8, 5, 0.06)}, 0.0, 0.0),
            PostureChoice::Comfort);
  EXPECT_TRUE(p.needs_probe(0.5));
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, -0.05), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.12)}, 0.0, 0.0),
            PostureChoice::Tight0);  // a gate the rigid comfort pose does not fit
  EXPECT_FALSE(p.needs_probe(0.5));
}

TEST(SidePicker, ComfortStaysWhenTheTightPoseIsBetterOnlyOnTheApproach)
{
  // Under a low beam a vertical pose can score better at the first conflict (its front edge sits farther back on
  // the approach) while it is the one that hits the beam further on: from Comfort a switch also needs the tight
  // pose's whole-horizon minimum to beat the comfort pose's.
  SidePicker p;
  ASSERT_EQ(p.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.06), profile(0.8, 5, 0.06)}, 0.0, 0.0),
            PostureChoice::Comfort);
  std::vector<double> comfort = profile(0.8, 5, 0.20), tight = profile(0.8, 5, 0.30);
  for (int k = 12; k < 21; ++k) comfort[static_cast<size_t>(k)] = 0.03, tight[static_cast<size_t>(k)] = -0.2;
  EXPECT_EQ(p.update(0.5, comfort, {tight, tight}, 0.0, 0.0), PostureChoice::Comfort);
}

TEST(SidePicker, ComfortTurnsTightForAGateWithABeamLaterInTheHorizon)
{
  // Task 10 review: from Comfort the tight pose is compared around the comfort pose's worst stage. A gate at stage 5
  // that the rigid comfort pose does not fit (comfort -0.08, tight 0.20) with a beam at stages 15.. that only the
  // tight pose meets (comfort 0.10, tight -0.10): the gate is comfort's worst conflict, the tight pose is better
  // there, so it switches.
  SidePicker p;
  ASSERT_EQ(p.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.06), profile(0.8, 5, 0.06)}, 0.0, 0.0),
            PostureChoice::Comfort);
  std::vector<double> comfort = profile(0.8, 5, -0.08), tight = profile(0.8, 5, 0.20);
  for (int k = 15; k < 21; ++k) comfort[static_cast<size_t>(k)] = 0.10, tight[static_cast<size_t>(k)] = -0.10;
  EXPECT_EQ(p.update(0.5, comfort, {tight, tight}, 0.0, 0.0), PostureChoice::Tight0);
}

TEST(SidePicker, ComfortStaysWhereTheRigidComfortPoseOnlyGetsClose)
{
  // Under a low beam the rigid comfort pose comes close (0.02 m) but does not collide; a vertical pose that scores
  // better there on the approach does not take over.
  SidePicker p;
  ASSERT_EQ(p.update(0.5, profile(0.8, 5, 0.10), {profile(0.8, 5, 0.06), profile(0.8, 5, 0.06)}, 0.0, 0.0),
            PostureChoice::Comfort);
  EXPECT_EQ(p.update(0.5, profile(0.8, 5, 0.02), {profile(0.8, 5, 0.20), profile(0.8, 5, 0.20)}, 0.0, 0.0),
            PostureChoice::Comfort);
}
