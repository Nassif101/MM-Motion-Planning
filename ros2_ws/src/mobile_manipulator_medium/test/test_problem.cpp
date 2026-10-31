// The medium problem (Phase 5A Task 7, spec Sections 3.2-3.3): trajectories, cost, constraints and their linearization.
#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <random>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "medium_support.hpp"

namespace
{
using namespace medium_test;

// A trajectory starting at `start` at rest whose parameters move it about `speed` m/s along +x (a least-squares fit of
// the x coordinate to x0 + speed t; every other coordinate holds still).
mmd::Trajectory moving(const mmd::MediumProblem & p, const mmm::State & start, double speed)
{
  Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
  rate(0) = speed;
  const mmd::Trajectory rest = p.from_parameters(start, rate, Eigen::MatrixXd::Zero(p.coordinates(), p.basis().free_parameters()));
  // Fit x(t) = x0 + speed t at the samples: minimise |S M p + S c0 - target|.
  const int K = p.samples();
  Eigen::MatrixXd SM(K, p.basis().free_parameters());
  Eigen::VectorXd r(K);
  for (int k = 0; k < K; ++k) {
    const Eigen::RowVectorXd b = p.basis().basis(p.sample_time(k), 0);
    SM.row(k) = b * p.basis().map();
    r(k) = start(0) + speed * p.sample_time(k) - b.dot(rest.control_points.row(0).transpose());
  }
  Eigen::MatrixXd params = Eigen::MatrixXd::Zero(p.coordinates(), p.basis().free_parameters());
  params.row(0) = SM.colPivHouseholderQr().solve(r).transpose();
  return p.from_parameters(start, rate, params);
}

mmd::RouteTask task_on(const mmm::Route & route)
{
  mmd::RouteTask t;
  t.route = &route;
  t.goal_s = route.length();
  t.goal_yaw = 0.0;
  t.s0 = 0.0;
  return t;
}

TEST(MediumConfig, DefaultsLoad)
{
  const mmd::MediumConfig c = mmd::load_medium_config(share("mobile_manipulator_medium") + "/config/medium.yaml");
  const mmd::MediumConfig d;
  EXPECT_DOUBLE_EQ(c.rate_hz, 5.0);
  EXPECT_EQ(c.bezier.degree, 3);
  EXPECT_EQ(c.bezier.segments, 6);  // user decision 2026-10-30 (6 joints, 6 segments, 15 iterations); 4 rang at stops
  EXPECT_DOUBLE_EQ(c.bezier.horizon_s, 8.0);
  EXPECT_EQ(c.bezier.continuity, 2);
  EXPECT_EQ(c.arm_joints, d.arm_joints);
  EXPECT_EQ(c.arm_joints.size(), 6u);
  EXPECT_DOUBLE_EQ(c.collision_dt, 0.16);  // the horizon's whole multiple nearest stage B's 0.15
  EXPECT_DOUBLE_EQ(c.base_margin_m, 0.12);
  EXPECT_DOUBLE_EQ(c.arm_margin_m, 0.05);
  EXPECT_FALSE(c.contact_points);  // Task 9 CP: available, off for now
  EXPECT_DOUBLE_EQ(c.contact_band_m, 0.02);
  EXPECT_DOUBLE_EQ(c.clearance_target_m, 0.25);
  EXPECT_DOUBLE_EQ(c.tight_speed_scale, 0.5);
  EXPECT_DOUBLE_EQ(c.weights.clearance, 8.0);  // Task 9 CP: 20 stalled the plans in a door
  EXPECT_DOUBLE_EQ(c.weights.comfort_free, 5.0);  // Task 12: 0.5 and 2 let the pan drift (a swing-side switch)
  EXPECT_DOUBLE_EQ(c.homotopy_hysteresis, 0.15);
  EXPECT_DOUBLE_EQ(c.solve_cap_s, 0.14);  // 10 ms for the last iteration's overrun: cycle p99 <= 150 ms
  EXPECT_DOUBLE_EQ(c.full_multistart_period_s, 2.0);
  EXPECT_DOUBLE_EQ(c.sample_ds, 0.1);
  EXPECT_DOUBLE_EQ(c.slack_feasible_m, 0.001);
  EXPECT_EQ(c.scp.max_iter, 15);
  EXPECT_TRUE(std::isinf(c.torque_limits_nm(0)));
  // 60 % of the URDF efforts (160, 90, 40, 30, 20 N m; Unity's arm drives use them): user decision 2026-10-30
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(1), 96.0);
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(2), 54.0);
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(3), 24.0);
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(4), 18.0);
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(5), 12.0);
  EXPECT_EQ(c.seeds, d.seeds);
  EXPECT_EQ(c.qp_backend, "daqp");     // Task 9 stage A
  EXPECT_DOUBLE_EQ(c.torque_dt, 0.16);  // every sample (spec Section 3.3)
  EXPECT_DOUBLE_EQ(c.speed_fraction, d.speed_fraction);
  EXPECT_DOUBLE_EQ(c.qp_tolerance, 1e-6);
  EXPECT_DOUBLE_EQ(c.weights.progress, d.weights.progress);
  EXPECT_DOUBLE_EQ(c.weights.skid, d.weights.skid);
  EXPECT_DOUBLE_EQ(c.weights.slack, d.weights.slack);
  EXPECT_DOUBLE_EQ(c.scp.trust_init, d.scp.trust_init);
  EXPECT_DOUBLE_EQ(c.scp.merit_tol, d.scp.merit_tol);
}

TEST(MediumConfig, InconsistentValuesThrow)
{
  const auto dir = std::filesystem::temp_directory_path();
  int i = 0;
  for (const char * text : {"scp:\n  trust_init: 5.0\n  trust_max: 2.0\n", "weights:\n  comfort: -1.0\n",
                            "speed_fraction: 1.5\n", "torque_dt: 0.25\n", "qp_backend: qpoases\n"}) {
    const auto path = dir / ("medium_bad_" + std::to_string(i++) + "_" + std::to_string(::getpid()) + ".yaml");
    std::ofstream(path) << text;
    EXPECT_THROW(mmd::load_medium_config(path.string()), std::runtime_error) << text;
  }
}

TEST(MediumConfig, UnknownKeyThrows)
{
  const auto path = std::filesystem::temp_directory_path() / ("medium_unknown_key_" + std::to_string(::getpid()) + ".yaml");
  std::ofstream(path) << "rate_hz: 5.0\nweights:\n  progres: 3.0\n";
  try {
    mmd::load_medium_config(path.string());
    FAIL() << "no exception";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("progres"), std::string::npos) << e.what();
  }
}

TEST(Problem, FromParametersHoldsTheStartState)
{
  mmd::MediumConfig c;
  c.arm_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_2_joint"};  // wrist_1 not planned
  const auto p = problem(c);
  const mmm::State start = state(1.0, -2.0, 0.3, pose("carry"));
  Eigen::Matrix<double, 9, 1> rate;
  rate << 0.2, 0.1, -0.05, 0.0, 0.1, -0.2, 0.0, 0.3, 0.0;
  std::mt19937 rng(1);
  std::normal_distribution<double> g(0.0, 0.3);
  Eigen::MatrixXd params(p.coordinates(), p.basis().free_parameters());
  for (int i = 0; i < params.size(); ++i) params(i) = g(rng);
  const mmd::Trajectory t = p.from_parameters(start, rate, params);
  EXPECT_LT((p.state_at(t, 0) - start).norm(), 1e-10);
  const Eigen::Matrix<double, 9, 1> r0 = p.rate_at(t, 0);
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(r0(i), rate(i), 1e-10);
  for (const int j : p.joints()) EXPECT_NEAR(r0(3 + j), rate(3 + j), 1e-10);
  EXPECT_LT((p.parameters(t) - params).norm(), 1e-10);
  // wrist_1 and wrist_3 are not planned: they stay at the start.
  EXPECT_DOUBLE_EQ(p.state_at(t, p.samples() - 1)(3 + 3), start(3 + 3));
}

TEST(Problem, ZeroParametersAreTheConstantVelocityCurve)
{
  // The curve the parameters deviate from: the start state continued at its rate (a straight line in each coordinate),
  // so a zero step keeps every joint where it is when it starts at rest.
  const auto p = problem();
  const mmm::State start = state(1.0, -2.0, 0.3, pose("carry"));
  Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
  rate(0) = 0.25;
  const mmd::Trajectory t = p.from_parameters(start, rate, Eigen::MatrixXd::Zero(p.coordinates(), p.basis().free_parameters()));
  for (int k = 0; k < p.samples(); ++k) {
    const mmm::State x = p.state_at(t, k);
    EXPECT_NEAR(x(0), 1.0 + 0.25 * p.sample_time(k), 1e-9);
    EXPECT_NEAR((x.tail<7>() - start.tail<7>()).norm(), 0.0, 1e-9);
  }
  EXPECT_TRUE(p.hull_ok(t));
}

TEST(Problem, UnknownJointThrows)
{
  mmd::MediumConfig c;
  c.arm_joints = {"shoulder_pan_joint", "elbow"};
  EXPECT_THROW(problem(c), std::invalid_argument);
}

TEST(Problem, CollisionRowsMatchFiniteDifferences)
{
  // A box ahead of the panel (carry pose, top 1.32 m): the panel and arm rows come within the activation distance. The
  // box and the robot are turned (generic witness points: with the panel's edge parallel to a face the distance has a
  // kink, where a central difference and the rows' one-sided gradient legitimately differ).
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const auto world = world_of({{"box", Eigen::Vector3d(2.2, 0.2, 1.0), Eigen::Vector3d(0.3, 2.0, 2.0), 0.4}});
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.1, pose("carry")), 0.3);
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world, 0.0, 1.0);
  mmm::CollisionConfig cc;
  cc.margin_m = p.config().arm_margin_m;
  cc.base_margin_m = p.config().base_margin_m;
  cc.contact_points = p.config().contact_points;
  const mmm::CollisionModel model(primitives(), cc);
  std::mt19937 rng(2);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::VectorXd v(lin.parameters);
  for (int i = 0; i < v.size(); ++i) v(i) = g(rng);
  v.normalize();
  const double h = 1e-4;  // the GJK distance is accurate to about 1e-9 m (as test_collision in mobile_manipulator_mpc)
  int checked = 0, close = 0, corners = 0;
  for (size_t i = 0; i < lin.rows.size(); ++i) {
    const auto & info = lin.rows[i];
    if (info.kind != mmd::RowKind::Collision) continue;
    if (info.collision.corner >= 0) {  // a hull point against the pair's plane: its own distance
      EXPECT_NEAR(lin.qp.lA(static_cast<int>(i)), info.collision.margin - info.collision.distance, 1e-12);
      ++corners;
      continue;
    }
    const double d0 = model.pair_distance(p.state_at(t, info.sample), info.collision, world, 0.0);
    const double up = model.pair_distance(p.state_at(p.apply(t, h * v), info.sample), info.collision, world, 0.0);
    const double down = model.pair_distance(p.state_at(p.apply(t, -h * v), info.sample), info.collision, world, 0.0);
    const double predicted = lin.qp.A.row(static_cast<int>(i)).head(lin.parameters).dot(v);
    // The mapping from the state to the parameters, exactly: the row is B5's state gradient times d state / d p.
    const mmm::State dx = (p.state_at(p.apply(t, h * v), info.sample) - p.state_at(p.apply(t, -h * v), info.sample)) / (2 * h);
    EXPECT_NEAR(predicted, info.collision.gradient.dot(dx), 1e-9) << "row " << i;
    // The true distance, within B5's gradient accuracy: FCL's GJK witness points lie up to about 1 cm from the true
    // ones on the panel box, so rotational derivatives are off by about 0.01 per radian (a pre-existing property of
    // the rows, found here; the merit uses exact distances).
    // (At a change of the closest feature, where the robot passes the box, the distance has a kink and the row's
    // one-sided gradient legitimately differs from a central difference: most rows, not every row.)
    close += std::abs(predicted - (up - down) / (2 * h)) <= 2e-2;
    // margin - d0 <= J dp + s
    EXPECT_NEAR(lin.qp.lA(static_cast<int>(i)), info.collision.margin - d0, 1e-9);
    EXPECT_DOUBLE_EQ(lin.qp.A(static_cast<int>(i), lin.parameters + mmd::kCollisionSlack), 1.0);
    ++checked;
  }
  EXPECT_GT(checked, 20);
  EXPECT_GE(close, 0.9 * checked) << close << " of " << checked;
}

TEST(Problem, ContactPointsAddCornerRows)
{
  // With contact_points the pairs near the box also get rows at the panel's and the base's corners, each priced at its
  // own distance to the pair's plane (never nearer than the pair).
  mmd::MediumConfig c;
  c.contact_points = true;
  const auto p = problem(c);
  const auto route = straight_route(0.0, 10.0);
  const auto world = world_of({{"box", Eigen::Vector3d(2.2, 0.2, 1.0), Eigen::Vector3d(0.3, 2.0, 2.0), 0.4}});
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.1, pose("carry")), 0.3);
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world, 0.0, 1.0);
  int corners = 0;
  for (size_t i = 0; i < lin.rows.size(); ++i) {
    const auto & info = lin.rows[i];
    if (info.kind != mmd::RowKind::Collision || info.collision.corner < 0) continue;
    EXPECT_NEAR(lin.qp.lA(static_cast<int>(i)), info.collision.margin - info.collision.distance, 1e-12);
    ++corners;
  }
  EXPECT_GT(corners, 0);
  const auto off = problem().linearize(t, task_on(route), world, 0.0, 1.0);
  for (const auto & info : off.rows) EXPECT_LT(info.collision.corner, 0);
}

TEST(Problem, TorqueRowsMatchFiniteDifferences)
{
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const auto world = world_of({});
  mmm::State start = state(0.0, 0.0, 0.0, pose("carry"));
  Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
  rate(3 + 1) = 0.2;  // shoulder_lift moving
  const mmd::Trajectory t = p.from_parameters(start, rate, Eigen::MatrixXd::Zero(p.coordinates(), p.basis().free_parameters()));
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world, 0.0, 1.0);
  std::mt19937 rng(3);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::VectorXd v(lin.parameters);
  for (int i = 0; i < v.size(); ++i) v(i) = g(rng);
  const double eps = 1e-6;
  int checked = 0;
  for (size_t i = 0; i < lin.rows.size(); ++i) {
    const auto & info = lin.rows[i];
    if (info.kind != mmd::RowKind::Torque) continue;
    const double tau0 = statics()->torque(p.state_at(t, info.sample).tail<6>())(info.joint);
    const double tau1 = statics()->torque(p.state_at(p.apply(t, eps * v), info.sample).tail<6>())(info.joint);
    // The rows are in fractions of the joint's limit (the shared slack is a fraction there).
    const double L = p.config().torque_limits_nm(info.joint);
    const double predicted = L * lin.qp.A.row(static_cast<int>(i)).head(lin.parameters).dot(eps * v);
    EXPECT_NEAR(predicted, tau1 - tau0, 1e-4 * eps * std::max(1.0, v.norm())) << "row " << i;
    ++checked;
  }
  EXPECT_GT(checked, 10);
}

TEST(Problem, SkidResidualZeroForStraightForwardMotion)
{
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const auto world = world_of({});
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.0, pose("carry")), 0.3);
  EXPECT_LT(p.cost_terms(t, task_on(route), world, 0.0).at("skid"), 1e-12);
  // Sideways (yaw +90 deg while moving along +x) pays it.
  const mmd::Trajectory sideways = moving(p, state(0.0, 0.0, M_PI / 2, pose("carry")), 0.3);
  EXPECT_GT(p.cost_terms(sideways, task_on(route), world, 0.0).at("skid"), 1e-3);
}

TEST(Problem, ViolationsArePerKind)
{
  mmd::MediumConfig c;
  c.torque_limits_nm(1) = 24.4;  // the shoulder limit level_extension exceeds (the Task 2 limit)
  const auto p = problem(c);
  const auto world = world_of({});
  const mmd::Violations fine = p.violations(moving(p, state(0.0, 0.0, 0.0, pose("carry")), 0.2), world, 0.0);
  EXPECT_LT(fine.collision_m, 0.0 + 1e-9);
  EXPECT_LT(fine.speed_mps, 1e-9);
  EXPECT_LT(fine.torque, 1e-9);
  EXPECT_LT(fine.hull, 1e-9);
  // Reversing: the forward-only rows.
  const mmd::Violations back = p.violations(moving(p, state(0.0, 0.0, 0.0, pose("carry")), -0.2), world, 0.0);
  EXPECT_NEAR(back.speed_mps, 0.2, 1e-6);
  EXPECT_NEAR(p.max_slack(moving(p, state(0.0, 0.0, 0.0, pose("carry")), -0.2), world, 0.0), 0.2, 1e-6);
  // level_extension: 95.7 N m at the shoulder against 24.4, as a fraction of the limit.
  const mmd::Violations heavy = p.violations(moving(p, state(0.0, 0.0, 0.0, pose("level_extension")), 0.1), world, 0.0);
  EXPECT_NEAR(heavy.torque, (95.7 - 24.4) / 24.4, 0.01);
}

TEST(Problem, TorqueSlackIsAFractionOfTheLimit)
{
  // The QP's slack and the merit's violation agree on the torque rows: with no room to move, the QP's slack is the
  // largest torque violation as a fraction of its limit (the other soft rows hold).
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.0, pose("level_extension")), 0.1);
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world_of({}), 0.0, 1e-9);
  const auto r = mmd::make_dense_solver("daqp")->solve(lin.qp, nullptr);
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_NEAR(r.x(lin.parameters + mmd::kTorqueSlack), p.violations(t, world_of({}), 0.0).torque, 1e-4);
}

TEST(Problem, SpeedRowsMatchFiniteDifferences)
{
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.2, pose("carry")), 0.2);
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world_of({}), 0.0, 1.0);
  std::mt19937 rng(5);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::VectorXd v(lin.parameters);
  for (int i = 0; i < v.size(); ++i) v(i) = g(rng);
  v.normalize();
  const double h = 1e-5;
  auto speed = [&](const mmd::Trajectory & tr, int k) {
    const mmm::State x = p.state_at(tr, k);
    const auto rate = p.rate_at(tr, k);
    return std::cos(x(2)) * rate(0) + std::sin(x(2)) * rate(1);
  };
  int checked = 0;
  for (size_t i = 0; i < lin.rows.size(); ++i) {
    if (lin.rows[i].kind != mmd::RowKind::Speed) continue;
    const int k = lin.rows[i].sample;
    const double fd = (speed(p.apply(t, h * v), k) - speed(p.apply(t, -h * v), k)) / (2 * h);
    EXPECT_NEAR(lin.qp.A.row(static_cast<int>(i)).head(lin.parameters).dot(v), fd, 1e-6) << "sample " << k;
    ++checked;
  }
  EXPECT_EQ(checked, 2 * (p.samples() - 1));
}

TEST(Problem, ProgressPullIsBounded)
{
  // Standing still costs at most the speed deficit over the horizon (B5's progress term), not a lag that grows with
  // time (a reference running ahead pulled B5 into a blocking obstacle, formulation.cpp).
  const auto p = problem();
  const auto route = straight_route(0.0, 50.0);
  const double v_nom = p.config().speed_fraction * limits().v_max;
  const auto still = p.cost_terms(moving(p, state(0.0, 0.0, 0.0, pose("carry")), 0.0), task_on(route), world_of({}), 0.0);
  EXPECT_LE(still.at("progress"), 0.5 * p.config().weights.progress * v_nom * v_nom * p.basis().horizon_s() * 1.001);
  const auto cruising =
    p.cost_terms(moving(p, state(0.0, 0.0, 0.0, pose("carry")), v_nom), task_on(route), world_of({}), 0.0);
  EXPECT_LT(cruising.at("progress"), 1e-9);
}

TEST(Problem, GoalHeadingRampsIn)
{
  // The goal heading's weight rises with closeness to the goal (no jump at kGoalHeadingM).
  const auto p = problem();
  const auto world = world_of({});
  auto at = [&](double goal_x) {
    const auto route = straight_route(0.0, goal_x);
    auto task = task_on(route);
    task.goal_yaw = 1.0;
    return p.cost_terms(moving(p, state(0.0, 0.0, 0.0, pose("carry")), 0.0), task, world, 0.0).at("goal_heading");
  };
  EXPECT_DOUBLE_EQ(at(5.0), 0.0);
  EXPECT_NEAR(at(mmm::kGoalHeadingM + 1e-6), at(mmm::kGoalHeadingM - 1e-6), 1e-4);
  EXPECT_GT(at(0.01), at(0.3));
}

TEST(Problem, GoalPositionRampsIn)
{
  const auto p = problem();
  auto at = [&](double goal_x) {
    const auto route = straight_route(0.0, goal_x);
    return p.cost_terms(moving(p, state(0.0, 0.0, 0.0, pose("carry")), 0.0), task_on(route), world_of({}), 0.0)
      .at("goal_position");
  };
  EXPECT_DOUBLE_EQ(at(5.0), 0.0);
  EXPECT_NEAR(at(mmm::kProgressTaperM + 1e-6), at(mmm::kProgressTaperM - 1e-6), 1e-4);
  EXPECT_NEAR(at(0.5), 0.5 * p.config().weights.goal_position * 0.5 * 0.25, 1e-9);  // half the ramp, 0.5 m away
}

TEST(Problem, QpStepReducesAHullViolation)
{
  // A seed past a joint limit: the violated hull rows are elastic (their own slack) and the QP step moves back in.
  // shoulder_pan: no gravity torque, so the hull is the only violation.
  const auto p = problem();
  Joints past = pose("carry");
  past(0) = limits().q_max(0) + 0.05;
  const auto route = straight_route(0.0, 10.0);
  const mmd::Trajectory t = moving(p, state(0, 0, 0, past), 0.2);
  // The start itself is fixed: move it back inside from the second control point on (the hull rows exclude the start).
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world_of({}), 0.0, 1.0);
  const auto r = mmd::make_dense_solver("daqp")->solve(lin.qp, nullptr);
  ASSERT_TRUE(r.ok) << r.status;
  const mmd::Trajectory next = p.apply(t, r.x.head(lin.parameters));
  EXPECT_LT(p.violations(next, world_of({}), 0.0).hull, 0.5 * p.violations(t, world_of({}), 0.0).hull);
  EXPECT_GT(p.merit(t, task_on(route), world_of({}), 0.0), p.merit(next, task_on(route), world_of({}), 0.0));
}

TEST(Problem, LinearizeReportsItsSplit)
{
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const auto world = world_of({{"box", Eigen::Vector3d(2.2, 0.2, 1.0), Eigen::Vector3d(0.3, 2.0, 2.0), 0.4}});
  const mmd::MediumQp lin = p.linearize(moving(p, state(0.0, 0.0, 0.1, pose("carry")), 0.2), task_on(route), world, 0.0, 1.0);
  const auto & t = lin.timing;
  EXPECT_GT(t.collision_s, 0.0);
  EXPECT_GT(t.torque_s, 0.0);
  EXPECT_GE(t.cost_s, 0.0);
  EXPECT_GE(t.assemble_s, 0.0);
  EXPECT_GT(t.total_s, 0.0);
  EXPECT_LE(t.collision_s + t.torque_s + t.cost_s + t.assemble_s, t.total_s * 1.001 + 1e-6);
}

TEST(Problem, TrustRegionAndSlackBounds)
{
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const mmd::MediumQp lin = p.linearize(moving(p, state(0, 0, 0, pose("carry")), 0.3), task_on(route), world_of({}), 0.0, 0.25);
  EXPECT_EQ(lin.qp.H.rows(), lin.parameters + mmd::kSlacks);  // the parameter steps, then a slack per kind
  EXPECT_EQ(lin.parameters, p.coordinates() * p.basis().free_parameters());
  for (int i = 0; i < lin.parameters; ++i) {
    EXPECT_DOUBLE_EQ(lin.qp.lb(i), -0.25);
    EXPECT_DOUBLE_EQ(lin.qp.ub(i), 0.25);
  }
  for (int i = lin.parameters; i < lin.parameters + mmd::kSlacks; ++i) {
    EXPECT_DOUBLE_EQ(lin.qp.lb(i), 0.0);
    EXPECT_TRUE(std::isinf(lin.qp.ub(i)));
  }
  // H positive definite (DAQP factorizes it).
  EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(lin.qp.H).eigenvalues().minCoeff(), 0.0);
}

TEST(Problem, HullRowsBoundJointLimitsBetweenSamples)
{
  // The comfort pose beyond the elbow's upper limit pulls the QP step towards it; the hull rows stop the whole curve
  // at the limit, not only the samples.
  Joints beyond = pose("carry");
  beyond(2) = limits().q_max(2) + 1.0;
  mmd::MediumConfig c;
  c.weights.comfort = 1e4;
  const mmd::MediumProblem p(c, limits(), primitives(), mmm::CollisionConfig{}, statics(), beyond);
  Joints near = pose("carry");
  near(2) = limits().q_max(2) - 0.05;
  const auto route = straight_route(0.0, 10.0);
  const mmd::Trajectory t = moving(p, state(0, 0, 0, near), 0.0);
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world_of({}), 0.0, 2.0);
  const auto r = mmd::make_dense_solver("daqp")->solve(lin.qp, nullptr);
  ASSERT_TRUE(r.ok) << r.status;
  const mmd::Trajectory next = p.apply(t, r.x.head(lin.parameters));
  EXPECT_TRUE(p.hull_ok(next));
  const int elbow = static_cast<int>(std::find(p.joints().begin(), p.joints().end(), 2) - p.joints().begin());
  double highest = -1e9;
  for (int i = 0; i <= 4000; ++i)
    highest = std::max(highest, p.basis().basis(p.basis().horizon_s() * i / 4000.0, 0).dot(next.control_points.row(3 + elbow).transpose()));
  EXPECT_LE(highest, limits().q_max(2) + 1e-6);
  EXPECT_GT(highest, limits().q_max(2) - 0.05);  // it did move towards the limit
}

TEST(Problem, ViolatedHardRowsKeepTheQpFeasible)
{
  // The nominal already past a joint limit: dp = 0 with the slacks is feasible.
  const auto p = problem();
  Joints past = pose("carry");
  past(2) = limits().q_max(2) + 0.02;
  const auto route = straight_route(0.0, 10.0);
  const mmd::Trajectory t = moving(p, state(0, 0, 0, past), 0.2);
  EXPECT_FALSE(p.hull_ok(t));
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world_of({}), 0.0, 0.5);
  Eigen::VectorXd zero = Eigen::VectorXd::Constant(lin.parameters + mmd::kSlacks, 1e3);  // large slacks satisfy every
  zero.head(lin.parameters).setZero();                                                    // soft and violated hull row
  const Eigen::VectorXd Ax = lin.qp.A * zero;
  for (int i = 0; i < Ax.size(); ++i) {
    EXPECT_GE(Ax(i), lin.qp.lA(i) - 1e-9) << "row " << i;
    EXPECT_LE(Ax(i), lin.qp.uA(i) + 1e-9) << "row " << i;
  }
}

TEST(Problem, QpGradientPredictsTheMeritChange)
{
  // Away from hinge kinks and with no violation, the QP's linear term is the exact merit gradient (Gauss-Newton:
  // J^T r), so a small step changes the merit by g . dp to first order.
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0, 0.1);
  const auto world = world_of({{"box", Eigen::Vector3d(3.0, 0.9, 1.0), Eigen::Vector3d(0.3, 0.3, 2.0), 0.5}});
  // 0.2 m/s: well inside the speed rows (at the envelope's v_max a speed row sits on its bound, a kink of the merit).
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.05, pose("carry")), 0.2);
  ASSERT_LT(p.max_slack(t, world, 0.0), 1e-9);
  const mmd::MediumQp lin = p.linearize(t, task_on(route), world, 0.0, 1.0);
  std::mt19937 rng(4);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::VectorXd v(lin.parameters);
  for (int i = 0; i < v.size(); ++i) v(i) = g(rng);
  v.normalize();
  const double eps = 1e-4;  // above the GJK distance noise in the clearance term
  const double m0 = p.merit(t, task_on(route), world, 0.0);
  const double m1 = p.merit(p.apply(t, eps * v), task_on(route), world, 0.0);
  const double m2 = p.merit(p.apply(t, -eps * v), task_on(route), world, 0.0);
  const double fd = (m1 - m2) / (2 * eps);
  EXPECT_NEAR(lin.qp.g.head(lin.parameters).dot(v), fd, 1e-3 * std::max(1.0, std::abs(fd))) << "merit " << m0;
}
}  // namespace

TEST(Problem, FitRecoversATrajectoryFromItsSamples)
{
  // Samples of a representable trajectory (its yaw wrapped into (-pi, pi], as recorded states are) fit back to it:
  // the oracle seed of the bench (Task 9 CP) and any seed from recorded motion.
  const auto p = problem();
  std::mt19937 rng(5);
  std::normal_distribution<double> g(0.0, 0.3);
  Eigen::MatrixXd params(p.coordinates(), p.basis().free_parameters());
  for (int i = 0; i < params.size(); ++i) params(i) = g(rng);
  mmm::State start = state(1.0, -2.0, 3.0, pose("carry"));  // near +pi: the samples wrap
  Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
  rate(0) = 0.2;
  rate(2) = 0.3;
  const mmd::Trajectory truth = p.from_parameters(start, rate, params);
  std::vector<double> times;
  std::vector<mmm::State> states;
  for (double t = 0.0; t <= p.basis().horizon_s() + 1e-9; t += 0.02) {
    times.push_back(t);
    mmm::State x = p.state_at(truth, 0);
    const Eigen::VectorXd values = truth.control_points * p.basis().basis(t, 0).transpose();
    x.head<3>() = values.head<3>();
    x(2) = std::remainder(x(2), 2.0 * M_PI);
    const auto & names = p.config().arm_joints;
    for (size_t m = 0; m < names.size(); ++m) {
      const auto j = std::find(mmm::kArmJoints.begin(), mmm::kArmJoints.end(), names[m]) - mmm::kArmJoints.begin();
      x(3 + j) = values(3 + static_cast<int>(m));
    }
    states.push_back(x);
  }
  ASSERT_GT(std::abs(states.back()(2) - states.front()(2)), 1.0);  // wrapped somewhere along the way
  const mmd::Trajectory fitted = p.fit(start, rate, times, states);
  EXPECT_LT((fitted.control_points - truth.control_points).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(Problem, SpeedScheduleSlowsThroughATightPassage)
{
  // A 1.05 m door 1.6 m along a straight route (within a plan's reach); carry is wider than the door: the schedule is 1
  // at the start, the tight scale in the door and already below 1 before it (the window looks ahead); 1 everywhere
  // without obstacles.
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  auto post = [](const char * name, double y0, double y1) {
    return mmn::Box{name, Eigen::Vector3d(1.6, 0.5 * (y0 + y1), 1.0), Eigen::Vector3d(0.2, y1 - y0, 2.0), 0.0};
  };
  const auto world = world_of({post("left", 0.525, 3.0), post("right", -3.0, -0.525)});
  mmd::RouteTask task = task_on(route);
  const mmm::State measured = state(0.0, 0.0, 0.0, pose("carry"));
  p.schedule_speed(task, measured, world, 0.0);
  ASSERT_GT(task.speed_scale.size(), 16u);
  EXPECT_DOUBLE_EQ(task.scale_s0, 0.0);
  EXPECT_NEAR(p.speed_scale(task, 1.6), p.config().tight_speed_scale, 1e-12);
  EXPECT_LT(p.speed_scale(task, 1.2), 1.0);
  EXPECT_GT(p.speed_scale(task, 0.0), p.speed_scale(task, 1.6));
  EXPECT_DOUBLE_EQ(p.speed_scale(task, 100.0), task.speed_scale.back());
  mmd::RouteTask open = task_on(route);
  p.schedule_speed(open, measured, world_of({}), 0.0);
  for (const double v : open.speed_scale) EXPECT_DOUBLE_EQ(v, 1.0);
}

TEST(Problem, ProgressIgnoresTheSpeedSchedule)
{
  // The schedule shapes the seeds only: the cost's reference stays the full one (a lower reference made waiting cheap).
  const auto p = problem();
  const auto route = straight_route(0.0, 20.0);
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.0, pose("carry")), 0.1);
  mmd::RouteTask full = task_on(route), slow = task_on(route);
  slow.speed_scale.assign(200, 0.5);
  EXPECT_DOUBLE_EQ(p.cost_terms(t, full, world_of({}), 0.0).at("progress"), p.cost_terms(t, slow, world_of({}), 0.0).at("progress"));
}

TEST(Problem, SpeedBoundsStartFromTheMeasuredSpeed)
{
  // A start already reversing at 0.12 m/s (B5 creeps back, recorded): the forward-only bound reaches 0 at the braking
  // limit from it (and v_max from above), so the first samples ask only what the base can do; a plan that keeps
  // reversing still violates it by the full 0.12 m/s.
  const auto p = problem();
  const double v0 = -0.12, decel = p.limits().decel, vmax = p.limits().v_max;
  EXPECT_DOUBLE_EQ(p.speed_bounds(v0, 0.1).first, v0 + decel * 0.1);
  EXPECT_DOUBLE_EQ(p.speed_bounds(v0, 1.0).first, 0.0);
  EXPECT_DOUBLE_EQ(p.speed_bounds(v0, 0.1).second, vmax);
  EXPECT_DOUBLE_EQ(p.speed_bounds(vmax + 0.1, 0.1).second, vmax + 0.1 - decel * 0.1);
  EXPECT_DOUBLE_EQ(p.speed_bounds(0.1, 0.0).first, 0.0);
  const mmm::State start = state(0.0, 0.0, 0.0, pose("carry"));
  Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
  rate(0) = v0;
  const mmd::Trajectory reversing =
    p.from_parameters(start, rate, Eigen::MatrixXd::Zero(p.coordinates(), p.basis().free_parameters()));
  EXPECT_NEAR(p.violations(reversing, world_of({}), 0.0).speed_mps, -v0, 1e-6);
  // the rows: the first sample's lower row at min(0, v0 + decel t) - v
  const auto lin = p.linearize(reversing, task_on(straight_route(0.0, 10.0)), world_of({}), 0.0, 1.0);
  bool found = false;
  for (size_t i = 0; i < lin.rows.size(); ++i)
    if (lin.rows[i].kind == mmd::RowKind::Speed && lin.rows[i].sample == 1 && std::isfinite(lin.qp.lA(static_cast<int>(i)))) {
      EXPECT_NEAR(lin.qp.lA(static_cast<int>(i)), v0 + decel * p.sample_time(1) - v0, 1e-9);
      found = true;
    }
  EXPECT_TRUE(found);
}

TEST(Problem, LinearizeFromTheMeritsCollisionRowsIsTheSame)
{
  // The SCP's merit check at an accepted step computes the collision rows its next linearization needs: handed over,
  // they give the same QP as a fresh pass (one collision pass per accepted step instead of two).
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const auto world = world_of({{"box", Eigen::Vector3d(2.2, 0.2, 1.0), Eigen::Vector3d(0.3, 2.0, 2.0), 0.4}});
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.1, pose("carry")), 0.3);
  mmd::CollisionRows rows;
  mmd::Violations v;
  const double m = p.merit(t, task_on(route), world, 0.0, &v, &rows);
  EXPECT_DOUBLE_EQ(m, p.merit(t, task_on(route), world, 0.0));
  ASSERT_FALSE(rows.empty());
  const auto fresh = p.linearize(t, task_on(route), world, 0.0, 1.0);
  const auto reused = p.linearize(t, task_on(route), world, 0.0, 1.0, &rows);
  EXPECT_EQ(fresh.qp.H, reused.qp.H);
  EXPECT_EQ(fresh.qp.g, reused.qp.g);
  EXPECT_EQ(fresh.qp.A, reused.qp.A);
  EXPECT_EQ(fresh.qp.lA, reused.qp.lA);
  EXPECT_EQ(fresh.qp.uA, reused.qp.uA);
  EXPECT_EQ(fresh.rows.size(), reused.rows.size());
}

TEST(Problem, ClearanceIsTheSmallestDistanceBeyondTheMargins)
{
  // The plan's clearance: over its samples (after the start) and obstacle rows, the least distance beyond the row's
  // margin; negative inside a margin; +infinity with nothing within the activation distance.
  const auto p = problem();
  const auto world = world_of({{"box", Eigen::Vector3d(2.2, 0.2, 1.0), Eigen::Vector3d(0.3, 2.0, 2.0), 0.4}});
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.1, pose("carry")), 0.3);
  mmd::CollisionRows rows;
  p.merit(t, task_on(straight_route(0.0, 10.0)), world, 0.0, nullptr, &rows);
  double least = std::numeric_limits<double>::infinity();
  for (const auto & row : rows)
    if (row.stage > 0 && row.obstacle != "floor" && row.obstacle.rfind("self/", 0) != 0)
      least = std::min(least, row.distance - row.margin);
  ASSERT_TRUE(std::isfinite(least));
  EXPECT_DOUBLE_EQ(p.clearance(t, world, 0.0), least);
  EXPECT_TRUE(std::isinf(p.clearance(t, world_of({}), 0.0)));
}

TEST(MediumConfig, TheFourJointOptionLoads)
{
  // The comparison option (user decision 2026-10-30): pan, lift, elbow and wrist_2 with 8 segments, every other value
  // the default's.
  const mmd::MediumConfig four = mmd::load_medium_config(share("mobile_manipulator_medium") + "/config/medium_4joint.yaml");
  const mmd::MediumConfig six = mmd::load_medium_config(share("mobile_manipulator_medium") + "/config/medium.yaml");
  EXPECT_EQ(four.arm_joints,
            (std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_2_joint"}));
  EXPECT_EQ(four.bezier.segments, 8);
  EXPECT_EQ(four.scp.max_iter, six.scp.max_iter);
  EXPECT_DOUBLE_EQ(four.collision_dt, six.collision_dt);
  EXPECT_DOUBLE_EQ(four.solve_cap_s, six.solve_cap_s);
  EXPECT_EQ(four.seeds, six.seeds);
}

TEST(Problem, ComfortPullHoldsInFreeSpaceAndReleasesBeforeAConflict)
{
  // Task 12 tuning (user 2026-10-31): where the comfort pose has clearance on the route the arm is pulled to it with
  // weights.comfort_free on top of the weak comfort weight; ahead of a place where comfort would conflict (a door
  // narrower than the panel) the pull releases, so a swing there stays cheap. Without a schedule: no extra pull.
  mmd::MediumConfig c;
  c.weights.comfort_free = 2.0;
  const auto p = problem(c);
  const auto route = straight_route(0.0, 10.0);
  auto post = [](const char * name, double y0, double y1) {
    return mmn::Box{name, Eigen::Vector3d(3.0, 0.5 * (y0 + y1), 1.0), Eigen::Vector3d(0.2, y1 - y0, 2.0), 0.0};
  };
  // (a door 3 m ahead: carry's panel reaches about 0.7 m ahead of the base)
  const auto world = world_of({post("left", 0.525, 3.0), post("right", -3.0, -0.525)});
  mmd::RouteTask task = task_on(route);
  p.schedule_speed(task, state(0.0, 0.0, 0.0, pose("carry")), world, 0.0);
  ASSERT_FALSE(task.comfort_scale.empty());
  EXPECT_DOUBLE_EQ(p.comfort_scale(task, 0.0), 1.0);  // the door beyond the look-ahead from the start
  EXPECT_DOUBLE_EQ(p.comfort_scale(task, 1.9), 0.0);  // released ahead of the door (carry conflicts in it)
  EXPECT_DOUBLE_EQ(p.comfort_scale(task_on(route), 0.0), 0.0);  // no schedule: no extra pull
  // the pan held 0.4 rad off comfort costs more where the pull holds
  Joints off = pose("carry");
  off(0) += 0.4;
  const mmd::Trajectory t = moving(p, state(0.0, 0.0, 0.0, off), 0.0);
  mmd::RouteTask free_task = task_on(route), conflict_task = task_on(route);
  free_task.comfort_scale.assign(200, 1.0);
  conflict_task.comfort_scale.assign(200, 0.0);
  const double free_cost = p.cost_terms(t, free_task, world_of({}), 0.0).at("comfort");
  const double conflict_cost = p.cost_terms(t, conflict_task, world_of({}), 0.0).at("comfort");
  EXPECT_NEAR(free_cost / conflict_cost, (c.weights.comfort + 2.0) / c.weights.comfort, 1e-6);
}
