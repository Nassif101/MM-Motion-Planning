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
  EXPECT_EQ(c.bezier.segments, 4);
  EXPECT_DOUBLE_EQ(c.bezier.horizon_s, 8.0);
  EXPECT_EQ(c.bezier.continuity, 2);
  EXPECT_EQ(c.arm_joints, d.arm_joints);
  EXPECT_DOUBLE_EQ(c.collision_dt, 0.1);
  EXPECT_DOUBLE_EQ(c.base_margin_m, 0.12);
  EXPECT_DOUBLE_EQ(c.arm_margin_m, 0.05);
  EXPECT_DOUBLE_EQ(c.clearance_target_m, 0.25);
  EXPECT_DOUBLE_EQ(c.homotopy_hysteresis, 0.15);
  EXPECT_DOUBLE_EQ(c.solve_cap_s, 0.15);
  EXPECT_DOUBLE_EQ(c.full_multistart_period_s, 2.0);
  EXPECT_DOUBLE_EQ(c.sample_ds, 0.1);
  EXPECT_DOUBLE_EQ(c.slack_feasible_m, 0.001);
  EXPECT_EQ(c.scp.max_iter, 10);
  EXPECT_TRUE(std::isinf(c.torque_limits_nm(0)));
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(1), 24.4);
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(2), 41.3);
  EXPECT_DOUBLE_EQ(c.torque_limits_nm(4), 10.7);
  EXPECT_EQ(c.seeds, d.seeds);
  EXPECT_EQ(c.qp_backend, "hpipm");
  EXPECT_DOUBLE_EQ(c.torque_dt, 0.1);  // every sample (spec Section 3.3)
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
  const auto p = problem();
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
  const mmm::CollisionModel model(primitives(), cc);
  std::mt19937 rng(2);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::VectorXd v(lin.parameters);
  for (int i = 0; i < v.size(); ++i) v(i) = g(rng);
  v.normalize();
  const double h = 1e-4;  // the GJK distance is accurate to about 1e-9 m (as test_collision in mobile_manipulator_mpc)
  int checked = 0, close = 0;
  for (size_t i = 0; i < lin.rows.size(); ++i) {
    const auto & info = lin.rows[i];
    if (info.kind != mmd::RowKind::Collision) continue;
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
    EXPECT_DOUBLE_EQ(lin.qp.A(static_cast<int>(i), lin.parameters), 1.0);
    ++checked;
  }
  EXPECT_GT(checked, 20);
  EXPECT_GE(close, 0.9 * checked) << close << " of " << checked;
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
  const auto p = problem();
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
  EXPECT_NEAR(r.x(lin.parameters), p.violations(t, world_of({}), 0.0).torque, 1e-4);
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
  const auto p = problem();
  Joints past = pose("carry");
  past(2) = limits().q_max(2) + 0.05;
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

TEST(Problem, TrustRegionAndSlackBounds)
{
  const auto p = problem();
  const auto route = straight_route(0.0, 10.0);
  const mmd::MediumQp lin = p.linearize(moving(p, state(0, 0, 0, pose("carry")), 0.3), task_on(route), world_of({}), 0.0, 0.25);
  EXPECT_EQ(lin.qp.H.rows(), lin.parameters + 2);  // the parameter steps, the soft-row slack, the hull slack
  EXPECT_EQ(lin.parameters, p.coordinates() * p.basis().free_parameters());
  for (int i = 0; i < lin.parameters; ++i) {
    EXPECT_DOUBLE_EQ(lin.qp.lb(i), -0.25);
    EXPECT_DOUBLE_EQ(lin.qp.ub(i), 0.25);
  }
  for (int i = lin.parameters; i < lin.parameters + 2; ++i) {
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
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(lin.parameters + 2);
  zero(lin.parameters) = 1e3;      // a large slack satisfies every soft row ...
  zero(lin.parameters + 1) = 1e3;  // ... and the hull slack every violated hull row
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
