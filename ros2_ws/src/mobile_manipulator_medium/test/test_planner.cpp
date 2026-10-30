// The medium planner (Phase 5A Task 8, spec Section 3.5) on synthetic scenes: multi-start SCP, selection, homotopy
// hysteresis, the solve cap and the arc-length samples. The closed loops advance the measured state along each
// published plan by 0.2 s (a kinematic stand-in for B5 and the robot).
#include <gtest/gtest.h>
#include <set>

#include <algorithm>
#include <cmath>

#include "mobile_manipulator_medium/planner.hpp"
#include "mobile_manipulator_medium/scp.hpp"
#include "medium_support.hpp"

namespace
{
using namespace medium_test;

mmd::MediumConfig test_config()
{
  mmd::MediumConfig c;
  c.torque_dt = 0.48;   // fewer torque rows (every third sample): these tests check behaviour, Task 9 the timing
  c.solve_cap_s = 5.0;  // no cap in behaviour tests
  return c;
}

mmd::MediumPlanner planner(mmd::MediumConfig c = test_config())
{
  std::map<std::string, Joints> poses;
  for (const auto & item : payload().at("poses_rad").items()) poses[item.key()] = pose(item.key());
  const Eigen::Vector3d half = primitives().back().dims;  // the payload box, appended last (half extents)
  return mmd::MediumPlanner(problem(c), mmd::make_dense_solver(c.qp_backend, mmd::DenseQpOptions{c.qp_tolerance}),
                            poses, half);
}

mmn::Box wall(double x0, double x1, double y0, double y1, double z0 = 0.0, double z1 = 2.0)
{
  return {"wall", Eigen::Vector3d(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
          Eigen::Vector3d(x1 - x0, y1 - y0, z1 - z0), 0.0};
}

struct Loop
{
  mmd::MediumPlanner & p;
  mmm::State x;
  Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
  double now = 0.0;

  // One cycle: plan, then move to the plan's state 0.2 s ahead.
  const mmd::PlanOutput & step(const mmm::World & world)
  {
    const auto & out = p.plan(now, x, rate, world);
    const mmd::Trajectory * t = p.last_trajectory();
    x = p.problem().state_at(*t, 2);
    rate = p.problem().rate_at(*t, 2);
    now += 0.2;
    return out;
  }
};

TEST(Planner, ShortRouteEndsAtGoal)
{
  // 1.5 m to the goal: the reference speed tapers over the last metre (as B5's), so the plans arrive over several
  // horizons (receding); the robot settles at the goal without overshooting it, slowing down on the way in.
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 1.5);
  pl.set_route(route, {1.5, 0.0, 0.0}, 1);
  Loop loop{pl, state(0, 0, 0, pose("carry"))};
  double furthest = 0.0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    const auto & out = loop.step(world_of({}));
    ASSERT_EQ(out.status, "ok") << "cycle " << cycle;
    furthest = std::max(furthest, loop.x(0));
    for (size_t i = 1; i < out.samples.size(); ++i)
      // slowing down, but never planned below the breakaway before the goal (as B5)
      if (out.samples[i].x > 1.0)
        EXPECT_LE(out.samples[i].speed, std::max(out.samples[i - 1].speed, limits().breakaway.linear_mps) + 1e-3)
          << out.samples[i].x;
    EXPECT_LE(out.splice_s, route.length() + 1e-9);
  }
  EXPECT_LT(std::abs(loop.x(0) - 1.5), 0.03) << loop.x(0);
  EXPECT_LE(furthest, 1.52);
}

TEST(Planner, TurnInPlaceKeepsYawSamples)
{
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 0.6);
  pl.set_route(route, {0.6, 0.0, M_PI / 2}, 1);
  const auto & out = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  ASSERT_EQ(out.status, "ok");
  EXPECT_LT(std::abs(std::remainder(out.samples.back().yaw - M_PI / 2, 2 * M_PI)), 0.15) << out.samples.back().yaw;
  for (size_t i = 1; i < out.samples.size(); ++i)
    EXPECT_LE(std::abs(out.samples[i].yaw - out.samples[i - 1].yaw), 0.11) << "sample " << i;
  // The turn at the goal: several samples at about the same arc length.
  int at_goal = 0;
  for (const auto & s : out.samples) at_goal += std::abs(s.s - out.samples.back().s) < 0.02;
  EXPECT_GE(at_goal, 5);
}

TEST(Planner, SeedsMerge)
{
  auto c = test_config();
  c.seeds = {"previous", "carry", "carry"};
  auto pl = planner(c);
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  const auto & out = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  ASSERT_EQ(out.seeds.size(), 3u);  // hold, carry, carry
  EXPECT_TRUE(out.seeds[2].merged);
  EXPECT_EQ(out.homotopy, "comfort");
}

TEST(Planner, BlockedPassageNeverPlansThroughTheWall)
{
  // A wall across the route 0.9 m ahead: an ok plan stops short of it (spec Section 3.5); whether the SCP finds that
  // stop within its iterations from seeds that drive on (infeasible then, the lowest-slack plan) varies with the
  // representation and the limits, so either status passes; B5's blocked detector owns the dead end. The infeasible
  // path itself: StartInsideAMarginPublishesInfeasible.
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  const auto world = world_of({wall(0.9, 1.1, -3.0, 3.0)});
  const auto & out = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world);
  ASSERT_FALSE(out.samples.empty());
  EXPECT_TRUE(out.status == "ok" || out.status == "infeasible") << out.status;
  if (out.status == "ok")
    for (const auto & s : out.samples) EXPECT_LT(s.x, 0.9) << s.s;
  EXPECT_NO_THROW(pl.plan(0.2, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world));
}

TEST(Planner, StartInsideAMarginPublishesInfeasible)
{
  // The panel already inside the wall's margin at the start (the start state is fixed): no feasible plan exists, the
  // lowest-slack one is published as infeasible, and planning goes on.
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  const auto world = world_of({wall(0.62, 0.82, -3.0, 3.0)});
  const auto & out = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world);
  EXPECT_EQ(out.status, "infeasible");
  EXPECT_FALSE(out.samples.empty());
  double least = 1e9;
  for (const auto & s : out.seeds) least = std::min(least, s.slack);
  EXPECT_GT(least, 0.001);
  EXPECT_NO_THROW(pl.plan(0.2, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world));
}

TEST(Planner, CapKeepsLastValidPlan)
{
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  const auto first = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  ASSERT_EQ(first.status, "ok");
  pl.set_solve_cap_s(1e-9);
  const auto & second = pl.plan(0.2, state(0.05, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  EXPECT_EQ(second.status, "stale_reused");
  EXPECT_TRUE(second.capped);
  ASSERT_EQ(second.samples.size(), first.samples.size());
  EXPECT_DOUBLE_EQ(second.samples.back().x, first.samples.back().x);
  EXPECT_GT(second.plan_id, first.plan_id);
  EXPECT_DOUBLE_EQ(second.planned_at, first.planned_at);  // B5 ages the reused plan from its planning time
  EXPECT_DOUBLE_EQ(first.planned_at, 0.0);
}

TEST(Planner, FirstPlanCappedStillPublishes)
{
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  pl.set_solve_cap_s(1e-9);
  const auto & out = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  EXPECT_TRUE(out.capped);
  EXPECT_EQ(out.status, "infeasible");  // nothing solved: the incumbent seed as it is
  EXPECT_FALSE(out.samples.empty());
}

TEST(Planner, RouteUpdateKeepsTheIncumbent)
{
  // B5 announces every replan as a new route_id: the same goal keeps the published plan as the incumbent and the pose
  // seeds go on in turn (seeds_per_cycle; with 0 a full multi-start follows); a new goal starts over (every seed).
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  pl.set_route(straight_route(0.0, 10.0, 0.02), {10.0, 0.0, 0.0}, 2);
  const auto & updated = pl.plan(0.2, state(0.04, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  // The incumbent is a seed of this cycle (listed, possibly merged into a seed that found the same solution cheaper).
  auto seeded = [](const mmd::PlanOutput & out, const std::string & name) {
    return std::any_of(out.seeds.begin(), out.seeds.end(), [&](const mmd::SeedStats & s) { return s.name == name; });
  };
  EXPECT_EQ(updated.route_id, 2u);
  EXPECT_TRUE(seeded(updated, "previous"));
  EXPECT_FALSE(seeded(updated, "hold"));
  EXPECT_FALSE(updated.full_multistart);
  EXPECT_EQ(updated.seeds.size(), 2u);
  pl.set_route(straight_route(0.0, 5.0), {5.0, 0.0, 0.0}, 3);
  const auto & fresh = pl.plan(0.4, state(0.08, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  EXPECT_TRUE(seeded(fresh, "hold"));
  EXPECT_FALSE(seeded(fresh, "previous"));
  EXPECT_TRUE(fresh.full_multistart);
  auto c = test_config();
  c.seeds_per_cycle = 0;
  auto periodic = planner(c);
  periodic.set_route(route, {10.0, 0.0, 0.0}, 1);
  periodic.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  periodic.set_route(straight_route(0.0, 10.0, 0.02), {10.0, 0.0, 0.0}, 2);
  EXPECT_TRUE(periodic.plan(0.2, state(0.04, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({})).full_multistart);
}

TEST(Planner, PreviousSeedSurvivesTheYawWrap)
{
  // Heading west the measured yaw wraps from +pi to -pi between two cycles: the carried plan follows the measured yaw
  // instead of spinning a whole turn.
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, -10.0);
  pl.set_route(route, {-10.0, 0.0, M_PI}, 1);
  pl.plan(0.0, state(0, 0, 3.12, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  const double wrapped = 3.15 - 2 * M_PI;
  const auto & out = pl.plan(0.2, state(-0.04, 0, wrapped, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  // the carried plan is a seed of the cycle (in either slot: a better seed it merges with takes the slot, SeedsMerge)
  const auto previous = std::find_if(out.seeds.begin(), out.seeds.end(), [](const auto & s) { return s.name == "previous"; });
  ASSERT_NE(previous, out.seeds.end());
  EXPECT_TRUE(previous->feasible);
  for (const auto & sample : out.samples) EXPECT_LT(std::abs(sample.yaw - wrapped), 0.3) << sample.yaw;
}

TEST(Planner, PredictedDecreaseCountsTheStartPenalty)
{
  // From a start inside an obstacle's margin the QP's model decrease includes the penalty the slack carries at dp = 0,
  // so it is positive and of the size of the merit's actual decrease for a small step.
  auto pl = planner();
  const auto & p = pl.problem();
  const mmm::Route route = straight_route(0.0, 10.0);
  mmd::RouteTask task;
  task.route = &route;
  task.goal_s = route.length();
  // In front of the panel at the start; the robot turned 0.2 rad (with the panel's edge parallel to the wall the
  // distance has a kink in yaw, where the linearization's one-sided gradient cannot predict a step).
  const auto world = world_of({wall(0.55, 0.75, -2.0, 2.0)});
  const mmd::Trajectory t =
    p.from_parameters(state(0, 0, 0.2, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(),
                      Eigen::MatrixXd::Zero(p.coordinates(), p.basis().free_parameters()));
  mmd::Violations at_start;
  const double m0 = p.merit(t, task, world, 0.0, &at_start);
  ASSERT_GT(at_start.soft(), 0.001);
  const mmd::MediumQp lin = p.linearize(t, task, world, 0.0, 0.01);
  const auto r = mmd::make_dense_solver("daqp")->solve(lin.qp, nullptr);
  ASSERT_TRUE(r.ok) << r.status;
  const double predicted = mmd::predicted_decrease(lin, r.x, at_start, p.config().weights);
  const double actual = m0 - p.merit(p.apply(t, r.x.head(lin.parameters)), task, world, 0.0);
  EXPECT_GT(predicted, 0.0);
  EXPECT_NEAR(actual / predicted, 1.0, 0.3);
}

TEST(Planner, LowBeamLowersWithinTorque)
{
  // A 1.23 m beam (bottom) across a straight route; carry's panel top is 1.32 m. The plan lowers the panel (fold_back,
  // top 1.18 m) within the torque limits and keeps the arm margin under the beam.
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 8.0);
  pl.set_route(route, {8.0, 0.0, 0.0}, 1);
  const auto world = world_of({wall(1.4, 1.6, -2.0, 2.0, 1.23, 1.43)});
  Loop loop{pl, state(0, 0, 0, pose("carry"))};
  const Joints limits = pl.problem().config().torque_limits_nm;
  mmm::CollisionConfig cc;
  const mmm::CollisionModel model(primitives(), cc);
  bool lowered = false;
  for (int cycle = 0; cycle < 120 && loop.x(0) < 2.5; ++cycle) {
    const auto & out = loop.step(world);
    lowered = lowered || out.homotopy.find("lowered") != std::string::npos;
    const Joints tau = statics()->torque(loop.x.tail<6>());
    for (int j = 1; j < 6; ++j) EXPECT_LE(std::abs(tau(j)), limits(j) * 1.001) << "cycle " << cycle << " joint " << j;
    const auto nearest = model.nearest(loop.x, world.at(0.0, 0.0));
    if (nearest) EXPECT_GE(nearest->distance, 0.045) << "cycle " << cycle << " " << out.homotopy;
  }
  EXPECT_TRUE(lowered);
  EXPECT_GT(loop.x(0), 2.5) << "did not pass the beam";
}

TEST(Planner, AngledDoorEntersAligned)
{
  // A 1.05 m door 1.2 m up a route that turns north 0.2 m ahead of the robot; carry is wider than the door. The base
  // crosses the door plane within 5 degrees of the door axis, with the margins kept.
  auto pl = planner();
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  pl.set_route(route, {3.0, 5.0, M_PI / 2}, 1);
  const auto world = world_of({wall(0.5, 2.475, 1.1, 1.3), wall(3.525, 5.5, 1.1, 1.3)});
  Loop loop{pl, state(2.8, 0, 0, pose("carry"))};
  mmm::CollisionConfig cc;
  const mmm::CollisionModel model(primitives(), cc);
  double yaw_at_door = std::nan("");
  for (int cycle = 0; cycle < 80 && loop.x(1) < 2.5; ++cycle) {
    const double y_before = loop.x(1);
    const auto & out = loop.step(world);
    if (y_before < 1.2 && loop.x(1) >= 1.2) yaw_at_door = loop.x(2);
    const auto nearest = model.nearest(loop.x, world.at(0.0, 0.0));
    if (nearest) EXPECT_GE(nearest->distance, 0.045) << "cycle " << cycle << " " << out.homotopy;
  }
  ASSERT_FALSE(std::isnan(yaw_at_door)) << "never crossed the door";
  EXPECT_LT(std::abs(std::remainder(yaw_at_door - M_PI / 2, 2 * M_PI)), 5.0 * M_PI / 180.0) << yaw_at_door;
}

TEST(Planner, SymmetricGateKeepsSide)
{
  // A 1.05 m gate 1.5 m ahead with a shelf along the left of the approach at panel height (v2_gate_right_swing_nav's
  // layout): a tight pose is needed, the right swing is the one the shelf leaves room for, and once chosen the side
  // does not flip over the following cycles (homotopy_switched counts side flips).
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 6.0);
  pl.set_route(route, {6.0, 0.0, 0.0}, 1);
  const auto world = world_of({wall(1.4, 1.6, 0.525, 3.0), wall(1.4, 1.6, -3.0, -0.525),
                               wall(0.3, 1.4, 0.4, 1.4, 0.75, 2.0)});
  Loop loop{pl, state(0, 0, 0, pose("carry"))};
  std::string side;
  int switches = 0;
  for (int cycle = 0; cycle < 30; ++cycle) {
    const auto & out = loop.step(world);
    switches += out.homotopy_switched;
    const std::string now = out.homotopy.rfind("left", 0) == 0 ? "left" : out.homotopy.rfind("right", 0) == 0 ? "right" : "";
    if (side.empty() && !now.empty()) side = now;
  }
  EXPECT_EQ(side, "right");
  EXPECT_EQ(switches, 0);
}
}  // namespace

namespace
{
std::vector<double> every(double dt, double horizon)
{
  std::vector<double> t;
  for (double s = 0.0; s <= horizon + 1e-9; s += dt) t.push_back(s);
  return t;
}
}  // namespace

TEST(PacedRoute, StraightRampsWithinTheLimitsAndStopsAtTheGoal)
{
  const auto & lim = limits();
  const mmm::Route route = straight_route(0.0, 2.0);
  const auto times = every(0.1, 16.0);
  const auto poses = mmd::paced_route(route, 0.0, 2.0, state(0, 0, 0, pose("carry")), 0.0, 0.0, [](double) { return 0.3; }, lim, times);
  ASSERT_EQ(poses.size(), times.size());
  double v_prev = 0.0, top = 0.0;
  for (size_t k = 1; k < poses.size(); ++k) {
    const double v = (poses[k] - poses[k - 1]).head<2>().norm() / 0.1;
    EXPECT_LE(v, 0.3 + 1e-6) << k;
    EXPECT_LE(v - v_prev, lim.accel * 0.1 + 0.02) << k;   // ramps up within the acceleration (sampling slack)
    EXPECT_LE(v_prev - v, lim.decel * 0.1 + 0.02) << k;   // and down within the deceleration
    EXPECT_LE(poses[k](0), 2.0 + 1e-9) << k;              // never past the goal
    top = std::max(top, v);
    v_prev = v;
  }
  EXPECT_GT(top, 0.29);
  EXPECT_NEAR(poses.back()(0), 2.0, 1e-3);
}

TEST(PacedRoute, TurnsInPlaceAtASharpCorner)
{
  // A 90 degree corner 1 m ahead: the base slows to a stop at the corner, turns within omega_max, then goes on along
  // the second leg.
  const auto & lim = limits();
  const mmm::Route route({Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 2.0)});
  const auto times = every(0.1, 16.0);
  const auto poses = mmd::paced_route(route, 0.0, route.length(), state(0, 0, 0, pose("carry")), 0.0, 0.0, [](double) { return 0.3; }, lim, times);
  double slowest_near_corner = 1e9;
  for (size_t k = 1; k < poses.size(); ++k) {
    const double v = (poses[k] - poses[k - 1]).head<2>().norm() / 0.1;
    const double w = std::abs(poses[k](2) - poses[k - 1](2)) / 0.1;
    EXPECT_LE(w, lim.omega_max + 1e-6) << k;
    if ((poses[k].head<2>() - Eigen::Vector2d(1.0, 0.0)).norm() < 0.05) slowest_near_corner = std::min(slowest_near_corner, v);
  }
  EXPECT_LT(slowest_near_corner, 0.03);
  EXPECT_NEAR(poses.back()(2), M_PI / 2, 0.05);
  EXPECT_GT(poses.back()(1), 1.0);  // well along the second leg
}

TEST(PacedRoute, WaitsUntilAlignedBeforeDriving)
{
  // Starting 1.2 rad off the route's heading: the base turns before it drives.
  const auto & lim = limits();
  const mmm::Route route = straight_route(0.0, 3.0);
  const auto times = every(0.1, 8.0);
  const auto poses = mmd::paced_route(route, 0.0, 3.0, state(0, 0, 1.2, pose("carry")), 0.0, 0.0, [](double) { return 0.3; }, lim, times);
  for (size_t k = 0; k < poses.size(); ++k)
    if (std::abs(poses[k](2)) > 0.6) EXPECT_LT(poses[k](0), 0.02) << k;  // hardly any progress while far off
  EXPECT_NEAR(poses.back()(2), 0.0, 0.05);
  EXPECT_GT(poses.back()(0), 0.5);
}

TEST(PacedRoute, FollowsAReferenceThatChangesAlongTheRoute)
{
  // 0.3 m/s for the first metre, 0.1 m/s beyond (a tight passage): the base is down to the lower speed once inside.
  const auto & lim = limits();
  const mmm::Route route = straight_route(0.0, 5.0);
  const auto times = every(0.1, 16.0);
  const auto poses = mmd::paced_route(route, 0.0, 5.0, state(0, 0, 0, pose("carry")), 0.0, 0.0,
                                      [](double s) { return s < 1.0 ? 0.3 : 0.1; }, lim, times);
  double top = 0.0;
  for (size_t k = 1; k < poses.size(); ++k) {
    const double v = (poses[k] - poses[k - 1]).head<2>().norm() / 0.1;
    if (poses[k - 1](0) < 0.9) top = std::max(top, v);
    if (poses[k - 1](0) > 1.3) EXPECT_LE(v, 0.1 + 1e-6) << k;
  }
  EXPECT_GT(top, 0.25);
}

TEST(Planner, PoseSeedsRotateBesideTheIncumbent)
{
  // seeds_per_cycle 1: a new route solves every seed; then each cycle the incumbent and the next pose seed in turn
  // (every pose seed once per len(seeds) - 1 cycles), with no periodic full multi-start; 0 keeps the periodic one.
  auto c = test_config();
  ASSERT_EQ(c.seeds_per_cycle, 1);
  auto pl = planner(c);
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  std::vector<std::string> poses;
  for (const auto & name : c.seeds)
    if (name != "previous") poses.push_back(name);
  Loop loop{pl, state(0, 0, 0, pose("carry"))};
  const auto & first = loop.step(world_of({}));
  EXPECT_TRUE(first.full_multistart);
  EXPECT_EQ(first.seeds.size(), poses.size() + 1);
  for (size_t cycle = 1; cycle <= 2 * poses.size() + 3; ++cycle) {  // past full_multistart_period_s (2 s at 0.2 s)
    const auto & out = loop.step(world_of({}));
    EXPECT_FALSE(out.full_multistart) << cycle;
    ASSERT_EQ(out.seeds.size(), 2u) << cycle;
    // (in either slot: a seed merging with a better one swaps slots, SeedsMerge)
    const std::set<std::string> names{out.seeds[0].name, out.seeds[1].name};
    EXPECT_EQ(names, (std::set<std::string>{"previous", poses[(cycle - 1) % poses.size()]})) << cycle;
  }
  c.seeds_per_cycle = 0;
  auto periodic = planner(c);
  periodic.set_route(route, {10.0, 0.0, 0.0}, 1);
  Loop old{periodic, state(0, 0, 0, pose("carry"))};
  int fulls = 0;
  for (int cycle = 0; cycle < 12; ++cycle) {
    const auto & out = old.step(world_of({}));
    fulls += out.full_multistart;
    EXPECT_EQ(out.seeds.size(), out.full_multistart ? poses.size() + 1 : 1u) << cycle;
  }
  EXPECT_EQ(fulls, 2);  // the first and the one 2 s later
}

TEST(PacedRoute, StartsFromTheMeasuredSpeedAndYawRate)
{
  // Moving at 0.24 m/s and turning at 0.3 rad/s where the reference is 0.12 m/s straight on: the speed comes down
  // within the deceleration and the turn rate within the yaw acceleration, from the measured values (the fit pins them).
  const auto & lim = limits();
  const mmm::Route route = straight_route(0.0, 5.0);
  const auto times = every(0.02, 2.0);
  const auto poses = mmd::paced_route(route, 0.0, 5.0, state(0, 0, 0, pose("carry")), 0.24, 0.3,
                                      [](double) { return 0.12; }, lim, times);
  const double v1 = (poses[1] - poses[0]).head<2>().norm() / 0.02;
  EXPECT_GT(v1, 0.24 - lim.decel * 0.02 - 0.005);
  EXPECT_GT(poses[1](2), 0.3 * 0.02 - 0.5 * lim.yaw_accel * 0.02 * 0.02 - 1e-4);  // still turning the measured way
  const double v_late = (poses.back() - poses[poses.size() - 2]).head<2>().norm() / 0.02;
  EXPECT_NEAR(v_late, 0.12, 0.01);
}

TEST(Planner, WristSeedsOffsetAQualifiedPose)
{
  // A seed "pose+joint:offset" is the qualified pose with that joint moved (seeds only, nothing qualified): with the
  // 6-joint mask, carry with the panel rolled (wrist_3) or pitched (wrist_1). The joint must be one the plan moves.
  auto c = test_config();
  c.arm_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  c.seeds = {"previous", "carry+wrist_3_joint:0.5", "carry+wrist_1_joint:-0.3", "vertical_carry"};
  auto pl = planner(c);
  Joints rolled = pose("carry");
  rolled(5) += 0.5;
  EXPECT_EQ(pl.seed_pose("carry+wrist_3_joint:0.5"), rolled);
  Joints pitched = pose("carry");
  pitched(3) -= 0.3;
  EXPECT_EQ(pl.seed_pose("carry+wrist_1_joint:-0.3"), pitched);
  EXPECT_EQ(pl.seed_pose("vertical_carry"), pose("vertical_carry"));
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  const auto & out = pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  EXPECT_EQ(out.seeds.size(), 4u);  // hold and the three
  auto four = test_config();  // the 4-joint mask does not move wrist_3: the offset would do nothing
  four.arm_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_2_joint"};
  four.seeds = {"previous", "carry+wrist_3_joint:0.5"};
  EXPECT_THROW(planner(four), std::invalid_argument);
  for (const char * bad : {"carry+wrist_3_joint", "nosuch+wrist_3_joint:0.5", "carry+nosuch_joint:0.5", "carry+wrist_3_joint:x"}) {
    c.seeds = {"previous", bad};
    EXPECT_THROW(planner(c), std::invalid_argument) << bad;
  }
}

TEST(Planner, AnInfeasibleIncumbentWidensTheCycleToEverySeed)
{
  // Pose seeds in turn: when the incumbent's solution turns infeasible (the last plan was ok) the cycle also solves every
  // other pose seed (one seed in turn could be the wrong side at a gate: SymmetricGateKeepsSide with 6 joints); while
  // it stays infeasible (a start inside a margin, a blocked route) the seeds go on in turn.
  auto c = test_config();
  auto pl = planner(c);
  std::vector<std::string> poses;
  for (const auto & name : c.seeds)
    if (name != "previous") poses.push_back(name);
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  pl.plan(0.0, state(0, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  const auto & calm = pl.plan(0.2, state(0.02, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), world_of({}));
  EXPECT_EQ(calm.seeds.size(), 2u);
  const auto blocked = world_of({wall(0.62, 0.82, -3.0, 3.0)});  // inside the panel's margin already
  const auto & out = pl.plan(0.4, state(0.04, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), blocked);
  EXPECT_EQ(out.status, "infeasible");
  EXPECT_EQ(out.seeds.size(), poses.size() + 1);
  EXPECT_FALSE(out.full_multistart);  // not a full multi-start: the rotation is not reset
  const auto & still = pl.plan(0.6, state(0.04, 0, 0, pose("carry")), Eigen::Matrix<double, 9, 1>::Zero(), blocked);
  EXPECT_EQ(still.status, "infeasible");
  EXPECT_EQ(still.seeds.size(), 2u);
}
