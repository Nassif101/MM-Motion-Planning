// The medium planner (Phase 5A Task 8, spec Section 3.5) on synthetic scenes: multi-start SCP, selection, homotopy
// hysteresis, the solve cap and the arc-length samples. The closed loops advance the measured state along each
// published plan by 0.2 s (a kinematic stand-in for B5 and the robot).
#include <gtest/gtest.h>

#include <cmath>

#include "mobile_manipulator_medium/planner.hpp"
#include "medium_support.hpp"

namespace
{
using namespace medium_test;

mmd::MediumConfig test_config()
{
  mmd::MediumConfig c;
  c.torque_dt = 0.4;    // fewer torque rows: these tests check behaviour, Task 9 the timing
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

TEST(Planner, BlockedPassagePublishesInfeasible)
{
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 10.0);
  pl.set_route(route, {10.0, 0.0, 0.0}, 1);
  const auto world = world_of({wall(0.9, 1.1, -3.0, 3.0)});
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
  // A symmetric 1.05 m gate 1.5 m ahead, the route 0.1 m off its centre: a tight pose is needed, and once chosen the
  // side does not switch over the following cycles.
  auto pl = planner();
  const mmm::Route route = straight_route(0.0, 6.0, 0.1);
  pl.set_route(route, {6.0, 0.1, 0.0}, 1);
  const auto world = world_of({wall(1.4, 1.6, 0.525, 3.0), wall(1.4, 1.6, -3.0, -0.525)});
  Loop loop{pl, state(0, 0.1, 0, pose("carry"))};
  std::string side;
  int switches = 0;
  for (int cycle = 0; cycle < 30; ++cycle) {
    const auto & out = loop.step(world);
    const std::string now = out.homotopy.rfind("left", 0) == 0 ? "left" : out.homotopy.rfind("right", 0) == 0 ? "right" : "";
    if (now.empty()) continue;
    if (!side.empty() && now != side) ++switches;
    side = now;
  }
  EXPECT_FALSE(side.empty()) << "never chose a side";
  EXPECT_EQ(switches, 0);
}
}  // namespace
