// The closed-loop kinematic bench (Phase 5A spec Sections 6.2 and 10, plan Task 12): B5 at 50 Hz on the kinematic plant
// following the medium planner at 5 Hz, on the scripted integration cells.
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "medium_support.hpp"
#include "mobile_manipulator_medium/closed_loop.hpp"
#include "mobile_manipulator_mpc/references.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/scene.hpp"
#include "pre_refactor_closed_loop.inc"

namespace
{
using namespace medium_test;

mmn::Box wall(double x0, double x1, double y0, double y1, double z0 = 0.0, double z1 = 2.0)
{
  return {"wall", Eigen::Vector3d(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
          Eigen::Vector3d(x1 - x0, y1 - y0, z1 - z0), 0.0};
}

struct Rig
{
  explicit Rig(mmd::MediumConfig config = {}) : medium_config(std::move(config)) { make_planner(); }

  mmm::MpcConfig b5_config = mmm::load_mpc_config(share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml");
  std::unique_ptr<mmm::Controller> b5 =
    mmm::build_controller(b5_config, limits(), medium_test::urdf(), payload(), 50.0, false);
  mmm::PostureTargets posture =
    mmm::posture_targets(payload(), b5_config.comfort_pose, b5_config.posture_mode, b5_config.tight_poses);
  mmd::MediumConfig medium_config;
  std::unique_ptr<mmd::MediumPlanner> medium;

  void make_planner()
  {
    std::map<std::string, Joints> poses;
    for (const auto & item : payload().at("poses_rad").items()) poses[item.key()] = pose(item.key());
    medium = std::make_unique<mmd::MediumPlanner>(
      problem(medium_config),
      mmd::make_dense_solver(medium_config.qp_backend, mmd::DenseQpOptions{medium_config.qp_tolerance}), poses,
      primitives().back().dims);
  }

  mmd::ClosedLoopResult run(const mmm::Route & route, const mmn::Pose2 & goal, const mmm::State & start,
                            const mmm::World & world, double timeout_s, double latency_s = 0.14,
                            double route_update_period_s = 0.0)
  {
    // deterministic: no cap and a fixed plan latency (the cap), whatever the host's load
    medium->set_solve_cap_s(10.0);
    mmd::ClosedLoopOptions o;
    o.timeout_s = timeout_s;
    o.fixed_latency_s = latency_s;
    o.route_update_period_s = route_update_period_s;
    return mmd::run_closed_loop(*b5, *medium, b5_config, posture, route, goal, start, world, primitives(), o);
  }
};

void expect_clean(const mmd::ClosedLoopResult & r)
{
  EXPECT_TRUE(r.reached) << "ended " << r.end_status << " after " << r.time_s << " s at (" << r.final_state(0) << ", "
                         << r.final_state(1) << ", " << r.final_state(2) << ")";
  EXPECT_EQ(r.executed_slack_plans, 0) << "largest executed slack " << r.max_executed_slack_m;
  EXPECT_GE(r.min_clearance_m, 0.045);
  EXPECT_GT(r.following_cycles, 0.9 * r.b5_cycles) << "fallbacks: " << r.fallback_summary();
  EXPECT_EQ(r.homotopy_switches_after_first, 0);
}
}  // namespace

TEST(ClosedLoop, AngledDoorReachesGoal)
{
  // The planner's angled 1.05 m door (Planner.AngledDoorEntersAligned): the route turns north 0.2 m ahead; carry is
  // wider than the door. B5 follows the medium plan through it to the goal, margins kept.
  Rig rig;
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  const auto r = rig.run(route, {3.0, 5.0, M_PI / 2}, state(2.8, 0.0, 0.0, pose("carry")),
                         world_of({wall(0.5, 2.475, 1.1, 1.3), wall(3.525, 5.5, 1.1, 1.3)}), 90.0);
  expect_clean(r);
}

TEST(ClosedLoop, SameGoalReplansKeepTheMediumPlan)
{
  // Nav2 replans the same goal about every 2.5 s near store_room's door S, and B5 announces each replan as a new
  // route_id (final review I1): until a plan for the new route arrives B5 fell back to the frozen B5 with stale
  // posture state, at the angled door too. The published plan now carries on under the new route_id at once, so
  // B5 falls back at most the one cycle the carried plan takes to arrive.
  Rig rig;
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  const auto r = rig.run(route, {3.0, 5.0, M_PI / 2}, state(2.8, 0.0, 0.0, pose("carry")),
                         world_of({wall(0.5, 2.475, 1.1, 1.3), wall(3.525, 5.5, 1.1, 1.3)}), 90.0, 0.14, 1.0);
  expect_clean(r);
  ASSERT_GT(r.route_updates, 5);
  const auto it = r.fallbacks.find("route_id");
  EXPECT_LE(it == r.fallbacks.end() ? 0 : it->second, r.route_updates) << "fallbacks: " << r.fallback_summary();
}

TEST(ClosedLoop, LowBeamReachesGoal)
{
  // A 1.23 m beam across a straight route (carry's panel top 1.32 m): the plan lowers the panel under it within the
  // torque limits; B5 follows to the goal.
  Rig rig;
  const mmm::Route route = straight_route(0.0, 6.0);
  const auto r = rig.run(route, {6.0, 0.0, 0.0}, state(0.0, 0.0, 0.0, pose("carry")),
                         world_of({wall(2.4, 2.6, -2.0, 2.0, 1.23, 1.43)}), 90.0);
  expect_clean(r);
  EXPECT_LE(r.max_torque_ratio, 1.0 + 1e-3);
}

TEST(ClosedLoop, SideTrackerCountsFlipsThroughSidelessLabels)
{
  // Task 12 review: the planner's homotopy_switched compares a plan with the last one only, so right -> lowered -> left
  // was no switch; the tracker keeps the last plan that had a side.
  mmd::SideTracker sides;
  for (const char * h : {"comfort", "right", "right+lowered", "lowered", "comfort", "left", "left", "right"})
    sides.observe(h);
  EXPECT_EQ(sides.first, "right");
  EXPECT_EQ(sides.switches, 2);  // right -> (lowered, comfort) -> left, left -> right
  EXPECT_EQ(sides.sequence, (std::vector<std::string>{"right", "left", "right"}));
}

TEST(ClosedLoop, DoorBOnTheHostProfileAtTheHotBudget)
{
  // v2_door_105_nav on the site V2 scene and its recorded Nav2 route (campaign 2, 2026-10-24) with the installed
  // medium.yaml (this host's 2.5 Hz profile), the cycles cut at 30 SCP iterations (the hot fanless host's compute,
  // live campaign A) and the plans' latency at the cap, B5's plan-age limit as navigation.launch.py derives it: the
  // same on any host. Live campaign A at 5 Hz stopped in front of door B (or in it).
  Rig rig(mmd::load_medium_config(share("mobile_manipulator_medium") + "/config/medium.yaml"));
  rig.b5_config.medium_stale_s = 1.0 / rig.medium_config.rate_hz + rig.medium_config.solve_cap_s + 0.16;
  rig.b5 = mmm::build_controller(rig.b5_config, limits(), medium_test::urdf(), payload(), 50.0, false);
  const mmm::Scene scene = mmm::load_scene(share("mobile_manipulator_navigation"), "", "v2_door_105_nav", "");
  mmm::World world;
  world.set_static(scene.boxes);
  const mmm::Route route({{12.000, 0.000}, {12.053, 0.003}, {12.106, 0.011}, {12.158, 0.026}, {13.550, 0.700},
                          {14.899, 1.375}, {15.140, 1.504}, {15.228, 1.560}, {15.309, 1.623}, {16.021, 2.326},
                          {16.088, 2.403}, {16.119, 2.442}, {16.146, 2.483}, {16.171, 2.525}, {16.191, 2.568},
                          {16.208, 2.613}, {16.222, 2.659}, {16.232, 2.706}, {16.239, 2.754}, {16.250, 4.300},
                          {16.250, 5.800}, {16.256, 7.346}, {16.269, 8.854}, {16.275, 9.500}});
  rig.medium->set_iteration_budget(30);
  const auto r = rig.run(route, {16.275, 9.5, M_PI / 2}, state(12.0, 0.0, 0.0, pose("carry")), world, 150.0,
                         rig.medium_config.solve_cap_s);
  expect_clean(r);
}


TEST(ClosedLoop, StepperMatchesThePreRefactorLoop)
{
  // the stepper against the loop as it was before the refactor (pre_refactor_closed_loop.inc, git f694544^), bit for
  // bit: the run_closed_loop of today runs the stepper, so StepperMatchesTheLoop alone compares it with itself
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  const mmn::Pose2 goal{3.0, 5.0, M_PI / 2};
  const mmm::State start = state(2.8, 0.0, 0.0, pose("carry"));
  const mmm::World world = world_of({wall(0.5, 2.475, 1.1, 1.3), wall(3.525, 5.5, 1.1, 1.3)});
  mmd::ClosedLoopOptions o;
  o.timeout_s = 90.0;
  o.fixed_latency_s = 0.14;
  Rig a;
  a.medium->set_solve_cap_s(10.0);
  const auto before = pre_refactor::run_closed_loop(*a.b5, *a.medium, a.b5_config, a.posture, route, goal, start, world,
                                                    primitives(), o);
  Rig b;
  b.medium->set_solve_cap_s(10.0);
  mmm::KinematicClosedLoop plant(start, b.medium->problem().limits().breakaway);
  mmd::ClosedLoopStepper stepper(*b.b5, *b.medium, b.b5_config, b.posture, route, goal, plant, world, primitives(), o);
  while (stepper.step()) {
  }
  const auto & now = stepper.result();
  EXPECT_EQ(now.reached, before.reached);
  EXPECT_EQ(now.end_status, before.end_status);
  EXPECT_EQ(now.b5_cycles, before.b5_cycles);
  EXPECT_EQ(now.medium_cycles, before.medium_cycles);
  EXPECT_EQ(now.time_s, before.time_s);
  EXPECT_EQ(now.final_state, before.final_state);  // bit for bit
  EXPECT_EQ(now.min_clearance_m, before.min_clearance_m);
}

TEST(ClosedLoop, StepperMatchesTheLoop)
{
  // The loop one B5 cycle at a time (the MuJoCo plant's session drives it so, MuJoCo plant plan Task 8): the same run,
  // bit for bit, as run_closed_loop
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  const mmn::Pose2 goal{3.0, 5.0, M_PI / 2};
  const mmm::State start = state(2.8, 0.0, 0.0, pose("carry"));
  const mmm::World world = world_of({wall(0.5, 2.475, 1.1, 1.3), wall(3.525, 5.5, 1.1, 1.3)});
  Rig a;
  const auto whole = a.run(route, goal, start, world, 90.0);
  Rig b;
  b.medium->set_solve_cap_s(10.0);
  mmd::ClosedLoopOptions o;
  o.timeout_s = 90.0;
  o.fixed_latency_s = 0.14;
  mmm::KinematicClosedLoop plant(start, b.medium->problem().limits().breakaway);
  mmd::ClosedLoopStepper stepper(*b.b5, *b.medium, b.b5_config, b.posture, route, goal, plant, world, primitives(), o);
  int cycles = 0;
  while (stepper.step()) {
    ++cycles;
    EXPECT_FALSE(stepper.done());
  }
  EXPECT_TRUE(stepper.done());
  EXPECT_FALSE(stepper.step());  // ended: no more cycles
  const auto & r = stepper.result();
  EXPECT_EQ(r.reached, whole.reached);
  EXPECT_EQ(r.end_status, whole.end_status);
  EXPECT_EQ(r.time_s, whole.time_s);
  EXPECT_EQ(r.b5_cycles, whole.b5_cycles);
  EXPECT_EQ(r.following_cycles, whole.following_cycles);
  EXPECT_EQ(r.final_state, whole.final_state);
  EXPECT_EQ(r.min_clearance_m, whole.min_clearance_m);
  EXPECT_EQ(cycles + 1, whole.b5_cycles);  // the last cycle ends the run
}

TEST(ClosedLoop, StepperContinuesFromItsStartTime)
{
  // a mission's next goal starts at the plant's clock (its commands are stamped with it): the cycles run from
  // start_time_s and the timeout counts from there
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  const mmm::State start = state(2.8, 0.0, 0.0, pose("carry"));
  const mmm::World world = world_of({wall(0.5, 2.475, 1.1, 1.3), wall(3.525, 5.5, 1.1, 1.3)});
  Rig rig;
  rig.medium->set_solve_cap_s(10.0);
  mmd::ClosedLoopOptions o;
  o.timeout_s = 90.0;
  o.fixed_latency_s = 0.14;
  o.start_time_s = 40.0;
  mmm::KinematicClosedLoop plant(start, rig.medium->problem().limits().breakaway);
  mmd::ClosedLoopStepper stepper(*rig.b5, *rig.medium, rig.b5_config, rig.posture, route, {3.0, 5.0, M_PI / 2}, plant, world,
                                 primitives(), o);
  EXPECT_DOUBLE_EQ(stepper.time(), 40.0);
  while (stepper.step()) {
  }
  EXPECT_TRUE(stepper.result().reached) << stepper.result().end_status;
  EXPECT_GT(stepper.result().time_s, 40.0);
  EXPECT_LT(stepper.result().time_s, 40.0 + 90.0);
}

TEST(ClosedLoop, StepperSaysItTimedOut)
{
  const mmm::Route route({Eigen::Vector2d(2.8, 0.0), Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(3.0, 5.0)});
  Rig rig;
  rig.medium->set_solve_cap_s(10.0);
  mmd::ClosedLoopOptions o;
  o.timeout_s = 0.5;
  o.fixed_latency_s = 0.14;
  mmm::KinematicClosedLoop plant(state(2.8, 0.0, 0.0, pose("carry")), rig.medium->problem().limits().breakaway);
  mmd::ClosedLoopStepper stepper(*rig.b5, *rig.medium, rig.b5_config, rig.posture, route, {3.0, 5.0, M_PI / 2}, plant,
                                 world_of({}), primitives(), o);
  while (stepper.step()) {
  }
  EXPECT_TRUE(stepper.timed_out());
  EXPECT_FALSE(stepper.result().reached);
}
