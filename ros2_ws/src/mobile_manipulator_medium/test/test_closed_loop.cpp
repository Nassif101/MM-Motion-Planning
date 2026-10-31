// The closed-loop kinematic bench (Phase 5A spec Sections 6.2 and 10, plan Task 12): B5 at 50 Hz on the kinematic plant
// following the medium planner at 5 Hz, on the scripted integration cells.
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "medium_support.hpp"
#include "mobile_manipulator_medium/closed_loop.hpp"
#include "mobile_manipulator_mpc/references.hpp"
#include "mobile_manipulator_mpc/replay.hpp"

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
  mmm::MpcConfig b5_config = mmm::load_mpc_config(share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml");
  std::unique_ptr<mmm::Controller> b5 =
    mmm::build_controller(b5_config, limits(), medium_test::urdf(), payload(), 50.0, false);
  mmm::PostureTargets posture =
    mmm::posture_targets(payload(), b5_config.comfort_pose, b5_config.posture_mode, b5_config.tight_poses);
  mmd::MediumConfig medium_config;
  std::unique_ptr<mmd::MediumPlanner> medium;

  Rig()
  {
    std::map<std::string, Joints> poses;
    for (const auto & item : payload().at("poses_rad").items()) poses[item.key()] = pose(item.key());
    medium = std::make_unique<mmd::MediumPlanner>(
      problem(medium_config),
      mmd::make_dense_solver(medium_config.qp_backend, mmd::DenseQpOptions{medium_config.qp_tolerance}), poses,
      primitives().back().dims);
  }

  mmd::ClosedLoopResult run(const mmm::Route & route, const mmn::Pose2 & goal, const mmm::State & start,
                            const mmm::World & world, double timeout_s)
  {
    // deterministic: no cap and a fixed plan latency (the node's p99 latency), whatever the host's load
    medium->set_solve_cap_s(10.0);
    mmd::ClosedLoopOptions o;
    o.timeout_s = timeout_s;
    o.fixed_latency_s = 0.14;
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
