#pragma once
// One closed-loop run as medium_bench closed sets it up (MuJoCo plant plan Task 4): a cell's route, goal and start
// state (from a recorded run's first goal, or the scenario's start and goal on a straight route), B5's configuration
// and the medium planner, run on any plant.
#include <memory>
#include <string>

#include "mobile_manipulator_medium/bench_setup.hpp"
#include "mobile_manipulator_medium/closed_loop.hpp"
#include "mobile_manipulator_mpc/closed_loop_plant.hpp"
#include "mobile_manipulator_mpc/route.hpp"

namespace mobile_manipulator_campaign
{
struct ClosedLoopCase
{
  std::shared_ptr<mobile_manipulator_medium::BenchSetup> bench;
  mobile_manipulator_mpc::MpcConfig b5_config;
  mobile_manipulator_mpc::Route route{{Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)}};
  mobile_manipulator_navigation::Pose2 goal{};
  mobile_manipulator_mpc::State start = mobile_manipulator_mpc::State::Zero();
};

// `bag` empty: the scenario's start and goal on a straight route (synthetic cells). `medium_config` empty: medium.yaml.
ClosedLoopCase make_case(const std::string & scenario, const std::string & bag, const std::string & medium_config = "");

struct CaseOptions
{
  bool medium = true;          // false: B5 alone
  // > 0: reproducible plans (cap 10 s), latency fixed at the profile's cap; 30 is the 2.5 Hz profile's gate budget, the
  // hot host's compute (Phase 5A, phase5a.md "Compute options")
  int iteration_budget = 30;
  double timeout_s = 120.0;
  std::function<void(double, const mobile_manipulator_mpc::State &, const mobile_manipulator_medium::PlanOutput &)> on_plan;
  std::function<void(double, const mobile_manipulator_mpc::State &)> on_cycle;
};

mobile_manipulator_medium::ClosedLoopResult run_case(const ClosedLoopCase & c, mobile_manipulator_mpc::ClosedLoopPlant & plant,
                                                     const CaseOptions & options);
}  // namespace mobile_manipulator_campaign
