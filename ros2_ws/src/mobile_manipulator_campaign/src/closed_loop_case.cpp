#include "mobile_manipulator_campaign/closed_loop_case.hpp"

#include <algorithm>
#include <stdexcept>

#include "mobile_manipulator_mpc/references.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/replay_bag.hpp"

namespace mobile_manipulator_campaign
{
namespace mmd = mobile_manipulator_medium;
namespace mmm = mobile_manipulator_mpc;

ClosedLoopCase make_case(const std::string & scenario, const std::string & bag, const std::string & medium_config)
{
  ClosedLoopCase c;
  const std::string config_path =
    medium_config.empty() ? mmd::package_share("mobile_manipulator_medium") + "/config/medium.yaml" : medium_config;
  c.bench = std::make_shared<mmd::BenchSetup>(mmd::load_medium_config(config_path), scenario);
  if (!bag.empty()) {
    std::array<std::string, 6> joints;
    for (int i = 0; i < 6; ++i) joints[i] = mmm::kArmJoints[i];
    const mmm::Recording rec = mmm::read_recording(bag, joints);
    const auto first = std::find_if(rec.goals.begin(), rec.goals.end(), [](const auto & g) { return g.points.size() >= 2; });
    if (first == rec.goals.end() || rec.cycles.empty()) throw std::runtime_error("make_case: no goal in " + bag);
    c.route = mmm::Route(first->points);
    c.goal = first->goal;
    const auto at = std::find_if(rec.cycles.begin(), rec.cycles.end(), [&](const auto & cy) { return cy.bag_time >= first->bag_time; });
    c.start = (at != rec.cycles.end() ? *at : rec.cycles.front()).x;
  } else {
    const auto & spec = *c.bench->scene().scenario;
    const auto s = spec.at("start"), g = spec.at("goal");
    c.goal = {g.at(0).get<double>(), g.at(1).get<double>(), g.at(2).get<double>()};
    c.route = mmm::Route({Eigen::Vector2d(s.at(0).get<double>(), s.at(1).get<double>()), Eigen::Vector2d(c.goal[0], c.goal[1])});
    c.start << s.at(0).get<double>(), s.at(1).get<double>(), s.at(2).get<double>(),
      c.bench->poses().at(spec.value("arm_pose", std::string("carry")));
  }
  c.b5_config = mmm::load_mpc_config(mmd::package_share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml");
  const mmd::MediumConfig & profile = c.bench->config();
  // B5's limit on a medium plan's age, as navigation.launch.py derives it (medium_bench closed)
  c.b5_config.medium_stale_s = 1.0 / profile.rate_hz + profile.solve_cap_s + 0.16;
  return c;
}

mmd::ClosedLoopResult run_case(const ClosedLoopCase & c, mmm::ClosedLoopPlant & plant, const CaseOptions & o)
{
  const auto & bench = *c.bench;
  const auto b5 = mmm::build_controller(c.b5_config, bench.limits(), bench.urdf(), bench.payload(), 50.0, false);
  const mmm::PostureTargets posture =
    mmm::posture_targets(bench.payload(), c.b5_config.comfort_pose, c.b5_config.posture_mode, c.b5_config.tight_poses);
  mmd::MediumPlanner medium = bench.planner();
  medium.set_iteration_budget(o.iteration_budget);
  mmd::ClosedLoopOptions co;
  co.timeout_s = o.timeout_s;
  co.medium = o.medium;
  if (o.iteration_budget > 0) {
    medium.set_solve_cap_s(10.0);  // a budget alone cuts the cycles
    co.fixed_latency_s = bench.config().solve_cap_s;
  }
  co.on_plan = o.on_plan;
  co.on_cycle = o.on_cycle;
  return mmd::run_closed_loop(*b5, medium, c.b5_config, posture, c.route, c.goal, plant, bench.world(), bench.primitives(), co);
}
}  // namespace mobile_manipulator_campaign
