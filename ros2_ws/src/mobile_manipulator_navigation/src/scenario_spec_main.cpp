// Resolve a navigation scenario and check that its start and goal poses are free in the static map.
//
// Prints one JSON object: the scenario, its site and Unity scene, its footprint polygon, and the start check. Exits
// non-zero when the scenario is unknown or inconsistent (2), or any map cell under the
// footprint (plus margin) at the start or goal pose is not free (1). Scenario obstacles
// (unmapped boxes placed in Play) must lie in free map space and clear of the start and
// goal footprints. Pure file access; no ROS graph.
//
// Usage: scenario_spec <name> [--arm-pose POSE] [--start-free-for PROFILE] | scenario_spec --list
// --arm-pose resolves the scenario starting in another qualified arm pose with the footprint
// profile of that name (run_nav_scenario.py --arm-pose; exit 2 for a pose without one).
// --start-free-for adds {"start_free_for": {"profile", "free"}}: whether the start is also free
// for another footprint profile (the runner teleports with the arm pose it holds).
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmn = mobile_manipulator_navigation;

int main(int argc, char ** argv)
{
  const mobile_manipulator_control::Args args(argc, argv);
  const mmn::ScenarioConfig config(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation"));
  if (args.has("list")) {
    std::cout << mmn::Json(config.names()).dump() << std::endl;
    return 0;
  }
  if (args.positional().size() != 1) {
    std::cerr << "usage: scenario_spec <scenario> [--arm-pose POSE] [--start-free-for PROFILE] | scenario_spec --list" << std::endl;
    return 2;
  }
  try {
    mmn::Json resolved = config.resolve(
      args.positional()[0], args.has("arm-pose") ? std::optional<std::string>(args.get("arm-pose")) : std::nullopt);
    if (args.has("start-free-for")) {
      const auto profile = args.get("start-free-for");
      const auto polygon = mmn::polygon_of(config.load("footprint_profiles.yaml").at("profiles").at(profile).at("polygon"));
      resolved["start_free_for"] = {
        {"profile", profile},
        {"free", mmn::start_is_free(config.map(resolved.at("site").get<std::string>()), polygon, mmn::pose_of(resolved.at("scenario").at("start"))).free}};
    }
    std::cout << resolved.dump() << std::endl;
    return resolved.at("start_free").get<bool>() ? 0 : 1;
  } catch (const std::exception & error) {  // unknown or invalid scenario
    std::cout << mmn::Json{{"error", error.what()}}.dump() << std::endl;
    return 2;
  }
}
