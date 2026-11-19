// Spec 2 Amendment 1, S6: every V2 scenario destination planned by the Smac Lattice port with the plant's routing
// inputs (the projected base footprint, the global costmap's padding and inflation, the Lattice block). Records
// iterations, the simulation-time hold at the default per-tick budget, wall time and the validation outcome to
// $ROUTE_MEASUREMENT_CSV (default /tmp/v2_routes.csv). A route that Smac finds but the swept validation rejects
// fails the test: the validation pass must not overrule Smac on the reference scenarios.
#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "mobile_manipulator_campaign/routing_inputs.hpp"
#include "mobile_manipulator_routing/search.hpp"

namespace mc = mobile_manipulator_campaign;
namespace mr = mobile_manipulator_routing;
namespace fs = std::filesystem;

namespace
{
std::string read(const fs::path & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("fixture input missing");
  std::ostringstream value;
  value << stream.rdbuf();
  return value.str();
}
}  // namespace

TEST(V2Routes, ScenarioDestinationsArePlannedAndNoFoundRouteIsRejected)
{
  const fs::path packages = PACKAGES_SOURCE;
  const auto navigation = packages / "mobile_manipulator_navigation";
  const auto inputs = mc::load_routing_inputs(navigation.string(),
    read(packages / "mobile_manipulator_description/urdf/mobile_manipulator.urdf"),
    nlohmann::ordered_json::parse(read(packages / "mobile_manipulator_control/config/qualified_payload.json")));
  mr::SearchConfiguration configuration;
  configuration.lattice = inputs.lattice;
  configuration.costmap = inputs.costmap;
  mr::smac::Footprint costmap_footprint;
  for (const auto & p : inputs.costmap_footprint) costmap_footprint.push_back({p.x, p.y});
  auto planner = std::make_shared<mr::RoutePlanner>(inputs.grid, costmap_footprint, inputs.footprint, configuration);

  const char * csv_path = std::getenv("ROUTE_MEASUREMENT_CSV");
  std::ofstream csv(csv_path ? csv_path : "/tmp/v2_routes.csv");
  csv << "scenario,status,reason,smoothed,iterations,ticks,hold_s,wall_s,poses,length_m,end_error_m\n";
  const mr::WorkBudget budget{256, 524288};
  int found = 0, rejected = 0, scenarios = 0;
  const auto file = YAML::LoadFile((navigation / "config/scenarios_site_v2.yaml").string());
  for (const auto & item : file["scenarios"]) {
    const auto & s = item.second;
    if (!s["start"] || !s["goal"] || s["start"].size() != 3 || s["goal"].size() != 3) continue;
    ++scenarios;
    const mr::Pose2 start{s["start"][0].as<double>(), s["start"][1].as<double>(), s["start"][2].as<double>()};
    const mr::Pose2 goal{s["goal"][0].as<double>(), s["goal"][1].as<double>(), s["goal"][2].as<double>()};
    mr::Search search(planner, {start, goal, {}});
    int ticks = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (search.result().status == mr::SearchStatus::Pending) {
      search.advance(budget);
      ++ticks;
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const auto & r = search.result();
    found += r.status == mr::SearchStatus::Found;
    rejected += r.status == mr::SearchStatus::Rejected;
    const double end_error = r.poses.empty() ? NAN : std::hypot(r.resolved_goal.x - goal.x, r.resolved_goal.y - goal.y);
    csv << item.first.as<std::string>() << ',' << mr::to_string(r.status) << ",\"" << r.reason << "\"," << r.smoothed
        << ',' << r.statistics.expansions << ',' << ticks << ',' << ticks * 0.02 << ',' << wall << ',' << r.poses.size()
        << ',' << r.length_m << ',' << end_error << '\n';
    EXPECT_NE(r.status, mr::SearchStatus::Rejected) << item.first.as<std::string>() << ": " << r.reason;
    if (r.status == mr::SearchStatus::Found) EXPECT_LE(end_error, 0.20 + 0.025 + 1e-6);
  }
  std::cout << scenarios << " scenarios, " << found << " found, " << rejected << " rejected\n";
  EXPECT_GE(scenarios, 30);
  EXPECT_GT(found, scenarios / 2);
}
