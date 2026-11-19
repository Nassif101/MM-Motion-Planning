// Spec 2 Amendment 1, S6: every V2 scenario destination planned by the Smac Lattice port with the plant's routing
// inputs (the projected base footprint, the global costmap's padding and inflation, the Lattice block). Records
// iterations, the simulation-time hold at the default per-tick budget, wall time and the validation outcome to
// $ROUTE_MEASUREMENT_CSV (default /tmp/v2_routes.csv). A route that Smac finds but the swept validation rejects
// fails the test: the validation pass must not overrule Smac on the reference scenarios.
#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "mobile_manipulator_campaign/routing_inputs.hpp"
#include "mobile_manipulator_routing/footprint.hpp"
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
  const mr::WorkBudget budget;  // the defaults: 4,096 iterations and 524,288 validation cell tests per tick
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

TEST(V2Routes, TheStoreRoomRouteFromTask13Seed4sStartIsAccepted)
{
  // Spec 2 Amendment 3: from seed 4's measured start Smac's path sweeps 6 mm into a wall cell's square between two
  // clear poses of the dead-end turn-around, inside the costmap's 0.01 m footprint padding; the swept validation
  // checks the geometric padding only and accepts it.
  const fs::path packages = PACKAGES_SOURCE;
  const auto inputs = mc::load_routing_inputs((packages / "mobile_manipulator_navigation").string(),
    read(packages / "mobile_manipulator_description/urdf/mobile_manipulator.urdf"),
    nlohmann::ordered_json::parse(read(packages / "mobile_manipulator_control/config/qualified_payload.json")));
  mr::SearchConfiguration configuration;
  configuration.lattice = inputs.lattice;
  configuration.costmap = inputs.costmap;
  mr::smac::Footprint costmap_footprint;
  for (const auto & p : inputs.costmap_footprint) costmap_footprint.push_back({p.x, p.y});
  auto planner = std::make_shared<mr::RoutePlanner>(inputs.grid, costmap_footprint, inputs.footprint, configuration);
  mr::Search search(planner, {{12.000571096711832, -9.233377091438666e-05, 0.00018850046942180685},
                              {13.575, 9.025, 0.0}, {}});
  while (search.result().status == mr::SearchStatus::Pending) search.advance({});
  EXPECT_EQ(search.result().status, mr::SearchStatus::Found) << search.result().reason;
  EXPECT_EQ(search.result().statistics.expansions, 128423u);  // the same Smac search as the rejected run
}

// Diagnostic (spec 2 Task 13, store_room lattice seed 4): where the swept validation rejects Smac's plan from the
// seed's start. Not a check: run with --gtest_also_run_disabled_tests --gtest_filter='*StoreRoom*'.
namespace
{
double distance_to_segment(mr::Point2 p, mr::Point2 a, mr::Point2 b)
{
  const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
  const double t = l2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
  return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
// signed distance from p to the footprint at pose (negative inside)
double footprint_distance(const mr::Polygon & fp, const mr::Pose2 & pose, mr::Point2 p)
{
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  std::vector<mr::Point2> w;
  for (const auto & v : fp) w.push_back({pose.x + c * v.x - s * v.y, pose.y + s * v.x + c * v.y});
  double d = INFINITY;
  bool inside = false;
  for (std::size_t i = 0, j = w.size() - 1; i < w.size(); j = i++) {
    d = std::min(d, distance_to_segment(p, w[j], w[i]));
    if ((w[i].y > p.y) != (w[j].y > p.y) && p.x < (w[j].x - w[i].x) * (p.y - w[i].y) / (w[j].y - w[i].y) + w[i].x)
      inside = !inside;
  }
  return inside ? -d : d;
}
}  // namespace

TEST(V2Routes, DISABLED_StoreRoomSeed4Diagnostic)
{
  const fs::path packages = PACKAGES_SOURCE;
  const auto inputs = mc::load_routing_inputs((packages / "mobile_manipulator_navigation").string(),
    read(packages / "mobile_manipulator_description/urdf/mobile_manipulator.urdf"),
    nlohmann::ordered_json::parse(read(packages / "mobile_manipulator_control/config/qualified_payload.json")));
  mr::SearchConfiguration configuration;
  configuration.lattice = inputs.lattice;
  configuration.costmap = inputs.costmap;
  mr::smac::Footprint costmap_footprint;
  for (const auto & p : inputs.costmap_footprint) costmap_footprint.push_back({p.x, p.y});
  const mr::Pose2 goal{13.575, 9.025, 0.0};
  const std::vector<std::pair<std::string, mr::Pose2>> starts = {
    {"exact", {12.0, 0.0, 0.0}},
    {"seed4 final", {12.000571096711832, -9.233377091438666e-05, 0.00018850046942180685}},
    {"seed4 perturbation", {12.0 + 0.0005710965798577461, -9.23405109605665e-05, 0.00018850125934238206}},
  };
  const auto & grid = *inputs.grid;
  const double res = grid.resolution();
  std::cout << "footprint vertices " << inputs.footprint.size() << ", resolution " << res << "\n";
  for (const auto & [name, start] : starts) {
    auto planner = std::make_shared<mr::RoutePlanner>(inputs.grid, costmap_footprint, inputs.footprint, configuration);
    mr::Search search(planner, {start, goal, {}});
    while (search.result().status == mr::SearchStatus::Pending) search.advance({});
    const auto & r = search.result();
    std::cout << "== " << name << ": " << mr::to_string(r.status) << " (" << r.reason << "), iterations "
              << r.statistics.expansions << "\n";
    const mr::FootprintChecker checker(planner->validation_grid, planner->validation_footprint);
    const auto & cm = planner->costmap->costmap;
    for (const bool smoothed : {true, false}) {
      const auto & plan = smoothed ? planner->smac->plan() : planner->smac->raw_plan();
      std::vector<mr::Pose2> poses;
      for (const auto & p : plan) poses.push_back({p.x, p.y, mr::smac::getYaw(p.orientation)});
      int blocked_segments = 0;
      for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
        auto check = checker.motion({poses[i], poses[i + 1]}, configuration.max_point_step_m);
        std::size_t budget = 1u << 30;
        if (check.advance(budget) != mr::CheckStatus::Blocked) continue;
        if (++blocked_segments > 3) continue;
        const auto & a = poses[i];
        const auto & b = poses[i + 1];
        std::cout << (smoothed ? "  smoothed" : "  raw") << " segment " << i << "/" << poses.size() - 1 << ": (" << a.x
                  << ", " << a.y << ", " << a.yaw << ") -> (" << b.x << ", " << b.y << ", " << b.yaw << "), step "
                  << std::hypot(b.x - a.x, b.y - a.y) << " m; poses clear " << checker.pose_clear(a) << "/"
                  << checker.pose_clear(b) << "\n";
        // the occupied cells nearest the footprint along the segment, with Smac's costmap cost there
        const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.0125)));
        const auto c0 = grid.world_to_cell({a.x, a.y});
        double best = INFINITY;
        std::string best_text;
        for (int k = 0; k <= n; ++k) {
          const double t = static_cast<double>(k) / n;
          double dyaw = std::remainder(b.yaw - a.yaw, 2 * M_PI);
          const mr::Pose2 q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.yaw + t * dyaw};
          const int R = static_cast<int>(std::ceil(checker.radius() / res)) + 3;
          for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
              const int x = c0->x + dx, y = c0->y + dy;
              if (!grid.contains(x, y) || !planner->validation_grid->blocked(x, y)) continue;
              const auto cc = grid.cell_center(x, y);
              const double d = footprint_distance(planner->validation_footprint, q, cc);
              if (d < best) {
                best = d;
                std::ostringstream text;
                text << "cell (" << x << ", " << y << ") centre (" << cc.x << ", " << cc.y << ") at t " << t
                     << ": centre-to-footprint " << d << " m (half cell " << res / 2 << ", half diagonal "
                     << res * M_SQRT1_2 << "), Smac cost " << int(cm.getCost(x, y));
                best_text = text.str();
              }
            }
        }
        std::cout << "    nearest occupied: " << best_text << "\n";
      }
      std::cout << (smoothed ? "  smoothed" : "  raw") << " plan: " << poses.size() << " poses, " << blocked_segments
                << " blocked segments\n";
    }
  }
}
