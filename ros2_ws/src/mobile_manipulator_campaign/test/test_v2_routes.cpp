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
  // Spec 2 Amendment 3: from seed 4's measured start, the conservative sweep check with both paddings blocked two
  // segments of the dead-end turn-around, although the padded footprint stays 8.6 mm clear of the wall cells (Task 13
  // review I2); with the geometric padding only the route is accepted.
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
// It locates the blocked segments. The printed centre-to-footprint distance only bounds proximity: a blocked segment
// is the conservative checker's verdict, not a geometric intersection (Task 13 review I2).
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

namespace
{
void print_validation_diagnostic(const std::vector<std::pair<std::string, mr::Pose2>> & starts, const mr::Pose2 & goal)
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
        // the exact gap: the footprint at each sample against the occupied cells' squares (boundary points every 1 mm;
        // negative: the footprint overlaps a cell)
        double gap = INFINITY;
        // samples dense in both the translation and the rotation (the footprint's far vertices move radius x dyaw)
        const double sweep = std::hypot(b.x - a.x, b.y - a.y) + checker.radius() * std::abs(std::remainder(b.yaw - a.yaw, 2 * M_PI));
        const int dense = std::max(1, static_cast<int>(std::ceil(sweep / 0.0005)));
        for (int k = 0; k <= dense; ++k) {
          const double t = static_cast<double>(k) / dense;
          const mr::Pose2 q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.yaw + t * std::remainder(b.yaw - a.yaw, 2 * M_PI)};
          const int R = static_cast<int>(std::ceil(checker.radius() / res)) + 3;
          for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
              const int x = c0->x + dx, y = c0->y + dy;
              if (!grid.contains(x, y) || !planner->validation_grid->blocked(x, y)) continue;
              const auto cc = grid.cell_center(x, y);
              for (int e = 0; e < 4; ++e)
                for (int m = 0; m <= 50; ++m) {
                  const double u = -res / 2 + res * m / 50.0;
                  const mr::Point2 pt = e == 0 ? mr::Point2{cc.x + u, cc.y - res / 2} : e == 1 ? mr::Point2{cc.x + u, cc.y + res / 2}
                                      : e == 2 ? mr::Point2{cc.x - res / 2, cc.y + u} : mr::Point2{cc.x + res / 2, cc.y + u};
                  gap = std::min(gap, footprint_distance(planner->validation_footprint, q, pt));
                }
            }
        }
        std::cout << "    exact gap to the occupied cells' squares over the swept motion (0.5 mm samples): " << gap << " m\n";
      }
      std::cout << (smoothed ? "  smoothed" : "  raw") << " plan: " << poses.size() << " poses, " << blocked_segments
                << " blocked segments\n";
    }
  }
}
}  // namespace

TEST(V2Routes, DISABLED_StoreRoomSeed4Diagnostic)
{
  print_validation_diagnostic({
    {"exact", {12.0, 0.0, 0.0}},
    {"seed4 final", {12.000571096711832, -9.233377091438666e-05, 0.00018850046942180685}},
    {"seed4 perturbation", {12.0 + 0.0005710965798577461, -9.23405109605665e-05, 0.00018850125934238206}},
  }, {13.575, 9.025, 0.0});
}

// Diagnostic (the route-refusal check after the medium diagnosis, 2026-11-25): v2_room_r4_nav's start with the seed's
// perturbation to the hallway points of docs/experiments/medium-diagnosis/corpus_door.json; seeds 11 (west) and 4
// (east) were refused, seed 1 accepted (experiment_runs/medium-diag/door). Not a check: run with
// --gtest_also_run_disabled_tests --gtest_filter='*Hallway*'.
TEST(V2Routes, DISABLED_HallwaySeedsDiagnostic)
{
  const mr::Pose2 start{5.275, -0.5, 1.570796};
  auto at = [&](double dx, double dy, double dyaw) { return mr::Pose2{start.x + dx, start.y + dy, start.yaw + dyaw}; };
  std::cout << "#### west goal (3.5, 12.65, 0)\n";
  print_validation_diagnostic({
    {"exact", start},
    {"seed1 (accepted)", at(-0.0007322467119749345, -0.0007271859272676056, -9.757019231092367e-05)},
    {"seed11 (refused)", at(-0.0006685737747910866, 0.0005468517123068786, -0.000243949601464698)},
  }, {3.5, 12.65, 0.0});
  std::cout << "#### east goal (11.5, 12.65, -pi)\n";
  print_validation_diagnostic({
    {"seed1 (accepted)", at(-0.0007322467119749345, -0.0007271859272676056, -9.757019231092367e-05)},
    {"seed4 (refused)", at(0.0005710965798577461, -9.23405109605665e-05, 0.00018850125934238206)},
  }, {11.5, 12.65, -3.141593});
}

// Route refusal retry (docs/superpowers/specs/2026-11-25-route-refusal-retry-design.md): the refused hallway routes are
// planned again around the refusing spot and validated; a route without a refusal is unchanged; no retries keep the
// old refusal.
namespace
{
struct Planned
{
  mr::SearchResult result;
  std::shared_ptr<mr::RoutePlanner> planner;
};
Planned plan_route(const mr::Pose2 & start, const mr::Pose2 & goal, int retries = -1)
{
  const fs::path packages = PACKAGES_SOURCE;
  const auto inputs = mc::load_routing_inputs((packages / "mobile_manipulator_navigation").string(),
    read(packages / "mobile_manipulator_description/urdf/mobile_manipulator.urdf"),
    nlohmann::ordered_json::parse(read(packages / "mobile_manipulator_control/config/qualified_payload.json")));
  mr::SearchConfiguration configuration;
  configuration.lattice = inputs.lattice;
  configuration.costmap = inputs.costmap;
  if (retries >= 0) configuration.refusal_retries = retries;
  mr::smac::Footprint costmap_footprint;
  for (const auto & p : inputs.costmap_footprint) costmap_footprint.push_back({p.x, p.y});
  auto planner = std::make_shared<mr::RoutePlanner>(inputs.grid, costmap_footprint, inputs.footprint, configuration);
  mr::Search search(planner, {start, goal, {}});
  while (search.result().status == mr::SearchStatus::Pending) search.advance({});
  return {search.result(), planner};
}
const mr::Pose2 kR4Start{5.275, -0.5, 1.570796};
mr::Pose2 r4_start(double dx, double dy, double dyaw) { return {kR4Start.x + dx, kR4Start.y + dy, kR4Start.yaw + dyaw}; }
}  // namespace

TEST(V2Routes, HallwayRefusalsAreReplanned)
{
  for (const auto & [start, goal] : std::vector<std::pair<mr::Pose2, mr::Pose2>>{
         {r4_start(-0.0006685737747910866, 0.0005468517123068786, -0.000243949601464698), {3.5, 12.65, 0.0}},      // seed 11
         {r4_start(0.0005710965798577461, -9.23405109605665e-05, 0.00018850125934238206), {11.5, 12.65, -3.141593}}}) {  // seed 4
    const auto p = plan_route(start, goal);
    ASSERT_EQ(p.result.status, mr::SearchStatus::Found) << p.result.reason;
    EXPECT_GT(p.result.statistics.refusal_retries, 0u);
    // the accepted route passes the unchanged validation on its own
    const mr::FootprintChecker checker(p.planner->validation_grid, p.planner->validation_footprint);
    auto check = checker.motion(p.result.poses, p.planner->configuration.max_point_step_m);
    std::size_t budget = std::size_t{1} << 34;
    EXPECT_EQ(check.advance(budget), mr::CheckStatus::Clear);
  }
}

TEST(V2Routes, NoRetryWithoutARefusalAndNoneWhenOff)
{
  const auto exact = plan_route(kR4Start, {3.5, 12.65, 0.0});
  ASSERT_EQ(exact.result.status, mr::SearchStatus::Found);
  EXPECT_EQ(exact.result.statistics.refusal_retries, 0u);
  EXPECT_EQ(exact.result.statistics.expansions, 196661u);  // the same Smac search as before the retry
  const auto off = plan_route(r4_start(-0.0006685737747910866, 0.0005468517123068786, -0.000243949601464698),
                              {3.5, 12.65, 0.0}, 0);
  EXPECT_EQ(off.result.status, mr::SearchStatus::Rejected);
  EXPECT_EQ(off.result.reason, "the smoothed and the raw plan failed the swept validation");
}

// Route refusal retry review I1: the marks can make the goal pose invalid for Smac, which then returns its best node
// within the goal tolerance; a re-plan must end where the first attempt ended (the review's synthetic case: a wall at
// y 6, the validation footprint scaled 1.1, the goal 0.47 m from the wall). Review M2: a re-plan that marks no new
// cell stops the retries.
namespace
{
std::shared_ptr<const mr::GridSnapshot> wall_map()
{
  const int n = 240;
  const double r = 0.05;
  std::vector<mr::Cell> known(n * n, mr::Cell::Free);
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x) {
      const double wx = (x + 0.5) * r, wy = (y + 0.5) * r;
      const bool border = x == 0 || y == 0 || x == n - 1 || y == n - 1;
      const bool wall = wy >= 6.0 && wy < 6.2 && !(std::abs(wx - 6.0) < 1.0);
      if (border || wall) known[y * n + x] = mr::Cell::Occupied;
    }
  return std::make_shared<const mr::GridSnapshot>(n, n, r, mr::Point2{0.0, 0.0}, known);
}
mr::SearchResult plan_on_wall_map(int retries)
{
  const fs::path packages = PACKAGES_SOURCE;
  const auto inputs = mc::load_routing_inputs((packages / "mobile_manipulator_navigation").string(),
    read(packages / "mobile_manipulator_description/urdf/mobile_manipulator.urdf"),
    nlohmann::ordered_json::parse(read(packages / "mobile_manipulator_control/config/qualified_payload.json")));
  mr::SearchConfiguration configuration;
  configuration.lattice = inputs.lattice;
  configuration.costmap = inputs.costmap;
  configuration.refusal_retries = retries;
  mr::smac::Footprint costmap_footprint;
  for (const auto & p : inputs.costmap_footprint) costmap_footprint.push_back({p.x, p.y});
  mr::Polygon validation = inputs.footprint;
  for (auto & p : validation) { p.x *= 1.1; p.y *= 1.1; }
  auto planner = std::make_shared<mr::RoutePlanner>(wall_map(), costmap_footprint, validation, configuration);
  mr::Search search(planner, {{3.0, 3.0, M_PI / 2}, {3.0, 5.53, 0.0}, {}});
  while (search.result().status == mr::SearchStatus::Pending) search.advance({});
  return search.result();
}
}  // namespace

TEST(V2Routes, AReplanMustEndWhereTheFirstAttemptEnded)
{
  const auto first = plan_on_wall_map(0), retried = plan_on_wall_map(3);
  ASSERT_EQ(first.status, mr::SearchStatus::Rejected) << first.reason;
  ASSERT_FALSE(first.poses.empty());
  if (retried.status == mr::SearchStatus::Found) {
    EXPECT_NEAR(retried.poses.back().x, first.poses.back().x, 1e-9);
    EXPECT_NEAR(retried.poses.back().y, first.poses.back().y, 1e-9);
  } else {
    EXPECT_EQ(retried.status, mr::SearchStatus::Rejected);
    EXPECT_NE(retried.reason.find("re-planned"), std::string::npos) << retried.reason;  // review M1: the re-plans named
    EXPECT_LE(retried.statistics.refusal_retries, 3u);
  }
}
