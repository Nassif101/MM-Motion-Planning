// S5 (spec 2 Amendment 1): end-to-end equivalence of the port with nav2_smac_planner::SmacPlannerLattice 1.3.13.
// Both planners see the same costmap (the port's, byte-identical to nav2_costmap_2d per S1), the reference Lattice
// settings and the same request sequence (Smac caches its obstacle heuristic per goal across requests, so the order
// matters). Pass: same outcome; same number of raw and smoothed poses; positions within 1e-4 m and yaw within
// 1e-4 rad. Smac's wall-time limit is raised so that only its iteration limit applies (A1.2 item 1). The port runs
// in slices of 37 iterations, which also checks that pausing does not change results.
#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "common.hpp"
#include "mobile_manipulator_routing/smac/planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace sm = mobile_manipulator_routing::smac;

namespace
{
struct Case
{
  std::string name;
  double sx, sy, syaw, gx, gy, gyaw;
};

struct Result
{
  std::string outcome;
  std::vector<std::array<double, 3>> raw, plan;
  double seconds = 0.0;
};

std::string footprint_string(const sm::Footprint & f)
{
  std::string s = "[";
  for (std::size_t i = 0; i < f.size(); ++i) {
    s += (i ? ", [" : "[") + std::to_string(f[i].x) + ", " + std::to_string(f[i].y) + "]";
  }
  return s + "]";
}

geometry_msgs::msg::PoseStamped stamped(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

class Harness
{
public:
  explicit Harness(const sm::Footprint & footprint)
  {
    layered_ = std::make_shared<sm::LayeredCostmap>(check::v2_costmap(footprint));
    const auto & map = layered_->costmap;

    rclcpp::NodeOptions costmap_options;
    costmap_options.arguments({"--ros-args", "-r", "__node:=global_costmap", "-r", "__ns:=/check"});
    costmap_options.parameter_overrides({
      {"global_frame", "map"}, {"robot_base_frame", "base_footprint"}, {"resolution", 0.05},
      {"rolling_window", false}, {"track_unknown_space", false}, {"footprint", footprint_string(footprint)},
      {"footprint_padding", 0.01}, {"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
      {"static_layer.plugin", "nav2_costmap_2d::StaticLayer"}, {"static_layer.map_topic", "/check_map_unused"},
      {"inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"}, {"inflation_layer.inflation_radius", 1.0},
      {"inflation_layer.cost_scaling_factor", 5.0}, {"inflation_layer.inflate_unknown", false},
      {"inflation_layer.inflate_around_unknown", false}});
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
    costmap_ros_->configure();
    costmap_ros_->getLayeredCostmap()->resizeMap(map.getSizeInCellsX(), map.getSizeInCellsY(), map.getResolution(),
                                                 map.getOriginX(), map.getOriginY());
    auto * master = costmap_ros_->getCostmap();
    for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y)
      for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) master->setCost(x, y, map.getCost(x, y));

    node_ = std::make_shared<nav2_util::LifecycleNode>("planner_check", "/check");
    const std::string lattice = std::string(ROUTING_DATA) + "/diff_5cm.json";
    settings_.lattice_filepath = lattice;
    const std::map<std::string, rclcpp::ParameterValue> params{
      {"tolerance", rclcpp::ParameterValue(0.20)}, {"allow_unknown", rclcpp::ParameterValue(false)},
      {"max_iterations", rclcpp::ParameterValue(1000000)},
      {"max_on_approach_iterations", rclcpp::ParameterValue(1000)},
      {"max_planning_time", rclcpp::ParameterValue(1000.0)},
      {"analytic_expansion_ratio", rclcpp::ParameterValue(3.5)},
      {"analytic_expansion_max_length", rclcpp::ParameterValue(3.0)},
      {"analytic_expansion_max_cost", rclcpp::ParameterValue(200.0)},
      {"analytic_expansion_max_cost_override", rclcpp::ParameterValue(false)},
      {"reverse_penalty", rclcpp::ParameterValue(2.0)}, {"change_penalty", rclcpp::ParameterValue(0.05)},
      {"non_straight_penalty", rclcpp::ParameterValue(1.2)}, {"cost_penalty", rclcpp::ParameterValue(1.0)},
      {"rotation_penalty", rclcpp::ParameterValue(5.0)}, {"retrospective_penalty", rclcpp::ParameterValue(0.015)},
      {"lookup_table_size", rclcpp::ParameterValue(20.0)}, {"cache_obstacle_heuristic", rclcpp::ParameterValue(true)},
      {"allow_reverse_expansion", rclcpp::ParameterValue(false)}, {"smooth_path", rclcpp::ParameterValue(true)},
      {"lattice_filepath", rclcpp::ParameterValue(lattice)},
      {"smoother.max_iterations", rclcpp::ParameterValue(1000)}, {"smoother.w_smooth", rclcpp::ParameterValue(0.3)},
      {"smoother.w_data", rclcpp::ParameterValue(0.2)}, {"smoother.tolerance", rclcpp::ParameterValue(1.0e-10)},
      {"smoother.do_refinement", rclcpp::ParameterValue(true)},
      {"smoother.refinement_num", rclcpp::ParameterValue(2)}};
    for (const auto & [name, value] : params) node_->declare_parameter("Lattice." + name, value);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    smac_ = std::make_unique<nav2_smac_planner::SmacPlannerLattice>();
    smac_->configure(node_, "Lattice", tf_, costmap_ros_);
    smac_->activate();
    raw_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "unsmoothed_plan", 10, [this](nav_msgs::msg::Path::SharedPtr msg) { last_raw_ = *msg; ++raw_count_; });
    port_ = std::make_unique<sm::SmacPlannerLattice>(settings_, map.getResolution());
  }

  ~Harness() { smac_->deactivate(); smac_->cleanup(); }

  Result run_smac(const Case & c)
  {
    Result r;
    const auto before = raw_count_;
    const auto t0 = std::chrono::steady_clock::now();
    try {
      const auto path = smac_->createPlan(stamped(c.sx, c.sy, c.syaw), stamped(c.gx, c.gy, c.gyaw),
                                          [] { return false; });
      r.outcome = "found";
      for (const auto & p : path.poses)
        r.plan.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
    } catch (const nav2_core::StartOutsideMapBounds &) { r.outcome = "start_outside_map_bounds";
    } catch (const nav2_core::GoalOutsideMapBounds &) { r.outcome = "goal_outside_map_bounds";
    } catch (const nav2_core::StartOccupied &) { r.outcome = "start_occupied";
    } catch (const nav2_core::NoValidPathCouldBeFound &) { r.outcome = "no_valid_path";
    } catch (const nav2_core::PlannerTimedOut &) { r.outcome = "exceeded_maximum_iterations";
    } catch (const std::exception & e) { r.outcome = std::string("exception: ") + e.what(); }
    r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    for (int i = 0; i < 50 && raw_count_ == before && r.outcome == "found"; ++i) {
      rclcpp::spin_some(node_->get_node_base_interface());
    }
    if (raw_count_ != before) {
      for (const auto & p : last_raw_.poses)
        r.raw.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
    }
    return r;
  }

  Result run_port(const Case & c)
  {
    Result r;
    const auto t0 = std::chrono::steady_clock::now();
    auto outcome = port_->begin(layered_, c.sx, c.sy, c.syaw, c.gx, c.gy, c.gyaw);
    while (outcome == sm::SmacPlannerLattice::Outcome::Pending) outcome = port_->advance(37);
    r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    r.outcome = sm::to_string(outcome);
    for (const auto & p : port_->plan()) r.plan.push_back({p.x, p.y, sm::getYaw(p.orientation)});
    for (const auto & p : port_->raw_plan()) r.raw.push_back({p.x, p.y, sm::getYaw(p.orientation)});
    iterations_ = port_->iterations();
    return r;
  }

  int port_iterations() const { return iterations_; }
  const sm::Costmap & costmap() const { return layered_->costmap; }

private:
  std::shared_ptr<sm::LayeredCostmap> layered_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_smac_planner::SmacPlannerLattice> smac_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_sub_;
  nav_msgs::msg::Path last_raw_;
  std::size_t raw_count_ = 0;
  sm::LatticeSettings settings_;
  std::unique_ptr<sm::SmacPlannerLattice> port_;
  int iterations_ = 0;
};

// Returns a description of the first difference, or "" when equivalent.
std::string compare(const std::vector<std::array<double, 3>> & a, const std::vector<std::array<double, 3>> & b)
{
  if (a.size() != b.size()) return "pose counts " + std::to_string(a.size()) + " vs " + std::to_string(b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double dyaw = std::abs(std::remainder(a[i][2] - b[i][2], 2.0 * M_PI));
    if (std::abs(a[i][0] - b[i][0]) > 1e-4 || std::abs(a[i][1] - b[i][1]) > 1e-4 || dyaw > 1e-4) {
      return "pose " + std::to_string(i) + ": (" + std::to_string(a[i][0]) + ", " + std::to_string(a[i][1]) + ", " +
             std::to_string(a[i][2]) + ") vs (" + std::to_string(b[i][0]) + ", " + std::to_string(b[i][1]) + ", " +
             std::to_string(b[i][2]) + ")";
    }
  }
  return "";
}

std::vector<Case> scenario_cases()
{
  std::vector<Case> out;
  const auto file = YAML::LoadFile(std::string(NAVIGATION_SOURCE) + "/config/scenarios_site_v2.yaml");
  for (const auto & item : file["scenarios"]) {
    const auto & s = item.second;
    if (!s["start"] || !s["goal"] || s["start"].size() != 3 || s["goal"].size() != 3) continue;
    out.push_back({item.first.as<std::string>(), s["start"][0].as<double>(), s["start"][1].as<double>(),
                   s["start"][2].as<double>(), s["goal"][0].as<double>(), s["goal"][1].as<double>(),
                   s["goal"][2].as<double>()});
  }
  return out;
}

std::vector<Case> random_cases(const sm::Costmap & map, std::uint32_t seed, int count)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(map.getOriginX() + 0.5,
                                            map.getOriginX() + map.getSizeInCellsX() * map.getResolution() - 0.5);
  std::uniform_real_distribution<double> uy(map.getOriginY() + 0.5,
                                            map.getOriginY() + map.getSizeInCellsY() * map.getResolution() - 0.5);
  std::uniform_real_distribution<double> uyaw(-M_PI, M_PI);
  const auto free_pose = [&](double & x, double & y) {
      for (;;) {
        x = ux(rng);
        y = uy(rng);
        unsigned int mx, my;
        if (map.worldToMap(x, y, mx, my) && map.getCost(mx, my) < 128) return;
      }
    };
  std::vector<Case> out;
  for (int i = 0; i < count; ++i) {
    Case c{"random_" + std::to_string(seed) + "_" + std::to_string(i), 0, 0, uyaw(rng), 0, 0, uyaw(rng)};
    free_pose(c.sx, c.sy);
    if (i % 5 == 0) {  // a near goal, often inside the 0.20 m tolerance or the analytic-expansion range
      std::uniform_real_distribution<double> near(-1.5, 1.5);
      c.gx = c.sx + near(rng);
      c.gy = c.sy + near(rng);
    } else {
      free_pose(c.gx, c.gy);
    }
    out.push_back(c);
  }
  return out;
}

std::vector<Case> edge_cases(const sm::Costmap & map)
{
  // a lethal goal, a goal outside the map, a start outside the map, a same-cell goal, a heading-only goal
  double lx = 0, ly = 0;
  for (unsigned int y = 100; y < map.getSizeInCellsY() && lx == 0; ++y)
    for (unsigned int x = 100; x < map.getSizeInCellsX(); ++x)
      if (map.getCost(x, y) == sm::kLethalObstacle) { map.mapToWorld(x, y, lx, ly); break; }
  return {{"lethal_goal", -6.0, -3.5, 0.0, lx, ly, 0.0},
          {"goal_outside", -6.0, -3.5, 0.0, 100.0, 0.0, 0.0},
          {"start_outside", 100.0, 0.0, 0.0, -6.0, -3.5, 0.0},
          {"same_cell", -6.0, -3.5, 0.0, -5.99, -3.49, 1.0},
          {"turn_in_place", -6.0, -3.5, 0.0, -6.0, -3.5, 3.1},
          {"start_lethal", lx, ly, 0.0, -6.0, -3.5, 0.0}};
}

void run_corpus(Harness & h, const std::vector<Case> & cases, const std::string & report)
{
  std::ofstream out(report);
  out << "case,smac_outcome,port_outcome,smac_poses,port_poses,raw_compared,port_iterations,smac_s,port_s,difference\n";
  int failures = 0, found = 0;
  for (const auto & c : cases) {
    const Result smac = h.run_smac(c);
    const Result port = h.run_port(c);
    std::string diff;
    if (smac.outcome != port.outcome) diff = "outcome";
    else if (smac.outcome == "found") {
      ++found;
      diff = compare(smac.plan, port.plan);
      if (diff.empty() && !smac.raw.empty()) {
        const auto raw = compare(smac.raw, port.raw);
        if (!raw.empty()) diff = "raw " + raw;
      }
    }
    out << c.name << ',' << smac.outcome << ',' << port.outcome << ',' << smac.plan.size() << ',' << port.plan.size()
        << ',' << (!smac.raw.empty()) << ',' << h.port_iterations() << ',' << smac.seconds << ',' << port.seconds
        << ",\"" << diff << "\"\n";
    if (!diff.empty()) {
      ++failures;
      ADD_FAILURE() << c.name << ": " << diff << " (smac " << smac.outcome << ", port " << port.outcome << ")";
    }
  }
  std::cout << report << ": " << cases.size() << " cases, " << found << " found, " << failures << " mismatches\n";
}
}  // namespace

TEST(PlannerEquivalence, ScenarioRandomAndEdgeCasesMatchSmacLattice)
{
  Harness h(check::home_footprint());
  auto cases = scenario_cases();
  ASSERT_GE(cases.size(), 30u);
  const auto random = random_cases(h.costmap(), 2026, 200);
  cases.insert(cases.end(), random.begin(), random.end());
  const auto edges = edge_cases(h.costmap());
  cases.insert(cases.end(), edges.begin(), edges.end());
  const char * dir = std::getenv("PLANNER_CHECK_REPORT_DIR");
  run_corpus(h, cases, std::string(dir ? dir : "/tmp") + "/planner_equivalence_home.csv");
}

TEST(PlannerEquivalence, AnIrregularFootprintMatchesSmacLattice)
{
  sm::Footprint f;  // a 16-vertex convex hull, off-centre, like the projected base links
  for (int i = 0; i < 16; ++i) {
    const double a = 2.0 * M_PI * i / 16.0 + 0.07;
    const double rx = std::cos(a) > 0 ? 0.50 : 0.62, ry = std::sin(a) > 0 ? 0.36 : 0.40;
    f.push_back({rx * std::cos(a) + 0.03, ry * std::sin(a)});
  }
  Harness h(f);
  auto cases = scenario_cases();
  const auto random = random_cases(h.costmap(), 7, 60);
  cases.insert(cases.end(), random.begin(), random.end());
  const char * dir = std::getenv("PLANNER_CHECK_REPORT_DIR");
  run_corpus(h, cases, std::string(dir ? dir : "/tmp") + "/planner_equivalence_irregular.csv");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
