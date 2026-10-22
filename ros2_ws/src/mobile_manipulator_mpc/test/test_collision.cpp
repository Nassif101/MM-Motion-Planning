#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;
using namespace mobile_manipulator_mpc;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace
{
const std::string kGate = "Environment/NavigationObstacles/ManipulationChallenge/ManipulationRequiredGate_1p05m/";
const std::string kPostLeft = kGate + "GatePostLeft";
const std::string kPostRight = kGate + "GatePostRight";
const Vector2d kGateCentre(-7.225, -7.725);  // between the 1.05 m posts (y -7.0 and -8.45, 0.4 m wide)

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

const nlohmann::ordered_json & qualified_payload()
{
  static const auto json = nlohmann::ordered_json::parse(read_file(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_control") + "/config/qualified_payload.json"));
  return json;
}

const std::vector<mmg::Primitive> & primitives()
{
  static const auto instance = mmg::load_primitives(
    read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
              "/urdf/mobile_manipulator.urdf"),
    mmg::payload_from_json(qualified_payload()));
  return instance;
}

CollisionModel model(CollisionConfig config = {})
{
  return CollisionModel(primitives(), config);
}

CollisionConfig unlimited()
{
  CollisionConfig config;
  config.pairs_per_stage = 1000;
  return config;
}

Eigen::Matrix<double, 6, 1> joints(const std::string & name)
{
  Eigen::Matrix<double, 6, 1> q;
  for (int i = 0; i < 6; ++i) q(i) = qualified_payload()["poses_rad"][name][i].get<double>();
  return q;
}

State state(double x, double y, double yaw, const Eigen::Matrix<double, 6, 1> & q)
{
  State s;
  s << x, y, yaw, q;
  return s;
}

const std::vector<mmn::Box> & map_boxes()
{
  static const auto boxes = mmn::known_boxes(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation") +
      "/maps/construction_site.boxes.yaml",
    nullptr);
  return boxes;
}

World map_world()
{
  World world;
  world.set_static(map_boxes());
  return world;
}

World world_of(const std::vector<mmn::Box> & boxes)
{
  World world;
  world.set_static(boxes);
  return world;
}

mmn::Box box(const std::string & name, const Vector3d & center, const Vector3d & size, double yaw = 0.0)
{
  return mmn::Box{name, center, size, yaw};
}

// Base position that puts the panel centre over `target` (x, y) with the given yaw and joints.
State with_panel_over(const Vector2d & target, double yaw, const Eigen::Matrix<double, 6, 1> & q)
{
  const State origin = state(0.0, 0.0, yaw, q);
  const Vector3d panel = link_pose(link_index("panel"), origin).translation();
  return state(target.x() - panel.x(), target.y() - panel.y(), yaw, q);
}

std::set<std::string> base_links()
{
  return {"base_link", "front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link",
          "rear_right_wheel_link", "arm_mount_link"};
}

bool is_base_link(int link)
{
  for (const auto & name : base_links())
    if (link == link_index(name)) return true;
  return false;
}

const DistanceRow * find_row(const std::vector<DistanceRow> & rows, int stage, int link, const std::string & obstacle)
{
  for (const auto & row : rows)
    if (row.stage == stage && row.link == link && row.obstacle == obstacle) return &row;
  return nullptr;
}

// Lowest point of the payload box (exact: a box corner) with the robot at x.
double panel_bottom(const State & x)
{
  const mmg::Primitive & payload = primitives().back();
  const Eigen::Isometry3d pose = link_pose(link_index(payload.link), x) * payload.pose;
  double lowest = std::numeric_limits<double>::infinity();
  for (const auto & corner : mmg::primitive_points(payload, 8)) lowest = std::min(lowest, (pose * corner).z());
  return lowest;
}
}  // namespace

TEST(Collision, PayloadIsTheLastPrimitive)
{
  // The helpers above rely on load_primitives appending the payload box on tool0.
  const auto & payload = primitives().back();
  EXPECT_EQ(payload.link, "tool0");
  EXPECT_LT((payload.dims - Vector3d(0.6, 0.6, 0.02)).norm(), 1e-12);
}

TEST(Collision, PayloadBoxOffThePanelFrameIsRejected)
{
  // The generated model's "panel" frame is the qualified payload's centre on tool0. A payload box
  // elsewhere on tool0 (another payload than the model was generated for) matches no panel alias:
  // the model must refuse it instead of silently building no panel rows.
  std::vector<mmg::Primitive> moved = primitives();
  moved.back().pose.translation() += Vector3d(0.1, 0.0, 0.0);
  EXPECT_THROW(CollisionModel(moved, CollisionConfig{}), std::invalid_argument);
  // Without a payload box (the URDF alone) there is nothing to alias.
  std::vector<mmg::Primitive> bare = primitives();
  bare.pop_back();
  EXPECT_NO_THROW(CollisionModel(bare, CollisionConfig{}));
  EXPECT_NO_THROW(CollisionModel(primitives(), CollisionConfig{}));
}

TEST(Collision, DistanceMatchesAnalyticBoxBox)
{
  const State x = state(1.0, -2.0, 0.0, joints("home"));
  const Eigen::Isometry3d panel = link_pose(link_index("panel"), x);
  ASSERT_LT((panel.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-9) << "home panel is axis-aligned";
  const double bottom = panel.translation().z() - 0.02;
  const double gap = 0.25;
  // A 0.2 m box under a panel corner, far from the wrist links on the panel's centre line.
  const Vector3d center = panel.translation() + Vector3d(0.4, 0.4, 0.0);
  const World world = world_of({box("box", Vector3d(center.x(), center.y(), bottom - gap - 0.15),
                                    Vector3d(0.2, 0.2, 0.3))});
  CollisionStats stats;
  const auto rows = model(unlimited()).rows({x}, 0.1, 0.0, world, &stats);
  const DistanceRow * row = find_row(rows, 0, link_index("panel"), "box");
  ASSERT_NE(row, nullptr);
  EXPECT_NEAR(row->distance, gap, 1e-6);
  for (const auto & other : rows) EXPECT_GE(other.distance, row->distance - 1e-9) << other.obstacle;
  // The normal is +z, so the gradient is the z row of the panel point's Jacobian: no base terms.
  EXPECT_NEAR(row->gradient(0), 0.0, 1e-6);
  EXPECT_NEAR(row->gradient(1), 0.0, 1e-6);
  EXPECT_EQ(stats.active_pairs, static_cast<int>(rows.size()));
  EXPECT_EQ(stats.dropped_pairs, 0);
}

TEST(Collision, GradientMatchesFiniteDifferences)
{
  // Random states around the 1.05 m gate posts; every kind of row (obstacle, panel versus
  // base, floor) checked against central differences of the reported distance.
  const World world = world_of({box("left", Vector3d(-7.225, -7.0, 1.2), Vector3d(0.4, 0.4, 2.4)),
                                box("right", Vector3d(-7.225, -8.45, 1.2), Vector3d(0.4, 0.4, 2.4))});
  const CollisionModel collision = model(unlimited());
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> along(-8.2, -6.2), across(-8.1, -7.35), yaw(-M_PI, M_PI), joint(-2.0, 2.0);
  const double h = 1e-4;  // the GJK distance is accurate to about 1e-9 m
  int obstacle_rows = 0, self_rows = 0, floor_rows = 0;
  for (int trial = 0; trial < 60; ++trial) {
    State x;
    x << along(rng), across(rng), yaw(rng), joint(rng), joint(rng), joint(rng), joint(rng), joint(rng), joint(rng);
    for (const auto & row : collision.rows({x}, 0.1, 0.0, world, nullptr)) {
      if (row.distance < 0.02) continue;  // nearest features of touching shapes are not smooth
      bool found = true;
      Eigen::Matrix<double, 1, kNx> numeric;
      for (int j = 0; j < kNx && found; ++j) {
        State plus = x, minus = x;
        plus(j) += h;
        minus(j) -= h;
        const auto rows_plus = collision.rows({plus}, 0.1, 0.0, world, nullptr);
        const auto rows_minus = collision.rows({minus}, 0.1, 0.0, world, nullptr);
        const DistanceRow * p = find_row(rows_plus, 0, row.link, row.obstacle);
        const DistanceRow * m = find_row(rows_minus, 0, row.link, row.obstacle);
        found = p && m;
        if (found) numeric(j) = (p->distance - m->distance) / (2.0 * h);
      }
      if (!found) continue;
      EXPECT_LT((numeric - row.gradient).cwiseAbs().maxCoeff(), 1e-4)
        << row.obstacle << " link " << row.link << " d " << row.distance << "\n analytic " << row.gradient
        << "\n numeric  " << numeric;
      if (row.obstacle == "floor") ++floor_rows;
      else if (row.obstacle.rfind("self/", 0) == 0) ++self_rows;
      else ++obstacle_rows;
    }
  }
  std::printf("checked rows: %d obstacle, %d panel-base, %d floor\n", obstacle_rows, self_rows, floor_rows);
  EXPECT_GT(obstacle_rows, 50);
  EXPECT_GT(self_rows, 5);
  EXPECT_GT(floor_rows, 5);
}

TEST(Collision, PanelThicknessNotInflated)
{
  // Vertical carry driving along -x through the 1.05 m gate as the narrow-gate scenario. The
  // plate hangs 0.225 m to the side of the arm, so with the base on the gate centreline the
  // two posts see it at 0.525 -+ 0.225 - 0.02 m: the per-side distances add up to the gate
  // width minus the 0.04 m plate thickness, with nothing inflated.
  const State centred_base = state(kGateCentre.x() + 0.08, kGateCentre.y(), M_PI, joints("vertical_carry"));
  const int panel = link_index("panel");
  const auto rows = model(unlimited()).rows({centred_base}, 0.1, 0.0, map_world(), nullptr);
  const DistanceRow * left = find_row(rows, 0, panel, kPostLeft);
  const DistanceRow * right = find_row(rows, 0, panel, kPostRight);
  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  EXPECT_NEAR(left->distance + right->distance, 1.05 - 0.04, 1e-6);
  EXPECT_NEAR(std::min(left->distance, right->distance), 0.28, 1e-6);
  for (const auto & row : rows) EXPECT_GE(row.distance, CollisionConfig{}.margin_m) << row.obstacle << " " << row.link;

  // With the plate itself centred in the gate it reports (1.05 - 0.04) / 2 = 0.505 m per side
  // (the base, then 0.225 m off the centreline, is what touches the posts).
  const auto centred = model(unlimited()).rows({with_panel_over(kGateCentre, M_PI, joints("vertical_carry"))}, 0.1,
                                               0.0, map_world(), nullptr);
  for (const auto & post : {kPostLeft, kPostRight}) {
    const DistanceRow * row = find_row(centred, 0, panel, post);
    ASSERT_NE(row, nullptr) << post;
    EXPECT_NEAR(row->distance, 0.505, 1e-6) << post;
  }
}

TEST(Collision, PenetratingRowPointsOutOfTheObstacle)
{
  // home spans 1.20 m across the 1.05 m gate: the panel overlaps both posts by about 0.075 m.
  const State x = with_panel_over(kGateCentre, M_PI, joints("home"));
  const auto rows = model(unlimited()).rows({x}, 0.1, 0.0, map_world(), nullptr);
  const DistanceRow * left = find_row(rows, 0, link_index("panel"), kPostLeft);
  const DistanceRow * right = find_row(rows, 0, link_index("panel"), kPostRight);
  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  EXPECT_NEAR(left->distance, -0.075, 0.03);
  EXPECT_NEAR(right->distance, -0.075, 0.03);
  // The left post is at larger y: moving the base towards -y frees the panel from it.
  EXPECT_LT(left->gradient(1), -0.5);
  EXPECT_GT(right->gradient(1), 0.5);
}

TEST(Collision, OnlyObstaclesWithinActivation)
{
  // home at the origin: the panel reaches x in [-0.68, 0.52] (centre -0.08), the base +-0.44.
  const State x = state(0.0, 0.0, 0.0, joints("home"));
  const World world = world_of({box("near", Vector3d(0.52 + 0.7 + 0.1, 0.0, 1.0), Vector3d(0.2, 0.4, 2.0)),
                                box("far", Vector3d(-0.68 - 0.9 - 0.1, 0.0, 1.0), Vector3d(0.2, 0.4, 2.0))});
  const auto rows = model(unlimited()).rows({x}, 0.1, 0.0, world, nullptr);
  bool near = false;
  for (const auto & row : rows) {
    EXPECT_NE(row.obstacle, "far");
    EXPECT_LE(row.distance, CollisionConfig{}.activation_m);
    near = near || row.obstacle == "near";
  }
  EXPECT_TRUE(near);
  const DistanceRow * panel = find_row(rows, 0, link_index("panel"), "near");
  ASSERT_NE(panel, nullptr);
  EXPECT_NEAR(panel->distance, 0.7, 1e-6);
}

TEST(Collision, PairBudgetKeepsClosestAndCountsDropped)
{
  // A ring of posts at different distances around a robot driving forward.
  std::vector<mmn::Box> ring;
  for (int i = 0; i < 12; ++i) {
    const double angle = 2.0 * M_PI * i / 12.0, radius = 1.1 + 0.03 * i;
    ring.push_back(box("post" + std::to_string(i), Vector3d(radius * std::cos(angle), radius * std::sin(angle), 1.0),
                       Vector3d(0.2, 0.2, 2.0)));
  }
  const World world = world_of(ring);
  std::vector<State> nominal;
  for (int k = 0; k < 3; ++k) nominal.push_back(state(0.02 * k, 0.0, 0.0, joints("vertical_carry")));

  CollisionStats all_stats, kept_stats;
  const auto all = model(unlimited()).rows(nominal, 0.1, 0.0, world, &all_stats);
  const auto kept = model().rows(nominal, 0.1, 0.0, world, &kept_stats);
  EXPECT_EQ(all_stats.dropped_pairs, 0);
  EXPECT_EQ(kept_stats.active_pairs, static_cast<int>(kept.size()));
  EXPECT_EQ(kept_stats.active_pairs + kept_stats.dropped_pairs, all_stats.active_pairs);
  for (int k = 0; k < 3; ++k) {
    std::vector<double> every, budget;
    for (const auto & row : all)
      if (row.stage == k) every.push_back(row.distance);
    for (const auto & row : kept)
      if (row.stage == k) budget.push_back(row.distance);
    ASSERT_GT(every.size(), 8u) << "stage " << k << " must exceed the budget";
    ASSERT_EQ(budget.size(), 8u);
    std::sort(every.begin(), every.end());
    std::sort(budget.begin(), budget.end());
    for (int i = 0; i < 8; ++i) EXPECT_NEAR(budget[i], every[i], 1e-12);
  }
}

TEST(Collision, BeamAboveBaseOnlyConstrainsArmAndPanel)
{
  // A beam whose bottom is at z 1.1 m just ahead of the base: the base (top 0.32 m, arm mount
  // 0.50 m) cannot reach it by driving, so only the arm and the panel get rows.
  const State x = state(0.0, 0.0, 0.0, joints("home"));
  const World beam = world_of({box("beam", Vector3d(0.6, 0.0, 1.2), Vector3d(0.3, 3.0, 0.2))});
  const auto rows = model(unlimited()).rows({x}, 0.1, 0.0, beam, nullptr);
  EXPECT_NE(find_row(rows, 0, link_index("panel"), "beam"), nullptr);
  for (const auto & row : rows) EXPECT_FALSE(is_base_link(row.link)) << row.obstacle << " " << row.link;

  // The same footprint on the ground does constrain the base.
  const World block = world_of({box("block", Vector3d(0.8, 0.0, 0.15), Vector3d(0.3, 3.0, 0.3))});
  bool base_row = false;
  for (const auto & row : model(unlimited()).rows({x}, 0.1, 0.0, block, nullptr))
    base_row = base_row || is_base_link(row.link);
  EXPECT_TRUE(base_row);
}

TEST(Collision, FloorRowForLoweredPanel)
{
  const World empty;
  // home keeps the panel 1.4 m above the floor plane: no floor row.
  for (const auto & row : model(unlimited()).rows({state(0, 0, 0, joints("home"))}, 0.1, 0.0, empty, nullptr))
    EXPECT_NE(row.obstacle, "floor");

  // level_extension with the shoulder lowered further tilts the panel down towards the floor.
  Eigen::Matrix<double, 6, 1> q = joints("level_extension");
  q(1) += 0.3;
  const State x = state(0, 0, 0, q);
  const double expected = panel_bottom(x) - CollisionConfig{}.floor_top_m;
  ASSERT_GT(expected, 0.0);
  ASSERT_LT(expected, CollisionConfig{}.activation_m);
  const auto rows = model(unlimited()).rows({x}, 0.1, 0.0, empty, nullptr);
  const DistanceRow * row = find_row(rows, 0, link_index("panel"), "floor");
  ASSERT_NE(row, nullptr);
  EXPECT_NEAR(row->distance, expected, 1e-9);
  EXPECT_NEAR(row->gradient(0), 0.0, 1e-12);  // the base moves in the plane
  EXPECT_NEAR(row->gradient(1), 0.0, 1e-12);
  EXPECT_NEAR(row->gradient(2), 0.0, 1e-12);
  EXPECT_LT(row->gradient(4), 0.0);  // lowering the shoulder further lowers the panel

  // Below the plane the distance turns negative.
  q(1) += 0.5;
  const auto lower = model(unlimited()).rows({state(0, 0, 0, q)}, 0.1, 0.0, empty, nullptr);
  const DistanceRow * penetrating = find_row(lower, 0, link_index("panel"), "floor");
  ASSERT_NE(penetrating, nullptr);
  EXPECT_NEAR(penetrating->distance, panel_bottom(state(0, 0, 0, q)) - CollisionConfig{}.floor_top_m, 1e-9);
  EXPECT_LT(penetrating->distance, 0.0);
}

TEST(Collision, PanelVersusBaseRow)
{
  // A panel folded down in front of the base gets one row against its closest base primitive.
  Eigen::Matrix<double, 6, 1> q = joints("level_extension");
  q(1) += 0.3;
  const auto rows = model(unlimited()).rows({state(0, 0, 0, q)}, 0.1, 0.0, World{}, nullptr);
  int self_rows = 0;
  for (const auto & row : rows) {
    if (row.obstacle.rfind("self/", 0) != 0) continue;
    ++self_rows;
    EXPECT_EQ(row.link, link_index("panel"));
    EXPECT_TRUE(base_links().count(row.obstacle.substr(5))) << row.obstacle;
    EXPECT_NEAR(row.gradient(0), 0.0, 1e-9) << "panel and base move together with the base pose";
    EXPECT_NEAR(row.gradient(1), 0.0, 1e-9);
    EXPECT_NEAR(row.gradient(2), 0.0, 1e-9);
  }
  EXPECT_EQ(self_rows, 1);
}

TEST(Collision, HomeClearanceDropsInNarrowGate)
{
  const CollisionModel collision = model();
  const World world = map_world();
  // Nominal base poses along the gate centreline in vertical carry, as the narrow-gate scenario.
  std::vector<State> through;
  for (int k = 0; k < 5; ++k)
    through.push_back(state(kGateCentre.x() + 0.2 - 0.1 * k, kGateCentre.y(), M_PI, joints("vertical_carry")));
  const double home = collision.clearance_with_joints(through, joints("home"), 0.1, 0.0, world);
  const double vertical = collision.clearance_with_joints(through, joints("vertical_carry"), 0.1, 0.0, world);
  std::printf("gate clearance: home %.3f m, vertical_carry %.3f m\n", home, vertical);
  EXPECT_LT(home, CollisionConfig{}.margin_m);
  EXPECT_NEAR(home, -0.075, 1e-6);  // the 1.20 m plate in the 1.05 m opening
  EXPECT_GT(vertical, CollisionConfig{}.margin_m);  // the plate 0.28 m from the nearer post

  // Open space at the narrow-gate scenario start.
  std::vector<State> open;
  for (int k = 0; k < 5; ++k) open.push_back(state(-4.2 - 0.1 * k, -7.725, M_PI, joints("vertical_carry")));
  const double open_home = collision.clearance_with_joints(open, joints("home"), 0.1, 0.0, world);
  std::printf("open clearance: home %.3f m\n", open_home);
  EXPECT_GT(open_home, 0.30);
  EXPECT_LE(open_home, CollisionConfig{}.activation_m);
}

// The overhead course of scenarios.yaml (Phase 4): its 1.05 m gate needs vertical carry, and its
// beam needs the panel lowered: vertical carry (panel top 1.92 m) runs into it while home (panel
// top 1.565 m) passes under it with at least 0.10 m.
TEST(Collision, OverheadCourseGateNeedsVerticalCarryAndBeamNeedsHome)
{
  const std::string share = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const mmn::Json scenario = mmn::load_yaml_file(share + "/config/scenarios.yaml").at("scenarios").at("overhead_course_nav");
  const World course = world_of(mmn::known_boxes(share + "/maps/construction_site.boxes.yaml", &scenario));
  const double y = scenario.at("start")[1].get<double>();
  std::optional<mmn::Box> beam;
  double gate_x = std::numeric_limits<double>::quiet_NaN();
  for (const auto & obstacle : scenario.at("obstacles")) {
    if (obstacle.at("name") == "beam") beam = mmn::scenario_box(obstacle);
    if (obstacle.at("name") == "gate_post_south") gate_x = obstacle.at("x").get<double>();
  }
  ASSERT_TRUE(beam.has_value());
  ASSERT_FALSE(std::isnan(gate_x));
  const CollisionModel collision = model();

  // Base poses on the course line (heading +x) while the panel passes under the beam.
  std::vector<State> under;
  for (int k = -10; k <= 10; ++k) under.push_back(state(beam->center.x() + 0.1 * k, y, 0.0, joints("home")));
  const World beam_only = world_of({*beam});
  const double home = collision.clearance_with_joints(under, joints("home"), 0.1, 0.0, beam_only);
  const double vertical = collision.clearance_with_joints(under, joints("vertical_carry"), 0.1, 0.0, beam_only);
  std::printf("beam clearance: home %.3f m, vertical_carry %.3f m\n", home, vertical);
  EXPECT_GE(home, 0.10);
  EXPECT_LT(vertical, 0.0);
  // Every course box and the map: home keeps the margin all the way under the beam.
  EXPECT_GE(collision.clearance_with_joints(under, joints("home"), 0.1, 0.0, course), 0.10);

  // Along the gate centreline, as HomeClearanceDropsInNarrowGate.
  std::vector<State> through;
  for (int k = 0; k < 5; ++k) through.push_back(state(gate_x - 0.2 + 0.1 * k, y, 0.0, joints("vertical_carry")));
  const double gate_home = collision.clearance_with_joints(through, joints("home"), 0.1, 0.0, course);
  const double gate_vertical = collision.clearance_with_joints(through, joints("vertical_carry"), 0.1, 0.0, course);
  std::printf("course gate clearance: home %.3f m, vertical_carry %.3f m\n", gate_home, gate_vertical);
  EXPECT_LT(gate_home, 0.0);
  EXPECT_GT(gate_vertical, CollisionConfig{}.margin_m);
}

TEST(Collision, MoverPredictionUsedPerStage)
{
  // A worker walking into the robot's lane appears in the late stages only.
  World world;
  const Vector3d size(0.5, 0.5, 1.8);
  world.update_mover("worker", Vector2d(3.0, 0.0), size, 0.0);
  world.update_mover("worker", Vector2d(2.9, 0.0), size, 0.1);  // 1 m/s towards the robot
  std::vector<State> nominal(20, state(0.0, 0.0, 0.0, joints("home")));
  const auto rows = model().rows(nominal, 0.1, 0.1, world, nullptr);
  std::set<int> stages;
  for (const auto & row : rows)
    if (row.obstacle == "worker") stages.insert(row.stage);
  ASSERT_FALSE(stages.empty());
  EXPECT_GT(*stages.begin(), 5);
  EXPECT_EQ(*stages.rbegin(), 19);
}

TEST(Collision, BoundsLoseNoPairWithinActivation)
{
  // The pruning by lower and upper bounds must give the same rows as evaluating every pair:
  // compare against an activation distance so large that every pair is evaluated exactly.
  World world = map_world();
  world.update_mover("worker", Vector2d(-7.0, -7.9), Vector3d(0.5, 0.5, 1.8), 0.0);
  CollisionConfig everything = unlimited();
  everything.activation_m = 100.0;
  const CollisionModel all_pairs = model(everything), within = model(unlimited()), budget = model();
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> along(-8.2, -6.2), across(-8.3, -7.15), yaw(-M_PI, M_PI), joint(-2.0, 2.0);
  for (int trial = 0; trial < 30; ++trial) {
    std::vector<State> nominal;
    for (int k = 0; k < 4; ++k) {
      State x;
      x << along(rng), across(rng), yaw(rng), joint(rng), joint(rng), joint(rng), joint(rng), joint(rng), joint(rng);
      nominal.push_back(x);
    }
    std::multiset<std::tuple<int, int, std::string, long>> expected, actual;
    auto key = [](const DistanceRow & row) {
      return std::make_tuple(row.stage, row.link, row.obstacle, std::lround(row.distance * 1e9));
    };
    for (const auto & row : all_pairs.rows(nominal, 0.1, 0.0, world, nullptr))
      if (row.distance <= CollisionConfig{}.activation_m) expected.insert(key(row));
    CollisionStats within_stats, budget_stats;
    const auto within_rows = within.rows(nominal, 0.1, 0.0, world, &within_stats);
    for (const auto & row : within_rows) actual.insert(key(row));
    EXPECT_EQ(actual, expected) << "trial " << trial;

    // The clearance prunes by the same bounds.
    const Eigen::Matrix<double, 6, 1> q = nominal[0].tail<6>();
    EXPECT_NEAR(budget.clearance_with_joints(nominal, q, 0.1, 0.0, world),
                std::min(CollisionConfig{}.activation_m, all_pairs.clearance_with_joints(nominal, q, 0.1, 0.0, world)),
                1e-12) << "trial " << trial;

    const auto budget_rows = budget.rows(nominal, 0.1, 0.0, world, &budget_stats);
    EXPECT_EQ(budget_stats.active_pairs + budget_stats.dropped_pairs, within_stats.active_pairs) << "trial " << trial;
    for (int k = 0; k < 4; ++k) {
      std::vector<double> every, kept;
      for (const auto & row : within_rows)
        if (row.stage == k) every.push_back(row.distance);
      for (const auto & row : budget_rows)
        if (row.stage == k) kept.push_back(row.distance);
      std::sort(every.begin(), every.end());
      ASSERT_EQ(kept.size(), std::min<size_t>(every.size(), 8u));
      for (size_t i = 0; i < kept.size(); ++i) EXPECT_NEAR(kept[i], every[i], 1e-12) << "trial " << trial;
    }
  }
}

TEST(Collision, RowsAreInStageOrder)
{
  // The slack gate maps slack j to the (j % budget)-th row of stage j / budget + 1 by walking the
  // rows in order (Controller::gate_passes, which also checks it): rows() must emit them by stage,
  // with the per-stage budget applied, also with movers and the full map.
  const CollisionModel collision = model();
  World world = map_world();
  world.update_mover("worker", Vector2d(-7.8, -6.6), Vector3d(0.5, 0.5, 1.8), 0.0);
  world.update_mover("worker", Vector2d(-7.8, -6.65), Vector3d(0.5, 0.5, 1.8), 0.1);
  std::vector<State> nominal;
  for (int k = 0; k < 20; ++k)
    nominal.push_back(with_panel_over(kGateCentre + Vector2d(0.3 - 0.03 * k, 0.0), M_PI, joints("vertical_carry")));
  const auto rows = collision.rows(nominal, 0.1, 0.1, world, nullptr);
  ASSERT_FALSE(rows.empty());
  std::vector<int> per_stage(nominal.size(), 0);
  for (size_t r = 0; r < rows.size(); ++r) {
    if (r > 0) ASSERT_LE(rows[r - 1].stage, rows[r].stage) << "row " << r;
    ASSERT_LE(++per_stage[static_cast<size_t>(rows[r].stage)], CollisionConfig{}.pairs_per_stage);
  }
}

TEST(Collision, TwentyStagesUnderOneMillisecond)
{
  // Vertical carry through the narrow gate at 0.3 m/s, 20 stages of 0.1 s, the full map and
  // one mover: median of 200 calls.
  const CollisionModel collision = model();
  World world = map_world();
  world.update_mover("worker", Vector2d(-7.8, -6.6), Vector3d(0.5, 0.5, 1.8), 0.0);
  world.update_mover("worker", Vector2d(-7.8, -6.65), Vector3d(0.5, 0.5, 1.8), 0.1);
  std::vector<State> nominal;
  for (int k = 0; k < 20; ++k)
    nominal.push_back(with_panel_over(kGateCentre + Vector2d(0.3 - 0.03 * k, 0.0), M_PI, joints("vertical_carry")));
  std::vector<double> times;
  CollisionStats stats;
  for (int i = 0; i < 200; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto rows = collision.rows(nominal, 0.1, 0.1, world, &stats);
    times.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    ASSERT_FALSE(rows.empty());
  }
  std::sort(times.begin(), times.end());
  const double median = times[times.size() / 2];
  std::printf("20 stages: median %.3f ms, p90 %.3f ms, max %.3f ms; %d rows, %d dropped (stats compute %.3f ms)\n",
              median * 1e3, times[180] * 1e3, times.back() * 1e3, stats.active_pairs, stats.dropped_pairs,
              stats.compute_s * 1e3);
  RecordProperty("median_ms", std::to_string(median * 1e3));
#if MPC_OPTIMIZED_BUILD
  EXPECT_LT(median, 1e-3);
#else
  GTEST_SKIP() << "timing only checked in an optimized build; median " << median * 1e3 << " ms";
#endif
}


TEST(Collision, PairDistanceIsTheRowsPairNow)
{
  // The slack gate's current violation: a row's pair evaluated at a state with the obstacles where
  // they are now. At the row's own state it is the row's distance, for obstacle, floor and self rows.
  Eigen::Matrix<double, 6, 1> q = joints("level_extension");
  q(1) += 0.3;  // the panel folded down in front of the base: floor and self rows
  const State x = state(-7.0, -7.725, M_PI, q);
  World world = map_world();
  // A worker 0.3 m ahead of the base's front (facing -x), walking towards it at 0.5 m/s.
  world.update_mover("worker", Vector2d(-8.17, -7.725), Vector3d(0.5, 0.5, 1.8), 0.0);
  world.update_mover("worker", Vector2d(-8.12, -7.725), Vector3d(0.5, 0.5, 1.8), 0.1);
  const CollisionModel collision = model(unlimited());
  const auto rows = collision.rows({x}, 0.1, 0.1, world, nullptr);
  std::set<std::string> kinds;
  for (const auto & row : rows) {
    ASSERT_GE(row.primitive, 0);
    EXPECT_NEAR(collision.pair_distance(x, row, world, 0.1), row.distance, 1e-9) << row.obstacle;
    kinds.insert(row.obstacle == "floor" ? "floor" : row.obstacle.rfind("self/", 0) == 0 ? "self" : row.obstacle);
  }
  EXPECT_TRUE(kinds.count("floor"));
  EXPECT_TRUE(kinds.count("self"));
  EXPECT_TRUE(kinds.count("worker"));
  // A later stage's row is evaluated with the mover where it is now, not where it is predicted.
  const auto ahead = collision.rows({x, x, x, x, x, x}, 0.1, 0.1, world, nullptr);
  int later = 0;
  for (const auto & row : ahead) {
    if (row.stage != 5 || row.obstacle != "worker") continue;
    ++later;
    EXPECT_GT(collision.pair_distance(x, row, world, 0.1), row.distance + 0.1);
  }
  EXPECT_GT(later, 0);
  // A row without a primitive stands for every primitive of its link (the closest one).
  DistanceRow any = rows.front();
  any.primitive = -1;
  EXPECT_LE(collision.pair_distance(x, any, world, 0.1), rows.front().distance + 1e-12);
  // An obstacle no longer in the world: no current distance.
  DistanceRow gone = rows.front();
  gone.obstacle = "left_the_scene";
  EXPECT_TRUE(std::isinf(collision.pair_distance(x, gone, world, 0.1)));
}

// Phase 4B-A spec Section 2: base rows carry the base margin (outside the StopZone), arm and panel rows
// the collision margin; floor and self rows keep the collision margin.
TEST(Collision, BaseRowsCarryTheBaseMargin)
{
  CollisionConfig cfg;
  cfg.base_margin_m = 0.07;
  cfg.pairs_per_stage = 1000;
  const CollisionModel model = ::model(cfg);
  const Eigen::Matrix<double, 6, 1> carry = joints("carry");
  // A tall wall beside the robot: the base, the arm and the panel all within the activation distance.
  const World world = world_of({mmn::Box{"wall", Eigen::Vector3d(0.0, -0.80, 1.0), Eigen::Vector3d(3.0, 0.2, 2.0), 0.0}});
  const auto rows = model.rows({state(0, 0, 0, carry)}, 0.1, 0.0, world, nullptr);
  bool base = false, other = false;
  for (const auto & row : rows) {
    if (row.obstacle == "floor" || row.obstacle.rfind("self/", 0) == 0) {
      EXPECT_DOUBLE_EQ(row.margin, 0.05) << row.obstacle;
      EXPECT_FALSE(row.base) << row.obstacle;
    } else if (model.is_base(row.primitive)) {
      base = true;
      EXPECT_DOUBLE_EQ(row.margin, 0.07) << row.link;
      EXPECT_TRUE(row.base) << row.link;  // the flag the inside slack weight skips (Phase 4B-A Task 11)
    } else {
      other = true;
      EXPECT_DOUBLE_EQ(row.margin, 0.05) << row.link;
      EXPECT_FALSE(row.base) << row.link;
    }
  }
  EXPECT_TRUE(base);
  EXPECT_TRUE(other);
}

TEST(Collision, BaseMarginDefaultsToTheMargin)
{
  const CollisionConfig cfg;
  EXPECT_DOUBLE_EQ(cfg.base_margin_m, cfg.margin_m);
}
