// The obstacle prefilter of the collision step: CollisionModel::rows, clearance_with_joints and
// pair_distance give exactly what the unfiltered step (collision_reference.hpp) gives, on the V1
// and V2 sites, standing and driving, with a mover; and the V2 site fits the 50 Hz cycle.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "collision_reference.hpp"
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
using Joints = Eigen::Matrix<double, 6, 1>;

namespace
{
constexpr int kStages = 21;  // horizon 20 at dt 0.1 s: stages 0..20
constexpr double kDt = 0.1;

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

Joints joints(const std::string & name)
{
  Joints q;
  for (int i = 0; i < 6; ++i) q(i) = qualified_payload()["poses_rad"][name][i].get<double>();
  return q;
}

State state(double x, double y, double yaw, const Joints & q)
{
  State s;
  s << x, y, yaw, q;
  return s;
}

World site(const std::string & name)
{
  World world;
  world.set_static(mmn::known_boxes(ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation") +
                                      "/maps/" + name + ".boxes.yaml",
                                    nullptr));
  return world;
}

const World & v1()
{
  static const World world = site("construction_site");
  return world;
}

const World & v2()
{
  static const World world = site("construction_site_v2");
  return world;
}

// Standing at `x` for the whole horizon.
std::vector<State> standing(const State & x)
{
  return std::vector<State>(kStages, x);
}

// Driving straight ahead from `x` at `speed`.
std::vector<State> driving(const State & x, double speed)
{
  std::vector<State> nominal;
  for (int k = 0; k < kStages; ++k) {
    State s = x;
    s(0) += speed * kDt * k * std::cos(x(2));
    s(1) += speed * kDt * k * std::sin(x(2));
    nominal.push_back(s);
  }
  return nominal;
}

struct Case
{
  std::string label;
  const World * world;
  std::vector<State> nominal;
};

std::vector<Case> cases()
{
  const Joints carry = joints("carry");
  return {
    {"V1 open (12, 0)", &v1(), standing(state(12.0, 0.0, M_PI, carry))},
    {"V1 narrow gate (-4.2, -7.725)", &v1(), standing(state(-4.2, -7.725, M_PI, carry))},
    {"V1 gate drive vertical_carry", &v1(), driving(state(-6.8, -7.725, M_PI, joints("vertical_carry")), 0.3)},
    {"V2 spawn (-21, 0)", &v2(), standing(state(-21.0, 0.0, 0.0, carry))},
    {"V2 spine (0, 0)", &v2(), standing(state(0.0, 0.0, 0.0, carry))},
    {"V2 door B approach (16.275, 3.5)", &v2(), standing(state(16.275, 3.5, M_PI / 2.0, carry))},
    {"V2 tunnel (-6.5, 6)", &v2(), standing(state(-6.5, 6.0, M_PI / 2.0, carry))},
    {"V2 door B (16.2, 5.27)", &v2(), standing(state(16.2, 5.27, 1.505, carry))},
    {"V2 tunnel drive 0.3 m/s", &v2(), driving(state(-6.5, 4.5, M_PI / 2.0, carry), 0.3)},
  };
}

void expect_same_rows(const std::vector<DistanceRow> & got, const std::vector<DistanceRow> & want,
                      const std::string & label)
{
  ASSERT_EQ(got.size(), want.size()) << label;
  for (size_t i = 0; i < got.size(); ++i) {
    EXPECT_EQ(got[i].stage, want[i].stage) << label << " row " << i;
    EXPECT_EQ(got[i].link, want[i].link) << label << " row " << i;
    EXPECT_EQ(got[i].primitive, want[i].primitive) << label << " row " << i;
    EXPECT_EQ(got[i].obstacle, want[i].obstacle) << label << " row " << i;
    EXPECT_EQ(got[i].distance, want[i].distance) << label << " row " << i;
    for (int j = 0; j < kNx; ++j) EXPECT_EQ(got[i].gradient(j), want[i].gradient(j)) << label << " row " << i;
  }
}

// rows, clearance_with_joints (with both comfort poses) and pair_distance of every row, against
// the reference at `now`.
void expect_same(const Case & c, double now)
{
  const CollisionModel model(primitives(), CollisionConfig{});
  const reference::Reference oracle(primitives(), CollisionConfig{});
  CollisionStats got_stats, want_stats;
  const auto got = model.rows(c.nominal, kDt, now, *c.world, &got_stats);
  const auto want = oracle.rows(c.nominal, kDt, now, *c.world, &want_stats);
  expect_same_rows(got, want, c.label);
  EXPECT_EQ(got_stats.active_pairs, want_stats.active_pairs) << c.label;
  EXPECT_EQ(got_stats.dropped_pairs, want_stats.dropped_pairs) << c.label;
  for (const std::string pose : {"carry", "home"}) {
    const double got_clearance = model.clearance_with_joints(c.nominal, joints(pose), kDt, now, *c.world);
    EXPECT_EQ(got_clearance, oracle.clearance_with_joints(c.nominal, joints(pose), kDt, now, *c.world))
      << c.label << " " << pose;
    // Phase 4B-A side picker: the per-stage clearances have the same minimum, one per stage.
    const auto per_stage = model.clearance_per_stage(c.nominal, joints(pose), kDt, now, *c.world);
    ASSERT_EQ(per_stage.size(), c.nominal.size()) << c.label;
    EXPECT_DOUBLE_EQ(*std::min_element(per_stage.begin(), per_stage.end()), got_clearance) << c.label << " " << pose;
  }
  for (const auto & row : want)
    EXPECT_EQ(model.pair_distance(c.nominal.front(), row, *c.world, now),
              oracle.pair_distance(c.nominal.front(), row, *c.world, now))
      << c.label << " " << row.obstacle;
  std::printf("%-40s %3zu rows, %2d dropped, clearance %.4f m\n", c.label.c_str(), got.size(),
              got_stats.dropped_pairs, model.clearance_with_joints(c.nominal, joints("carry"), kDt, now, *c.world));
}

template <typename F>
double mean_ms(F && f, int calls)
{
  f();  // warm up
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < calls; ++i) f();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / calls;
}
}  // namespace

TEST(CollisionPrefilter, SameAsUnfilteredOnBothSites)
{
  for (const Case & c : cases()) expect_same(c, 0.0);
}

TEST(CollisionPrefilter, SameAsUnfilteredWithAMover)
{
  // A worker 1.2 m ahead of the robot walking along -x (predicted per stage), and one that went
  // stale (left out of both).
  for (Case c : cases()) {
    World world = *c.world;
    const Vector2d ahead = c.nominal.front().head<2>() + 1.2 * Vector2d(std::cos(c.nominal.front()(2)),
                                                                          std::sin(c.nominal.front()(2)));
    world.update_mover("worker", ahead, Vector3d(0.5, 0.5, 1.8), 0.0);
    world.update_mover("worker", ahead - Vector2d(0.05, 0.0), Vector3d(0.5, 0.5, 1.8), 0.1);
    world.update_mover("stale", ahead + Vector2d(0.0, 0.5), Vector3d(0.5, 0.5, 1.8), -5.0);
    c.world = &world;
    c.label += " + mover";
    expect_same(c, 0.1);
  }
}

TEST(CollisionPrefilter, V2CycleFitsTheBudget)
{
  // rows() + clearance_with_joints() as one controller cycle calls them: mean of 50 calls,
  // standing in the carry pose. The unfiltered reference is timed alongside for comparison.
  const CollisionModel model(primitives(), CollisionConfig{});
  const reference::Reference oracle(primitives(), CollisionConfig{});
  const Joints carry = joints("carry");
  constexpr int kCalls = 50;
  double worst_checked = 0.0;
  for (const Case & c : cases()) {
    CollisionStats stats;
    const double rows = mean_ms([&] { model.rows(c.nominal, kDt, 0.0, *c.world, &stats); }, kCalls);
    const double clearance =
      mean_ms([&] { model.clearance_with_joints(c.nominal, carry, kDt, 0.0, *c.world); }, kCalls);
    const double old_rows = mean_ms([&] { oracle.rows(c.nominal, kDt, 0.0, *c.world, &stats); }, kCalls);
    const double old_clearance =
      mean_ms([&] { oracle.clearance_with_joints(c.nominal, carry, kDt, 0.0, *c.world); }, kCalls);
    std::printf("%-40s rows %6.3f ms (was %6.3f), clearance %6.3f ms (was %6.3f), total %6.3f ms (was %6.3f)\n",
                c.label.c_str(), rows, old_rows, clearance, old_clearance, rows + clearance,
                old_rows + old_clearance);
    if (c.label.rfind("V2 spawn", 0) == 0 || c.label.rfind("V2 tunnel (", 0) == 0)
      worst_checked = std::max(worst_checked, rows + clearance);
  }
  RecordProperty("worst_ms", std::to_string(worst_checked));
#if MPC_OPTIMIZED_BUILD
  EXPECT_LT(worst_checked, 1.5);
#else
  GTEST_SKIP() << "timing only checked in an optimized build; worst " << worst_checked << " ms";
#endif
}
