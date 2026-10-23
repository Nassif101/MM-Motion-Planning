// The frozen B5 (tag b5-frozen-2026-10-22) as a fixture (Phase 4B-B spec Sections 2 and 10): two closed loops
// built from the frozen wholebody_mpc.yaml (test/data/frozen_wholebody_mpc.yaml) must reproduce the recorded
// statuses, iterations and commands of every cycle, so every new switch at its default leaves B5 unchanged.
// MPC_WRITE_GOLDEN=1 rewrites test/data/frozen_golden.json instead (only on the frozen code).
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "closed_loop.hpp"
#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "test_support.hpp"

using namespace mobile_manipulator_mpc;
using namespace mpc_test;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace
{
const std::string kDataDir = MPC_TEST_DATA_DIR;

const MpcConfig & frozen()
{
  static const MpcConfig instance = load_mpc_config(kDataDir + "/frozen_wholebody_mpc.yaml");
  return instance;
}

// As the node builds it (build_controller, wholebody_mpc_node.cpp), from the frozen configuration, at the loop's
// rate (one cycle per plan stage).
std::unique_ptr<Controller> frozen_controller()
{
  return build_controller(frozen(), limits(), mpc_test::urdf(), payload(), 1.0 / frozen().dt, false);
}

// The fixture holds one architecture's numbers: the 500-iteration cycles and 1e-9 commands depend on floating-point
// detail (FMA, vectorization), so another architecture skips rather than fail. Inputs not pinned to the tag: the
// installed URDF, payload, operating envelope and construction_site boxes (a change there is no B5 change).
const char * architecture()
{
#if defined(__aarch64__)
  return "aarch64";
#elif defined(__x86_64__)
  return "x86_64";
#else
  return "other";
#endif
}

PostureTargets frozen_targets()
{
  return posture_targets(payload(), frozen().comfort_pose, frozen().posture_mode, frozen().tight_poses);
}

// Every cycle's status, iterations and command of a closed loop of `cycles` cycles at the plan's dt.
Json run_loop(Controller & c, Plant & plant, const World & world, int cycles)
{
  Json out = Json::array();
  const double period = frozen().dt;
  for (int i = 0; i < cycles; ++i) {
    const CycleOutput & o = c.step(i * period, plant.x, true, world, plant.at_rest());
    Json cycle;
    cycle["status"] = to_string(o.status);
    cycle["iterations"] = o.iterations;
    cycle["command"] = std::vector<double>(o.command.data(), o.command.data() + o.command.size());
    out.push_back(cycle);
    if (o.status == CycleStatus::GoalReached || o.status == CycleStatus::Failed) break;
    plant.step(o.command, period);
  }
  return out;
}

// Guard.LowBeamKeepsComfortAndLowers's world: the panel is lowered under a 1.34 m beam.
Json beam_loop()
{
  auto c = frozen_controller();
  const World world = world_of({mmn::Box{"beam", Vector3d(4.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
  const PostureTargets targets = frozen_targets();
  Plant plant{state(0.0, 0.0, 0.0, targets.comfort)};
  c->start(Route({Vector2d(0, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets);
  return run_loop(*c, plant, world, 160);
}

// RightSwing.ShelfLeftOfTheApproachSwingsRight's world: the side picker swings right at the 1.05 m gate.
Json shelf_loop()
{
  auto c = frozen_controller();
  const World world = world_of_map_plus({mmn::Box{"shelf", Vector3d(-5.0, kGateY - 0.32 - 0.5, 1.475),
                                                  Vector3d(4.0, 1.0, 1.45), 0.0}});
  const PostureTargets targets = frozen_targets();
  Plant plant{state(-1.0, kGateY, M_PI, targets.comfort)};
  c->start(Route({Vector2d(-1.0, kGateY), Vector2d(-13.0, kGateY)}), {-13.0, kGateY, M_PI}, {}, targets);
  return run_loop(*c, plant, world, 130);
}

void expect_same(const Json & expected, const Json & actual, const std::string & name)
{
  ASSERT_EQ(expected.size(), actual.size()) << name << ": cycle count";
  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_EQ(expected[i]["status"], actual[i]["status"]) << name << " cycle " << i;
    ASSERT_EQ(expected[i]["iterations"], actual[i]["iterations"]) << name << " cycle " << i;
    for (size_t j = 0; j < expected[i]["command"].size(); ++j) {
      ASSERT_NEAR(expected[i]["command"][j].get<double>(), actual[i]["command"][j].get<double>(), 1e-9)
        << name << " cycle " << i << " command " << j;
    }
  }
}
}  // namespace

TEST(Golden, DefaultsReproduceFrozenOutput)
{
  Json actual;
  actual["arch"] = architecture();
  actual["beam"] = beam_loop();
  actual["shelf"] = shelf_loop();
  const std::string path = kDataDir + "/frozen_golden.json";
  const char * write = std::getenv("MPC_WRITE_GOLDEN");
  if (write != nullptr && std::string(write) == "1") {
    std::ofstream(path) << actual.dump(1) << "\n";
    GTEST_SKIP() << "wrote " << path;
  }
  std::ifstream file(path);
  ASSERT_TRUE(file.good()) << "missing " << path << " (generate it on the frozen code with MPC_WRITE_GOLDEN=1)";
  const Json expected = Json::parse(file);
  if (expected.value("arch", std::string()) != architecture()) {
    GTEST_SKIP() << "the fixture was generated on " << expected.value("arch", std::string("?")) << ", this is "
                 << architecture();
  }
  expect_same(expected["beam"], actual["beam"], "beam");
  expect_same(expected["shelf"], actual["shelf"], "shelf");
}

TEST(MpcConfig, FrozenYamlLoadsWithDefaults)
{
  // The frozen file has none of the Phase 4B-B keys: every switch takes the frozen behaviour.
  const MpcConfig & c = frozen();
  EXPECT_EQ(c.solver.backend, "osqp");
  EXPECT_TRUE(c.solver.warm_start);
  EXPECT_FALSE(c.solver.deadline_cap);
  EXPECT_DOUBLE_EQ(c.solver.publish_margin_s, 0.002);
  EXPECT_EQ(c.solver.hpipm.partial_condensing_blocks, 0);
  EXPECT_EQ(c.qp_iterations, 1);
  EXPECT_DOUBLE_EQ(c.iter2_budget, 0.5);
  EXPECT_DOUBLE_EQ(c.iter2_cost_tolerance, 0.01);
  EXPECT_FALSE(c.rti_split);
  EXPECT_DOUBLE_EQ(c.rti_max_deviation.base_m, 0.05);
  EXPECT_DOUBLE_EQ(c.rti_max_deviation.yaw_rad, 0.05);
  EXPECT_DOUBLE_EQ(c.rti_max_deviation.joint_rad, 0.05);
  EXPECT_FALSE(c.collision.self_rows);
  EXPECT_EQ(c.collision.self_pairs_per_stage, 2);
  EXPECT_TRUE(c.collision.self_links.empty());
}

TEST(MpcConfig, SweepKeysLoadAndDefaultToFrozen)
{
  // Phase 4B-B Stage A factors: OSQP's rho adaptation interval and the collision fidelity, optional in the YAML.
  EXPECT_EQ(frozen().solver.osqp_adaptive_rho_interval, 100);
  EXPECT_EQ(frozen().collision.pairs_per_stage, 8);
  EXPECT_DOUBLE_EQ(frozen().collision.activation_m, 0.8);
  const std::string path = ::testing::TempDir() + "/sweep_keys.yaml";
  std::string text;
  {
    std::ifstream in(kDataDir + "/frozen_wholebody_mpc.yaml");
    text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  text.replace(text.find("solver:\n"), 8, "solver:\n  adaptive_rho_interval: 25\n");
  text.replace(text.find("collision:\n"), 11, "collision:\n  pairs_per_stage: 12\n  activation_m: 1.0\n  margin_m: 0.07\n");
  std::ofstream(path) << text;
  const MpcConfig c = load_mpc_config(path);
  EXPECT_EQ(c.solver.osqp_adaptive_rho_interval, 25);
  EXPECT_EQ(c.collision.pairs_per_stage, 12);
  EXPECT_DOUBLE_EQ(c.collision.activation_m, 1.0);
  EXPECT_DOUBLE_EQ(c.collision.margin_m, 0.07);
  EXPECT_DOUBLE_EQ(frozen().collision.margin_m, 0.05);
}
