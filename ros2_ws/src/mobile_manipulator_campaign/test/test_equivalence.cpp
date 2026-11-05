// E1 and E2 (MuJoCo plant spec Section 8, plan Task 4): the MuJoCo plant in ideal mode with the breakaway is
// indistinguishable from the kinematic bench on the corpus cells, and the same seed gives the same run bit for bit.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <set>
#include <fstream>
#include <vector>

#include "mobile_manipulator_campaign/closed_loop_case.hpp"
#include "mobile_manipulator_campaign/mujoco_plant.hpp"
#include "mobile_manipulator_campaign/scenario_setup.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

namespace
{
std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

struct Cell
{
  std::string cell, scenario, bag;  // bag: absolute, "" synthetic
};

std::vector<Cell> corpus()
{
  const auto json = mobile_manipulator_navigation::Json::parse(std::ifstream(std::string(REPO_ROOT) + "/" + CORPUS));
  std::vector<Cell> cells;
  for (const auto & c : json.at("cells"))
    cells.push_back({c.at("cell").get<std::string>(), c.at("scenario").get<std::string>(),
                     c.contains("bag") ? std::string(REPO_ROOT) + "/" + c.at("bag").get<std::string>() : ""});
  return cells;
}

struct Trace
{
  std::vector<double> t;
  std::vector<mmm::State> x;
};

mmc::MujocoPlant ideal_plant()
{
  auto profile = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share);
  profile.ideal_breakaway = true;  // the kinematic plant's static friction
  return mmc::MujocoPlant(profile, share("mobile_manipulator_plant") + "/models/site_v2.xml", mmp::PlantMode::Ideal);
}

mobile_manipulator_medium::ClosedLoopResult run(const mmc::ClosedLoopCase & c, mmm::ClosedLoopPlant & plant, Trace & trace)
{
  mmc::CaseOptions o;
  o.on_cycle = [&](double t, const mmm::State & x) {
    trace.t.push_back(t);
    trace.x.push_back(x);
  };
  return mmc::run_case(c, plant, o);
}
}  // namespace

class Equivalence : public ::testing::TestWithParam<int>
{
};

// The closed loop amplifies rounding-sized differences (a start yaw moved by 1e-15 rad changes a run by up to metres and
// seconds, and in two cells turns success into a timeout), so no plant can match the kinematic bench trajectory for
// trajectory. The plant model itself is gated open loop (OpenLoopReplayMatchesKinematicPlant). Closed loop (provisional,
// the user's decision at CP1): the MuJoCo run ends in a status the rounding ensemble (the unperturbed kinematic run and
// eight with the start yaw moved by +-1..4 x 1e-15 rad) reaches; time and deviation are reported against it.
TEST_P(Equivalence, IdealPlantMatchesKinematicBench)
{
  const auto cells = corpus();
  ASSERT_LT(GetParam(), static_cast<int>(cells.size()));
  const Cell & cell = cells[GetParam()];
  if (!cell.bag.empty() && !std::filesystem::exists(cell.bag)) GTEST_SKIP() << cell.cell << ": recording absent (" << cell.bag << ")";
  const mmc::ClosedLoopCase c = mmc::make_case(cell.scenario, cell.bag);

  Trace kin_trace;
  mmm::KinematicClosedLoop kinematic(c.start, c.bench->limits().breakaway);
  const auto kin = run(c, kinematic, kin_trace);
  auto deviation = [&](const Trace & other) {
    double dev = 0.0;
    for (size_t k = 0; k < std::min(kin_trace.x.size(), other.x.size()); ++k)
      dev = std::max(dev, (kin_trace.x[k].head<2>() - other.x[k].head<2>()).norm());
    return dev;
  };
  double t_lo = kin.time_s, t_hi = kin.time_s, dev_hi = 0.0;
  bool same_status = true;
  std::set<std::string> statuses{kin.end_status};
  for (int k : {-4, -3, -2, -1, 1, 2, 3, 4}) {
    mmm::State moved = c.start;
    moved(2) += k * 1e-15;
    Trace trace;
    mmm::KinematicClosedLoop perturbed(moved, c.bench->limits().breakaway);
    const auto r = run(c, perturbed, trace);
    same_status = same_status && r.end_status == kin.end_status;
    statuses.insert(r.end_status);
    t_lo = std::min(t_lo, r.time_s);
    t_hi = std::max(t_hi, r.time_s);
    dev_hi = std::max(dev_hi, deviation(trace));
  }
  Trace mj_trace;
  mmc::MujocoPlant mujoco = ideal_plant();
  mujoco.reset(mmc::scenario_setup(c.bench->scene(), cell.scenario), c.start, 1);
  const auto mj = run(c, mujoco, mj_trace);
  const double mj_dev = deviation(mj_trace);
  std::cout << cell.cell << ": kinematic " << kin.end_status << " " << kin.time_s << " s (ensemble " << t_lo << ".." << t_hi
            << " s, deviation up to " << dev_hi << " m" << (same_status ? "" : ", mixed end states") << "); MuJoCo "
            << mj.end_status << " " << mj.time_s << " s, deviation " << mj_dev << " m, min true clearance "
            << mj.plant_min_clearance_m << " m, contact cycles " << mj.plant_contacts << "\n";
  // the MuJoCo run is one more sample of the rounding ensemble: its end is one the ensemble reaches
  EXPECT_EQ(statuses.count(mj.end_status), 1u) << cell.cell << ": " << mj.end_status;
  const bool inside = mj.time_s >= t_lo - 0.02 && mj.time_s <= t_hi + 0.02 && mj_dev <= dev_hi + 1e-3;
  std::cout << cell.cell << ": MuJoCo " << (inside ? "inside" : "OUTSIDE") << " the rounding ensemble\n";
}

INSTANTIATE_TEST_SUITE_P(Corpus, Equivalence, ::testing::Range(0, 7));

TEST(Determinism, SameSeedSameRun)
{
  const auto cells = corpus();
  const auto it = std::find_if(cells.begin(), cells.end(), [](const Cell & c) { return c.bag.empty(); });  // synthetic
  ASSERT_NE(it, cells.end());
  const mmc::ClosedLoopCase c = mmc::make_case(it->scenario, "");
  Trace a, b;
  for (Trace * trace : {&a, &b}) {
    mmc::MujocoPlant plant = ideal_plant();
    plant.reset(mmc::scenario_setup(c.bench->scene(), it->scenario), c.start, 42);
    run(c, plant, *trace);
  }
  ASSERT_EQ(a.x.size(), b.x.size());
  for (size_t k = 0; k < a.x.size(); ++k) ASSERT_TRUE((a.x[k].array() == b.x[k].array()).all()) << "cycle " << k;
}

// Diagnostic (not a gate): the kinematic bench against itself with the start yaw moved by 1e-15 rad. Run with
// --gtest_also_run_disabled_tests to measure how the closed loop amplifies a rounding-sized difference.
TEST(Equivalence, DISABLED_KinematicBenchAgainstItselfPerturbed)
{
  for (const Cell & cell : corpus()) {
    if (!cell.bag.empty() && !std::filesystem::exists(cell.bag)) continue;
    const mmc::ClosedLoopCase c = mmc::make_case(cell.scenario, cell.bag);
    Trace a, b;
    mmm::KinematicClosedLoop first(c.start, c.bench->limits().breakaway);
    const auto ra = run(c, first, a);
    mmm::State moved = c.start;
    moved(2) += 1e-15;
    mmm::KinematicClosedLoop second(moved, c.bench->limits().breakaway);
    const auto rb = run(c, second, b);
    double max_dev = 0.0;
    for (size_t k = 0; k < std::min(a.x.size(), b.x.size()); ++k) max_dev = std::max(max_dev, (a.x[k].head<2>() - b.x[k].head<2>()).norm());
    std::cout << cell.cell << ": " << ra.end_status << " " << ra.time_s << " s vs " << rb.end_status << " " << rb.time_s
              << " s, max base deviation " << max_dev << " m\n";
  }
}

namespace
{
// The kinematic plant, recording every command B5 gives it.
class RecordingPlant : public mmm::ClosedLoopPlant
{
public:
  RecordingPlant(const mmm::State & start, const mmm::Breakaway & b) : inner_(start, b) {}
  const mmm::State & state() const override { return inner_.state(); }
  const mmm::Control & executed() const override { return inner_.executed(); }
  bool at_rest() const override { return inner_.at_rest(); }
  void step(const mmm::Control & command, const mmm::CycleOutput & cycle, double t, double dt) override
  {
    commands.push_back({command, t, dt});
    inner_.step(command, cycle, t, dt);
  }
  struct Command
  {
    mmm::Control u;
    double t, dt;
  };
  std::vector<Command> commands;

private:
  mmm::KinematicClosedLoop inner_;
};
}  // namespace

// E1's gate on the plant model (Task 4 review): a cell's recorded command sequence (B5 closed loop on the kinematic
// plant) replayed open loop through the kinematic plant and the MuJoCo plant in ideal mode with the breakaway. Open loop
// nothing amplifies a difference, so the trajectories, the executed commands and the at-rest flags must agree every
// cycle (base within 1e-6 m and rad: RK4 over a cycle against the exact arc; joints within 1e-9 rad).
class OpenLoop : public ::testing::TestWithParam<int>
{
};

TEST_P(OpenLoop, ReplayMatchesKinematicPlant)
{
  const auto cells = corpus();
  ASSERT_LT(GetParam(), static_cast<int>(cells.size()));
  const Cell & cell = cells[GetParam()];
  if (!cell.bag.empty() && !std::filesystem::exists(cell.bag)) GTEST_SKIP() << cell.cell << ": recording absent";
  const mmc::ClosedLoopCase c = mmc::make_case(cell.scenario, cell.bag);
  RecordingPlant recorder(c.start, c.bench->limits().breakaway);
  mmc::CaseOptions o;
  mmc::run_case(c, recorder, o);
  ASSERT_GT(recorder.commands.size(), 100u);

  mmm::KinematicClosedLoop kinematic(c.start, c.bench->limits().breakaway);
  mmc::MujocoPlant mujoco = ideal_plant();
  mujoco.reset(mmc::scenario_setup(c.bench->scene(), cell.scenario), c.start, 1);
  const mmm::CycleOutput none;
  double max_base = 0.0, max_joint = 0.0;
  int rest_mismatch = 0, executed_mismatch = 0;
  for (const auto & cmd : recorder.commands) {
    kinematic.step(cmd.u, none, cmd.t, cmd.dt);
    mujoco.step(cmd.u, none, cmd.t, cmd.dt);
    max_base = std::max(max_base, (kinematic.state().head<3>() - mujoco.state().head<3>()).cwiseAbs().maxCoeff());
    max_joint = std::max(max_joint, (kinematic.state().tail<6>() - mujoco.state().tail<6>()).cwiseAbs().maxCoeff());
    rest_mismatch += kinematic.at_rest() != mujoco.at_rest();
    executed_mismatch += (kinematic.executed() - mujoco.executed()).cwiseAbs().maxCoeff() > 1e-12;
  }
  std::cout << cell.cell << ": " << recorder.commands.size() << " commands, max base difference " << max_base
            << ", max joint difference " << max_joint << ", at-rest mismatches " << rest_mismatch
            << ", executed mismatches " << executed_mismatch << "\n";
  EXPECT_LE(max_base, 1e-6) << cell.cell;
  EXPECT_LE(max_joint, 1e-9) << cell.cell;
  EXPECT_EQ(rest_mismatch, 0) << cell.cell;
  EXPECT_EQ(executed_mismatch, 0) << cell.cell;
}

INSTANTIATE_TEST_SUITE_P(Corpus, OpenLoop, ::testing::Range(0, 7));

// The realistic plant closes the loop with the node's arm command path: one synthetic cell runs to its end with the
// arm tracking its reference (few re-anchors) and the ground truth recorded.
TEST(Realistic, ClosesTheLoopOnACell)
{
  const auto cells = corpus();
  const auto it = std::find_if(cells.begin(), cells.end(), [](const Cell & c) { return c.bag.empty(); });
  ASSERT_NE(it, cells.end());
  const mmc::ClosedLoopCase c = mmc::make_case(it->scenario, "");
  auto profile = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share);
  mmc::MujocoPlant plant(profile, share("mobile_manipulator_plant") + "/models/site_v2.xml", mmp::PlantMode::Realistic);
  mmc::CaseOptions o;
  // run_case configures the arm path; the reset needs the scenario
  plant.configure_arm_path(c.bench->limits(), c.b5_config, c.bench->primitives());
  plant.reset(mmc::scenario_setup(c.bench->scene(), it->scenario), c.start, 1);
  const auto r = mmc::run_case(c, plant, o);
  const Eigen::Vector2d goal(c.goal[0], c.goal[1]);
  const double start_gap = (c.start.head<2>() - goal).norm(), end_gap = (r.final_state.head<2>() - goal).norm();
  std::cout << it->cell << " realistic: " << r.end_status << " " << r.time_s << " s, " << end_gap << " m from the goal, true min clearance "
            << r.plant_min_clearance_m << " m, contact cycles " << r.plant_contacts << ", re-anchors " << plant.reanchors() << "\n";
  EXPECT_LT(end_gap, 0.5 * start_gap);
  EXPECT_LT(plant.reanchors(), r.b5_cycles / 20);
}
