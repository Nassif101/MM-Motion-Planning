// E1 and E2 (MuJoCo plant spec Section 8, plan Task 4): the MuJoCo plant in ideal mode with the breakaway is
// indistinguishable from the kinematic bench on the corpus cells, and the same seed gives the same run bit for bit.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
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
// trajectory. E1 here (provisional, the criterion is the user's decision at CP1): the MuJoCo run ends as the unperturbed
// kinematic run does; it is reported against a fixed ensemble of eight kinematic runs whose start yaw is moved by
// +-1..4 x 1e-15 rad (time range, largest deviation), not asserted against it.
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
  for (int k : {-4, -3, -2, -1, 1, 2, 3, 4}) {
    mmm::State moved = c.start;
    moved(2) += k * 1e-15;
    Trace trace;
    mmm::KinematicClosedLoop perturbed(moved, c.bench->limits().breakaway);
    const auto r = run(c, perturbed, trace);
    same_status = same_status && r.end_status == kin.end_status;
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
  EXPECT_EQ(mj.end_status, kin.end_status) << cell.cell;
  EXPECT_EQ(mj.reached, kin.reached) << cell.cell;
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
