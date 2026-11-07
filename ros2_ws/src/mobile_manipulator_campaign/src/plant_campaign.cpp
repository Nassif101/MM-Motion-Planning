// One campaign run on the MuJoCo plant (MuJoCo plant plan Task 8): a cell through a Session to its end (every recorded
// goal in order with a bag, else the scenario's start and goal), DIR/summary.json and DIR/run.mmlog.
//   plant_campaign --scenario NAME [--bag DIR] --mode ideal|realistic [--b5-only] [--medium-config FILE] [--seed N]
//                  [--lidar] [--profile FILE] [--timeout-s T] [--profile-timing] [--stall-s 45] [--cell NAME] --out DIR
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/session.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;

int main(int argc, char ** argv)
{
  try {
    mmc::SessionConfig config;
    config.stall_s = 45.0;  // campaigns: a run without progress for 45 s ends as stalled (user, 2026-11-07); 0: off
    std::string scenario, bag, out, mode;
    uint64_t seed = 1;
    std::vector<std::string> args;  // the run's arguments but --out, into its summary
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--out") ++i;
      else args.push_back(argv[i]);
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
        return argv[++i];
      };
      if (a == "--scenario") scenario = next();
      else if (a == "--bag") bag = next();
      else if (a == "--mode") mode = next();
      else if (a == "--b5-only") config.medium = false;
      else if (a == "--medium-config") config.medium_config = next();
      else if (a == "--seed") seed = std::stoull(next());
      else if (a == "--lidar") config.lidar = true;
      else if (a == "--profile") config.profile = next();
      else if (a == "--timeout-s") config.timeout_s = std::stod(next());
      else if (a == "--profile-timing") config.profile_timing = true;
      else if (a == "--stall-s") config.stall_s = std::stod(next());
      else if (a == "--cell") config.cell = next();
      else if (a == "--out") out = next();
      else throw std::invalid_argument("unknown option " + a);
    }
    if (scenario.empty() || out.empty() || (mode != "ideal" && mode != "realistic"))
      throw std::invalid_argument("--scenario, --mode ideal|realistic and --out are required");
    config.mode = mode == "realistic" ? mmp::PlantMode::Realistic : mmp::PlantMode::Ideal;
    config.provenance = mmc::git_provenance();
    std::filesystem::create_directories(out);
    config.log_path = out + "/run.mmlog";
    nlohmann::json summary;
    {
      mmc::Session session(config);
      session.reset(scenario, bag, std::nullopt, seed);
      while (session.step_cycle()) {
      }
      summary = session.summary();
    }  // the run log is closed before the summary marks the run done
    summary["args"] = args;  // what tools/plant/plant_campaign.py's resume compares
    const std::string done = out + "/summary.json", partial = done + ".partial";
    {
      std::ofstream file(partial);
      file << summary.dump(1) << "\n";
      if (!file.flush()) throw std::runtime_error("cannot write " + partial);
    }
    std::filesystem::rename(partial, done);  // atomic: a summary is whole or absent
    std::cout << scenario << " seed " << seed << ": " << summary.at("end_status").get<std::string>() << " after "
              << summary.at("time_s").get<double>() << " s (" << summary.at("real_time_factor").get<double>() << "x real time)\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "plant_campaign: " << e.what() << "\n";
    return 1;
  }
}
