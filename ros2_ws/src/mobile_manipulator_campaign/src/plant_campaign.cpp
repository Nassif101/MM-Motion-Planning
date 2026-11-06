// One campaign run on the MuJoCo plant (MuJoCo plant plan Task 8): a cell through a Session to its end (every recorded
// goal in order with a bag, else the scenario's start and goal), DIR/summary.json and DIR/run.mmlog.
//   plant_campaign --scenario NAME [--bag DIR] --mode ideal|realistic [--b5-only] [--medium-config FILE] [--seed N]
//                  [--lidar] [--profile FILE] [--timeout-s T] --out DIR
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mobile_manipulator_campaign/session.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;

int main(int argc, char ** argv)
{
  try {
    mmc::SessionConfig config;
    std::string scenario, bag, out, mode;
    uint64_t seed = 1;
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
      else if (a == "--out") out = next();
      else throw std::invalid_argument("unknown option " + a);
    }
    if (scenario.empty() || out.empty() || (mode != "ideal" && mode != "realistic"))
      throw std::invalid_argument("--scenario, --mode ideal|realistic and --out are required");
    config.mode = mode == "realistic" ? mmp::PlantMode::Realistic : mmp::PlantMode::Ideal;
    config.provenance = mmc::git_provenance();
    std::filesystem::create_directories(out);
    config.log_path = out + "/run.mmlog";
    mmc::Session session(config);
    session.reset(scenario, bag, std::nullopt, seed);
    while (session.step_cycle()) {
    }
    const auto summary = session.summary();
    std::ofstream(out + "/summary.json") << summary.dump(1) << "\n";
    std::cout << scenario << " seed " << seed << ": " << summary.at("end_status").get<std::string>() << " after "
              << summary.at("time_s").get<double>() << " s (" << summary.at("real_time_factor").get<double>() << "x real time)\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "plant_campaign: " << e.what() << "\n";
    return 1;
  }
}
