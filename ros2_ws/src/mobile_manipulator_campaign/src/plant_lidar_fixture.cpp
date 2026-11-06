// The plant's scans at a fixed pose for gate L (MuJoCo plant plan Task 10; tools/plant/compare_lidar.py plant).
//   plant_lidar_fixture --scenario NAME --pose X Y YAW --out FILE.mmlog [--scans 20] [--seed 1] [--mode realistic|ideal]
//                       [--profile FILE]
#include <iostream>
#include <stdexcept>
#include <string>

#include "mobile_manipulator_campaign/lidar_fixture.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;

int main(int argc, char ** argv)
{
  try {
    mmc::SessionConfig config;
    config.mode = mmp::PlantMode::Realistic;
    std::string scenario, out;
    mmp::Pose2 pose;
    bool posed = false;
    int scans = 20;
    uint64_t seed = 1;
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
        return argv[++i];
      };
      if (a == "--scenario") scenario = next();
      else if (a == "--pose") {
        pose.x = std::stod(next());
        pose.y = std::stod(next());
        pose.yaw = std::stod(next());
        posed = true;
      } else if (a == "--out") out = next();
      else if (a == "--scans") scans = std::stoi(next());
      else if (a == "--seed") seed = std::stoull(next());
      else if (a == "--profile") config.profile = next();
      else if (a == "--mode") {
        const std::string mode = next();
        if (mode != "ideal" && mode != "realistic") throw std::invalid_argument("--mode ideal|realistic");
        config.mode = mode == "ideal" ? mmp::PlantMode::Ideal : mmp::PlantMode::Realistic;
      } else throw std::invalid_argument("unknown option " + a);
    }
    if (scenario.empty() || out.empty() || !posed) throw std::invalid_argument("--scenario, --pose and --out are required");
    const auto r = mmc::lidar_fixture(config, scenario, pose, scans, seed);
    mmc::write_lidar_fixture(r, out);
    std::cout << scenario << " at (" << pose.x << ", " << pose.y << ", " << pose.yaw << "): " << r.scans.size()
              << " scans, contacts " << r.header.at("contacts") << ", drift " << r.header.at("drift_m") << " m\n";
    return r.header.at("contacts").get<int>() == 0 ? 0 : 2;
  } catch (const std::exception & e) {
    std::cerr << "plant_lidar_fixture: " << e.what() << "\n";
    return 1;
  }
}
