// plant_replay (MuJoCo plant plan Task 6): recorded commands replayed open loop through the realistic plant, the
// calibration's engine. ROS-free: tools/plant/calibrate_plant.py extracts the commands from the bags.
//   plant_replay --input FILE.json --out FILE.json [--profile PROFILE.yaml] [--param key=value ...]
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>

#include "mobile_manipulator_campaign/replay.hpp"

namespace
{
std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }
}

int main(int argc, char ** argv)
{
  try {
    std::string input, output, profile_path = share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml";
    nlohmann::json params = nlohmann::json::object();
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
        return argv[++i];
      };
      if (a == "--input") input = next();
      else if (a == "--out") output = next();
      else if (a == "--profile") profile_path = next();
      else if (a == "--param") {
        const std::string kv = next();
        const auto eq = kv.find('=');
        if (eq == std::string::npos) throw std::invalid_argument("--param key=value");
        params[kv.substr(0, eq)] = nlohmann::json::parse(kv.substr(eq + 1));
      } else {
        throw std::invalid_argument("unknown option " + a);
      }
    }
    if (input.empty() || output.empty()) throw std::invalid_argument("--input and --out are required");
    nlohmann::json in = nlohmann::json::parse(std::ifstream(input));
    for (const auto & [k, v] : params.items()) in["params"][k] = v;  // the command line wins
    const auto profile = mobile_manipulator_plant::load_profile(profile_path, share);
    const auto out = mobile_manipulator_campaign::replay(in, profile, share("mobile_manipulator_plant") + "/models/site_v2.xml");
    std::ofstream(output) << out.dump() << "\n";
    std::cout << out.at("t").size() << " samples -> " << output << "\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "plant_replay: " << e.what() << "\n";
    return 1;
  }
}
