// Footprint update cost of the mesh and disc models (Sagar et al., Table II analogue).
//
// Usage: footprint_benchmark [--evaluations N] [--output FILE]
// For each model and each qualified pose, times N evaluations of footprint(pose, padding_m)
// and writes {model: {pose: {mean_us, min_us, max_us, p99_us, points, vertices, area_m2}},
// disc_to_mesh_area: {pose: ratio}} (stdout without --output).
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_navigation/footprint_config.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;
using Clock = std::chrono::steady_clock;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  int evaluations = 5000;
  std::string output;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--evaluations" && i + 1 < argc) {
      evaluations = std::stoi(argv[++i]);
    } else if (arg == "--output" && i + 1 < argc) {
      output = argv[++i];
    } else {
      std::cerr << "usage: footprint_benchmark [--evaluations N] [--output FILE]\n";
      return 1;
    }
  }
  const auto share = [](const std::string & p) { return ament_index_cpp::get_package_share_directory(p); };
  const auto payload = mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
  const auto projector = std::make_shared<const mmg::FootprintProjector>(
    read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf"), mmg::payload_from_json(payload));
  const auto parameters = mmn::ScenarioConfig(share("mobile_manipulator_navigation"))
                            .load("dynamic_footprint.yaml").at("dynamic_footprint_node").at("ros__parameters");
  const double padding = parameters.at("padding_m").get<double>();

  mmn::Json out;
  std::map<std::string, std::map<std::string, double>> areas;
  for (const std::string model_name : {"mesh", "disc"}) {
    const auto model = mmn::footprint_model(model_name, parameters, projector);
    for (const auto & [pose_name, values] : payload["poses_rad"].items()) {
      mmg::JointMap joints;
      for (size_t i = 0; i < payload["joint_order"].size(); ++i) {
        joints[payload["joint_order"][i].get<std::string>()] = values[i].get<double>();
      }
      std::vector<double> times_us;
      mmg::Polygon footprint;
      for (int k = 0; k < evaluations; ++k) {
        const auto start = Clock::now();
        footprint = model->footprint(joints, padding);
        times_us.push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());
      }
      std::sort(times_us.begin(), times_us.end());
      const double mean = std::accumulate(times_us.begin(), times_us.end(), 0.0) / times_us.size();
      const size_t p99 = std::min(times_us.size() - 1, static_cast<size_t>(0.99 * times_us.size()));
      areas[model_name][pose_name] = mmg::area(footprint);
      out[model_name][pose_name] = {{"mean_us", mean}, {"min_us", times_us.front()}, {"max_us", times_us.back()},
                                    {"p99_us", times_us[p99]}, {"points", model->points(joints).size()},
                                    {"vertices", footprint.size()}, {"area_m2", areas[model_name][pose_name]}};
    }
  }
  for (const auto & [pose_name, mesh_area] : areas["mesh"]) out["disc_to_mesh_area"][pose_name] = areas["disc"][pose_name] / mesh_area;
  out["evaluations"] = evaluations;
  out["padding_m"] = padding;
  if (output.empty()) {
    std::cout << out.dump(2) << "\n";
  } else {
    std::ofstream(output) << out.dump(2) << "\n";
  }
  return 0;
}
