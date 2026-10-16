// Highest point of the robot and its panel for a set of arm poses, in base_footprint.
//
// Usage: robot_heights --poses FILE
// FILE is {"name": [6 joints], ...} in the joint_order of qualified_payload.json. Prints
// {"name": top_z_m, ...}. Pure file access; no ROS graph.
#include <fstream>
#include <iostream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_geometry/footprint_projection.hpp"

namespace mmg = mobile_manipulator_geometry;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("Cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  const mobile_manipulator_control::Args args(argc, argv);
  if (!args.has("poses")) {
    std::cerr << "usage: robot_heights --poses FILE" << std::endl;
    return 2;
  }
  try {
    const auto qualified = nlohmann::ordered_json::parse(read_file(
      ament_index_cpp::get_package_share_directory("mobile_manipulator_control") + "/config/qualified_payload.json"));
    const mmg::FootprintProjector projector(
      read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
                "/urdf/mobile_manipulator.urdf"),
      mmg::payload_from_json(qualified));
    const auto & order = qualified.at("joint_order");
    const auto poses = nlohmann::ordered_json::parse(read_file(args.values("poses", 1)[0]));
    nlohmann::ordered_json tops = nlohmann::ordered_json::object();
    for (const auto & [name, values] : poses.items()) {
      if (values.size() != order.size()) throw std::runtime_error("Pose " + name + " needs " + std::to_string(order.size()) + " joints");
      mmg::JointMap joints;
      for (size_t i = 0; i < order.size(); ++i) joints[order[i].get<std::string>()] = values[i].get<double>();
      tops[name] = projector.top_z(joints);
    }
    std::cout << tops.dump() << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 2;
  }
}
