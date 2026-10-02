// Panel-centre pose in base_footprint for a qualified arm pose, for writing mission goals.
//
// Usage: panel_pose --pose NAME   (NAME from qualified_payload.json poses_rad)
// Prints {"pose": NAME, "xyz": [...], "rpy": [...], "quaternion_xyzw": [...]}.
#include <fstream>
#include <iostream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_navigation/footprint_projection.hpp"

namespace mmn = mobile_manipulator_navigation;

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
  if (argc != 3 || std::string(argv[1]) != "--pose") {
    std::cerr << "usage: panel_pose --pose NAME\n";
    return 1;
  }
  const std::string name = argv[2];
  const auto payload = mmn::Json::parse(read_file(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_control") + "/config/qualified_payload.json"));
  if (!payload["poses_rad"].contains(name)) {
    std::cerr << "unknown qualified pose " << name << "\n";
    return 1;
  }
  mmn::JointMap joints;
  for (size_t i = 0; i < payload["joint_order"].size(); ++i) {
    joints[payload["joint_order"][i].get<std::string>()] = payload["poses_rad"][name][i].get<double>();
  }
  const mmn::FootprintProjector projector(
    read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
              "/urdf/mobile_manipulator.urdf"),
    mmn::payload_from_json(payload));
  const auto pose = projector.panel_pose(joints);
  const auto rpy = mmn::rpy_of(pose.linear());
  const Eigen::Quaterniond q(pose.linear());
  const auto & t = pose.translation();
  mmn::Json out;
  out["pose"] = name;
  out["xyz"] = mmn::Json::array({t.x(), t.y(), t.z()});
  out["rpy"] = mmn::Json::array({rpy[0], rpy[1], rpy[2]});
  out["quaternion_xyzw"] = mmn::Json::array({q.x(), q.y(), q.z(), q.w()});
  std::cout << out.dump() << "\n";
  return 0;
}
