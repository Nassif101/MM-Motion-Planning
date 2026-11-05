#include "mobile_manipulator_plant/robot_profile.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace mobile_manipulator_plant
{
namespace
{
struct Reader
{
  std::string file;

  YAML::Node at(const YAML::Node & node, const std::string & key, const std::string & where) const
  {
    if (!node.IsMap() || !node[key]) throw std::runtime_error(file + ": missing key " + where + key);
    return node[key];
  }

  template <class T>
  T get(const YAML::Node & node, const std::string & key, const std::string & where = "") const
  {
    try {
      return at(node, key, where).as<T>();
    } catch (const YAML::Exception & e) {
      throw std::runtime_error(file + ": bad value of " + where + key + ": " + e.what());
    }
  }

  // A fixed-length list of numbers; null entries become NaN when `null_is_nan`.
  template <int N>
  Eigen::Matrix<double, N, 1> vec(const YAML::Node & node, const std::string & key, const std::string & where = "",
                                  bool null_is_nan = false) const
  {
    const YAML::Node list = at(node, key, where);
    if (!list.IsSequence() || list.size() != N)
      throw std::runtime_error(file + ": " + where + key + " must list " + std::to_string(N) + " numbers");
    Eigen::Matrix<double, N, 1> v;
    for (int i = 0; i < N; ++i) {
      if (null_is_nan && list[i].IsNull()) v(i) = std::numeric_limits<double>::quiet_NaN();
      else v(i) = list[i].as<double>();
    }
    return v;
  }
};

// "package/relative/path" -> share(package)/relative/path
std::string resolve(const std::string & reference, const std::function<std::string(const std::string &)> & share)
{
  const auto slash = reference.find('/');
  if (slash == std::string::npos || slash == 0) throw std::runtime_error("not a package-relative path: " + reference);
  return share(reference.substr(0, slash)) + reference.substr(slash);
}

Eigen::Vector3d json_vector3(const nlohmann::json & j) { return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()}; }
}  // namespace

RobotProfile load_profile(const std::string & profile_yaml, const std::function<std::string(const std::string &)> & package_share)
{
  const Reader r{profile_yaml};
  YAML::Node root;
  try {
    root = YAML::LoadFile(profile_yaml);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(profile_yaml + ": " + e.what());
  }
  RobotProfile p;
  p.robot = r.get<std::string>(root, "robot");
  p.urdf_path = resolve(r.get<std::string>(root, "urdf"), package_share);
  p.payload_json_path = resolve(r.get<std::string>(root, "payload_json"), package_share);
  p.model_path = (std::filesystem::path(profile_yaml).parent_path() / r.get<std::string>(root, "model")).string();
  const YAML::Node joints = r.at(root, "arm_joints", "");
  if (!joints.IsSequence() || joints.size() != 6) throw std::runtime_error(profile_yaml + ": arm_joints must list 6 names");
  for (int i = 0; i < 6; ++i) p.arm_joints[i] = joints[i].as<std::string>();
  p.torque_limits_nm = r.vec<6>(root, "torque_limits_nm", "", true);

  const YAML::Node base = r.at(root, "base", "");
  const YAML::Node limits = r.at(base, "limits", "base.");
  p.base_limits = {r.get<double>(limits, "v_max_fwd", "base.limits."), r.get<double>(limits, "v_max_rev", "base.limits."),
                   r.get<double>(limits, "w_max", "base.limits."),     r.get<double>(limits, "accel", "base.limits."),
                   r.get<double>(limits, "decel", "base.limits."),     r.get<double>(limits, "yaw_accel", "base.limits."),
                   r.get<double>(limits, "yaw_decel", "base.limits."), r.get<double>(limits, "stale_s", "base.limits.")};
  const YAML::Node breakaway = r.at(base, "breakaway", "base.");
  p.breakaway_linear_mps = r.get<double>(breakaway, "linear_mps", "base.breakaway.");
  p.breakaway_yaw_radps = r.get<double>(breakaway, "yaw_radps", "base.breakaway.");
  p.base_servo_kv = r.vec<3>(base, "servo_kv", "base.");

  const YAML::Node arm = r.at(root, "arm", "");
  p.kp = r.vec<6>(arm, "kp", "arm.");
  p.kd = r.vec<6>(arm, "kd", "arm.");
  p.armature = r.vec<6>(arm, "armature", "arm.");
  p.gravity_ff = r.get<bool>(arm, "gravity_ff", "arm.");
  p.motion_ff = r.get<bool>(arm, "motion_ff", "arm.");

  const YAML::Node lidar = r.at(root, "lidar", "");
  p.lidar = {r.get<std::string>(lidar, "site", "lidar."),     r.get<std::string>(lidar, "pattern", "lidar."),
             r.get<double>(lidar, "rate_hz", "lidar."),       r.get<int>(lidar, "points_per_scan", "lidar."),
             r.get<double>(lidar, "min_range_m", "lidar."),   r.get<double>(lidar, "max_range_m", "lidar."),
             r.get<double>(lidar, "sigma_m", "lidar."),       r.get<double>(lidar, "dropout", "lidar.")};

  p.ideal_breakaway = r.get<bool>(r.at(root, "ideal", ""), "breakaway", "ideal.");
  const YAML::Node realistic = r.at(root, "realistic", "");
  p.command_delay_s = r.get<double>(realistic, "command_delay_s", "realistic.");
  p.state_delay_s = r.get<double>(realistic, "state_delay_s", "realistic.");
  p.base_lag_s = r.vec<2>(realistic, "base_lag_s", "realistic.");
  p.base_slip = r.vec<2>(realistic, "base_slip", "realistic.");

  std::ifstream in(p.payload_json_path);
  if (!in) throw std::runtime_error(profile_yaml + ": cannot read " + p.payload_json_path);
  try {
    const nlohmann::json payload = nlohmann::json::parse(in);
    const auto & box = payload.at("payload");
    p.payload_mass_kg = box.at("mass_kg").get<double>();
    p.payload_size = json_vector3(box.at("dimensions_tool_ros_m"));
    p.payload_center = json_vector3(box.at("com_tool_ros_m"));
    for (int i = 0; i < 6; ++i)
      if (payload.at("joint_order").at(i).get<std::string>() != p.arm_joints[i])
        throw std::runtime_error("joint_order differs from the profile's arm_joints");
    for (const auto & item : payload.at("poses_rad").items()) {
      Vec6 q;
      for (int i = 0; i < 6; ++i) q(i) = item.value().at(i).get<double>();
      p.poses[item.key()] = q;
    }
  } catch (const nlohmann::json::exception & e) {
    throw std::runtime_error(p.payload_json_path + ": " + e.what());
  }
  return p;
}
}  // namespace mobile_manipulator_plant
