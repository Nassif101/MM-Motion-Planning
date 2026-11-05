#include "mobile_manipulator_plant/robot_profile.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
      if (null_is_nan && list[i].IsNull()) {
        v(i) = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      try {
        v(i) = list[i].as<double>();
      } catch (const YAML::Exception &) {
        throw std::runtime_error(file + ": " + where + key + "[" + std::to_string(i) + "] is not a number");
      }
    }
    return v;
  }

  // Throws naming the key when `ok` is false.
  void check(bool ok, const std::string & key, const std::string & what) const
  {
    if (!ok) throw std::runtime_error(file + ": " + key + " " + what);
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
  for (int i = 0; i < 6; ++i) {
    if (!joints[i].IsScalar()) throw std::runtime_error(profile_yaml + ": arm_joints[" + std::to_string(i) + "] is not a name");
    p.arm_joints[i] = joints[i].as<std::string>();
  }
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
  p.rest_linear_mps = r.get<double>(breakaway, "rest_linear_mps", "base.breakaway.");
  p.rest_yaw_radps = r.get<double>(breakaway, "rest_yaw_radps", "base.breakaway.");
  p.base_servo_kv = r.vec<3>(base, "servo_kv", "base.");
  p.drive_force_limit_n = r.get<double>(base, "drive_force_limit_n", "base.");
  p.drive_torque_limit_nm = r.get<double>(base, "drive_torque_limit_nm", "base.");
  p.wheel_radius_m = r.get<double>(base, "wheel_radius_m", "base.");
  p.effective_track_m = r.get<double>(base, "effective_track_m", "base.");
  p.wheel_speed_max = r.get<double>(base, "wheel_speed_max", "base.");
  r.check(p.drive_force_limit_n > 0.0 && p.drive_torque_limit_nm > 0.0 && p.wheel_radius_m > 0.0 && p.effective_track_m > 0.0 &&
            p.wheel_speed_max > 0.0,
          "base drive", "limits and geometry must be positive");

  const YAML::Node arm = r.at(root, "arm", "");
  p.kp = r.vec<6>(arm, "kp", "arm.");
  p.kd = r.vec<6>(arm, "kd", "arm.");
  p.armature = r.vec<6>(arm, "armature", "arm.");
  p.damping = r.vec<6>(arm, "damping", "arm.");
  p.frictionloss = r.vec<6>(arm, "frictionloss", "arm.");
  r.check((p.kp.array() > 0.0).all(), "arm.kp", "must be positive");
  r.check((p.kd.array() >= 0.0).all(), "arm.kd", "must not be negative");
  r.check((p.armature.array() >= 0.0).all() && (p.damping.array() >= 0.0).all() && (p.frictionloss.array() >= 0.0).all(),
          "arm.armature/damping/frictionloss", "must not be negative");
  p.gravity_ff = r.get<bool>(arm, "gravity_ff", "arm.");
  p.motion_ff = r.get<bool>(arm, "motion_ff", "arm.");
  p.jtc_rate_hz = r.get<double>(arm, "jtc_rate_hz", "arm.");
  p.ff_accel_limit = r.vec<6>(arm, "ff_accel_limit", "arm.");
  r.check(p.jtc_rate_hz > 0.0, "arm.jtc_rate_hz", "must be positive");
  r.check((p.ff_accel_limit.array() >= 0.0).all(), "arm.ff_accel_limit", "must not be negative");

  const YAML::Node lidar = r.at(root, "lidar", "");
  p.lidar = {r.get<std::string>(lidar, "site", "lidar."),     r.get<std::string>(lidar, "pattern", "lidar."),
             r.get<double>(lidar, "rate_hz", "lidar."),       r.get<int>(lidar, "points_per_scan", "lidar."),
             r.get<double>(lidar, "min_range_m", "lidar."),   r.get<double>(lidar, "max_range_m", "lidar."),
             r.get<double>(lidar, "sigma_m", "lidar."),       r.get<double>(lidar, "dropout", "lidar.")};

  p.lidar.scan_mode = r.get<std::string>(lidar, "scan_mode", "lidar.");
  p.lidar.static_bvh = r.get<bool>(lidar, "static_bvh", "lidar.");
  r.check(p.lidar.scan_mode == "sweep" || p.lidar.scan_mode == "instant", "lidar.scan_mode", "must be sweep or instant");
  p.lidar_pattern_path = package_share("mobile_manipulator_plant") + "/" + p.lidar.pattern;
  r.check(p.lidar.rate_hz > 0.0, "lidar.rate_hz", "must be positive");
  r.check(p.lidar.points_per_scan > 0, "lidar.points_per_scan", "must be positive");
  r.check(p.lidar.min_range_m >= 0.0 && p.lidar.min_range_m < p.lidar.max_range_m, "lidar.min_range_m",
          "must lie in [0, max_range_m)");
  r.check(p.lidar.sigma_m >= 0.0, "lidar.sigma_m", "must not be negative");
  r.check(p.lidar.dropout >= 0.0 && p.lidar.dropout <= 1.0, "lidar.dropout", "must lie in [0, 1]");

  p.ideal_breakaway = r.get<bool>(r.at(root, "ideal", ""), "breakaway", "ideal.");
  const YAML::Node realistic = r.at(root, "realistic", "");
  p.base_command_delay_s = r.get<double>(realistic, "base_command_delay_s", "realistic.");
  p.arm_command_delay_s = r.get<double>(realistic, "arm_command_delay_s", "realistic.");
  p.state_delay_s = r.get<double>(realistic, "state_delay_s", "realistic.");
  p.base_lag_s = r.vec<2>(realistic, "base_lag_s", "realistic.");
  p.base_slip = r.vec<2>(realistic, "base_slip", "realistic.");
  p.base_breakaway = r.get<bool>(realistic, "base_breakaway", "realistic.");
  const YAML::Node skid = r.at(realistic, "base_skid", "realistic.");
  auto list = [&](const char * key) {
    const YAML::Node n = r.at(skid, key, "realistic.base_skid.");
    if (!n.IsSequence() || n.size() == 0) throw std::runtime_error(profile_yaml + ": realistic.base_skid." + key + " must list numbers");
    std::vector<double> v;
    for (const auto & x : n) {
      try {
        v.push_back(x.as<double>());
      } catch (const YAML::Exception &) {
        throw std::runtime_error(profile_yaml + ": realistic.base_skid." + key + " must list numbers");
      }
    }
    return v;
  };
  p.skid_share = list("share");
  p.skid_v = list("v");
  p.skid_w = list("w");
  p.skid_w_in_place = r.get<double>(skid, "w_in_place", "realistic.base_skid.");
  r.check(p.skid_v.size() == p.skid_share.size() && p.skid_w.size() == p.skid_share.size(), "realistic.base_skid",
          "share, v and w must have equal lengths");
  r.check(std::is_sorted(p.skid_share.begin(), p.skid_share.end()) &&
            std::adjacent_find(p.skid_share.begin(), p.skid_share.end()) == p.skid_share.end(),
          "realistic.base_skid", "share knots must ascend");
  r.check(p.base_command_delay_s >= 0.0, "realistic.base_command_delay_s", "must not be negative");
  r.check(p.arm_command_delay_s >= 0.0, "realistic.arm_command_delay_s", "must not be negative");
  r.check(p.state_delay_s >= 0.0, "realistic.state_delay_s", "must not be negative");
  r.check((p.base_lag_s.array() >= 0.0).all(), "realistic.base_lag_s", "must not be negative");
  r.check(p.base_limits.v_max_fwd > 0.0 && p.base_limits.v_max_rev > 0.0 && p.base_limits.w_max > 0.0 &&
            p.base_limits.accel > 0.0 && p.base_limits.decel > 0.0 && p.base_limits.yaw_accel > 0.0 &&
            p.base_limits.yaw_decel > 0.0 && p.base_limits.stale_s > 0.0,
          "base.limits", "must all be positive");
  for (const std::string & file : {p.urdf_path, p.payload_json_path, p.model_path})
    r.check(std::filesystem::exists(file), file, "(referenced) does not exist");

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
        throw std::runtime_error("joint_order differs from the profile's arm_joints (" + profile_yaml + ")");
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
