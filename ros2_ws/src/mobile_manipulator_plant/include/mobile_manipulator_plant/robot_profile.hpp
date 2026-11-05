#pragma once
// The robot as data (MuJoCo plant spec Section 3.3): profile.yaml plus the files it references (URDF, qualified
// payload), resolved through a package-share lookup so the plant stays ROS-free.
#include <array>
#include <functional>
#include <map>
#include <string>

#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_plant
{
struct BaseLimits
{
  double v_max_fwd = 0.0, v_max_rev = 0.0, w_max = 0.0, accel = 0.0, decel = 0.0, yaw_accel = 0.0, yaw_decel = 0.0,
         stale_s = 0.0;
};

struct LidarParams
{
  std::string site, pattern;  // the model's site; the pattern file relative to the plant's share directory
  double rate_hz = 0.0;
  int points_per_scan = 0;
  double min_range_m = 0.0, max_range_m = 0.0, sigma_m = 0.0, dropout = 0.0;
};

struct RobotProfile
{
  std::string robot;
  std::string urdf_path, payload_json_path, model_path;  // resolved, absolute
  std::array<std::string, 6> arm_joints;
  Vec6 torque_limits_nm;  // NaN: no limit
  BaseLimits base_limits;
  double breakaway_linear_mps = 0.0, breakaway_yaw_radps = 0.0;
  Eigen::Vector3d base_servo_kv = Eigen::Vector3d::Zero();
  Vec6 kp = Vec6::Zero(), kd = Vec6::Zero(), armature = Vec6::Zero();
  bool gravity_ff = true, motion_ff = true;
  LidarParams lidar;
  bool ideal_breakaway = false;
  double command_delay_s = 0.0, state_delay_s = 0.0;
  Eigen::Vector2d base_lag_s = Eigen::Vector2d::Zero(), base_slip = Eigen::Vector2d::Ones();
  double payload_mass_kg = 0.0;
  Eigen::Vector3d payload_size = Eigen::Vector3d::Zero(), payload_center = Eigen::Vector3d::Zero();  // in tool0
  std::map<std::string, Vec6> poses;  // qualified poses, the arm_joints order
};

// `package_share` maps a package name to its share directory. Throws std::runtime_error naming the file and key for
// a missing or malformed entry.
RobotProfile load_profile(const std::string & profile_yaml, const std::function<std::string(const std::string &)> & package_share);
}  // namespace mobile_manipulator_plant
