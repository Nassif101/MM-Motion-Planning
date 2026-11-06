#pragma once
// The robot as data (MuJoCo plant spec Section 3.3): profile.yaml plus the files it references (URDF, qualified
// payload), resolved through a package-share lookup so the plant stays ROS-free.
#include <array>
#include <functional>
#include <map>
#include <string>
#include <vector>

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
  std::string scan_mode = "sweep";  // sweep | instant
  bool static_bvh = true;           // static geoms through a BVH, MuJoCo's rays for the rest (CP2)
};

// The realistic base (amendment 2026-11-06): planar joints with a calibrated drive model, or the base on its wheels.
enum class BaseModel
{
  Planar,
  Wheels
};

// Servo gains by rule: per joint kp = I w^2, kd = 2 zeta I w at `pose` (resolve_arm_gains); off: the profile's kp, kd.
struct GainRule
{
  bool on = false;
  double bandwidth_hz = 0.0, zeta = 0.0;
  std::string pose;
};

struct RobotProfile
{
  std::string robot;
  std::string urdf_path, payload_json_path, model_path, model_wheels_path;  // resolved, absolute
  std::string lidar_pattern_path;                         // resolved (built at the plant's build, not checked here)
  std::array<std::string, 6> arm_joints;
  Vec6 torque_limits_nm;  // NaN: no limit
  BaseLimits base_limits;
  double breakaway_linear_mps = 0.0, breakaway_yaw_radps = 0.0, rest_linear_mps = 0.0, rest_yaw_radps = 0.0;
  Eigen::Vector3d base_servo_kv = Eigen::Vector3d::Zero();
  double drive_force_limit_n = 0.0, drive_torque_limit_nm = 0.0;
  double wheel_radius_m = 0.0, effective_track_m = 0.0, wheel_speed_max = 0.0;
  double wheel_kv = 0.0;  // the wheel motors' velocity gain (N m s/rad) on the wheels base
  Vec6 kp = Vec6::Zero(), kd = Vec6::Zero();  // servo gains (N m/rad, N m s/rad); resolved at reset under a gain rule
  GainRule gain_rule;
  Vec6 armature = Vec6::Zero(), damping = Vec6::Zero(), frictionloss = Vec6::Zero();  // passive joint terms
  bool gravity_ff = true, motion_ff = true;
  double jtc_rate_hz = 50.0;
  Vec6 ff_accel_limit = Vec6::Constant(4.0);  // per joint (rad/s^2)
  LidarParams lidar;
  bool ideal_breakaway = false;
  double base_command_delay_s = 0.0, arm_command_delay_s = 0.0;
  double base_state_delay_s = 0.0, arm_state_delay_s = 0.0;  // odometry and joint states: separate paths
  Eigen::Vector2d base_lag_s = Eigen::Vector2d::Zero(), base_slip = Eigen::Vector2d::Ones();
  bool base_breakaway = false;
  BaseModel base_model = BaseModel::Planar;
  double contact_mu = 1.0, contact_impratio = 1.0;  // the wheels base's ground friction and elliptic cone
  std::vector<double> skid_share{0.0}, skid_v{1.0}, skid_w{1.0};  // the base's skid map (BaseDriveParams)
  double skid_w_in_place = 1.0;
  double payload_mass_kg = 0.0;
  Eigen::Vector3d payload_size = Eigen::Vector3d::Zero(), payload_center = Eigen::Vector3d::Zero();  // in tool0
  std::map<std::string, Vec6> poses;  // qualified poses, the arm_joints order
};

// `package_share` maps a package name to its share directory. Throws std::runtime_error naming the file and key for
// a missing, malformed or out-of-range entry, or a referenced file that does not exist.
RobotProfile load_profile(const std::string & profile_yaml, const std::function<std::string(const std::string &)> & package_share);
}  // namespace mobile_manipulator_plant
