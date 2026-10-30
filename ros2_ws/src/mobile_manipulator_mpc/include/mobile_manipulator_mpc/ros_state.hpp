#pragma once
// The robot's measured state as the nodes read it (wholebody_mpc_node, medium_horizon_node; Phase 5A Task 4, moved
// out of wholebody_mpc_node unchanged): TF map -> base_footprint (latest, without waiting; the last pose stands in when
// a lookup fails, and it ages), the six arm joints merged by name from /joint_states (non-finite values ignored),
// /odom for the measured velocity, and the JTC's controller_state for arm control alive. Thread safe: the input
// callbacks and measure() may run on different threads.
#include <array>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>

#include "mobile_manipulator_mpc/model.hpp"

namespace mobile_manipulator_mpc
{
constexpr double kStateStaleS = 0.5;  // joint states, odometry, TF and arm control older than this are stale

struct MeasuredState
{
  State x = State::Zero();
  // Joint velocities from the latest message that had the joint; zero when that message had no finite velocity.
  Eigen::Matrix<double, 6, 1> qd = Eigen::Matrix<double, 6, 1>::Zero();
  bool complete = false;  // every part received at least once
  bool fresh = false;
  std::string missing, stale;  // descriptions for the result (set on failure only)
  bool tf_problem = false;
  double v = 0.0, omega = 0.0;  // odometry: planar speed and yaw rate
};

class StateTracker
{
public:
  // `steady_s` arguments: the steady clock in seconds (receipt times); `ros_now_s`: the node clock (TF stamps).
  void on_joints(const sensor_msgs::msg::JointState & m, double steady_s);
  void on_odom(const nav_msgs::msg::Odometry & m, double steady_s);
  void on_arm_state(double steady_s);  // arm control alive is checked only once it was seen
  MeasuredState measure(const tf2_ros::Buffer & tf, double ros_now_s, double steady_s);

private:
  std::mutex mutex_;
  Eigen::Matrix<double, 6, 1> joints_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> velocities_ = Eigen::Matrix<double, 6, 1>::Zero();
  std::array<double, 6> joint_received_{NAN, NAN, NAN, NAN, NAN, NAN};
  double odom_v_ = 0.0, odom_omega_ = 0.0, odom_received_ = NAN;
  double arm_state_received_ = NAN;
  std::optional<std::array<double, 3>> pose_;  // the last transform (measure() only)
  double pose_stamp_ = 0.0;
};
}  // namespace mobile_manipulator_mpc
