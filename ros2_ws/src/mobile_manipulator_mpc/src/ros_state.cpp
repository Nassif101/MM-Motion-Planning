#include "mobile_manipulator_mpc/ros_state.hpp"

#include <cmath>

#include <rclcpp/time.hpp>
#include <tf2/exceptions.h>

namespace mobile_manipulator_mpc
{
namespace
{
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

void StateTracker::on_joints(const sensor_msgs::msg::JointState & m, double steady_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
    if (!std::isfinite(m.position[i])) continue;
    for (size_t j = 0; j < kArmJoints.size(); ++j) {
      if (m.name[i] != kArmJoints[j]) continue;
      joints_(static_cast<int>(j)) = m.position[i];
      velocities_(static_cast<int>(j)) = i < m.velocity.size() && std::isfinite(m.velocity[i]) ? m.velocity[i] : 0.0;
      joint_received_[j] = steady_s;
      break;
    }
  }
}

void StateTracker::on_odom(const nav_msgs::msg::Odometry & m, double steady_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_v_ = std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y);
  odom_omega_ = m.twist.twist.angular.z;
  odom_received_ = steady_s;
}

void StateTracker::on_arm_state(double steady_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  arm_state_received_ = steady_s;
}

MeasuredState StateTracker::measure(const tf2_ros::Buffer & tf, double ros_now_s, double steady_s)
{
  MeasuredState m;
  std::lock_guard<std::mutex> lock(mutex_);
  m.x.tail<6>() = joints_;
  m.qd = velocities_;
  m.v = odom_v_;
  m.omega = odom_omega_;
  m.complete = true;
  m.fresh = true;
  for (size_t j = 0; j < kArmJoints.size(); ++j) {
    if (!std::isfinite(joint_received_[j])) {
      m.complete = false;
      m.missing += (m.missing.empty() ? "" : ", ") + std::string(kArmJoints[j]);
    } else if (steady_s - joint_received_[j] > kStateStaleS) {
      m.fresh = false;
      m.stale += (m.stale.empty() ? "joint states " : ", ") + std::string(kArmJoints[j]);
    }
  }
  if (!std::isfinite(odom_received_)) {
    m.complete = false;
    m.missing += (m.missing.empty() ? "" : ", ") + std::string("odometry");
  } else if (steady_s - odom_received_ > kStateStaleS) {
    m.fresh = false;
    m.stale += (m.stale.empty() ? "" : "; ") + std::string("odometry");
  }
  if (std::isfinite(arm_state_received_) && steady_s - arm_state_received_ > kStateStaleS) {
    m.fresh = false;
    m.stale += (m.stale.empty() ? "" : "; ") + std::string("arm control (/arm_controller/controller_state)");
  }
  try {
    if (tf.canTransform("map", "base_footprint", tf2::TimePointZero)) {
      const auto t = tf.lookupTransform("map", "base_footprint", tf2::TimePointZero);
      pose_ = {t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
      pose_stamp_ = rclcpp::Time(t.header.stamp).seconds();
    }
  } catch (const tf2::TransformException &) {
    // the last pose stands in, and it ages
  }
  if (!pose_) {
    m.complete = false;
    m.tf_problem = true;
    m.missing += (m.missing.empty() ? "" : ", ") + std::string("TF map -> base_footprint");
  } else {
    m.x(0) = (*pose_)[0];
    m.x(1) = (*pose_)[1];
    m.x(2) = (*pose_)[2];
    if (ros_now_s - pose_stamp_ > kStateStaleS) {
      m.fresh = false;
      m.tf_problem = true;
      m.stale += (m.stale.empty() ? "" : "; ") + std::string("TF map -> base_footprint");
    }
  }
  return m;
}
}  // namespace mobile_manipulator_mpc
