// The measured state the nodes assemble (ros_state.hpp, Phase 5A Task 4: moved out of wholebody_mpc_node).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>

#include "mobile_manipulator_mpc/ros_state.hpp"

namespace
{
namespace mmm = mobile_manipulator_mpc;

sensor_msgs::msg::JointState joints(std::vector<std::string> names, std::vector<double> positions,
                                    std::vector<double> velocities = {})
{
  sensor_msgs::msg::JointState m;
  m.name = std::move(names);
  m.position = std::move(positions);
  m.velocity = std::move(velocities);
  return m;
}

const std::vector<std::string> kModel(mmm::kArmJoints.begin(), mmm::kArmJoints.end());

class RosState : public ::testing::Test
{
protected:
  void SetUp() override { buffer_ = std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)); }

  void base_pose(double x, double y, double yaw, double stamp_s)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_footprint";
    t.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_s * 1e9), RCL_ROS_TIME);
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    buffer_->setTransform(t, "test", false);
  }

  void all_parts(mmm::StateTracker & tracker, double steady)
  {
    tracker.on_joints(joints(kModel, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}), steady);
    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = 0.3;
    odom.twist.twist.linear.y = 0.4;
    odom.twist.twist.angular.z = -0.2;
    tracker.on_odom(odom, steady);
    base_pose(1.0, 2.0, 0.5, 10.0);
  }

  std::unique_ptr<tf2_ros::Buffer> buffer_;
};

TEST_F(RosState, MergesJointsByNameIntoModelOrder)
{
  mmm::StateTracker tracker;
  all_parts(tracker, 0.0);
  tracker.on_joints(joints({"elbow_joint", "front_left_wheel_joint", "shoulder_pan_joint"}, {-1.0, 9.0, 0.7}), 0.1);
  const auto m = tracker.measure(*buffer_, 10.1, 0.1);
  EXPECT_TRUE(m.complete);
  EXPECT_TRUE(m.fresh);
  EXPECT_DOUBLE_EQ(m.x(3), 0.7);   // shoulder_pan
  EXPECT_DOUBLE_EQ(m.x(4), 0.2);   // shoulder_lift from the first message
  EXPECT_DOUBLE_EQ(m.x(5), -1.0);  // elbow
  EXPECT_NEAR(m.x(0), 1.0, 1e-12);
  EXPECT_NEAR(m.x(1), 2.0, 1e-12);
  EXPECT_NEAR(m.x(2), 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(m.v, 0.5);
  EXPECT_DOUBLE_EQ(m.omega, -0.2);
}

TEST_F(RosState, IgnoresNonFiniteJoints)
{
  mmm::StateTracker tracker;
  all_parts(tracker, 0.0);
  tracker.on_joints(joints({"wrist_1_joint"}, {NAN}), 0.1);
  EXPECT_DOUBLE_EQ(tracker.measure(*buffer_, 10.1, 0.1).x(6), 0.4);
}

TEST_F(RosState, JointVelocitiesWhenTheMessageHasThem)
{
  mmm::StateTracker tracker;
  all_parts(tracker, 0.0);
  EXPECT_TRUE(tracker.measure(*buffer_, 10.0, 0.0).qd.isZero());
  tracker.on_joints(joints(kModel, {0, 0, 0, 0, 0, 0}, {1, 2, 3, 4, 5, 6}), 0.1);
  EXPECT_DOUBLE_EQ(tracker.measure(*buffer_, 10.1, 0.1).qd(2), 3.0);
}

TEST_F(RosState, NamesWhatIsMissing)
{
  mmm::StateTracker tracker;
  tracker.on_joints(joints({"shoulder_pan_joint"}, {0.0}), 0.0);
  const auto m = tracker.measure(*buffer_, 10.0, 0.0);
  EXPECT_FALSE(m.complete);
  EXPECT_TRUE(m.tf_problem);
  EXPECT_NE(m.missing.find("shoulder_lift_joint"), std::string::npos) << m.missing;
  EXPECT_NE(m.missing.find("odometry"), std::string::npos) << m.missing;
  EXPECT_NE(m.missing.find("TF map -> base_footprint"), std::string::npos) << m.missing;
}

TEST_F(RosState, StaleAfterHalfASecond)
{
  mmm::StateTracker tracker;
  all_parts(tracker, 0.0);
  EXPECT_TRUE(tracker.measure(*buffer_, 10.5, 0.5).fresh);
  const auto joints_old = tracker.measure(*buffer_, 10.0, 0.51);
  EXPECT_FALSE(joints_old.fresh);
  EXPECT_NE(joints_old.stale.find("joint states"), std::string::npos) << joints_old.stale;
  EXPECT_NE(joints_old.stale.find("odometry"), std::string::npos) << joints_old.stale;
  const auto tf_old = tracker.measure(*buffer_, 10.51, 0.0);
  EXPECT_FALSE(tf_old.fresh);
  EXPECT_TRUE(tf_old.tf_problem);
}

TEST_F(RosState, ArmControlIsCheckedOnlyOnceSeen)
{
  mmm::StateTracker tracker;
  all_parts(tracker, 1.0);
  EXPECT_TRUE(tracker.measure(*buffer_, 10.0, 1.0).fresh);  // never seen: not checked
  tracker.on_arm_state(0.0);
  const auto m = tracker.measure(*buffer_, 10.0, 1.0);
  EXPECT_FALSE(m.fresh);
  EXPECT_NE(m.stale.find("arm control"), std::string::npos) << m.stale;
}
}  // namespace
