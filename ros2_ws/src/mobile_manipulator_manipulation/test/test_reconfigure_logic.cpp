#include <gtest/gtest.h>

#include <cmath>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "mobile_manipulator_interfaces/reconfigure_panel_codes.hpp"
#include "mobile_manipulator_manipulation/reconfigure_logic.hpp"

namespace mmm = mobile_manipulator_manipulation;

namespace
{
geometry_msgs::msg::Vector3 vec(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

builtin_interfaces::msg::Time stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

// Samples every 0.05 s from `from` to `to` (inclusive) at the given speeds.
void feed(mmm::BaseMotionWindow & window, double from, double to, double linear, double angular = 0.0)
{
  for (double t = from; t <= to + 1e-9; t += 0.05) window.add(t, linear, angular);
}
}  // namespace

TEST(PanelGoal, ConstraintsUseThePanelSubframe)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_footprint";
  pose.pose.position.x = -0.08;
  pose.pose.position.y = 0.225;
  pose.pose.position.z = 1.32;
  pose.pose.orientation.w = 1.0;
  const auto constraints = mmm::panel_goal_constraints(pose, vec(0.05, 0.04, 0.03), vec(0.1, 0.2, 0.3));

  ASSERT_EQ(constraints.position_constraints.size(), 1u);
  const auto & position = constraints.position_constraints[0];
  EXPECT_EQ(position.link_name, "panel/panel");
  EXPECT_EQ(position.header.frame_id, "base_footprint");
  ASSERT_EQ(position.constraint_region.primitives.size(), 1u);
  const auto & box = position.constraint_region.primitives[0].dimensions;
  EXPECT_DOUBLE_EQ(box[0], 0.10);
  EXPECT_DOUBLE_EQ(box[1], 0.08);
  EXPECT_DOUBLE_EQ(box[2], 0.06);
  EXPECT_DOUBLE_EQ(position.constraint_region.primitive_poses[0].position.z, 1.32);
  EXPECT_DOUBLE_EQ(position.weight, 1.0);

  ASSERT_EQ(constraints.orientation_constraints.size(), 1u);
  const auto & orientation = constraints.orientation_constraints[0];
  EXPECT_EQ(orientation.link_name, "panel/panel");
  // MoveIt rejects XYZ Euler angles for subframes and then silently drops the constraints.
  EXPECT_EQ(orientation.parameterization, moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR);
  EXPECT_DOUBLE_EQ(orientation.absolute_x_axis_tolerance, 0.1);
  EXPECT_DOUBLE_EQ(orientation.absolute_y_axis_tolerance, 0.2);
  EXPECT_DOUBLE_EQ(orientation.absolute_z_axis_tolerance, 0.3);
  EXPECT_DOUBLE_EQ(orientation.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(orientation.weight, 1.0);
}

TEST(BaseMotionWindow, StoppedAfterHalfSecondBelowThresholds)
{
  mmm::BaseMotionWindow window;
  feed(window, 0.0, 0.6, 0.005);
  EXPECT_TRUE(window.stopped(0.6));

  mmm::BaseMotionWindow moving;
  feed(moving, 0.0, 0.25, 0.005);
  moving.add(0.3, 0.02, 0.0);
  feed(moving, 0.35, 0.6, 0.005);
  EXPECT_FALSE(moving.stopped(0.6));
  feed(moving, 0.65, 0.85, 0.005);
  EXPECT_TRUE(moving.stopped(0.85));
}

TEST(BaseMotionWindow, NeedsAFullWindowOfSamples)
{
  mmm::BaseMotionWindow window;
  feed(window, 0.0, 0.3, 0.0);
  EXPECT_FALSE(window.stopped(0.3));
  EXPECT_FALSE(mmm::BaseMotionWindow().stopped(10.0));
}

// Minor 10: /odom pausing inside the window hides whatever the base did meanwhile.
TEST(BaseMotionWindow, GapInsideTheWindowDoesNotCount)
{
  mmm::BaseMotionWindow window;
  feed(window, 0.0, 0.2, 0.0);
  feed(window, 0.45, 0.6, 0.0);  // nothing from 0.2 to 0.45
  EXPECT_FALSE(window.stopped(0.6));
  feed(window, 0.62, 0.8, 0.0);
  EXPECT_FALSE(window.stopped(0.8));  // the gap is still inside the window
  feed(window, 0.82, 0.96, 0.0);
  EXPECT_TRUE(window.stopped(0.96));
}

TEST(BaseMotionWindow, StaleSamplesDoNotCount)
{
  mmm::BaseMotionWindow window;
  feed(window, 0.0, 1.0, 0.0);
  EXPECT_FALSE(window.stopped(3.0));  // /odom stopped arriving
}

TEST(BaseMotionWindow, YawRateCounts)
{
  mmm::BaseMotionWindow window;
  feed(window, 0.0, 0.6, 0.0, 0.03);
  EXPECT_FALSE(window.stopped(0.6));
}

// Review Focus 1: a teleport or new simulation epoch resets /clock.
TEST(BaseMotionWindow, ResetsWhenTimeGoesBackwards)
{
  mmm::BaseMotionWindow window;
  feed(window, 9.0, 10.0, 0.0);
  window.add(1.0, 0.0, 0.0);
  EXPECT_FALSE(window.stopped(1.0));
  feed(window, 1.05, 1.5, 0.0);
  EXPECT_TRUE(window.stopped(1.5));
}

// Final review I1: two odom callbacks handled out of order must not look like a new epoch.
TEST(BaseMotionWindow, SmallReorderingDoesNotReset)
{
  mmm::BaseMotionWindow window;
  feed(window, 0.0, 0.6, 0.0);
  window.add(0.55, 0.0, 0.0);  // a slightly older sample handled late
  EXPECT_TRUE(window.stopped(0.6));
  window.add(0.58, 0.05, 0.0);  // a late moving sample still counts
  EXPECT_FALSE(window.stopped(0.6));
}

// Review Focus 2: a Unity stream pause right after execution.
TEST(FreshState, RejectsSamplesBeforeTrajectoryEnd)
{
  EXPECT_FALSE(mmm::fresh_after(stamp(10, 0), stamp(10, 1)));
  EXPECT_TRUE(mmm::fresh_after(stamp(10, 1), stamp(10, 1)));
  EXPECT_TRUE(mmm::fresh_after(stamp(11, 0), stamp(10, 999999999)));
}

TEST(TrajectoryMetrics, JointPathLengthSumsAbsoluteSteps)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"a", "b"};
  for (const auto & q : std::vector<std::vector<double>>{{0.0, 0.0}, {0.5, -0.2}, {0.25, 0.1}}) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = q;
    trajectory.points.push_back(point);
  }
  EXPECT_NEAR(mmm::joint_path_length(trajectory), 0.5 + 0.2 + 0.25 + 0.3, 1e-12);
}

namespace
{
Eigen::Isometry3d pose_at(double x, double y, double z, const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = q.toRotationMatrix();
  return pose;
}
}  // namespace

// The server's own check that a plan reaches the requested panel pose (MoveIt dropped
// unsupported constraints without failing the plan).
TEST(PanelGoal, MetWithinTolerances)
{
  const Eigen::Quaterniond target(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.1;
  goal.pose.position.y = 0.2;
  goal.pose.position.z = 1.3;
  goal.pose.orientation.x = target.x();
  goal.pose.orientation.y = target.y();
  goal.pose.orientation.z = target.z();
  goal.pose.orientation.w = target.w();
  const auto tol = vec(0.01, 0.01, 0.01);
  const auto ori = vec(0.01, 0.01, 0.01);
  EXPECT_TRUE(mmm::panel_goal_met(pose_at(0.105, 0.195, 1.309, target), goal, tol, ori));
  EXPECT_FALSE(mmm::panel_goal_met(pose_at(0.12, 0.2, 1.3, target), goal, tol, ori));
  const Eigen::Quaterniond tilted = target * Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(mmm::panel_goal_met(pose_at(0.1, 0.2, 1.3, tilted), goal, tol, ori));
  const Eigen::Quaterniond nudged = target * Eigen::Quaterniond(Eigen::AngleAxisd(0.008, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(mmm::panel_goal_met(pose_at(0.1, 0.2, 1.3, nudged), goal, tol, ori));
}

// Follow-up 2a: trajectories run in simulation time; a slow simulation must not abort a
// healthy execution, while a frozen one still ends within a bounded wall time.
TEST(ExecutionDeadline, FollowsSimTimeWithAWallCap)
{
  const double budget = 10.0;
  EXPECT_FALSE(mmm::execution_overdue(5.0, 12.0, budget));   // RTF 0.4: sim still within budget
  EXPECT_TRUE(mmm::execution_overdue(10.5, 12.0, budget));   // sim budget spent
  EXPECT_FALSE(mmm::execution_overdue(1.0, 39.0, budget));   // nearly frozen, wall cap not reached
  EXPECT_TRUE(mmm::execution_overdue(1.0, 40.5, budget));    // frozen: wall cap (4 x budget)
}

// A plan MoveIt rejects after smoothing (reported as FAILURE) is worth repeating; an IK or
// goal-constraint failure is not.
TEST(Replan, OnlyAfterPlanningFailures)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  EXPECT_TRUE(mmm::replan_after(Codes::FAILURE));
  EXPECT_TRUE(mmm::replan_after(Codes::PLANNING_FAILED));
  EXPECT_TRUE(mmm::replan_after(Codes::INVALID_MOTION_PLAN));
  EXPECT_FALSE(mmm::replan_after(Codes::SUCCESS));
  EXPECT_FALSE(mmm::replan_after(Codes::NO_IK_SOLUTION));
  EXPECT_FALSE(mmm::replan_after(Codes::GOAL_CONSTRAINTS_VIOLATED));
  EXPECT_FALSE(mmm::replan_after(Codes::PREEMPTED));
}

// Follow-up 3a: a hold with no joint samples is unmeasured, not perfect.
TEST(HoldError, NaNWithoutSamples)
{
  mmm::HoldErrorTracker hold({0.0, 1.0});
  EXPECT_TRUE(std::isnan(hold.value()));
  hold.add({0.01, 0.97});
  hold.add({-0.02, 1.0});
  EXPECT_NEAR(hold.value(), 0.03, 1e-12);
  EXPECT_EQ(hold.samples(), 2u);
}

// Reports and run summaries spell result codes by these names (CLI and mission share them).
TEST(ReconfigureCodes, NamesFollowTheActionConstants)
{
  using Result = mobile_manipulator_interfaces::action::ReconfigurePanel::Result;
  using mobile_manipulator_interfaces::reconfigure_code_name;
  EXPECT_STREQ(reconfigure_code_name(Result::SUCCESS), "SUCCESS");
  EXPECT_STREQ(reconfigure_code_name(Result::BASE_NOT_STOPPED), "BASE_NOT_STOPPED");
  EXPECT_STREQ(reconfigure_code_name(Result::PROFILE_VIOLATED_AFTER_EXECUTION), "PROFILE_VIOLATED_AFTER_EXECUTION");
  EXPECT_STREQ(reconfigure_code_name(Result::CANCELED), "CANCELED");
  EXPECT_STREQ(reconfigure_code_name(200), "UNKNOWN");
}

// Minor 14: a relaunched Nav2 keeps its launch footprint until the server republishes.
