#include "mobile_manipulator_mpc/replay_bag.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mobile_manipulator_interfaces/msg/whole_body_plan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "mobile_manipulator_mpc/arm_stream.hpp"
#include "mobile_manipulator_mpc/executed_command.hpp"

namespace mobile_manipulator_mpc
{
namespace
{
namespace mmi = mobile_manipulator_interfaces;
constexpr size_t kCyclePadding = 128;  // more than every field added to MpcCycle since the 4B-A recordings

double seconds(const builtin_interfaces::msg::Time & t) { return t.sec + 1e-9 * t.nanosec; }

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

template <class T>
T decode(const rosbag2_storage::SerializedBagMessage & message)
{
  T out;
  const rclcpp::SerializedMessage serialized(*message.serialized_data);
  rclcpp::Serialization<T>().deserialize_message(&serialized, &out);
  return out;
}

// The plan's states in the model's joint order (`order[j]`: the message's index of model joint j).
std::vector<State> plan_states(const mmi::msg::WholeBodyPlan & plan, const std::array<std::string, 6> & joint_names)
{
  std::array<size_t, 6> order{};
  for (size_t j = 0; j < 6; ++j) {
    const auto it = std::find(plan.joint_names.begin(), plan.joint_names.end(), joint_names[j]);
    if (it == plan.joint_names.end()) return {};
    order[j] = static_cast<size_t>(it - plan.joint_names.begin());
  }
  const size_t n = plan.base.size(), width = plan.joint_names.size();
  if (plan.positions.size() < n * width) return {};
  std::vector<State> states(n);
  for (size_t k = 0; k < n; ++k) {
    states[k](0) = plan.base[k].x;
    states[k](1) = plan.base[k].y;
    states[k](2) = plan.base[k].theta;
    for (size_t j = 0; j < 6; ++j) states[k](3 + static_cast<int>(j)) = plan.positions[k * width + order[j]];
  }
  return states;
}
}  // namespace

mmi::msg::MpcCycle decode_cycle(const std::vector<uint8_t> & cdr)
{
  rclcpp::SerializedMessage serialized(cdr.size() + kCyclePadding);
  auto & raw = serialized.get_rcl_serialized_message();
  std::copy(cdr.begin(), cdr.end(), raw.buffer);
  std::fill(raw.buffer + cdr.size(), raw.buffer + cdr.size() + kCyclePadding, 0);
  raw.buffer_length = cdr.size() + kCyclePadding;
  mmi::msg::MpcCycle out;
  rclcpp::Serialization<mmi::msg::MpcCycle>().deserialize_message(&serialized, &out);
  return out;
}

Recording read_recording(const std::string & bag_uri, const std::array<std::string, 6> & joint_names)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  Recording recording;
  ExecutedCommand executed;
  // Snapshot at each sent command (the cycle's /cmd_vel_smoothed precedes its plan and cycle messages): the previous
  // base command, the stream and the odometry as they were before that cycle's step.
  struct Pending
  {
    double t_send = 0.0;
    Eigen::Vector2d previous = Eigen::Vector2d::Zero();
    std::optional<trajectory_msgs::msg::JointTrajectory> trajectory;
    double trajectory_t = 0.0;
    double odom_v = 0.0, odom_omega = 0.0;
  };
  std::optional<Pending> pending;
  std::optional<trajectory_msgs::msg::JointTrajectory> trajectory;
  double trajectory_t = 0.0, odom_v = 0.0, odom_omega = 0.0;
  std::map<int64_t, std::vector<State>> plans;      // by header stamp (ns), awaiting their cycle
  std::map<int64_t, size_t> cycle_at_stamp;         // cycles awaiting their plan
  const auto key = [](const builtin_interfaces::msg::Time & t) { return rclcpp::Time(t).nanoseconds(); };

  while (reader.has_next()) {
    const auto message = reader.read_next();
    const double t = 1e-9 * static_cast<double>(message->recv_timestamp);
    const std::string & topic = message->topic_name;
    if (topic == "/plan") {
      const auto path = decode<nav_msgs::msg::Path>(*message);
      if (path.poses.empty()) continue;
      RecordedGoal goal;
      goal.bag_time = t;
      for (const auto & p : path.poses) goal.points.emplace_back(p.pose.position.x, p.pose.position.y);
      const auto & last = path.poses.back().pose;
      goal.goal = {last.position.x, last.position.y, yaw_of(last.orientation)};
      recording.goals.push_back(std::move(goal));
    } else if (topic == "/scenario/movers") {
      const auto movers = decode<geometry_msgs::msg::PoseArray>(*message);
      MoverSample sample{t, seconds(movers.header.stamp), {}};
      for (const auto & p : movers.poses) sample.positions.emplace_back(p.position.x, p.position.y);
      recording.movers.push_back(std::move(sample));
    } else if (topic == "/odom") {
      const auto odom = decode<nav_msgs::msg::Odometry>(*message);
      odom_v = odom.twist.twist.linear.x;
      odom_omega = odom.twist.twist.angular.z;
    } else if (topic == "/cmd_vel_smoothed") {
      const auto sent = decode<geometry_msgs::msg::Twist>(*message);
      pending = Pending{t, executed.previous(t, kExecutedStaleS), trajectory, trajectory_t, odom_v, odom_omega};
      executed.on_sent(t, Eigen::Vector2d(sent.linear.x, sent.angular.z));
    } else if (topic == "/cmd_vel") {
      const auto command = decode<geometry_msgs::msg::Twist>(*message);
      executed.on_executed(t, Eigen::Vector2d(command.linear.x, command.angular.z));
    } else if (topic == "/arm_controller/joint_trajectory") {
      trajectory = decode<trajectory_msgs::msg::JointTrajectory>(*message);
      trajectory_t = t;
    } else if (topic == "/wholebody_mpc/plan") {
      const auto plan = decode<mmi::msg::WholeBodyPlan>(*message);
      std::vector<State> states = plan_states(plan, joint_names);
      const auto waiting = cycle_at_stamp.find(key(plan.header.stamp));
      if (waiting != cycle_at_stamp.end()) {
        RecordedCycle & c = recording.cycles[waiting->second];
        c.plan = std::move(states);
        if (!c.plan.empty()) c.x = c.plan.front();
        cycle_at_stamp.erase(waiting);
      } else {
        plans[key(plan.header.stamp)] = std::move(states);
      }
    } else if (topic == "/wholebody_mpc/cycle") {
      const auto & data = *message->serialized_data;
      const auto cycle = decode_cycle(std::vector<uint8_t>(data.buffer, data.buffer + data.buffer_length));
      RecordedCycle c;
      c.stamp = seconds(cycle.header.stamp);
      c.status = cycle.status;
      c.posture_choice = cycle.posture_choice;
      c.iterations = cycle.iterations;
      c.deadline_missed = cycle.deadline_missed;
      c.state_to_command_s = cycle.state_to_command_s;
      const double t_send = pending ? pending->t_send : t;
      c.bag_time = t_send - cycle.state_to_command_s;
      if (pending) {
        c.executed = pending->previous;
        c.odom_v = pending->odom_v;
        c.odom_omega = pending->odom_omega;
        if (pending->trajectory && !pending->trajectory->points.empty()) {
          Eigen::Matrix<double, 6, 1> q;
          sample_trajectory(*pending->trajectory, c.bag_time - pending->trajectory_t, q, c.arm_v);
        }
      }
      const auto plan = plans.find(key(cycle.header.stamp));
      if (plan != plans.end()) {
        c.plan = std::move(plan->second);
        plans.erase(plan);
        if (!c.plan.empty()) c.x = c.plan.front();
      } else {
        cycle_at_stamp[key(cycle.header.stamp)] = recording.cycles.size();
      }
      recording.cycles.push_back(std::move(c));
      pending.reset();
    }
  }
  // Cycles whose plan never arrived have no state to replay from.
  recording.cycles.erase(std::remove_if(recording.cycles.begin(), recording.cycles.end(),
                                        [](const RecordedCycle & c) { return c.plan.empty(); }),
                         recording.cycles.end());
  std::vector<double> spacing;
  for (size_t i = 1; i < recording.cycles.size(); ++i) {
    const double d = recording.cycles[i].stamp - recording.cycles[i - 1].stamp;
    if (d > 0.0) spacing.push_back(d);
  }
  if (!spacing.empty()) {
    std::nth_element(spacing.begin(), spacing.begin() + static_cast<long>(spacing.size() / 2), spacing.end());
    recording.period_s = spacing[spacing.size() / 2];
  }
  return recording;
}
}  // namespace mobile_manipulator_mpc
