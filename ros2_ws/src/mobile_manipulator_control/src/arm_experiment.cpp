// Bounded commissioning goals through the standard FollowJointTrajectory action.
//
// Usage: arm_experiment [--joint NAME --delta RAD | --positions Q1..Q6] [--duration 1..30]
//                       [--cancel-after S] [--hold-seconds 0..60]
//                       [--disturbance none|compact|extended|gate] [--output FILE]
// Sends one synchronized cubic segment (zero end velocities) after checking it against the
// URDF limits and a conservative reference-payload acceleration envelope, optionally holds
// the goal while driving a bounded simulation-only base sequence (requires a cleared test
// area), and prints a JSON report as the last line. Exit 1 when the goal fails (unless it
// was cancelled on purpose) or the experiment cannot run; 2 for invalid arguments.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include "mobile_manipulator_control/cli.hpp"

using Json = nlohmann::ordered_json;
using Clock = std::chrono::steady_clock;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

namespace
{
const std::array<std::string, 6> kNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
// Conservative reference-payload operating envelope; JTC itself is not a trajectory retimer.
const std::array<double, 6> kAcceleration = {2.0, 1.5, 2.0, 3.0, 3.0, 4.0};

// (duration in simulation seconds, forward m/s, yaw rad/s). Always finish stopped.
const std::map<std::string, std::vector<std::array<double, 3>>> kSchedules = {
  {"none", {}},
  {"compact", {{3, .3, 0}, {1, 0, 0}, {3, -.3, 0}, {1, 0, 0}, {2, 0, .4}, {1, 0, 0}, {2, 0, -.4}, {1, 0, 0},
               {2, .15, .2}, {1, 0, 0}, {2, .15, -.2}, {1, 0, 0}, {2, -.3, 0}, {2, 0, 0}}},
  {"extended", {{2, .15, 0}, {1, 0, 0}, {2, -.15, 0}, {1, 0, 0}, {2, 0, .2}, {1, 0, 0}, {2, 0, -.2}, {3, 0, 0}}},
  {"gate", {{18, .2, 0}, {3, 0, 0}}},
};

double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

struct ArmState
{
  std::map<std::string, double> position;
  std::optional<Clock::time_point> received;
  bool complete() const
  {
    return std::all_of(kNames.begin(), kNames.end(), [this](const auto & n) { return position.count(n); });
  }
};

int usage(const std::string & message)
{
  std::cerr << "arm_experiment: " << message << std::endl;
  rclcpp::shutdown();
  return 2;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const double delta = args.number("delta", 0.05), duration = args.number("duration", 4.0);
  double hold_seconds = args.number("hold-seconds", 0.0);
  const std::string joint = args.get("joint", ""), disturbance = args.get("disturbance", "none");
  const std::string output = args.get("output", "/tmp/arm-action-result.json");
  if (!joint.empty() && std::find(kNames.begin(), kNames.end(), joint) == kNames.end()) return usage("unknown --joint");
  if (!kSchedules.count(disturbance)) return usage("--disturbance must be none, compact, extended or gate");
  if (!(duration >= 1 && duration <= 30) || std::abs(delta) > 1 || !(hold_seconds >= 0 && hold_seconds <= 60)) {
    return usage("Use duration 1..30 seconds and delta <= 1 rad");
  }

  auto node = std::make_shared<rclcpp::Node>(
    "arm_commissioning", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  ArmState actual;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/state", 1, [&actual](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) actual.position[m->name[i]] = m->position[i];
      actual.received = Clock::now();
    });
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(node, "/arm_controller/follow_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto fresh = [&actual] { return actual.received && since(*actual.received) <= 0.5; };
  const auto sim_now = [&node] { return node->get_clock()->now().seconds(); };

  try {
    const auto deadline = Clock::now() + std::chrono::seconds(10);
    while (!actual.complete() && Clock::now() < deadline) executor.spin_once(std::chrono::milliseconds(100));
    if (!actual.complete() || !fresh()) throw std::runtime_error("No fresh physical arm state");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) throw std::runtime_error("Arm action server unavailable");

    std::vector<double> start, target;
    for (const auto & n : kNames) start.push_back(actual.position.at(n));
    if (args.has("positions")) {
      target = args.numbers("positions", 6);
    } else {
      for (size_t i = 0; i < kNames.size(); ++i) {
        target.push_back(start[i] + (joint.empty() || joint == kNames[i] ? delta : 0.0));
      }
    }
    urdf::Model model;
    const auto urdf_path = ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
                           "/urdf/mobile_manipulator.urdf";
    if (!model.initFile(urdf_path)) throw std::runtime_error("Cannot read " + urdf_path);
    for (size_t i = 0; i < kNames.size(); ++i) {
      const auto urdf_joint = model.getJoint(kNames[i]);
      if (!urdf_joint || !urdf_joint->limits) throw std::runtime_error("No limits for " + kNames[i]);
      const auto & limit = *urdf_joint->limits;
      const double displacement = std::abs(target[i] - start[i]), segment = duration - 0.1;
      if (!std::isfinite(target[i]) || !(limit.lower <= target[i] && target[i] <= limit.upper) ||
          1.5 * displacement / segment > 0.5 * limit.velocity ||
          6 * displacement / (segment * segment) > 0.5 * kAcceleration[i]) {
        throw std::invalid_argument("Unsafe cubic segment for " + kNames[i] +
                                    "; increase duration or reduce displacement");
      }
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names.assign(kNames.begin(), kNames.end());
    // Explicit zero endpoint velocities yield synchronized cubic interpolation in JTC.
    const std::vector<std::pair<double, std::vector<double>>> points = {{0.1, start}, {duration, target}};
    for (const auto & [time, positions] : points) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions;
      point.velocities.assign(6, 0.0);
      point.time_from_start.sec = static_cast<int32_t>(time);
      point.time_from_start.nanosec = static_cast<uint32_t>(std::lround((time - static_cast<int>(time)) * 1e9));
      goal.trajectory.points.push_back(point);
    }
    auto handle_future = client->async_send_goal(goal);
    if (executor.spin_until_future_complete(handle_future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS ||
        !handle_future.get()) {
      throw std::runtime_error("Goal rejected or response timed out");
    }
    const auto handle = handle_future.get();
    auto result_future = client->async_get_result(handle);
    const auto done = [&result_future] {
      return result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    };
    const auto began = Clock::now();
    bool cancelled = false;
    const auto cancel = [&] {
      auto future = client->async_cancel_goal(handle);
      executor.spin_until_future_complete(future, std::chrono::seconds(3));
    };
    while (!done() && since(began) < 120) {
      executor.spin_once(std::chrono::milliseconds(50));
      if (args.has("cancel-after") && !cancelled && since(began) >= args.number("cancel-after", 0.0)) {
        cancel();
        cancelled = true;
      }
    }
    if (!done()) {
      cancel();
      throw std::runtime_error("Experiment timed out; cancellation requested");
    }
    const auto response = result_future.get();
    const int error_code = response.result ? response.result->error_code : -1;
    const std::string error_string = response.result ? response.result->error_string : "";

    const double hold_start = sim_now();
    std::vector<double> hold_max(6, 0.0);
    const auto & schedule = kSchedules.at(disturbance);
    double total = 0.0;
    for (const auto & step : schedule) total += step[0];
    hold_seconds = std::max(hold_seconds, total);
    auto base = schedule.empty() ? nullptr : node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    const auto hold_deadline = Clock::now() + std::chrono::duration<double>(std::max(10.0, hold_seconds * 5));
    double last_publish = -1.0;
    std::string hold_failure;
    while (error_code == 0 && sim_now() - hold_start < hold_seconds) {
      executor.spin_once(std::chrono::milliseconds(10));
      if (Clock::now() > hold_deadline || !fresh()) {
        hold_failure = "Hold observation lost feedback or simulation stopped";
        break;
      }
      const double elapsed = sim_now() - hold_start;
      if (base && elapsed - last_publish >= .05) {
        geometry_msgs::msg::Twist command;
        double boundary = 0.0;
        for (const auto & [seconds, v, w] : schedule) {
          boundary += seconds;
          if (elapsed < boundary) {
            command.linear.x = v;
            command.angular.z = w;
            break;
          }
        }
        base->publish(command);
        last_publish = elapsed;
      }
      for (size_t i = 0; i < kNames.size(); ++i) {
        hold_max[i] = std::max(hold_max[i], std::abs(actual.position.at(kNames[i]) - target[i]));
      }
    }
    if (base) {
      for (int i = 0; i < 3; ++i) {
        base->publish(geometry_msgs::msg::Twist());
        executor.spin_once(std::chrono::milliseconds(50));
      }
    }
    if (!hold_failure.empty()) throw std::runtime_error(hold_failure);

    std::vector<double> final;
    for (const auto & n : kNames) final.push_back(actual.position.at(n));
    const Json report = {{"status", static_cast<int>(response.code)}, {"error_code", error_code},
                         {"error_string", error_string}, {"start", start}, {"target", target}, {"final", final},
                         {"wall_seconds", since(began)}, {"hold_seconds", hold_seconds},
                         {"hold_max_error", hold_max}, {"disturbance", disturbance}};
    std::ofstream(output) << report.dump(2) << "\n";
    std::cout << report.dump() << std::endl;
    rclcpp::shutdown();
    return error_code != 0 && !cancelled ? 1 : 0;
  } catch (const std::exception & error) {
    std::cerr << "arm_experiment: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
