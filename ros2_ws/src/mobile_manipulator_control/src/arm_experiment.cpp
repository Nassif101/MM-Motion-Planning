// Bounded commissioning goals through the standard FollowJointTrajectory action.
//
// Usage: arm_experiment [--joint NAME --delta RAD | --positions Q1..Q6] [--duration 1..30]
//                       [--cancel-after S] [--hold-seconds 0..60]
//                       [--disturbance none|compact|extended|gate] [--drive-during-motion]
//                       [--cpu-load-threads N] [--output FILE]
//        arm_experiment --stream-rate HZ --stream-duration S [--stream-amplitude RAD]
//                       [--stream-frequency HZ] [--stream-stop-after S] [--cpu-load-threads N]
// Sends one synchronized cubic segment (zero end velocities) after checking it against the
// URDF limits and a conservative reference-payload acceleration envelope, optionally holds
// the goal while driving a bounded simulation-only base sequence (requires a cleared test
// area), and prints a JSON report as the last line. Exit 1 when the goal fails (unless it
// was cancelled on purpose) or the experiment cannot run; 2 for invalid arguments.
//
// --drive-during-motion starts the --disturbance sequence when the goal is sent, so the arm
// moves while the base does; the sequence then continues into the hold.
// --cpu-load-threads N runs N busy threads (not pinned to a core) for the whole run.
// --stream-rate/--stream-duration replace the goal with overlapping trajectory segments of
// a sinusoid about the current pose on /arm_controller/joint_trajectory, each published at the
// current time. A segment is 1.5 s: 1 s of the sinusoid plus a 0.5 s braking tail that ends at
// rest, because JTC rejects a segment that ends moving. --stream-stop-after stops publishing
// early (mid-motion) and the tool then observes the arm settle at the last braking-tail end.
// The report adds max_path_error_rad (largest |controller-state error| per joint while the
// trajectory ran), position_jumps (controller-state samples whose commanded position changed by
// more than 0.01 rad beyond what the commanded velocity explains) and cpu_load_threads.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <atomic>
#include <map>
#include <stdexcept>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>

#include "mobile_manipulator_control/cli.hpp"

using Json = nlohmann::ordered_json;
using Clock = std::chrono::steady_clock;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using ControllerState = control_msgs::msg::JointTrajectoryControllerState;

namespace
{
const std::array<std::string, 6> kNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
// Conservative reference-payload operating envelope; JTC itself is not a trajectory retimer.
const std::array<double, 6> kAcceleration = {2.0, 1.5, 2.0, 3.0, 3.0, 4.0};
// Duration of the braking tail appended to each streamed segment (seconds).
constexpr double kStreamTail = 0.5;

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

// Controller-state statistics over an observation window: the largest tracking error per joint
// and the commanded-position discontinuities. A commanded position that moves by what the
// commanded velocity explains is not a jump; the threshold applies to the remainder.
struct StateTracker
{
  static constexpr double kJumpRad = 0.01;
  std::array<double, 6> max_error{}, max_velocity{};
  int jumps = 0;
  size_t samples = 0;
  double max_jump = 0.0;
  bool active = false;

  void begin()
  {
    max_error.fill(0.0);
    max_velocity.fill(0.0);
    jumps = 0;
    samples = 0;
    max_jump = 0.0;
    have_last_ = false;
    active = true;
  }

  void update(const ControllerState & m)
  {
    if (!active) return;
    std::array<double, 6> position{}, velocity{}, error{};
    for (size_t j = 0; j < m.joint_names.size(); ++j) {
      const auto it = std::find(kNames.begin(), kNames.end(), m.joint_names[j]);
      if (it == kNames.end()) continue;
      const size_t i = static_cast<size_t>(it - kNames.begin());
      if (j < m.reference.positions.size()) position[i] = m.reference.positions[j];
      if (j < m.reference.velocities.size()) velocity[i] = m.reference.velocities[j];
      if (j < m.error.positions.size()) error[i] = m.error.positions[j];
    }
    const double time = rclcpp::Time(m.header.stamp).seconds();
    ++samples;
    for (size_t i = 0; i < 6; ++i) {
      max_error[i] = std::max(max_error[i], std::abs(error[i]));
      max_velocity[i] = std::max(max_velocity[i], std::abs(velocity[i]));
    }
    if (have_last_ && time > last_time_) {
      const double dt = time - last_time_;
      double worst = 0.0;
      for (size_t i = 0; i < 6; ++i) {
        const double explained = 0.5 * (velocity[i] + last_velocity_[i]) * dt;
        worst = std::max(worst, std::abs(position[i] - last_position_[i] - explained));
      }
      max_jump = std::max(max_jump, worst);
      if (worst > kJumpRad) ++jumps;
    }
    last_position_ = position;
    last_velocity_ = velocity;
    last_time_ = time;
    have_last_ = true;
  }

private:
  bool have_last_ = false;
  double last_time_ = 0.0;
  std::array<double, 6> last_position_{}, last_velocity_{};
};

// Busy threads that stay scheduler-managed (no affinity) for the lifetime of the object.
class CpuLoad
{
public:
  explicit CpuLoad(int threads)
  {
    for (int i = 0; i < threads; ++i) {
      workers_.emplace_back([this] {
        volatile double x = 1.0;
        while (!stop_) x = x * 1.0000001 + 1e-9;
      });
    }
  }
  ~CpuLoad()
  {
    stop_ = true;
    for (auto & worker : workers_) worker.join();
  }
  CpuLoad(const CpuLoad &) = delete;
  CpuLoad & operator=(const CpuLoad &) = delete;

private:
  std::atomic<bool> stop_{false};
  std::vector<std::thread> workers_;
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
  const bool drive_during_motion = args.has("drive-during-motion");
  const double cpu_threads = args.number("cpu-load-threads", 0.0);
  const bool streaming = args.has("stream-rate") || args.has("stream-duration");
  const double stream_rate = args.number("stream-rate", 0.0), stream_duration = args.number("stream-duration", 0.0);
  const double stream_amplitude = args.number("stream-amplitude", 0.2), stream_frequency = args.number("stream-frequency", 0.25);
  const double stream_stop_after = args.number("stream-stop-after", -1.0);
  if (!joint.empty() && std::find(kNames.begin(), kNames.end(), joint) == kNames.end()) return usage("unknown --joint");
  if (!kSchedules.count(disturbance)) return usage("--disturbance must be none, compact, extended or gate");
  if (!(duration >= 1 && duration <= 30) || std::abs(delta) > 1 || !(hold_seconds >= 0 && hold_seconds <= 60)) {
    return usage("Use duration 1..30 seconds and delta <= 1 rad");
  }
  if (drive_during_motion && disturbance == "none") return usage("--drive-during-motion needs a --disturbance sequence");
  if (!(cpu_threads >= 0 && cpu_threads <= 64 && cpu_threads == std::floor(cpu_threads))) {
    return usage("--cpu-load-threads must be an integer 0..64");
  }
  if (streaming) {
    if (!(stream_rate >= 1 && stream_rate <= 100) || !(stream_duration >= 2 && stream_duration <= 60) ||
        !(stream_amplitude > 0 && stream_amplitude <= 0.5) || !(stream_frequency > 0 && stream_frequency <= 1) ||
        (args.has("stream-stop-after") && !(stream_stop_after >= 1 && stream_stop_after < stream_duration))) {
      return usage("Use --stream-rate 1..100 with --stream-duration 2..60, amplitude <= 0.5 rad, frequency <= 1 Hz, "
                   "and --stream-stop-after within the duration");
    }
    if (args.has("positions") || args.has("joint") || drive_during_motion) {
      return usage("Streaming takes no --positions, --joint or --drive-during-motion");
    }
  }

  auto node = std::make_shared<rclcpp::Node>(
    "arm_commissioning", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  ArmState actual;
  StateTracker tracker;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/state", 1, [&actual](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) actual.position[m->name[i]] = m->position[i];
      actual.received = Clock::now();
    });
  auto state_subscription = node->create_subscription<ControllerState>(
    "/arm_controller/controller_state", 100,
    [&tracker](ControllerState::ConstSharedPtr m) { tracker.update(*m); });
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
      if (streaming) {
        // A sinusoid about the current pose: peak velocity A*w, peak acceleration A*w^2, and the braking tail.
        const double w = 2.0 * M_PI * stream_frequency;
        if (!(limit.lower <= start[i] - stream_amplitude && start[i] + stream_amplitude <= limit.upper) ||
            stream_amplitude * w > 0.5 * limit.velocity || stream_amplitude * w * w > 0.5 * kAcceleration[i] ||
            stream_amplitude * w / kStreamTail > 0.5 * kAcceleration[i]) {
          throw std::invalid_argument("Unsafe stream for " + kNames[i] + "; reduce amplitude or frequency");
        }
        continue;
      }
      const double displacement = std::abs(target[i] - start[i]), segment = duration - 0.1;
      if (!std::isfinite(target[i]) || !(limit.lower <= target[i] && target[i] <= limit.upper) ||
          1.5 * displacement / segment > 0.5 * limit.velocity ||
          6 * displacement / (segment * segment) > 0.5 * kAcceleration[i]) {
        throw std::invalid_argument("Unsafe cubic segment for " + kNames[i] +
                                    "; increase duration or reduce displacement");
      }
    }

    CpuLoad cpu_load(static_cast<int>(cpu_threads));
    const auto make_point = [](double time, const std::vector<double> & positions, const std::vector<double> & velocities) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions;
      point.velocities = velocities;
      point.time_from_start.sec = static_cast<int32_t>(time);
      point.time_from_start.nanosec = static_cast<uint32_t>(std::lround((time - static_cast<int>(time)) * 1e9));
      return point;
    };

    if (streaming) {
      // Overlapping 1 s segments of q0 + A sin(2 pi f t), each sampled at the publish time. A zero
      // header stamp starts each one at the controller's current time, replacing the previous one.
      auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
      const double w = 2.0 * M_PI * stream_frequency, publish_for = stream_stop_after > 0 ? stream_stop_after : stream_duration;
      const double period = 1.0 / stream_rate;
      std::vector<double> tail_end = start;
      int segments = 0;
      // Let the publisher match the subscriber before the first segment.
      for (int i = 0; i < 20; ++i) executor.spin_once(std::chrono::milliseconds(50));
      tracker.begin();
      const double t0 = sim_now();
      double next_publish = 0.0, last_publish_time = t0;
      std::vector<double> sinusoid_error(6, 0.0);
      while (sim_now() - t0 < publish_for) {
        executor.spin_once(std::chrono::milliseconds(2));
        if (!fresh()) throw std::runtime_error("Stream observation lost feedback or simulation stopped");
        const double elapsed = sim_now() - t0;
        for (size_t i = 0; i < 6; ++i) {
          sinusoid_error[i] = std::max(
            sinusoid_error[i], std::abs(actual.position.at(kNames[i]) - start[i] - stream_amplitude * std::sin(w * elapsed)));
        }
        // The simulation clock ticks at 50 Hz; keep the average rate by scheduling, not by resetting.
        if (elapsed < next_publish - 1e-6) continue;
        next_publish = std::max(next_publish + period, elapsed - period);
        trajectory_msgs::msg::JointTrajectory segment;
        segment.joint_names.assign(kNames.begin(), kNames.end());
        std::vector<double> q(6), v(6);
        for (const double tau : {0.0, 0.25, 0.5, 0.75, 1.0}) {
          for (size_t i = 0; i < 6; ++i) {
            q[i] = start[i] + stream_amplitude * std::sin(w * (elapsed + tau));
            v[i] = stream_amplitude * w * std::cos(w * (elapsed + tau));
          }
          segment.points.push_back(make_point(tau, q, v));
        }
        // JTC (allow_nonzero_velocity_at_trajectory_end: false) rejects a segment that ends moving, and with
        // true it keeps commanding that velocity once the segment is over. A braking tail at constant
        // deceleration ends the segment at rest; it never runs while newer segments keep arriving.
        for (size_t i = 0; i < 6; ++i) q[i] += 0.5 * v[i] * kStreamTail;
        tail_end = q;
        segment.points.push_back(make_point(1.0 + kStreamTail, q, std::vector<double>(6, 0.0)));
        publisher->publish(segment);
        ++segments;
        last_publish_time = sim_now();
      }
      const int jumps_stream = tracker.jumps;
      const double jump_stream = tracker.max_jump;
      // The last segment (1 s of sinusoid plus the braking tail) ends 1.5 s after it was published; allow 2.5 s more to settle.
      const double settle_until = last_publish_time + 1.0 + kStreamTail + 2.5;
      const auto settle_deadline = Clock::now() + std::chrono::seconds(60);
      while (sim_now() < settle_until) {
        executor.spin_once(std::chrono::milliseconds(10));
        if (!fresh() || Clock::now() > settle_deadline) {
          throw std::runtime_error("Settle observation lost feedback or simulation stopped");
        }
      }
      std::vector<double> final, settle_error;
      for (size_t i = 0; i < 6; ++i) {
        final.push_back(actual.position.at(kNames[i]));
        settle_error.push_back(std::abs(final[i] - tail_end[i]));
      }
      const Json report = {{"mode", "stream"}, {"start", start}, {"final", final},
                           {"stream", {{"rate_hz", stream_rate}, {"publish_seconds", publish_for},
                                       {"amplitude_rad", stream_amplitude}, {"frequency_hz", stream_frequency},
                                       {"stopped_early", stream_stop_after > 0}, {"segments", segments},
                                       {"achieved_rate_hz", segments / publish_for},
                                       {"max_error_vs_sinusoid_rad", sinusoid_error},
                                       {"braking_tail_end", tail_end}, {"settle_error_rad", settle_error}}},
                           {"max_path_error_rad", tracker.max_error},
                           {"max_reference_velocity_radps", tracker.max_velocity},
                           {"position_jumps", jumps_stream}, {"max_position_jump_rad", jump_stream},
                           {"position_jumps_after_stream", tracker.jumps - jumps_stream},
                           {"controller_state_samples", tracker.samples}, {"cpu_load_threads", static_cast<int>(cpu_threads)}};
      std::ofstream(output) << report.dump(2) << "\n";
      std::cout << report.dump() << std::endl;
      rclcpp::shutdown();
      return 0;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names.assign(kNames.begin(), kNames.end());
    // Explicit zero endpoint velocities yield synchronized cubic interpolation in JTC.
    const std::vector<double> zero_velocity(6, 0.0);
    goal.trajectory.points.push_back(make_point(0.1, start, zero_velocity));
    goal.trajectory.points.push_back(make_point(duration, target, zero_velocity));
    const auto & schedule = kSchedules.at(disturbance);
    auto base = schedule.empty() ? nullptr : node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // The base sequence runs on simulation time from drive_origin, which is the goal for
    // --drive-during-motion and the start of the hold otherwise.
    double drive_origin = -1.0, last_publish = -1.0;
    const auto drive = [&] {
      if (!base || drive_origin < 0) return;
      const double elapsed = sim_now() - drive_origin;
      if (elapsed - last_publish < .05) return;
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
    };
    const auto stop_base = [&] {
      if (!base) return;
      for (int i = 0; i < 3; ++i) {
        base->publish(geometry_msgs::msg::Twist());
        executor.spin_once(std::chrono::milliseconds(50));
      }
    };

    tracker.begin();
    if (drive_during_motion) drive_origin = sim_now();
    auto handle_future = client->async_send_goal(goal);
    if (executor.spin_until_future_complete(handle_future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS ||
        !handle_future.get()) {
      stop_base();
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
      executor.spin_once(std::chrono::milliseconds(drive_during_motion ? 10 : 50));
      drive();
      if (args.has("cancel-after") && !cancelled && since(began) >= args.number("cancel-after", 0.0)) {
        cancel();
        cancelled = true;
      }
    }
    if (!done()) {
      stop_base();
      cancel();
      throw std::runtime_error("Experiment timed out; cancellation requested");
    }
    const auto response = result_future.get();
    const int error_code = response.result ? response.result->error_code : -1;
    const std::string error_string = response.result ? response.result->error_string : "";
    tracker.active = false;

    const double hold_start = sim_now();
    std::vector<double> hold_max(6, 0.0);
    double total = 0.0;
    for (const auto & step : schedule) total += step[0];
    if (drive_origin < 0) drive_origin = hold_start;
    // The sequence must finish: what is left of it after the motion is held at the goal.
    hold_seconds = std::max(hold_seconds, total - (hold_start - drive_origin));
    const auto hold_deadline = Clock::now() + std::chrono::duration<double>(std::max(10.0, hold_seconds * 5));
    std::string hold_failure;
    while (error_code == 0 && sim_now() - hold_start < hold_seconds) {
      executor.spin_once(std::chrono::milliseconds(10));
      if (Clock::now() > hold_deadline || !fresh()) {
        hold_failure = "Hold observation lost feedback or simulation stopped";
        break;
      }
      drive();
      for (size_t i = 0; i < kNames.size(); ++i) {
        hold_max[i] = std::max(hold_max[i], std::abs(actual.position.at(kNames[i]) - target[i]));
      }
    }
    stop_base();
    if (!hold_failure.empty()) throw std::runtime_error(hold_failure);

    std::vector<double> final;
    for (const auto & n : kNames) final.push_back(actual.position.at(n));
    const Json report = {{"status", static_cast<int>(response.code)}, {"error_code", error_code},
                         {"error_string", error_string}, {"start", start}, {"target", target}, {"final", final},
                         {"wall_seconds", since(began)}, {"hold_seconds", hold_seconds},
                         {"hold_max_error", hold_max}, {"disturbance", disturbance},
                         {"drive_during_motion", drive_during_motion},
                         {"max_path_error_rad", tracker.max_error},
                         {"max_reference_velocity_radps", tracker.max_velocity},
                         {"position_jumps", tracker.jumps}, {"max_position_jump_rad", tracker.max_jump},
                         {"controller_state_samples", tracker.samples},
                         {"cpu_load_threads", static_cast<int>(cpu_threads)}};
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
