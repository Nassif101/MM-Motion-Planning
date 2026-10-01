// Bounded /cmd_vel step, brake, watchdog, and breakaway tests for the Unity skid-steer base.
//
// Simulation only. Requires Unity in Play with the base stopped in a surveyed open area
// (for example after `unity command arm_test_place_open`), no other /cmd_vel publisher, and
// fresh ground-truth TF. Every case drives out and back so the robot ends near its start.
// Ground truth comes from Unity's odom -> base_footprint TF and the wheel joint velocities
// on /joint_states; all timing uses simulation time. With --breakaway-speeds /
// --breakaway-yaw-rates it runs only small steps from rest, to find the smallest command
// that starts the base moving.
//
// Usage: base_step_test --output-dir DIR --prefix NAME [--speeds V...] [--yaw-rate W]
//                       [--breakaway-speeds V...] [--breakaway-yaw-rates W...]
//                       [--breakaway-seconds S]
// Writes <prefix>-<case>.csv per case and <prefix>-summary.json.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mobile_manipulator_control/cli.hpp"

namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;

namespace
{
constexpr double kWheelRadius = 0.14;
constexpr double kCommandPeriod = 0.05;
const std::array<std::string, 4> kWheels = {"front_left_wheel_joint", "rear_left_wheel_joint",
                                            "front_right_wheel_joint", "rear_right_wheel_joint"};

struct Segment
{
  double v, w, seconds;
  bool publish;
};
using Case = std::pair<std::string, std::vector<Segment>>;

// stamp, x, y, yaw, roll, pitch, 4 wheel velocities, cmd v, cmd w, publishing
using Sample = std::array<double, 13>;

struct Row
{
  double t, x, y, yaw, roll, pitch, v, lateral, w, rim_left, rim_right, cmd_v, cmd_w;
  bool publishing;
  std::optional<double> a, alpha;
};

std::string fixed(double value, int digits)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.*f", digits, value);
  return text;
}

std::vector<Case> step_cases(double speed, double yaw_rate)
{
  const double drive = std::max(4.0, 1.5 / speed);
  const auto s = fixed(speed, 2);
  return {
    {"forward-brake-" + s, {{speed, 0, drive, true}, {0, 0, 3, true}, {-speed, 0, drive, true}, {0, 0, 3, true}}},
    {"reverse-brake-" + s, {{-speed, 0, drive, true}, {0, 0, 3, true}, {speed, 0, drive, true}, {0, 0, 3, true}}},
    {"watchdog-stop-" + s, {{speed, 0, drive, true}, {0, 0, 3, false}, {-speed, 0, drive, true}, {0, 0, 3, false}}},
    {"yaw-brake-" + fixed(yaw_rate, 2),
     {{0, yaw_rate, 4, true}, {0, 0, 3, true}, {0, -yaw_rate, 4, true}, {0, 0, 3, true}}},
  };
}

// Small steps from rest, out and back: does the base start moving, and how fast?
std::vector<Case> breakaway_cases(const std::vector<double> & speeds, const std::vector<double> & yaw_rates,
                                  double seconds)
{
  std::vector<Case> out;
  for (const double v : speeds) {
    out.push_back({"breakaway-v-" + fixed(v, 4), {{v, 0, seconds, true}, {0, 0, 2, true}, {-v, 0, seconds, true}, {0, 0, 2, true}}});
  }
  for (const double w : yaw_rates) {
    out.push_back({"breakaway-w-" + fixed(w, 4), {{0, w, seconds, true}, {0, 0, 2, true}, {0, -w, seconds, true}, {0, 0, 2, true}}});
  }
  return out;
}

class Recorder : public rclcpp::Node
{
public:
  Recorder() : Node("base_step_test", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}))
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 50, [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) { on_tf(*m); });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { on_joints(*m); });
    executor_.add_node(get_node_base_interface());
  }

  double now_s() { return get_clock()->now().seconds(); }
  void spin_once(double seconds) { executor_.spin_once(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds))); }

  void spin_for(double seconds, double v, double w, bool publish)
  {
    command_ = {v, w, publish ? 1.0 : 0.0};
    const double end = now_s() + seconds;
    double next_publish = now_s();
    while (now_s() < end) {
      if (publish && now_s() >= next_publish) {
        geometry_msgs::msg::Twist message;
        message.linear.x = v;
        message.angular.z = w;
        publisher_->publish(message);
        next_publish += kCommandPeriod;
      }
      spin_once(0.01);
    }
  }

  std::vector<Sample> samples;

private:
  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    std::map<std::string, size_t> index;
    for (size_t i = 0; i < msg.name.size(); ++i) index[msg.name[i]] = i;
    std::array<double, 4> wheels{};
    for (size_t k = 0; k < kWheels.size(); ++k) {
      const auto it = index.find(kWheels[k]);
      if (it == index.end() || it->second >= msg.velocity.size()) return;
      wheels[k] = msg.velocity[it->second];
    }
    wheels_ = wheels;
  }

  void on_tf(const tf2_msgs::msg::TFMessage & msg)
  {
    for (const auto & t : msg.transforms) {
      if (t.child_frame_id != "base_footprint" || !wheels_) continue;
      const auto & q = t.transform.rotation;
      const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
      const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
      const auto & w = *wheels_;
      samples.push_back({t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, t.transform.translation.x,
                         t.transform.translation.y, yaw, roll, pitch, w[0], w[1], w[2], w[3], command_[0],
                         command_[1], command_[2]});
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::optional<std::array<double, 4>> wheels_;
  std::array<double, 3> command_{0.0, 0.0, 0.0};
};

// Body-frame velocities and accelerations from 50 Hz ground truth (centred differences).
std::vector<Row> derive(const std::vector<Sample> & samples)
{
  std::vector<Row> rows;
  for (size_t i = 2; i + 2 < samples.size(); ++i) {
    const auto & s0 = samples[i - 2];
    const auto & s1 = samples[i + 2];
    const double dt = s1[0] - s0[0];
    if (dt <= 0) continue;
    const double heading = samples[i][3];
    const double vx = (s1[1] - s0[1]) / dt, vy = (s1[2] - s0[2]) / dt;
    Row r;
    r.t = samples[i][0];
    r.x = samples[i][1];
    r.y = samples[i][2];
    r.yaw = heading;
    r.roll = samples[i][4];
    r.pitch = samples[i][5];
    r.v = std::cos(heading) * vx + std::sin(heading) * vy;
    r.lateral = -std::sin(heading) * vx + std::cos(heading) * vy;
    r.w = std::atan2(std::sin(s1[3] - s0[3]), std::cos(s1[3] - s0[3])) / dt;
    r.rim_left = kWheelRadius * (samples[i][6] + samples[i][7]) / 2;
    r.rim_right = kWheelRadius * (samples[i][8] + samples[i][9]) / 2;
    r.cmd_v = samples[i][10];
    r.cmd_w = samples[i][11];
    r.publishing = samples[i][12] != 0.0;
    rows.push_back(r);
  }
  for (size_t i = 2; i + 2 < rows.size(); ++i) {
    const double dt = rows[i + 2].t - rows[i - 2].t;
    rows[i].a = dt > 0 ? (rows[i + 2].v - rows[i - 2].v) / dt : 0.0;
    rows[i].alpha = dt > 0 ? (rows[i + 2].w - rows[i - 2].w) / dt : 0.0;
  }
  if (rows.size() < 4) return {};
  return {rows.begin() + 2, rows.end() - 2};
}

// Per-segment steady state, acceleration, stopping distance and slip.
Json summarize(const std::vector<Row> & rows, const std::vector<Segment> & segments)
{
  Json result = Json::array();
  if (rows.empty()) return result;
  double elapsed = 0.0;
  const double start = rows.front().t;
  for (const auto & s : segments) {
    const double begin = start + elapsed, end = begin + s.seconds;
    elapsed += s.seconds;
    std::vector<const Row *> part;
    for (const auto & r : rows) {
      if (begin <= r.t && r.t < end) part.push_back(&r);
    }
    if (part.size() < 10) continue;
    const bool linear = std::abs(s.v) > 0 || (s.v == 0 && s.w == 0 && std::abs(part.front()->v) > 0.02);
    const auto key = [linear](const Row * r) { return linear ? r->v : r->w; };
    const auto accel = [linear](const Row * r) { return (linear ? r->a : r->alpha).value_or(0.0); };
    double peak = 0.0, lateral = 0.0, tilt = 0.0;
    for (const auto * r : part) {
      peak = std::max(peak, std::abs(accel(r)));
      lateral = std::max(lateral, std::abs(r->lateral));
      tilt = std::max({tilt, std::abs(r->roll), std::abs(r->pitch)});
    }
    Json entry = {{"command", {s.v, s.w}}, {"published", s.publish}, {"duration_s", end - begin},
                  {"peak_abs_accel", peak}, {"max_abs_lateral_mps", lateral},
                  {"max_tilt_deg", tilt * 180.0 / M_PI}};
    if (s.v != 0 || s.w != 0) {
      double sum = 0.0;
      int count = 0;
      for (const auto * r : part) {
        if (r->t > end - 1.5) {
          sum += key(r);
          ++count;
        }
      }
      const double target = sum / count;
      entry["steady_state"] = target;
      const auto first = [&](double fraction) -> std::optional<double> {
        for (const auto * r : part) {
          if (std::abs(key(r)) >= fraction * std::abs(target)) return r->t;
        }
        return std::nullopt;
      };
      const auto lo = first(0.1), hi = first(0.9);
      if (lo && hi && *hi > *lo) entry["mean_accel_10_90"] = 0.8 * std::abs(target) / (*hi - *lo);
    } else {
      const double initial = key(part.front());
      entry["initial"] = initial;
      const auto stop = std::find_if(part.begin(), part.end(), [&](const Row * r) { return std::abs(key(r)) < 0.01; });
      if (stop != part.end()) {
        const double time_to_stop = (*stop)->t - begin;
        entry["time_to_stop_s"] = time_to_stop;
        if (linear) {
          entry["stopping_distance_m"] = std::hypot((*stop)->x - part.front()->x, (*stop)->y - part.front()->y);
        }
        if (time_to_stop > 0) entry["mean_decel"] = std::abs(initial) / time_to_stop;
      }
      if (linear) {
        // Positive slip: wheel rims turn faster (same sign) than the chassis moves.
        double slip = 0.0;
        for (const auto * r : part) {
          if (std::abs(r->v) > 0.02) slip = std::max(slip, std::abs((r->rim_left + r->rim_right) / 2 - r->v));
        }
        entry["max_abs_rim_minus_body_mps"] = slip;
      }
    }
    result.push_back(entry);
  }
  return result;
}

// Shortest round-trip decimal, as Python writes floats to CSV.
std::string number(double value) { return Json(value).dump(); }

void write_csv(const fs::path & path, const std::vector<Row> & rows)
{
  std::ofstream out(path);
  out << "t,x,y,yaw,roll,pitch,v,lateral,w,rim_left,rim_right,cmd_v,cmd_w,publishing,a,alpha\n";
  for (const auto & r : rows) {
    for (const double value : {r.t, r.x, r.y, r.yaw, r.roll, r.pitch, r.v, r.lateral, r.w, r.rim_left, r.rim_right,
                               r.cmd_v, r.cmd_w}) {
      out << number(value) << ",";
    }
    out << (r.publishing ? "True" : "False") << "," << number(r.a.value_or(0.0)) << ","
        << number(r.alpha.value_or(0.0)) << "\n";
  }
}

int refuse(const std::string & message)
{
  std::cerr << message << std::endl;
  rclcpp::shutdown();
  return 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const std::vector<double> speeds = args.has("speeds") ? args.numbers("speeds", 0) : std::vector<double>{0.3};
  const double yaw_rate = args.number("yaw-rate", 0.4);
  const std::vector<double> breakaway_speeds =
    args.has("breakaway-speeds") ? args.numbers("breakaway-speeds", 0) : std::vector<double>{};
  const std::vector<double> breakaway_yaw_rates =
    args.has("breakaway-yaw-rates") ? args.numbers("breakaway-yaw-rates", 0) : std::vector<double>{};
  const double breakaway_seconds = args.number("breakaway-seconds", 4.0);
  const fs::path output = args.get("output-dir");
  const std::string prefix = args.get("prefix");
  fs::create_directories(output);

  auto node = std::make_shared<Recorder>();
  while (node->now_s() == 0.0) node->spin_once(0.1);  // wait for the first /clock
  node->spin_for(2.0, 0, 0, false);
  const auto others = static_cast<long>(node->count_publishers("/cmd_vel")) - 1;
  if (others != 0) return refuse("Refusing to command: " + std::to_string(others) + " other /cmd_vel publisher(s)");
  if (node->samples.size() < 20) return refuse("Refusing to command: no fresh odom -> base_footprint TF");
  if (node->samples.size() >= 15) {
    const std::vector<Sample> recent_samples(node->samples.end() - 15, node->samples.end());
    for (const auto & r : derive(recent_samples)) {
      if (std::abs(r.v) > 0.02 || std::abs(r.w) > 0.02) return refuse("Refusing to command: base is not stationary");
    }
  }

  Json report = {{"wheel_radius_m", kWheelRadius}, {"command_rate_hz", 1 / kCommandPeriod}, {"cases", Json::object()}};
  std::vector<Case> selected;
  if (!breakaway_speeds.empty() || !breakaway_yaw_rates.empty()) {
    selected = breakaway_cases(breakaway_speeds, breakaway_yaw_rates, breakaway_seconds);
  } else {
    for (const double s : speeds) {
      for (const auto & c : step_cases(s, yaw_rate)) {
        if (c.first.rfind("yaw", 0) != 0) selected.push_back(c);
      }
    }
    selected.push_back(step_cases(speeds.front(), yaw_rate).back());
  }
  for (const auto & [name, segments] : selected) {
    const std::string stem = prefix + "-" + name;
    if (fs::exists(output / (stem + ".csv"))) return refuse("Refusing to overwrite evidence: " + stem + ".csv");
    node->samples.clear();
    for (const auto & s : segments) node->spin_for(s.seconds, s.v, s.w, s.publish);
    node->spin_for(1.0, 0, 0, true);
    const auto rows = derive(node->samples);
    write_csv(output / (stem + ".csv"), rows);
    report["cases"][name] = summarize(rows, segments);
    std::cout << name << " " << report["cases"][name].dump(1) << std::endl;
  }
  std::ofstream(output / (prefix + "-summary.json")) << report.dump(2);
  rclcpp::shutdown();
  return 0;
}
