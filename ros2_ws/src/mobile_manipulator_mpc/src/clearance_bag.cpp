#include "mobile_manipulator_mpc/clearance_bag.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>

#include <Eigen/Geometry>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace mobile_manipulator_mpc
{
namespace
{
constexpr int kMaxChain = 16;

struct Edge
{
  std::string parent;
  Eigen::Isometry3d transform;  // parent -> child
};

Eigen::Isometry3d isometry(const geometry_msgs::msg::Transform & t)
{
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  if (q.norm() < 1e-9) q = Eigen::Quaterniond::Identity();
  q.normalize();
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return out;
}

// map -> frame through the latest edges, or nothing while the chain is incomplete.
std::optional<Eigen::Isometry3d> in_map(const std::map<std::string, Edge> & edges, const std::string & frame)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  std::string current = frame;
  for (int depth = 0; depth < kMaxChain; ++depth) {
    if (current == "map") return pose;
    const auto edge = edges.find(current);
    if (edge == edges.end()) return std::nullopt;
    pose = edge->second.transform * pose;
    current = edge->second.parent;
  }
  return std::nullopt;
}
}  // namespace

std::vector<ClearanceSample> read_clearance_samples(const std::string & bag_uri,
                                                    const std::array<std::string, 6> & joint_names, double period_s)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;
  rclcpp::Serialization<sensor_msgs::msg::JointState> joint_serialization;

  std::map<std::string, Edge> edges;
  std::array<double, 6> q{};
  std::array<bool, 6> known{};
  std::vector<ClearanceSample> samples;
  double last = -std::numeric_limits<double>::infinity();
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const rclcpp::SerializedMessage serialized(*message->serialized_data);
    if (message->topic_name == "/tf" || message->topic_name == "/tf_static") {
      tf2_msgs::msg::TFMessage tf;
      tf_serialization.deserialize_message(&serialized, &tf);
      for (const auto & t : tf.transforms) {
        std::string child = t.child_frame_id;
        if (!child.empty() && child.front() == '/') child.erase(0, 1);
        std::string parent = t.header.frame_id;
        if (!parent.empty() && parent.front() == '/') parent.erase(0, 1);
        edges[child] = {parent, isometry(t.transform)};
      }
    } else if (message->topic_name == "/joint_states") {
      sensor_msgs::msg::JointState joints;
      joint_serialization.deserialize_message(&serialized, &joints);
      for (size_t i = 0; i < joints.name.size() && i < joints.position.size(); ++i) {
        if (!std::isfinite(joints.position[i])) continue;
        for (size_t j = 0; j < joint_names.size(); ++j) {
          if (joints.name[i] != joint_names[j]) continue;
          q[j] = joints.position[i];
          known[j] = true;
        }
      }
      const double stamp = joints.header.stamp.sec + joints.header.stamp.nanosec * 1e-9;
      if (stamp - last < period_s) continue;
      if (std::find(known.begin(), known.end(), false) != known.end()) continue;
      const auto base = in_map(edges, "base_footprint");
      if (!base) continue;
      State x;
      const Eigen::Vector3d heading = base->linear().col(0);
      x << base->translation().x(), base->translation().y(), std::atan2(heading.y(), heading.x()), q[0], q[1],
        q[2], q[3], q[4], q[5];
      samples.push_back({stamp, x});
      last = stamp;
    }
  }
  return samples;
}
}  // namespace mobile_manipulator_mpc
