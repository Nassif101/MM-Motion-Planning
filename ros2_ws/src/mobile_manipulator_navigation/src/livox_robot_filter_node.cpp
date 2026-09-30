// Filter /livox/lidar for navigation consumers (local-costmap perception contract).
//
// Drops UnitySensors zero-point misses and robot self-returns, then republishes the
// remaining points unchanged in livox_frame on /livox/points_filtered so costmaps keep the
// sensor origin for ray tracing. Self-returns are points inside the URDF collision
// primitives or the attached reference panel, posed from the current TF: either inside a
// primitive enlarged by `margin`, or on a ray that first hits the robot and no more than
// `noise_band` (4 sigma of the lidar range noise) in front of that surface. Obstacles
// between the sensor and the robot or off its rays stay visible, even inside the
// footprint rectangle. Ground and overhead returns are kept for the consumers' own height
// handling.
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "mobile_manipulator_navigation/lidar_robot_filter.hpp"

namespace mmn = mobile_manipulator_navigation;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("Cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

Eigen::Vector3d vector3(const YAML::Node & node)
{
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}
}  // namespace

class LivoxRobotFilter : public rclcpp::Node
{
public:
  LivoxRobotFilter()
  : Node("livox_robot_filter"),
    margin_(declare_parameter("margin", 0.03)),
    // UnitySensors Mid-360 range noise is Gaussian with sigma 0.02 m; 4 sigma.
    noise_band_(declare_parameter("noise_band", 0.08)),
    base_frame_(declare_parameter("base_frame", std::string("base_footprint"))),
    buffer_(get_clock()), listener_(buffer_)
  {
    const auto description = ament_index_cpp::get_package_share_directory("mobile_manipulator_description");
    const auto control = ament_index_cpp::get_package_share_directory("mobile_manipulator_control");
    // qualified_payload.json is JSON, which yaml-cpp reads as YAML.
    const auto payload = YAML::LoadFile(control + "/config/qualified_payload.json")["payload"];
    primitives_ = mmn::load_primitives(
      read_file(description + "/urdf/mobile_manipulator.urdf"),
      mmn::Payload{"tool0", vector3(payload["dimensions_tool_ros_m"]), vector3(payload["com_tool_ros_m"])});
    std::set<std::string> links;
    for (const auto & primitive : primitives_) links.insert(primitive.link);
    links_.assign(links.begin(), links.end());

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/livox/points_filtered", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(*cloud); });
    RCLCPP_INFO(get_logger(), "Self-model: %zu primitives on %zu links incl. payload panel, "
                "margin %.3f m, noise band %.3f m", primitives_.size(), links_.size(), margin_, noise_band_);
  }

private:
  // Latest sensor and link poses in the base frame; false if any transform is missing.
  bool posed_primitives(const std::string & sensor_frame, Eigen::Isometry3d & sensor,
                        std::vector<mmn::PosedPrimitive> & posed)
  {
    try {
      sensor = tf2::transformToEigen(buffer_.lookupTransform(base_frame_, sensor_frame, tf2::TimePointZero));
      std::map<std::string, Eigen::Isometry3d> links;
      for (const auto & link : links_) {
        links[link] = tf2::transformToEigen(buffer_.lookupTransform(base_frame_, link, tf2::TimePointZero));
      }
      posed.clear();
      for (const auto & primitive : primitives_) {
        posed.push_back(mmn::posed_primitive(links[primitive.link] * primitive.pose, primitive.shape, primitive.dims));
      }
    } catch (const tf2::TransformException &) {
      return false;
    }
    return true;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto started = std::chrono::steady_clock::now();
    Eigen::Isometry3d sensor;
    std::vector<mmn::PosedPrimitive> posed;
    if (!posed_primitives(cloud.header.frame_id, sensor, posed)) {
      ++skipped_;
      return;
    }
    int offset[3] = {-1, -1, -1};
    for (const auto & field : cloud.fields) {
      if (field.name == "x") offset[0] = static_cast<int>(field.offset);
      if (field.name == "y") offset[1] = static_cast<int>(field.offset);
      if (field.name == "z") offset[2] = static_cast<int>(field.offset);
    }
    if (std::min({offset[0], offset[1], offset[2]}) < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Cloud without x/y/z fields");
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = cloud.header;
    out.height = 1;
    out.fields = cloud.fields;
    out.is_bigendian = cloud.is_bigendian;
    out.point_step = cloud.point_step;
    out.is_dense = true;
    out.data.reserve(cloud.data.size());
    const size_t points = cloud.data.size() / cloud.point_step;
    for (size_t i = 0; i < points; ++i) {
      const uint8_t * point = cloud.data.data() + i * cloud.point_step;
      float xyz[3];
      for (int axis = 0; axis < 3; ++axis) std::memcpy(&xyz[axis], point + offset[axis], sizeof(float));
      if (mmn::keep_point(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]), sensor, posed, margin_, noise_band_)) {
        out.data.insert(out.data.end(), point, point + cloud.point_step);
      }
    }
    out.width = static_cast<uint32_t>(out.data.size() / cloud.point_step);
    out.row_step = out.width * out.point_step;
    const double kept = static_cast<double>(out.width) / std::max<size_t>(1, points);
    publisher_->publish(out);

    durations_.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    kept_.push_back(kept);
    if (durations_.size() == 100) {
      std::sort(durations_.begin(), durations_.end());
      RCLCPP_INFO(get_logger(), "100 scans: processing p50 %.2f ms, p99 %.2f ms, max %.2f ms; "
                  "kept %.1f %% of points; skipped %zu scans without TF",
                  durations_[49] * 1e3, durations_[98] * 1e3, durations_.back() * 1e3,
                  100.0 * std::accumulate(kept_.begin(), kept_.end(), 0.0) / kept_.size(), skipped_);
      durations_.clear();
      kept_.clear();
    }
  }

  const double margin_, noise_band_;
  const std::string base_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::vector<mmn::Primitive> primitives_;
  std::vector<std::string> links_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::vector<double> durations_, kept_;
  size_t skipped_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxRobotFilter>());
  rclcpp::shutdown();
  return 0;
}
