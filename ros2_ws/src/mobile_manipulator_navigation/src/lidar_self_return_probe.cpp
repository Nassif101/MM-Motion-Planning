// Classify one /livox/lidar scan relative to the robot for the local-costmap contract.
//
// Read-only. Separates misses (encoded by UnitySensors as zero points), transforms the
// returns into base_footprint with TF, and reports robot self-returns (inside the active
// footprint polygon and above the ground), ground returns, and the nearest ground /
// obstacle ranges. Run with the robot stationary in an open area.
//
// Usage: lidar_self_return_probe [--profile home] [--ground-band 0.05] [--output FILE]
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace fs = std::filesystem;
namespace mmn = mobile_manipulator_navigation;
using mmn::Json;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const std::string profile = args.get("profile", "home");
  const double ground_band = args.number("ground-band", 0.05);

  const auto share = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const auto polygon = mmn::load_yaml_file(share + "/config/footprint_profiles.yaml").at("profiles").at(profile).at("polygon");
  double x_min = std::numeric_limits<double>::infinity(), x_max = -x_min, y_min = x_min, y_max = -x_min;
  for (const auto & p : polygon) {
    x_min = std::min(x_min, p[0].get<double>());
    x_max = std::max(x_max, p[0].get<double>());
    y_min = std::min(y_min, p[1].get<double>());
    y_max = std::max(y_max, p[1].get<double>());
  }

  auto node = std::make_shared<rclcpp::Node>("lidar_self_return_probe");
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer, node);
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", rclcpp::SensorDataQoS(), [&clouds](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { clouds.push_back(m); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (clouds.size() < 3) executor.spin_once(std::chrono::milliseconds(100));
  const auto cloud = clouds.back();
  while (!buffer._frameExists(cloud->header.frame_id) ||
         !buffer.canTransform("base_footprint", cloud->header.frame_id, tf2::TimePointZero)) {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  const auto transform = buffer.lookupTransform("base_footprint", cloud->header.frame_id, tf2::TimePointZero);
  rclcpp::shutdown();
  const Eigen::Isometry3d sensor_to_base = tf2::transformToEigen(transform);

  long total = 0, misses = 0, self_returns = 0, ground = 0;
  double self_low = std::numeric_limits<double>::infinity(), self_high = -self_low;
  std::optional<double> nearest_ground, nearest_other;
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    const Eigen::Vector3d raw(*x, *y, *z);
    if (!raw.allFinite()) continue;  // skip_nans
    ++total;
    // UnitySensors encodes misses and out-of-range returns as (0, 0, 0) in the sensor frame.
    if (raw.norm() == 0.0) {
      ++misses;
      continue;
    }
    const Eigen::Vector3d p = sensor_to_base * raw;
    const double planar = std::hypot(p.x(), p.y());
    const bool is_ground = p.z() < ground_band;
    const bool inside = p.x() >= x_min && p.x() <= x_max && p.y() >= y_min && p.y() <= y_max;
    if (is_ground) {
      ++ground;
      if (!nearest_ground || planar < *nearest_ground) nearest_ground = planar;
    } else if (inside) {
      ++self_returns;
      self_low = std::min(self_low, p.z());
      self_high = std::max(self_high, p.z());
    } else if (!nearest_other || planar < *nearest_other) {
      nearest_other = planar;
    }
  }

  const auto & t = transform.transform.translation;
  const Json report = {
    {"profile", profile},
    {"frame_id", cloud->header.frame_id},
    {"points", total},
    {"misses_encoded_at_sensor_origin", misses},
    {"self_returns_inside_footprint", self_returns},
    {"self_return_height_range_m",
     self_returns ? Json::array({mmn::round_digits(self_low, 3), mmn::round_digits(self_high, 3)}) : Json(nullptr)},
    {"ground_returns", ground},
    {"nearest_ground_return_m", nearest_ground ? Json(mmn::round_digits(*nearest_ground, 3)) : Json(nullptr)},
    {"nearest_non_robot_return_m", nearest_other ? Json(mmn::round_digits(*nearest_other, 3)) : Json(nullptr)},
    {"sensor_height_m", mmn::round_digits(t.z, 3)},
    {"predicted_ground_blind_radius_m", mmn::round_digits(t.z / std::tan(7.212303 * M_PI / 180.0) + t.x, 3)},
  };
  std::cout << report.dump(2) << std::endl;
  if (args.has("output")) {
    const fs::path output = args.get("output");
    if (output.has_parent_path()) fs::create_directories(output.parent_path());
    std::ofstream(output) << report.dump(2) << "\n";
  }
  return 0;
}
