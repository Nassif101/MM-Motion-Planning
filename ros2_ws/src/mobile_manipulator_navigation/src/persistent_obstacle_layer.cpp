#include "mobile_manipulator_navigation/persistent_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>

namespace mobile_manipulator_navigation
{
void PersistentObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) throw std::runtime_error("PersistentObstacleLayer: lifecycle node expired");
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/livox/points_filtered")));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.05));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("obstacle_range", rclcpp::ParameterValue(5.0));
  declareParameter("persistence", rclcpp::ParameterValue(2.0));
  declareParameter("max_gap", rclcpp::ParameterValue(1.0));
  declareParameter("decay", rclcpp::ParameterValue(10.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  PersistenceParameters parameters;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".min_obstacle_height", min_height_);
  node->get_parameter(name_ + ".max_obstacle_height", max_height_);
  node->get_parameter(name_ + ".obstacle_range", obstacle_range_);
  node->get_parameter(name_ + ".persistence", parameters.persistence_s);
  node->get_parameter(name_ + ".max_gap", parameters.max_gap_s);
  node->get_parameter(name_ + ".decay", parameters.decay_s);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  grid_ = PersistenceGrid(parameters);
  clock_ = node->get_clock();

  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();
  current_ = true;

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(cloud); }, options);
  RCLCPP_INFO(node->get_logger(),
              "%s: persistent obstacles from %s (%.2f-%.2f m, %.1f m range): confirmed after %.1f s "
              "(gaps up to %.1f s), cleared %.1f s after the last observation",
              name_.c_str(), topic_.c_str(), min_height_, max_height_, obstacle_range_, parameters.persistence_s,
              parameters.max_gap_s, parameters.decay_s);
}

void PersistentObstacleLayer::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (!enabled_) return;
  const std::string & global_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(global_frame, cloud->header.frame_id, cloud->header.stamp,
                                     tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException &) {
    ++skipped_clouds_;
    return;
  }
  const Eigen::Isometry3d sensor_to_global = tf2::transformToEigen(transform);
  const double stamp = cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9;
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
  std::lock_guard<std::mutex> lock(mutex_);
  for (; x != x.end(); ++x, ++y, ++z) {
    const Eigen::Vector3d sensor(*x, *y, *z);
    if (!sensor.allFinite() || sensor.norm() > obstacle_range_) continue;
    const Eigen::Vector3d point = sensor_to_global * sensor;
    if (point.z() < min_height_ || point.z() > max_height_) continue;
    unsigned int mx, my;
    if (!worldToMap(point.x(), point.y(), mx, my)) continue;
    grid_.observe(getIndex(mx, my), stamp);
  }
}

void PersistentObstacleLayer::updateBounds(double, double, double, double * min_x, double * min_y,
                                           double * max_x, double * max_y)
{
  if (!enabled_) return;
  const double now = clock_->now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto touch_cell = [&](unsigned index, unsigned char cost) {
    unsigned int mx, my;
    indexToCells(index, mx, my);
    setCost(mx, my, cost);
    double wx, wy;
    mapToWorld(mx, my, wx, wy);
    touch(wx, wy, min_x, min_y, max_x, max_y);
  };
  for (const unsigned index : grid_.prune(now)) touch_cell(index, nav2_costmap_2d::FREE_SPACE);
  for (const unsigned index : grid_.obstacles(now)) touch_cell(index, nav2_costmap_2d::LETHAL_OBSTACLE);
}

void PersistentObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
                                          int max_i, int max_j)
{
  if (!enabled_) return;
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void PersistentObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  grid_.clear();
  resetMaps();
  current_ = true;
}

void PersistentObstacleLayer::matchSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  CostmapLayer::matchSize();
  grid_.clear();  // indices refer to the old geometry
}
}  // namespace mobile_manipulator_navigation

PLUGINLIB_EXPORT_CLASS(mobile_manipulator_navigation::PersistentObstacleLayer, nav2_costmap_2d::Layer)
