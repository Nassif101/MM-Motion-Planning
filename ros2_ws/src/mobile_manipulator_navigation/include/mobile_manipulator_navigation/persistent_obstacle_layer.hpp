#pragma once
// Costmap layer that marks only persistent lidar obstacles (see persistence_grid.hpp).
//
// For the global costmap: transient obstacles such as a worker crossing the route stay in
// the local costmap only, while obstacles that stay put (a blocked gate, a box, a worker
// who stops) reach the global planner and change the route. Points of the filtered Livox
// cloud within [min_obstacle_height, max_obstacle_height] in the global frame and within
// obstacle_range of the sensor are counted per costmap cell.
#include <mutex>
#include <string>

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mobile_manipulator_navigation/persistence_grid.hpp"

namespace mobile_manipulator_navigation
{
class PersistentObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
                    double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void matchSize() override;
  bool isClearable() override { return true; }

private:
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  std::string topic_;
  double min_height_ = 0.05, max_height_ = 2.0, obstacle_range_ = 5.0, transform_tolerance_ = 0.2;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::mutex mutex_;
  PersistenceGrid grid_;
  size_t skipped_clouds_ = 0;
};
}  // namespace mobile_manipulator_navigation
