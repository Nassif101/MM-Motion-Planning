#pragma once
// Spec 2 Task 10: the lidar observation the routing core consumes (for the perception phase's self-filter and
// observed layer: plan Task 11, deferred). Plain values only: no simulator, no ROS, and no ground-truth labels (the
// plant's per-point geom and class stay in the campaign adapter's validation data and never reach filtering or
// mapping).
//
// Frames and times:
// - points are in the sensor frame (the lidar's, x forward, z up), each at its batch's acquisition time;
// - a batch's sensor pose and robot configuration are in the map frame, at that time. The base pose is the
//   simulator's ground truth (the experiments' declared localization assumption), the arm joints are the joint
//   encoders, and the rigid payload follows the arm;
// - acquisition times are the plant's simulation clock; t_received is when the session drained the scan (also
//   simulation time); the time a consumer applies an observation is its own and recorded by it.
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mobile_manipulator_routing/types.hpp"

namespace mobile_manipulator_routing
{
struct SensorPose
{
  double x = 0.0, y = 0.0, z = 0.0;            // position in the map frame
  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;  // orientation (unit quaternion), sensor to map
};

struct RobotConfiguration
{
  Pose2 base;
  std::array<double, 6> q{};  // shoulder_pan ... wrist_3
};

// One acquisition: points[first_point, first_point + points) were measured at time t from `sensor`, with the robot in
// `robot`.
struct ObservationBatch
{
  double t = 0.0;
  std::size_t first_point = 0, points = 0;
  SensorPose sensor;
  RobotConfiguration robot;
};

struct LidarObservation
{
  std::uint64_t scan = 0;
  double t_start = 0.0, t_end = 0.0;  // the scan's acquisition interval
  double t_received = 0.0;            // when it was drained
  int rays = 0;                       // rays cast (returns and no-returns)
  std::vector<ObservationBatch> batches;  // in time order; they partition `points`
  std::vector<std::array<float, 3>> points;
};

// A point of `batch` in the map frame.
std::array<double, 3> to_map(const ObservationBatch & batch, const std::array<float, 3> & point);
// The sensor's position in the map frame.
inline std::array<double, 3> sensor_origin(const ObservationBatch & batch)
{
  return {batch.sensor.x, batch.sensor.y, batch.sensor.z};
}
// std::invalid_argument when the batches do not partition the points in time order, a batch time lies outside
// [t_start, t_end], t_received precedes t_end, or a value is not finite.
void validate(const LidarObservation & observation);
}  // namespace mobile_manipulator_routing
