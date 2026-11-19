// Spec 2 Task 10: the plain lidar observation contract.
#include <gtest/gtest.h>

#include <cmath>

#include "mobile_manipulator_routing/observation.hpp"

namespace mr = mobile_manipulator_routing;

namespace
{
mr::ObservationBatch batch(double t, std::size_t first, std::size_t points, double yaw)
{
  mr::ObservationBatch b;
  b.t = t;
  b.first_point = first;
  b.points = points;
  b.sensor = {1.0, 2.0, 0.5, std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)};
  b.robot.base = {1.0, 2.0, yaw};
  return b;
}
}  // namespace

TEST(Observation, PointsGoToTheMapThroughTheirBatchsSensorPose)
{
  const auto b = batch(0.1, 0, 1, M_PI / 2);  // facing +y
  const auto p = mr::to_map(b, {2.0f, 0.0f, 0.25f});  // 2 m ahead, 0.25 m up in the sensor frame
  EXPECT_NEAR(p[0], 1.0, 1e-9);
  EXPECT_NEAR(p[1], 4.0, 1e-9);
  EXPECT_NEAR(p[2], 0.75, 1e-9);
  const auto o = mr::sensor_origin(b);
  EXPECT_EQ(o[0], 1.0);
}

TEST(Observation, ValidationRequiresOrderedBatchesThatPartitionThePoints)
{
  mr::LidarObservation o;
  o.t_start = 0.0;
  o.t_end = 0.1;
  o.points = {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  o.batches = {batch(0.05, 0, 2, 0.0), batch(0.1, 2, 1, 0.0)};
  EXPECT_NO_THROW(mr::validate(o));
  auto gap = o;
  gap.batches[1].first_point = 3;  // a point outside every batch
  EXPECT_THROW(mr::validate(gap), std::invalid_argument);
  auto order = o;
  order.batches[1].t = 0.01;  // out of time order
  EXPECT_THROW(mr::validate(order), std::invalid_argument);
  auto outside = o;
  outside.batches[1].t = 0.2;  // after the scan
  EXPECT_THROW(mr::validate(outside), std::invalid_argument);
  auto pose = o;
  pose.batches[0].sensor.qw = 2.0;  // not a unit quaternion
  EXPECT_THROW(mr::validate(pose), std::invalid_argument);
  auto point = o;
  point.points[0][2] = NAN;
  EXPECT_THROW(mr::validate(point), std::invalid_argument);
}
