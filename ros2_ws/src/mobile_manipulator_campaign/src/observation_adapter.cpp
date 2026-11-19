#include "mobile_manipulator_campaign/observation_adapter.hpp"

#include <utility>

namespace mobile_manipulator_campaign
{
namespace mr = mobile_manipulator_routing;

AdaptedScan adapt_scan(const mobile_manipulator_plant::PlantScan & scan, double t_received)
{
  AdaptedScan out;
  auto & o = out.observation;
  o.scan = scan.scan;
  o.t_start = scan.t_start;
  o.t_end = scan.t_end;
  o.t_received = t_received;
  o.rays = scan.rays;
  for (const auto & b : scan.batches) {
    if (!b.robot.known) {
      ++out.dropped_batches;
      out.dropped_points += static_cast<std::size_t>(b.points);
      continue;
    }
    mr::ObservationBatch batch;
    batch.t = b.t;
    batch.first_point = o.points.size();
    batch.points = static_cast<std::size_t>(b.points);
    batch.sensor = {b.sensor_position.x(), b.sensor_position.y(), b.sensor_position.z(), b.sensor_rotation.w(),
                    b.sensor_rotation.x(), b.sensor_rotation.y(), b.sensor_rotation.z()};
    batch.robot.base = {b.robot.x, b.robot.y, b.robot.yaw};
    for (int i = 0; i < 6; ++i) batch.robot.q[i] = b.robot.q(i);
    for (int i = b.first_point; i < b.first_point + b.points; ++i) {
      const auto & p = scan.points[i];
      o.points.push_back({p.x(), p.y(), p.z()});
      out.labels.geom.push_back(scan.geom[i]);
      out.labels.cls.push_back(scan.cls[i]);
    }
    o.batches.push_back(batch);
  }
  mr::validate(o);
  return out;
}

bool ObservationQueue::push(AdaptedScan scan)
{
  counts_.missing_state_batches += scan.dropped_batches;
  counts_.missing_state_points += scan.dropped_points;
  auto & o = scan.observation;
  if (any_ && o.t_end <= last_t_end_) {  // the defined out-of-order policy: refuse, never reorder or merge
    ++counts_.out_of_order;
    return false;
  }
  any_ = true;
  last_t_end_ = o.t_end;
  ++counts_.scans;
  counts_.points += o.points.size();
  pending_.push_back(std::move(o));
  while (pending_.size() > capacity_) {
    pending_.pop_front();
    ++counts_.overflow;
  }
  return true;
}

std::vector<mr::LidarObservation> ObservationQueue::take()
{
  std::vector<mr::LidarObservation> out(std::make_move_iterator(pending_.begin()), std::make_move_iterator(pending_.end()));
  pending_.clear();
  return out;
}

void ObservationQueue::reset()
{
  pending_.clear();
  counts_ = ObservationCounts{};
  last_t_end_ = -1.0;
  any_ = false;
}
}  // namespace mobile_manipulator_campaign
