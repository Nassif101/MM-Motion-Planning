#include "mobile_manipulator_routing/observation.hpp"

#include <cmath>
#include <stdexcept>

namespace mobile_manipulator_routing
{
std::array<double, 3> to_map(const ObservationBatch & batch, const std::array<float, 3> & point)
{
  const auto & s = batch.sensor;
  const double px = point[0], py = point[1], pz = point[2];
  // v' = q v q* for a unit quaternion
  const double tx = 2.0 * (s.qy * pz - s.qz * py), ty = 2.0 * (s.qz * px - s.qx * pz), tz = 2.0 * (s.qx * py - s.qy * px);
  return {s.x + px + s.qw * tx + (s.qy * tz - s.qz * ty), s.y + py + s.qw * ty + (s.qz * tx - s.qx * tz),
          s.z + pz + s.qw * tz + (s.qx * ty - s.qy * tx)};
}

void validate(const LidarObservation & o)
{
  if (!std::isfinite(o.t_start) || !std::isfinite(o.t_end) || !std::isfinite(o.t_received) || o.t_end < o.t_start ||
      o.rays < 0)
    throw std::invalid_argument("observation: times must be finite and ordered");
  std::size_t next = 0;
  double last = -INFINITY;
  for (const auto & b : o.batches) {
    if (b.first_point != next) throw std::invalid_argument("observation: batches must partition the points in order");
    next += b.points;
    if (!std::isfinite(b.t) || b.t < last || b.t < o.t_start - 1e-9 || b.t > o.t_end + 1e-9)
      throw std::invalid_argument("observation: batch times must be finite, ordered and inside the scan");
    last = b.t;
    const auto & s = b.sensor;
    const double n = s.qw * s.qw + s.qx * s.qx + s.qy * s.qy + s.qz * s.qz;
    if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.z) || !std::isfinite(n) ||
        std::abs(n - 1.0) > 1e-6 || !finite(b.robot.base))
      throw std::invalid_argument("observation: a batch's sensor pose or base pose is not finite or not unit");
    for (const double q : b.robot.q)
      if (!std::isfinite(q)) throw std::invalid_argument("observation: a batch's joints are not finite");
  }
  if (next != o.points.size()) throw std::invalid_argument("observation: batches must cover every point");
  for (const auto & p : o.points)
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
      throw std::invalid_argument("observation: a point is not finite");
}
}  // namespace mobile_manipulator_routing
