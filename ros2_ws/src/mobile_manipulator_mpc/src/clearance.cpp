#include "mobile_manipulator_mpc/clearance.hpp"

namespace mobile_manipulator_mpc
{
Clearance minimum_clearance(const CollisionModel & model, const std::vector<ClearanceSample> & samples,
                            const std::vector<ObstacleBox> & boxes)
{
  Clearance result;
  for (const auto & sample : samples) {
    ++result.samples;
    const auto pair = model.nearest(sample.state, boxes);
    if (pair && pair->distance < result.distance_m) {
      result.distance_m = pair->distance;
      result.link = pair->link;
      result.obstacle = pair->obstacle;
      result.time_s = sample.time_s;
    }
  }
  return result;
}
}  // namespace mobile_manipulator_mpc
