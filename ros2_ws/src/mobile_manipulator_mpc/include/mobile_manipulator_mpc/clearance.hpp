#pragma once
// Minimum 3D clearance of a recorded run: the robot's collision primitives (the MPC's
// CollisionModel, without margin) against the known boxes over a series of robot states. Scores
// B1-B5 alike; the 2D footprint clearance of the navigate task cannot see an overhead beam or the
// arm and panel.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/collision.hpp"

namespace mobile_manipulator_mpc
{
struct ClearanceSample
{
  double time_s;  // simulation time of the state
  State state;    // base_footprint pose in map and the six arm joints
};

struct Clearance
{
  double distance_m = std::numeric_limits<double>::infinity();  // signed; infinity without a pair
  std::string link, obstacle;                                     // at the minimum
  double time_s = std::numeric_limits<double>::quiet_NaN();       // sample time of the minimum
  size_t samples = 0;                                             // states evaluated
};

Clearance minimum_clearance(const CollisionModel & model, const std::vector<ClearanceSample> & samples,
                            const std::vector<ObstacleBox> & boxes);
}  // namespace mobile_manipulator_mpc
