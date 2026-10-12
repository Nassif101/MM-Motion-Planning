#pragma once
// Robot states from a rosbag2 recording for clearance_3d: /tf and /tf_static give the map ->
// base_footprint pose (chained through every parent, e.g. map -> odom -> base_footprint, with
// the latest value of each edge), /joint_states the six arm joints by name.
#include <array>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/clearance.hpp"

namespace mobile_manipulator_mpc
{
// One sample per joint-state message at least `period_s` (header time) after the previous
// sample, once all six joints and the transform chain are known. `joint_names` in the state's
// joint order (qualified_payload.json joint_order). Throws std::runtime_error if the bag cannot be
// opened.
std::vector<ClearanceSample> read_clearance_samples(const std::string & bag_uri,
                                                    const std::array<std::string, 6> & joint_names,
                                                    double period_s);
}  // namespace mobile_manipulator_mpc
