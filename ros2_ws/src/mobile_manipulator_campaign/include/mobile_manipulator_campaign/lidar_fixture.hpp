#pragma once
// The plant's lidar at a fixed pose for gate L (MuJoCo plant plan Task 10): a Session reset to `scenario` with the robot
// at `pose` (no start perturbation) in the scenario's arm pose, held still while `scans` scans complete. The header
// carries what tools/plant/compare_lidar.py needs to label Unity's points at the same pose: the base and the lidar
// poses (4 x 4, row-major), the arm joints, the robot's contacts, the base's drift over the hold and every site,
// scenario and robot primitive in the world frame ({name, kind box|cylinder|sphere, label site|scenario|robot|panel,
// pos, rot 3 x 3, size as MuJoCo's geom_size}).
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/session.hpp"
#include "mobile_manipulator_plant/lidar.hpp"

namespace mobile_manipulator_campaign
{
struct LidarFixtureResult
{
  nlohmann::json header;
  std::vector<mobile_manipulator_plant::PlantScan> scans;
};

LidarFixtureResult lidar_fixture(SessionConfig config, const std::string & scenario, const mobile_manipulator_plant::Pose2 & pose,
                                 int scans, uint64_t seed);

// A run log: the header, then one Scan frame per scan (the stream's layout: points in the lidar frame; classes).
void write_lidar_fixture(const LidarFixtureResult & result, const std::string & path);
}  // namespace mobile_manipulator_campaign
