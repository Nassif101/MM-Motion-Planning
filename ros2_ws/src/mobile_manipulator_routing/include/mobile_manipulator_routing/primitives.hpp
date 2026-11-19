#pragma once
#include <filesystem>
#include <vector>
#include "mobile_manipulator_routing/types.hpp"
namespace mobile_manipulator_routing
{
struct MotionPrimitive
{
  std::size_t id = 0, start_heading = 0, end_heading = 0;
  int dx = 0, dy = 0;
  double length_m = 0.0, angular_travel = 0.0;
  bool pure_rotation = false, turning = false;
  std::vector<Pose2> poses;  // relative XY, absolute map-heading angles; explicit origin first
};
class PrimitiveSet
{
public:
  static PrimitiveSet load(const std::filesystem::path & path, double required_resolution = 0.05);
  double resolution = 0.05, turning_radius = 0.5;
  std::vector<double> headings;
  std::vector<MotionPrimitive> motions;
  std::vector<std::vector<std::size_t>> successors;  // indices into motions; sorted by trajectory ID
  std::size_t nearest_heading(double yaw) const;
  std::size_t storage_bytes() const;
};
}  // namespace mobile_manipulator_routing
