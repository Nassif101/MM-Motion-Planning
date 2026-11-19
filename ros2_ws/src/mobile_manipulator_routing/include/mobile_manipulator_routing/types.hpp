#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace mobile_manipulator_routing
{
constexpr double kPi = 3.14159265358979323846;
struct Point2 { double x = 0.0, y = 0.0; };
struct Pose2 { double x = 0.0, y = 0.0, yaw = 0.0; };
using Polygon = std::vector<Point2>;
inline double wrap_angle(double angle) { return std::remainder(angle, 2.0 * kPi); }
inline bool finite(const Pose2 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
inline double distance(const Point2 & a, const Point2 & b) { return std::hypot(a.x-b.x, a.y-b.y); }

struct Revisions
{
  std::uint64_t epoch = 0, goal = 0, request = 0, map = 0, footprint = 0, configuration = 0;
  bool operator==(const Revisions & other) const
  {
    return epoch == other.epoch && goal == other.goal && request == other.request &&
      map == other.map && footprint == other.footprint && configuration == other.configuration;
  }
  bool operator!=(const Revisions & other) const { return !(*this == other); }
};

struct WorkBudget
{
  // Smac iterations per call; 4,096 per 20 ms tick bounds the V2 holds near 0.9 s at ~1.3 us per iteration (2026-11-19)
  std::size_t expansions = 4096;
  std::size_t cell_tests = 524288;  // the validation pass's swept-footprint cell tests per call
};
}  // namespace mobile_manipulator_routing
