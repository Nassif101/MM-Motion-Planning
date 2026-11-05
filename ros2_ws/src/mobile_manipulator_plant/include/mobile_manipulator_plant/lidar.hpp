#pragma once
// The Livox Mid-360 lidar (MuJoCo plant spec Section 5): Livox's own scan pattern (mid360.csv, as UnitySensors uses it),
// a window of points_per_scan directions per scan advancing through the pattern, ray casts against everything in the
// model (self-hits included), Unity's range rules and seeded noise, a ground-truth label per point.
#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_plant/robot_profile.hpp"

namespace mobile_manipulator_plant
{
// Directions in the lidar's ROS frame (x forward, z up), unit length, the pattern's order.
struct ScanPattern
{
  std::vector<Eigen::Vector3d> directions;
};

// The binary written by make_scan_pattern: "MMPSCAN1", uint32 count, count x float32 (azimuth, zenith) in degrees as
// the CSV holds them. Direction = (sin z cos a, -sin z sin a, cos z): UnitySensors' Quaternion.Euler(z - 90, a, 0) *
// forward in Unity's left-handed frame, mapped to ROS (x = Unity z, y = -Unity x, z = Unity y). Throws std::runtime_error.
ScanPattern load_pattern(const std::string & path);

enum class HitClass : uint8_t { Site, Scenario, Mover, Ground, Robot, Panel, Other };

// One scan: its points in the lidar frame at their cast time, each with its time offset from t_start, the geom hit and
// its class. Rays without a hit or outside the range rules leave no point (Unity publishes them as zeros).
struct PlantScan
{
  uint32_t scan = 0;
  double t_start = 0.0, t_end = 0.0;
  int rays = 0;
  std::vector<Eigen::Vector3f> points;
  std::vector<float> dt;
  std::vector<int> ray;  // the ray's index within the scan (the noise index)
  std::vector<int> geom;
  std::vector<HitClass> cls;
};

// sweep: every advance casts the rays due since the last one from the sensor's current pose (a real Mid-360's motion
// distortion); instant: a whole scan at once when it is due (Unity's IidNoiseRaycastLiDARSensor).
enum class ScanMode { Sweep, Instant };

class Lidar
{
public:
  Lidar(const LidarParams & params, ScanPattern pattern, ScanMode mode);

  // The model's lidar site and every geom's class (by name prefix); a new run: scan 0, window at the pattern's start.
  void attach(const mjModel * m, uint64_t seed);
  // The rays due over (t - dt, t] at the sensor's pose in `d` (positions computed).
  void advance(const mjModel * m, mjData * d, double t, double dt);
  // A completed scan, once.
  std::optional<PlantScan> take_scan();

  ScanMode mode() const { return mode_; }

private:
  void cast(const mjModel * m, mjData * d, int first, int count, double t);
  void finish(double t);

  LidarParams params_;
  ScanPattern pattern_;
  ScanMode mode_;
  int site_ = -1;
  std::vector<HitClass> classes_;
  uint32_t seed32_ = 0;
  uint32_t scan_ = 0;
  long window_ = 0;  // the pattern index of the scan's first ray
  int cast_ = 0;     // rays cast in the current scan
  double t_start_ = 0.0;
  PlantScan current_;
  std::optional<PlantScan> done_;
  std::vector<mjtNum> vec_, dist_, normal_;
  std::vector<int> geomid_;
};
}  // namespace mobile_manipulator_plant
