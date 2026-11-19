#pragma once
// The Livox Mid-360 lidar (MuJoCo plant spec Section 5): Livox's own scan pattern (mid360.csv, as UnitySensors uses it),
// a window of points_per_scan directions per scan advancing through the pattern, ray casts against everything in the
// model (self-hits included), Unity's range rules and seeded noise, a ground-truth label per point.
#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_plant/robot_profile.hpp"
#include "mobile_manipulator_plant/static_bvh.hpp"

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

// Spec 2 Task 10: the robot's configuration when a batch of rays was cast. The base pose is the simulator's ground
// truth (the experiments' localization assumption) and the arm joints are the joint encoders (MuJoCo's positions); the
// rigid payload follows the arm. `known` is false when the caller supplied none.
struct AcquisitionState
{
  bool known = false;
  double x = 0.0, y = 0.0, yaw = 0.0;
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
};

// Spec 2 Task 10: one physics-time cast of a scan (sweep: one per advance; instant: one per scan). Its points are
// points[first_point, first_point + points), cast at time t from the sensor's world pose; no per-point transforms.
struct CastBatch
{
  double t = 0.0;
  int first_point = 0, points = 0;
  int first_ray = 0, rays = 0;
  Eigen::Vector3d sensor_position = Eigen::Vector3d::Zero();      // the lidar site in the world (map) frame
  Eigen::Quaterniond sensor_rotation = Eigen::Quaterniond::Identity();
  AcquisitionState robot;
};

// One scan: its points in the lidar frame at their cast time, each with its time offset from t_start, the geom hit and
// its class. Rays without a hit or outside the range rules leave no point (Unity publishes them as zeros).
struct PlantScan
{
  uint32_t scan = 0;
  double t_start = 0.0, t_end = 0.0;
  int rays = 0;
  std::vector<CastBatch> batches;  // in cast order; they partition the points
  std::vector<Eigen::Vector3f> points;
  std::vector<float> dt;
  std::vector<int> ray;  // the ray's index within the scan (the noise index)
  std::vector<int> geom;
  std::vector<HitClass> cls;
  double cast_s = 0.0;  // wall time spent casting this scan's rays (gate P)
};

// sweep: every advance casts the rays due since the last one from the sensor's current pose (a real Mid-360's motion
// distortion); instant: a whole scan at once when it is due (Unity's IidNoiseRaycastLiDARSensor).
enum class ScanMode { Sweep, Instant };

class Lidar
{
public:
  Lidar(const LidarParams & params, ScanPattern pattern, ScanMode mode);

  // The model's lidar site and every geom's class (by name prefix); a new run from time `t`: scan 0, the window at the
  // pattern's start, the first scan due at t + 1 / rate.
  // `d` holds the static geoms' poses (kinematics computed): the BVH is built from them.
  void attach(const mjModel * m, const mjData * d, uint64_t seed, double t = 0.0);
  // The rays due over (t - dt, t] from the sensor's pose in `d`: its kinematics must be current (after mj_step, call
  // mj_kinematics or mj_fwdPosition first, or the origin is one step old). With static_bvh: false, MuJoCo's ray
  // preparation drops the ground plane (no bounding radius) beyond the cutoff from its origin: fine on V2 (+-25 m); the
  // BVH always tests it.
  // `robot`: the configuration the batches cast in this call record (Spec 2 Task 10); unknown by default.
  void advance(const mjModel * m, mjData * d, double t, double dt, const AcquisitionState & robot = {});
  // The oldest completed scan not yet taken (scans queue: a caller reading less often than 10 Hz loses none).
  std::optional<PlantScan> take_scan();

  ScanMode mode() const { return mode_; }

private:
  void cast(const mjModel * m, mjData * d, int first, int count, double t, const AcquisitionState & robot);
  void finish(double t);

  LidarParams params_;
  ScanPattern pattern_;
  ScanMode mode_;
  int site_ = -1;
  std::vector<HitClass> classes_;
  std::unique_ptr<StaticBvh> bvh_;
  uint32_t seed32_ = 0;
  uint32_t scan_ = 0;
  long window_ = 0;  // the pattern index of the scan's first ray
  int cast_ = 0;     // rays cast in the current scan
  double t_start_ = 0.0;
  PlantScan current_;
  std::deque<PlantScan> done_;
  std::vector<mjtNum> vec_, dist_, normal_;
  std::vector<int> geomid_;
};
}  // namespace mobile_manipulator_plant
