#pragma once
// The MPC's obstacle world in the map frame: the known boxes (map export plus scenario) and
// the movers from /scenario/movers, predicted at constant velocity over the horizon.
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mobile_manipulator_navigation/known_world.hpp"

namespace mobile_manipulator_mpc
{
// Box in the map frame: centre pose (yaw only for known boxes) and full size along its axes.
struct ObstacleBox
{
  std::string name;
  Eigen::Isometry3d pose;
  Eigen::Vector3d size;
};

// Axis-aligned bounding box in the map frame.
struct Bounds
{
  Eigen::Vector3d lo, hi;
};

// The bounding box of an obstacle box.
Bounds bounds_of(const ObstacleBox & box);

constexpr double kMoverJumpM = 1.0;   // a larger step between samples resets the velocity
constexpr double kMoverStaleS = 1.0;  // movers whose last sample is older are dropped

// A box for World::update_box from a full orientation (the demo's draggable obstacles, which a user can tip over):
// an upright box (tilted less than kUprightTiltRad) keeps its yaw; a tilted one becomes its axis-aligned bounds, a
// conservative stand-in, since World boxes turn about z only.
constexpr double kUprightTiltRad = 0.05;
struct YawedBox
{
  Eigen::Vector3d center;
  double yaw;
  Eigen::Vector3d size;
};
YawedBox yawed_box(const Eigen::Vector3d & center, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & size);

class World
{
public:
  // Replaces the known boxes (and their bounds: known boxes never move).
  void set_static(const std::vector<mobile_manipulator_navigation::Box> & boxes);

  // A mover sample: centre position on the ground (the box stands on z = 0, axis-aligned),
  // full size, sample time. The velocity comes from the last two samples; it is zero for the
  // first sample and after a jump of more than kMoverJumpM. A sample with a stamp not later
  // than the previous one replaces the position and keeps the velocity.
  void update_mover(const std::string & name, const Eigen::Vector2d & position, const Eigen::Vector3d & size,
                    double stamp);

  // A box with a full pose (the demo's draggable obstacles, which may float at panel height): centre in the map
  // frame, yaw, full size, sample time. Velocity, jumps and staleness as for a mover, in 3D; a mover is such a
  // box standing on z = 0 without yaw.
  void update_box(const std::string & name, const Eigen::Vector3d & center, double yaw, const Eigen::Vector3d & size,
                  double stamp);

  // The known boxes, then the movers (by name) predicted to now + t_ahead; movers whose last
  // sample is more than kMoverStaleS before `now` are left out.
  std::vector<ObstacleBox> at(double t_ahead, double now) const;

  // The two parts of at() without copying the known boxes: the known boxes and, index for index,
  // their bounds; the movers alone, predicted and filtered as in at().
  const std::vector<ObstacleBox> & statics() const { return static_; }
  const std::vector<Bounds> & static_bounds() const { return static_bounds_; }
  std::vector<ObstacleBox> movers_at(double t_ahead, double now) const;

private:
  struct Mover
  {
    Eigen::Vector3d center;
    Eigen::Vector3d velocity;
    double yaw;
    Eigen::Vector3d size;
    double stamp;
  };
  std::vector<ObstacleBox> static_;
  std::vector<Bounds> static_bounds_;
  std::map<std::string, Mover> movers_;
};
}  // namespace mobile_manipulator_mpc
