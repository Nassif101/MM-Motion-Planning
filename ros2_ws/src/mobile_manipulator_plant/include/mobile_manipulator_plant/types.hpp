#pragma once
// Value types of the MuJoCo plant's interface (MuJoCo plant spec Section 2.1). ROS-free.
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <vector>

namespace mobile_manipulator_plant
{
using Vec6 = Eigen::Matrix<double, 6, 1>;

// A planar pose in the map frame: base_footprint's x, y (m) and yaw (rad).
struct Pose2
{
  double x = 0.0, y = 0.0, yaw = 0.0;
};

enum class PlantMode { Ideal, Realistic };

// A point of a streamed arm trajectory, as the JTC receives it: time from the trajectory's start (s), positions,
// velocities.
struct StreamPoint
{
  double t;
  Vec6 q, v;
};

// The arm's command: a streamed trajectory (start time on the plant's clock and its points; held at the last point
// after it) or, when `joint_velocity` is set, joint velocities.
struct ArmReference
{
  double start_time = 0.0;
  std::vector<StreamPoint> points;
  std::optional<Vec6> joint_velocity;
};

// One command from the framework: its stamp on the plant's clock, the base twist (v m/s, omega rad/s) and the arm.
struct PlantCommand
{
  double stamp = 0.0;
  double v = 0.0, omega = 0.0;
  ArmReference arm;
};

// The plant's state as reported to the framework.
struct PlantState
{
  double t = 0.0;
  Pose2 base;
  double v = 0.0, omega = 0.0;  // body-frame forward speed and yaw rate
  Vec6 q = Vec6::Zero(), qd = Vec6::Zero();
  Eigen::Isometry3d panel = Eigen::Isometry3d::Identity();  // the panel box's centre pose in the map frame
};

// A box of the known world: full size, centre in the map frame, yaw about the vertical.
struct SceneBox
{
  std::string name;
  Eigen::Vector3d center = Eigen::Vector3d::Zero(), size = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

// A scenario mover (ScenarioMoverMotion.cs): a box walking start -> end on the ground.
struct MoverSpec
{
  std::string name;
  Eigen::Vector2d start = Eigen::Vector2d::Zero(), end = Eigen::Vector2d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();  // size_x, size_y, height
  double speed_mps = 0.0, trigger_distance_m = 0.0;
  int crossings = 1;
};

// What a scenario adds to the site: its obstacle boxes and its movers.
struct ScenarioSetup
{
  std::string name;
  std::vector<SceneBox> boxes;
  std::vector<MoverSpec> movers;
};
}  // namespace mobile_manipulator_plant
