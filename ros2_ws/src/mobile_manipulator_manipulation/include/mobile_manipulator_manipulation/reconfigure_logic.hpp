#pragma once
// Pure helpers of the ReconfigurePanel server (no ROS graph).
#include <deque>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace mobile_manipulator_manipulation
{
// The attached panel's centre subframe, as MoveIt names it in constraints.
inline constexpr const char * kPanelFrame = "panel/panel";

// Goal constraints putting the panel centre inside a box of half extents
// `position_tolerance` around `pose` and its orientation within per-axis rotation-vector
// tolerances. ResolveConstraintFrames maps the subframe onto tool0 for planning; it
// rejects XYZ-Euler tolerances for subframes and would then drop the constraints.
moveit_msgs::msg::Constraints panel_goal_constraints(const geometry_msgs::msg::PoseStamped & pose,
                                                     const geometry_msgs::msg::Vector3 & position_tolerance,
                                                     const geometry_msgs::msg::Vector3 & orientation_tolerance);

// Whether `reached` (panel centre in the goal's frame) satisfies the same tolerances:
// the position inside the box and each rotation-vector component of the orientation error
// within its tolerance. Guards against constraints MoveIt ignored.
bool panel_goal_met(const Eigen::Isometry3d & reached, const geometry_msgs::msg::PoseStamped & goal,
                    const geometry_msgs::msg::Vector3 & position_tolerance,
                    const geometry_msgs::msg::Vector3 & orientation_tolerance);

// Whether the base has been stopped: every /odom sample of the last `window_s` below the
// speed limits, and samples throughout it (from the window start to now, none more than
// 0.1 s apart). A sample more than 0.5 s older than the newest (teleport, new simulation
// epoch) restarts it; a slightly older one (callbacks handled out of order) is inserted in
// time order.
class BaseMotionWindow
{
public:
  void add(double t, double linear, double angular);
  bool stopped(double now, double window_s = 0.5, double v_max = 0.01, double w_max = 0.02) const;

private:
  struct Sample
  {
    double t, linear, angular;
  };
  std::deque<Sample> samples_;
};

// Whether an execution has run too long: its simulation-time budget is spent, or the
// simulation has (nearly) frozen and 4 x the budget has passed in wall time. A slow but
// running simulation (real-time factor down to 0.25) does not abort a healthy trajectory.
bool execution_overdue(double sim_elapsed_s, double wall_elapsed_s, double budget_s);

// Largest |measured - target| joint error over the samples of a hold; NaN (unmeasured) when
// no sample arrived, so a stalled feedback stream cannot read as a perfect hold.
class HoldErrorTracker
{
public:
  explicit HoldErrorTracker(std::vector<double> target) : target_(std::move(target)) {}
  void add(const std::vector<double> & measured);
  double value() const;
  std::size_t samples() const { return samples_; }

private:
  std::vector<double> target_;
  double max_error_ = 0.0;
  std::size_t samples_ = 0;
};

// Plan requests per reconfiguration, and whether a failed request is worth repeating. OMPL's
// path is collision-free, but time parameterization and Ruckig smoothing can bend it into an
// obstacle it grazed; MoveIt's ValidateSolution then rejects the plan (reported as FAILURE).
// A new random plan usually passes. IK and goal-constraint failures would only repeat.
constexpr int kMaxPlanRequests = 3;
bool replan_after(int moveit_error_code);

// True when `sample` is not older than `trajectory_end`.
bool fresh_after(const builtin_interfaces::msg::Time & sample, const builtin_interfaces::msg::Time & trajectory_end);

// Sum over consecutive points and joints of |delta q| (rad).
double joint_path_length(const trajectory_msgs::msg::JointTrajectory & trajectory);
}  // namespace mobile_manipulator_manipulation
