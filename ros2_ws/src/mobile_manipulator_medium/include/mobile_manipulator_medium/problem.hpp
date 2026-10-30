#pragma once
// The medium-horizon problem (Phase 5A spec Section 3): the base pose (x, y, yaw) and the masked arm joints as
// trajectories in a TrajectoryBasis over the horizon, a cost and constraints evaluated at time samples, and their
// linearization around a nominal trajectory as one dense QP in the step of the free parameters (SCP, Task 8).
//
// QP variables: the parameter steps of every coordinate (coordinate-major, free_parameters() each), then one slack
// s >= 0 shared by every soft row (collision, torque, base speed) with an L1 and a quadratic weight; the trust region
// is a box on the parameter steps. Hard rows: the convex-hull bounds of the masked joints (position, velocity,
// acceleration), the yaw rate and the base accelerations. A hard row the nominal already violates is not made worse
// (its bound moves to the nominal), so the QP stays feasible; feasibility is judged on the exact trajectory.
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_medium/bezier.hpp"
#include "mobile_manipulator_medium/dense_qp.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/route.hpp"
#include "mobile_manipulator_mpc/statics.hpp"
#include "mobile_manipulator_mpc/world.hpp"

namespace mobile_manipulator_medium
{
namespace mmm = mobile_manipulator_mpc;
using Joints = Eigen::Matrix<double, 6, 1>;

struct MediumWeights
{
  double progress = 10.0;      // lag towards the time-parameterized route reference (per second of horizon)
  double contour = 0.5;        // normal offset from the route (weak)
  double goal_heading = 5.0;   // final yaw towards the goal yaw within kGoalHeadingM of the goal
  double clearance = 20.0;     // hinge below clearance_target_m (base, arm and panel rows)
  double comfort = 0.5;        // |q - q_comfort|^2 over the masked joints
  double smooth_base = 1.0;    // integral of the squared acceleration of x, y and yaw
  double smooth_arm = 1.0;     // ... and of the masked joints
  double skid = 50.0;          // squared body-frame lateral velocity
  double slack = 1000.0;       // L1 weight of the shared slack (and of a violation in the merit)
  double slack_quadratic = 1e4;
};

struct ScpConfig
{
  int max_iter = 10;
  double trust_init = 0.5;   // box on the parameter steps (m or rad of a control point: the map is orthonormal)
  double trust_min = 1e-3;
  double trust_max = 2.0;
  double step_tol = 1e-3;    // converged when the accepted step's largest component is below this
  double merit_tol = 1e-4;   // ... or the merit changes by less than this, relative
};

// medium.yaml (spec Sections 3 and 6; every key optional with these defaults; an unknown key throws
// std::runtime_error naming it).
struct MediumConfig
{
  double rate_hz = 5.0;
  BezierSpec bezier{3, 4, 8.0, 2};
  std::vector<std::string> arm_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_2_joint"};
  double collision_dt = 0.1;   // collision and cost samples (s)
  double torque_dt = 0.4;      // torque rows every this (s)
  double base_margin_m = 0.12;
  double arm_margin_m = 0.05;
  double clearance_target_m = 0.25;
  double speed_fraction = 0.8;  // the route reference moves at this share of the envelope's v_max
  MediumWeights weights;
  ScpConfig scp;
  double slack_feasible_m = 0.001;
  double homotopy_hysteresis = 0.15;
  double solve_cap_s = 0.15;
  double full_multistart_period_s = 2.0;
  double sample_ds = 0.1;
  double sample_dyaw = 0.1;
  // N m per joint in the model's order; infinity: none (shoulder_pan). Defaults: the Task 2 checkpoint (2026-10-30).
  Joints torque_limits_nm = (Joints() << std::numeric_limits<double>::infinity(), 24.4, 41.3, 10.7, 10.7, 10.7).finished();
  std::vector<std::string> seeds = {"previous", "vertical_carry", "vertical_carry_right", "fold", "fold_back",
                                    "level_extension"};
  std::string qp_backend = "hpipm";
  double qp_tolerance = 1e-6;
};
MediumConfig load_medium_config(const std::string & yaml_path);

// The coordinates of a trajectory: x, y, yaw, then the masked joints (indices into the model's six).
std::vector<int> joint_indices(const std::vector<std::string> & arm_joints);

// Control points per coordinate (rows: x, y, yaw, then the masked joints in joint_indices order); the joints not
// masked stay at q_rest. Built with Trajectory::from_parameters or by the planner's seeds.
struct Trajectory
{
  Eigen::MatrixXd control_points;  // coordinates x control_points()
  Joints q_rest = Joints::Zero();
};

// What a QP row is (MediumQp::rows, index for index with qp.A's rows).
enum class RowKind { Collision, Torque, Speed, Hull };
struct RowInfo
{
  RowKind kind;
  int sample;                    // time sample index (t = sample * collision_dt); hull rows: -1
  mmm::DistanceRow collision{};  // collision rows: the pair
  int joint = -1;                // torque rows: the joint
};

struct MediumQp
{
  DenseQp qp;
  std::vector<RowInfo> rows;
  int parameters = 0;  // the parameter steps; the slack is variable `parameters`
};

// The route to plan along: the B5 route, the goal's arc length and yaw, and the arc length the robot is at.
struct RouteTask
{
  const mmm::Route * route = nullptr;
  double goal_s = 0.0;
  double goal_yaw = 0.0;
  double s0 = 0.0;
};

class MediumProblem
{
public:
  // The robot's collision `primitives` with `collision` (pairs per sample, activation, self rows; its margins are
  // replaced by the config's base_margin_m and arm_margin_m), `statics` the torques, `q_comfort` the comfort pose.
  // Throws std::invalid_argument for an unknown arm joint name, a non-positive sample time or an activation distance
  // below clearance_target_m.
  MediumProblem(MediumConfig config, mmm::Limits limits,
                const std::vector<mobile_manipulator_geometry::Primitive> & primitives, mmm::CollisionConfig collision,
                std::shared_ptr<const mmm::StaticTorqueModel> statics, Joints q_comfort);

  const MediumConfig & config() const { return config_; }
  const TrajectoryBasis & basis() const { return *basis_; }
  int coordinates() const { return 3 + static_cast<int>(joints_.size()); }
  const std::vector<int> & joints() const { return joints_; }
  int samples() const { return static_cast<int>(times_.size()); }  // including t = 0
  double sample_time(int k) const { return times_[k]; }

  // A trajectory through `start` (base pose and joints) with `start_rate` (d/dt of x, y, yaw and the six joints) whose
  // free parameters `params` (coordinates x free_parameters) are its deviation, through the map, from the start state
  // continued at its rate (zero: p0 + v0 t in every coordinate).
  Trajectory from_parameters(const mmm::State & start, const Eigen::Matrix<double, 9, 1> & start_rate,
                             const Eigen::MatrixXd & params) const;
  Eigen::MatrixXd parameters(const Trajectory & t) const;  // coordinates x free_parameters
  mmm::State state_at(const Trajectory & t, int sample) const;
  // d/dt of (x, y, yaw) at a sample, and of the six joints (zero for the joints not masked).
  Eigen::Matrix<double, 9, 1> rate_at(const Trajectory & t, int sample) const;

  MediumQp linearize(const Trajectory & nominal, const RouteTask & task, const mmm::World & world, double now,
                     double trust) const;
  // The cost at `t` (route projections, references and collision pairs evaluated at t itself) plus weights.slack times
  // the largest violation of a soft row.
  double merit(const Trajectory & t, const RouteTask & task, const mmm::World & world, double now) const;
  // The cost terms at `t` by name (progress, contour, goal_heading, clearance, comfort, smooth, skid), each
  // sum_k 0.5 w r^2 (samples weighted by collision_dt) as merit() adds them.
  std::map<std::string, double> cost_terms(const Trajectory & t, const RouteTask & task, const mmm::World & world,
                                           double now) const;
  // The largest violation of a soft row at `t`: collision margin (m), torque (N m / limit, as a fraction), base speed
  // (m/s).
  double max_slack(const Trajectory & t, const mmm::World & world, double now) const;
  // True when the hull rows (joint position, velocity, acceleration; yaw rate; base acceleration) hold at `t`.
  bool hull_ok(const Trajectory & t) const;
  // t with the parameter step dp (the first parameters entries of a QP solution; the slack is ignored).
  Trajectory apply(const Trajectory & t, const Eigen::VectorXd & dp) const;

private:
  // The route reference arc lengths per sample (nominal speed from task.s0, tapered over the last metre).
  std::vector<double> reference_s(const RouteTask & task) const;

  MediumConfig config_;
  mmm::Limits limits_;
  mmm::CollisionModel collision_;
  std::shared_ptr<const mmm::StaticTorqueModel> statics_;
  Joints q_comfort_;
  std::unique_ptr<TrajectoryBasis> basis_;
  std::vector<int> joints_;
  std::vector<double> times_;
  std::vector<Eigen::MatrixXd> sm_;  // per derivative 0..2: samples x free_parameters (basis rows times the map)
  std::vector<Eigen::MatrixXd> s_;   // per derivative 0..2: samples x control_points
  std::vector<Eigen::MatrixXd> hull_;  // per derivative 0..2: the hull rows (derivative control points) a bound needs
  Eigen::MatrixXd gram_;              // free_parameters x free_parameters: map^T G map
  std::vector<int> torque_samples_;
};
}  // namespace mobile_manipulator_medium
