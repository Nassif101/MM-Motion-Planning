#pragma once
// The B5 controller loop without ROS: one MPC cycle per step (nominal from the previous plan,
// collision rows, tightness, QP build and solve), the command output stage (rate limit relative
// to the command sent last, breakaway from rest, dead zone) and the failure handling (fallback to
// the shifted previous plan, braking, stale state, goal). The node (wholebody_mpc_node) owns the
// ROS interfaces and calls step() at its rate.
#include <memory>
#include <optional>
#include <vector>

#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/qp.hpp"
#include "mobile_manipulator_mpc/references.hpp"
#include "mobile_manipulator_mpc/route.hpp"
#include "mobile_manipulator_mpc/world.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mobile_manipulator_mpc
{
constexpr int kDefaultFallbackCycles = 3;
constexpr double kGoalXyM = 0.15;    // as the baselines' goal checker
constexpr double kGoalYawRad = 0.15;

// The command sent to the base: per base component, |c| below the dead zone becomes zero; then,
// while the base is measured at rest and no remaining component reaches its breakaway, the base
// command (v, omega) is scaled up until the larger one (relative to its breakaway) reaches it,
// which keeps the commanded curvature. The arm velocities pass unchanged.
Control breakaway_output(const Control & planned, bool base_at_rest, const Breakaway & b);

// The base counts as at rest when its measured |v| and |omega| are below Breakaway::rest_*.
bool base_at_rest(double linear_mps, double yaw_radps, const Breakaway & b);

enum class CycleStatus { Ok, Fallback, Braking, Failed, GoalReached, StaleState };
const char * to_string(CycleStatus status);

// Seconds per part of a cycle: linearize = advancing the nominal (shifted previous plan rolled out
// from the measured state), collision = rows and the home clearance, setup = the QP build (model
// Jacobians and assembly), solve = the backend's update and solve, total = the whole step.
struct CycleTiming
{
  double linearize_s = 0.0, collision_s = 0.0, setup_s = 0.0, solve_s = 0.0, total_s = 0.0;
};

struct CycleOutput
{
  CycleStatus status = CycleStatus::Failed;
  Control command = Control::Zero();  // to send now (base: v, omega; arm: joint velocities)
  // From the state used this cycle: base poses and joint positions (x), base commands and joint
  // velocities (u). Braking, failed and goal plans hold the arm (zero joint velocities).
  Plan plan;
  CycleTiming timing;
  CollisionStats collision;
  int iterations = 0;
  double max_slack = 0.0;
  double manipulability = 0.0;  // of the measured state
  double tightness = 0.0;       // alpha of the posture preference this cycle
};

class Controller
{
public:
  // `formulation` and `collision` must use the same per-stage row budget; the tightness filter
  // takes its margin from `collision`. Throws std::invalid_argument for a null solver, a budget
  // mismatch, a negative fallback count or a non-positive goal tolerance.
  Controller(Formulation formulation, CollisionModel collision, std::unique_ptr<QpSolver> solver,
             Breakaway breakaway, int fallback_cycles = kDefaultFallbackCycles, double goal_xy_m = kGoalXyM,
             double goal_yaw_rad = kGoalYawRad);
  // Not copyable or movable: the QP inputs point at the controller's own route.
  Controller(const Controller &) = delete;
  Controller & operator=(const Controller &) = delete;

  // A new goal (also preempting a running one): the route (map frame), the goal pose, the panel
  // references (empty: unscripted) and the posture targets. The plan restarts from the measured
  // state; the command sent last (rate limits) and the tightness filter carry over.
  void start(const Route & route, const mobile_manipulator_navigation::Pose2 & goal, std::vector<PanelReference> refs,
             PostureTargets posture);

  // One cycle at time `now` (seconds, monotonic) from the measured state `x`. `state_fresh` false
  // (missing or stale joint states, odometry or TF) brakes at once and fails once stopped.
  // The output stays valid until the next call (no allocation for it per cycle).
  // Statuses: Ok; Fallback (solver failure: the shifted previous plan, up to fallback_cycles in a
  // row); Braking (after that: the base decelerates at the envelope's braking limits, the arm
  // holds); StaleState (braking because of `state_fresh`); Failed (braking finished: zero command,
  // latched until start); GoalReached (within the goal tolerance: zero command, latched until
  // start). Throws std::logic_error before the first start().
  const CycleOutput & step(double now, const State & x, bool state_fresh, const World & world, bool base_at_rest);

  // The command sent last (after the output stage).
  const Control & last_command() const { return last_command_; }
  const Formulation & formulation() const { return formulation_; }

private:
  enum class Mode { Tracking, Braking, Failed, GoalReached };

  void brake(double elapsed, const State & x);
  void hold(const State & x);
  bool at_goal(const State & x) const;

  Formulation formulation_;
  CollisionModel collision_;
  std::unique_ptr<QpSolver> solver_;
  Breakaway breakaway_;
  int fallback_cycles_;
  double goal_xy_m_, goal_yaw_rad_;
  TightnessFilter tightness_;

  std::optional<Route> route_;
  Inputs inputs_{};  // per goal: route, goal, references, posture; per cycle: x0, last command, tightness
  Eigen::Vector2d goal_xy_ = Eigen::Vector2d::Zero();
  Mode mode_ = Mode::Tracking;
  bool fresh_plan_ = true;  // the next cycle starts from hold(x) instead of the previous plan
  int failures_ = 0;        // solver failures in a row
  double plan_time_ = 0.0;  // time of the plan's stage 0
  double last_time_ = 0.0;  // time of the last command
  bool have_last_time_ = false;
  Control last_command_ = Control::Zero();
  bool solver_ready_ = false;
  QpSolution warm_;
  bool have_warm_ = false;
  Plan nominal_;  // the plan the next QP linearizes around (the latest plan, advanced to now)
  CycleOutput out_;
};
}  // namespace mobile_manipulator_mpc
