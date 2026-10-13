#pragma once
// The B5 controller loop without ROS: one MPC cycle per step (nominal from the previous plan,
// collision rows, tightness, QP build and solve), the command output stage (rate limit relative
// to the command sent last, breakaway from rest, dead zone), the slack gate (holding instead of a
// plan that pushes into an obstacle) and the failure handling (fallback to the shifted previous
// plan, braking, stale state, goal). The node (wholebody_mpc_node) owns the
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

// The cycles a loop at rate_hz runs in `seconds`: ceil(seconds * rate_hz), an exact product not
// rounded up (e.g. the node's statistics reserve, an hour at its rate). Throws
// std::invalid_argument for a non-positive time or rate.
long cycles_in(double seconds, double rate_hz);
// The fallback budget in cycles for a time budget at a loop rate (ruling R27): cycles_in(seconds,
// rate_hz), so the controller rides out the same stall at any rate (0.3 s: 6 cycles at 20 Hz,
// 15 at 50 Hz). Throws std::invalid_argument for a non-positive time or rate.
int fallback_cycles_for(double seconds, double rate_hz);
// The arm's braking hold (rulings R21, R24): from the joint velocities v to zero at a constant
// deceleration over T = max(kArmHoldMinS, max_j |v_j| / qdd_max_j), all joints together.
constexpr double kArmHoldMinS = 0.5;
constexpr double kGoalXyM = 0.15;    // as the baselines' goal checker
constexpr double kGoalYawRad = 0.15;

// The command sent to the base: per base component, |c| below the dead zone becomes zero; then,
// while the base is measured at rest and no remaining component reaches its breakaway, the base
// command (v, omega) is scaled up until the larger one (relative to its breakaway) reaches it,
// which keeps the commanded curvature: that one becomes exactly +-breakaway. The arm velocities
// pass unchanged.
Control breakaway_output(const Control & planned, bool base_at_rest, const Breakaway & b);

// The base counts as at rest when its measured |v| and |omega| are below Breakaway::rest_*.
bool base_at_rest(double linear_mps, double yaw_radps, const Breakaway & b);

enum class CycleStatus { Ok, Fallback, Braking, Failed, GoalReached, StaleState, Holding };
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
  // velocities (u). Holding, braking, failed and goal plans hold the arm (zero joint velocities).
  Plan plan;
  CycleTiming timing;
  CollisionStats collision;
  int iterations = 0;
  double max_slack = 0.0;
  // Slack gate of a solved cycle (0 otherwise): among the rows of the stages within the gate's
  // horizon, the one whose slack exceeds its pair's current violation the most when any slack is
  // above the tolerance (its slack and that violation), else the largest slack there and 0.
  double gate_slack = 0.0;
  double gate_violation = 0.0;
  double manipulability = 0.0;  // of the measured state
  double tightness = 0.0;       // alpha of the posture preference this cycle
  double plan_time = 0.0;       // time of the plan's stage 0: the step's now in every status (a
                                // Fallback plan's controls are shifted by whole stages, but its
                                // states are rolled out from the state measured now)
};

class Controller
{
public:
  // `formulation` and `collision` must use the same per-stage row budget; the tightness filter
  // and the slack gate take their margin from `collision`. Throws std::invalid_argument for a null
  // solver, a budget mismatch, a negative fallback count, a non-positive goal tolerance or gate
  // horizon, or a negative gate tolerance.
  Controller(Formulation formulation, CollisionModel collision, std::unique_ptr<QpSolver> solver,
             Breakaway breakaway, int fallback_cycles = kDefaultFallbackCycles, double goal_xy_m = kGoalXyM,
             double goal_yaw_rad = kGoalYawRad, SlackGate slack_gate = SlackGate{});
  // Not copyable or movable: the QP inputs point at the controller's own route.
  Controller(const Controller &) = delete;
  Controller & operator=(const Controller &) = delete;

  // A new goal (also preempting a running one): the route (map frame), the goal pose, the panel
  // references (empty: unscripted) and the posture targets. The plan restarts from the measured
  // state; the command sent last (rate limits) and the tightness filter carry over. The first
  // cycle projects the measured state onto the whole route; later cycles search around the
  // previous cycle's progress (Inputs::s0_hint).
  void start(const Route & route, const mobile_manipulator_navigation::Pose2 & goal, std::vector<PanelReference> refs,
             PostureTargets posture);

  // The active goal's route, goal pose and references change in place (a preempting FollowPath,
  // e.g. a replanned path): the mode, the failures in a row, the warm plan and its nominal, the
  // tightness filter and the posture targets carry over. A latched Failed or GoalReached stays
  // latched (start() begins a new goal). The first cycle after it projects the measured state
  // onto the whole new route, later cycles around the previous cycle's progress again.
  void update_goal(const Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                   std::vector<PanelReference> refs);

  // One cycle at time `now` (seconds, monotonic) from the measured state `x`. The controller is
  // single-threaded: call start, step, reset_command and note_failure from one thread. The
  // returned reference aliases the controller's own output buffer, which the next step
  // overwrites: copy what must outlive it (for example before handing it to another thread). `state_fresh` false
  // (missing or stale joint states, odometry or TF) brakes at once and fails once stopped.
  // The output stays valid until the next call (no allocation for it per cycle).
  // Statuses: Ok; Fallback (solver failure: the shifted previous plan, up to fallback_cycles in a
  // row); Holding (the solve succeeded but its plan failed the slack gate: it is not executed; the
  // base decelerates at the envelope's braking limits from the command sent last, the arm holds
  // (zero joint velocities), the plan continues braking, and the next cycle plans again from this
  // braking plan; no failure, so it neither uses the fallback budget nor ends the goal); Braking (after that: the base decelerates at the envelope's braking limits, the arm
  // holds); StaleState (braking because of `state_fresh`); Failed (braking finished: zero command,
  // latched until start); GoalReached (within the goal tolerance: zero command, latched until
  // start). Throws std::logic_error before the first start().
  const CycleOutput & step(double now, const State & x, bool state_fresh, const World & world, bool base_at_rest);

  // The node sent a base command itself (a zero on cancel, on a failure it detected or at the
  // goal's end): the next cycle's rate band and braking start from `sent`, not from the command
  // the controller output last.
  void reset_command(const Control & sent = Control::Zero());

  // Counts the last step as a failure although its output was used (the node calls it when the
  // cycle overran its period). A step whose solve already failed is not counted twice. More than
  // fallback_cycles failures in a row (solver failures or missed deadlines) brake from the next
  // step on, as in step(). No effect outside tracking (braking, failed, goal reached).
  void note_failure();

  // The command sent last (after the output stage).
  const Control & last_command() const { return last_command_; }
  const Formulation & formulation() const { return formulation_; }

private:
  enum class Mode { Tracking, Braking, Failed, GoalReached };

  void set_goal(const Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                std::vector<PanelReference> refs);
  // The base at the envelope's braking limits from the command sent last; the arm holds (zero
  // joint velocities) or, with arm_hold, follows the braking hold started at the hold's entry.
  void brake(double elapsed, const State & x, bool arm_hold = false);
  void hold(const State & x);
  bool at_goal(const State & x) const;
  // The slack gate on a solution's slack for `rows` (the rows the QP was built from) at the
  // measured state x0: true when no row of a stage within the gate's horizon has more slack than
  // its pair's current violation plus the tolerance. Sets out_.gate_slack and gate_violation.
  bool gate_passes(const std::vector<DistanceRow> & rows, const Eigen::VectorXd & slack, const State & x0,
                   const World & world, double now, double horizon_s);

  Formulation formulation_;
  CollisionModel collision_;
  std::unique_ptr<QpSolver> solver_;
  Breakaway breakaway_;
  int fallback_cycles_;
  double goal_xy_m_, goal_yaw_rad_;
  SlackGate gate_;
  TightnessFilter tightness_;

  std::optional<Route> route_;
  Inputs inputs_{};  // per goal: route, goal, references, posture; per cycle: x0, last command, tightness,
                     // the progress hint
  Eigen::Vector2d goal_xy_ = Eigen::Vector2d::Zero();
  Mode mode_ = Mode::Tracking;
  bool fresh_plan_ = true;  // the next cycle starts from hold(x) instead of the previous plan
  int failures_ = 0;        // failures in a row (solver failures and noted missed deadlines)
  int failures_at_step_ = 0;  // failures_ before the last step (note_failure counts a cycle once)
  double plan_time_ = 0.0;  // time of the plan's stage 0
  double last_time_ = 0.0;  // time of the last command
  bool have_last_time_ = false;
  Control last_command_ = Control::Zero();
  bool was_holding_ = false;  // the last step held (the arm's braking hold continues)
  double resume_since_ = std::numeric_limits<double>::quiet_NaN();  // holding: plans pass since then
  Eigen::Matrix<double, 6, 1> arm_hold_v0_ = Eigen::Matrix<double, 6, 1>::Zero();
  double arm_hold_duration_ = kArmHoldMinS, arm_hold_t_ = 0.0;
  bool solver_ready_ = false;
  QpSolution warm_;
  bool have_warm_ = false;
  Plan nominal_;  // the plan the next QP linearizes around (the latest plan, advanced to now)
  CycleOutput out_;
};
}  // namespace mobile_manipulator_mpc
