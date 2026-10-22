#pragma once
// The B5 controller loop without ROS: one MPC cycle per step (nominal from the previous plan,
// collision rows, tightness, QP build and solve), the command output stage (rate limit relative
// to the command sent last, breakaway from rest, dead zone), the slack gate (holding instead of a
// plan that pushes into an obstacle) and the failure handling (fallback to the shifted previous
// plan, braking, stale state, goal). The node (wholebody_mpc_node) owns the
// ROS interfaces and calls step() at its rate.
#include <chrono>
#include <limits>
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

// The arm's braking hold checked against the obstacles (Phase 4B-A Task 10: an unchecked hold extrapolated
// q + v T / 2 into a shelf, a contact). From the reference (q0 = x's joints, v0) with the base at x braking from
// base_vw at the envelope's limits (brake_base; its travel is part of the check): the constant-deceleration stop
// above (the JTC's Hermite to q0 + v0 T / 2), sampled at kHoldSamples points evenly in displacement and once more
// where the base has stopped; when it would bring the arm or panel closer than min(the clearance with the arm
// frozen at q0 along the base's path, the margin) - kHoldTolM, the stop is made kHoldFaster times faster
// (duration / k), and failing that the arm stops at q0 within kHoldEmergencyS (a Hermite from (q0, v0) to (q0, 0),
// which overshoots by about 4/27 v0 T; checked and reported too). Stops faster than the qualified 0.61 rad/s^2
// (factors > 1) are unqualified emergency stops: they reduce the risk, their physical tracking is not verified.
// decel_factor is k (infinity for the stop at q0); clearance_m the smallest clearance along the chosen stop.
constexpr int kHoldSamples = 8;
constexpr double kHoldTolM = 1e-3, kHoldEmergencyS = 0.1;
constexpr double kHoldFaster[] = {2.0, 4.0, 8.0};
struct ArmHold
{
  Eigen::Matrix<double, 6, 1> target;
  double duration_s;
  double decel_factor;
  double clearance_m;  // the smallest clearance along the chosen stop
};
// The second QP iteration's step check: the largest (margin - true distance) of `rows` on stages 1..last_stage, 0 when
// none is inside its margin; a second iterate may exceed the first's by kIter2ViolationTolM (solver noise).
constexpr double kIter2ViolationTolM = 1e-6;
double worst_violation(const std::vector<DistanceRow> & rows, int last_stage);
// An iterate's measures for the step check: its worst violation on the first kIter2NearStages stages (the motion about
// to be executed) and over the gate's horizon, and its objective. The second iterate is kept only if neither violation
// is larger (beyond kIter2ViolationTolM) and its cost is at most (1 + cost_tolerance) times the first's. The near-term
// condition is a Phase 4B-B implementation ruling: with the horizon alone, iterates that cut far-stage violation while
// taking the next stages closer were kept, and the unit loop under a 1.34 m beam came to 0.042 m (one iteration 0.051).
constexpr int kIter2NearStages = 3;
struct Iter2Measure
{
  double near = 0.0, horizon = 0.0, cost = 0.0;
};
bool accept_second_iterate(const Iter2Measure & first, const Iter2Measure & second, double cost_tolerance);

// The slack gate's allowance above a pair's current violation when the pair is already inside the
// margin: the solver's numerical noise only (a plan may not take such a pair deeper; a pair outside
// the margin gets SlackGate::tolerance_m).
constexpr double kGateInsideAllowanceM = 1e-4;
constexpr double kGoalXyM = 0.15;    // as the baselines' goal checker
constexpr double kGoalYawRad = 0.15;

// The command sent to the base: per base component, |c| below the dead zone becomes zero; then,
// while the base is measured at rest and no remaining component reaches its breakaway, the base
// command (v, omega) is scaled up until the larger one (relative to its breakaway) reaches it,
// which keeps the commanded curvature: that one becomes exactly +-breakaway. The arm velocities
// pass unchanged.
Control breakaway_output(const Control & planned, bool base_at_rest, const Breakaway & b);

// The base command one braking step later: (v, omega) towards zero by the envelope's braking limits
// (decel, yaw_decel) over elapsed_s, never past zero, components below the dead zone zeroed. Holds,
// braking and the node's ramp after a goal ends use it.
Eigen::Vector2d brake_base(const Eigen::Vector2d & from, double elapsed_s, const Limits & l, const Breakaway & b);

ArmHold safe_arm_hold(const CollisionModel & collision, const World & world, double now, const State & x,
                      const Eigen::Matrix<double, 6, 1> & v0, const Limits & l,
                      const Eigen::Vector2d & base_vw = Eigen::Vector2d::Zero());

// The base counts as at rest when its measured |v| and |omega| are below Breakaway::rest_*.
bool base_at_rest(double linear_mps, double yaw_radps, const Breakaway & b);

enum class CycleStatus { Ok, Fallback, Braking, Failed, GoalReached, StaleState, Holding, Blocked };
const char * to_string(CycleStatus status);

// Seconds per part of a cycle: linearize = advancing the nominal (shifted previous plan rolled out
// from the measured state), collision = rows and the comfort pose's clearance, setup = the QP build
// (model Jacobians and assembly), solve = the backend's update and solve, total = the whole step.
// With the preparation/feedback split (Phase 4B-B), prep_s is the preparation this cycle used (made after the
// previous cycle's command) and feedback_s the in-cycle part; both 0 without it.
// diagnostics_s: the bench's measurements (ControllerOptions::diagnostics), left out of total_s.
struct CycleTiming
{
  double linearize_s = 0.0, collision_s = 0.0, setup_s = 0.0, solve_s = 0.0, total_s = 0.0;
  double prep_s = 0.0, feedback_s = 0.0, diagnostics_s = 0.0;
};

struct CycleOutput
{
  CycleStatus status = CycleStatus::Failed;
  Control command = Control::Zero();  // to send now (base: v, omega; arm: joint velocities)
  // From the state used this cycle: base poses and joint positions (x), base commands and joint
  // velocities (u). Braking, failed and goal plans hold the arm (zero joint velocities); a holding
  // plan follows the arm's braking hold (from the joint velocities sent last to zero).
  Plan plan;
  CycleTiming timing;
  CollisionStats collision;
  int iterations = 0;
  bool solve_failed = false;  // a solve was due this cycle and failed (or the deadline cap skipped it)
  bool time_limited = false;  // the deadline cap ended or skipped the solve (status time_limit)
  // A second QP iteration ran (qp_iterations 2) and its iterate was kept, or it ran and the step check kept the first.
  bool iter2_used = false, iter2_rejected = false;
  // With ControllerOptions::diagnostics: the largest |linearized - true| distance of the solved plan's collision
  // rows within the slack gate's horizon (m); NaN without diagnostics, without a solve or without such a row.
  double lin_error_m = std::numeric_limits<double>::quiet_NaN();
  double max_slack = 0.0;
  // Slack gate of a solved cycle with the gate on (0 and false otherwise: with the gate off no pair
  // distance is computed, as in the recorded B5's cycle): among the rows of the stages within the
  // gate's horizon, the one whose slack exceeds its allowance the most (its slack and its pair's
  // current violation) when any does, else the largest slack there and 0. The allowance is the
  // tolerance for a pair outside the margin (no current violation) and the violation (plus
  // kGateInsideAllowanceM) for a pair inside it (no deeper). gate_unsafe: some slack exceeds its
  // allowance (while holding, over the resume horizon).
  double gate_slack = 0.0;
  double gate_violation = 0.0;
  bool gate_unsafe = false;
  double manipulability = 0.0;  // of the measured state
  double tightness = 0.0;       // alpha of the posture preference this cycle
  // The side picker's choice this cycle and, from the cycle that chose, its scores (comfort, tight...).
  PostureChoice posture_choice = PostureChoice::None;
  std::vector<double> probe_scores;
  double plan_time = 0.0;       // time of the plan's stage 0: the step's now in every status (a
                                // Fallback plan's controls are shifted by whole stages, but its
                                // states are rolled out from the state measured now)
};

// Phase 4B-B switches of the cycle (spec Section 4); the defaults are the frozen B5. period_s is the node's cycle
// period: 0 turns the time-based parts off (the deadline cap and the second iteration's budget).
struct ControllerOptions
{
  int qp_iterations = 1;
  double iter2_budget = 0.5;
  double iter2_cost_tolerance = 0.01;
  bool warm_start = true;
  bool deadline_cap = false;
  double period_s = 0.0;
  double publish_margin_s = 0.002;
  bool rti_split = false;
  RtiDeviation rti_max_deviation;
  // The replay bench's measurements (CycleOutput::lin_error_m); they never change a cycle's outcome.
  bool diagnostics = false;
};

// The options of wholebody_mpc.yaml for a loop at period_s.
ControllerOptions controller_options(const MpcConfig & config, double period_s);

class Controller
{
public:
  // `formulation` and `collision` must use the same per-stage row budget; the tightness filter
  // and the slack gate take their margin from `collision`. Throws std::invalid_argument for a null
  // solver, a budget mismatch, a negative fallback count, a non-positive goal tolerance or gate
  // horizon, or a negative gate tolerance.
  Controller(Formulation formulation, CollisionModel collision, std::unique_ptr<QpSolver> solver,
             Breakaway breakaway, int fallback_cycles = kDefaultFallbackCycles, double goal_xy_m = kGoalXyM,
             double goal_yaw_rad = kGoalYawRad, SlackGate slack_gate = SlackGate{},
             ControllerOptions options = ControllerOptions{});
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
  // Statuses: Ok; Fallback (solver failure while tracking: the shifted previous plan, up to
  // fallback_cycles in a row); Holding (the solve succeeded but its plan failed the slack gate, or
  // passed it within the resume dwell: it is not executed; the base decelerates at the envelope's
  // braking limits from the command sent last, the arm follows its braking hold from the joint
  // velocities sent last (started once per hold), the plan continues braking, and the next cycle
  // linearizes around the held plan; a held solve is no failure, so it neither uses the fallback
  // budget nor ends the goal. A solver failure while holding also holds (the held plan is never
  // executed), counts towards the fallback budget and restarts the resume dwell; missed deadlines
  // noted while holding count too); Braking (more than fallback_cycles failures in a row: the base
  // decelerates at the envelope's braking limits, the arm holds); StaleState (braking because of
  // `state_fresh`); Failed (braking finished: zero command, latched until start); GoalReached
  // (within the goal tolerance: zero command, latched until start). Throws std::logic_error before
  // the first start().
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

  // Blocked by the collision monitor (Phase 4B-A, the node's BlockedDetector): while on, every cycle holds
  // as a slack-gate hold does (no plan executed, base braking, the arm's braking hold, plans every cycle)
  // and reports CycleStatus::Blocked; off resumes through the gate's resume rule.
  void set_external_hold(bool on) { external_hold_ = on; }

  // The start of the caller's cycle (the node's timer callback), for the deadline cap of the next step() only: the
  // budget is the period less what the cycle used before the solve. Without it the next step() counts from its own
  // start.
  void set_cycle_start(std::chrono::steady_clock::time_point start) { cycle_start_ = start; }

  // The command sent last (after the output stage).
  const Control & last_command() const { return last_command_; }
  const Formulation & formulation() const { return formulation_; }
  const CollisionModel & collision() const { return collision_; }
  const ControllerOptions & options() const { return options_; }

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
  // its allowance: the tolerance for a pair outside the margin, its current violation for a pair
  // inside it. Sets out_.gate_slack, gate_violation and gate_unsafe.
  // The largest |row.distance + gradient dx - true distance at the applied plan| over the rows of stages
  // 1..the gate's horizon (NaN without such a row). After apply_in_place: nominal_ is the plan.
  double linearization_error(const std::vector<DistanceRow> & rows, const QpSolution & solution, const World & world,
                             double now) const;
  bool gate_passes(const std::vector<DistanceRow> & rows, const Eigen::VectorXd & slack, const State & x0,
                   const World & world, double now, double horizon_s);

  Formulation formulation_;
  CollisionModel collision_;
  std::unique_ptr<QpSolver> solver_;
  Breakaway breakaway_;
  int fallback_cycles_;
  double goal_xy_m_, goal_yaw_rad_;
  SlackGate gate_;
  ControllerOptions options_;
  TightnessFilter tightness_;
  SidePicker picker_;  // carries over goals like the tightness filter
  std::vector<double> probe_comfort_;
  std::vector<std::vector<double>> probe_tights_;
  RouteProbe probe_route_;            // the route probe of the choosing cycle (Phase 4B-A Task 11)
  std::vector<State> route_samples_;  // its base poses

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
  bool was_holding_ = false;  // the last step held, by the gate or blocked (the arm's braking hold continues)
  bool external_hold_ = false;
  std::optional<std::chrono::steady_clock::time_point> cycle_start_;
  double resume_since_ = std::numeric_limits<double>::quiet_NaN();  // holding: plans pass since then
  Eigen::Matrix<double, 6, 1> arm_hold_v0_ = Eigen::Matrix<double, 6, 1>::Zero();
  double arm_hold_duration_ = kArmHoldMinS, arm_hold_t_ = 0.0;
  bool solver_ready_ = false;
  QpSolution warm_;
  bool have_warm_ = false;
  Plan nominal_;  // the plan the next QP linearizes around (the latest solved plan, held or not,
                  // advanced to now; executed by a Fallback only outside a hold)
  CycleOutput out_;
};
}  // namespace mobile_manipulator_mpc
