#pragma once
// The B5 whole-body MPC formulation: one linearized, stage-wise QP per cycle around a nominal
// plan (the shifted previous solution), in the deviations from that plan. The formulation knows
// nothing about the QP backend.
//
// QP state per stage: z = (x, u_prev), the model state (kNx) and the control of the previous
// stage (kNu), so control changes (smoothness cost, rate limits) stay stage-wise; u_prev of
// stage 0 is the last command sent, fixed. QP control: the stage's control u.
//
// Cost per stage (0.5 * w * r^2 for each residual r):
//  - route, linearized at each stage's projection on the route: contouring (normal offset),
//    lag (along the tangent, against the point one step at the progress speed ahead of the
//    projection, clamped at the goal: a bounded forward pull that tapers with the progress speed
//    and becomes the goal point at the goal),
//    progress (v towards the progress speed, scaled by the heading alignment; the speed tapers
//    linearly over the last kProgressTaperM but never below the linear breakaway
//    (Limits::breakaway) until the goal, so the approach never plans a creep the base would not
//    start from), heading (towards the route tangent with its weight scaled by
//    (1 - closeness)^2, closeness rising linearly from 0 at kGoalHeadingM from the goal point to
//    1 at it; near the goal the goal heading instead, once the measured base is near the goal
//    point: a forward-only base must arrive first and turn in place). The
//    tangent at a projection is the chord over +-kTangentChordM, so it turns continuously
//    through route corners (a vertex's own heading is tie-dependent).
//  - panel reference (weight reference_weight at the stage's route progress): panel position
//    error in base_footprint and the rotation vector log(R_ref^T R);
//  - posture, scaled by (1 - reference weight), by PostureTargets::mode: blend (the recorded B5)
//    (1 - alpha)|q - comfort|^2 + alpha|q - tight|^2, comfort_only |q - comfort|^2, none nothing;
//  - effort (yaw rate and joint velocities) and smoothness (control change);
//  - optional clearance (Weights::clearance, default 0 = off; spec Section 3, amended
//    2026-10-13): for each obstacle row of stages 1..N whose nominal distance d is below
//    clearance_target_m, the Gauss-Newton term 0.5 w (target - (d + gradient * dx))^2. It rewards
//    distance beyond the margin; floor and self rows are not obstacles and get no term. It only
//    adds to the stage's Q and q, so the QP pattern is unchanged.
// Constraints: control boxes (forward-only base); joint position limits; rate limits relative to
// the previous stage's control (stage 0: relative to the last command sent, never the measured
// velocity; yaw uses the smaller accelerating limit both ways); collision rows as slacked
// inequalities d + gradient * dx >= the row's margin (DistanceRow::margin), a fixed number per stage (stages 1..N; stage 0 is
// fixed), unused ones padded with satisfied zero rows.
#include <limits>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_mpc/blocked_detector.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/qp.hpp"
#include "mobile_manipulator_mpc/references.hpp"
#include "mobile_manipulator_mpc/route.hpp"
#include "mobile_manipulator_mpc/solver_config.hpp"

namespace mobile_manipulator_mpc
{
constexpr double kProgressTaperM = 1.0;  // the progress speed tapers linearly over the last metre
constexpr double kGoalHeadingM = 0.5;    // stages this close to the goal turn towards the goal heading,
constexpr double kGoalAlignStartM = 0.25;  // once the measured base is this close to the goal point
constexpr double kGoalAlignFullM = 0.10;   // (fully from here; the goal checker accepts 0.15 m)
constexpr double kTangentChordM = 0.3;   // half chord of the route tangent

struct Plan
{
  std::vector<State> x;    // N + 1 states
  std::vector<Control> u;  // N controls
};

// The base's static friction (operating envelope tracking.breakaway_*): commands below these
// do not start the base from rest. Used by the controller's output stage (Task 9).
struct Breakaway
{
  double linear_mps, yaw_radps, rest_linear_mps = 0.01, rest_yaw_radps = 0.02, dead_linear_mps = 0.005,
                                dead_yaw_radps = 0.01;
};

struct Limits
{
  double v_max, omega_max, accel, decel, yaw_accel, yaw_decel;
  Eigen::Matrix<double, 6, 1> q_min, q_max, qd_max, qdd_max;
  Breakaway breakaway;
};

// Base speeds (the smallest over the envelope's profiles), rates and breakaway from
// nav_operating_envelope.yaml (a file path), arm velocity and acceleration from its
// wholebody_arm block, joint position limits from the URDF (text) intersected with
// wholebody_arm.joint_bounds when present. Joints in the model's order (shoulder_pan ..
// wrist_3). Throws std::runtime_error for a missing field or joint.
Limits load_limits(const std::string & envelope_yaml, const std::string & urdf_text);

struct Weights
{
  double contour, lag, progress, heading, goal_heading, panel_position, panel_orientation, posture, effort_base,
    effort_arm, smooth_base, smooth_arm;
  double slack = 100.0;             // L1 weight of a collision row's slack (QpProblem::slack_weight)
  // L1 weight of the slack of an arm or panel row whose pair is already inside its margin at the nominal (Phase 4B-A
  // Task 10: those rows' slack let the true clearance creep; Task 11: not the base rows); negative: the same as slack.
  double slack_inside = -1.0;
  double slack_quadratic = 1000.0;  // its quadratic weight (QpProblem::slack_quadratic)
  double clearance = 0.0;           // optional clearance cost (0: off, the recorded B5)
  double clearance_target_m = 0.25;  // obstacle rows closer than this get the clearance cost
};

// The slack gate (spec Section 3, amended 2026-10-13; ADR 0011): after each solve, a collision row
// of a stage within the first horizon_s of the plan whose slack exceeds its allowance would push the
// robot into an obstacle: the controller does not execute that plan but holds
// (CycleStatus::Holding). The allowance is tolerance_m for a pair outside the margin and its
// current violation (margin - the pair's distance at the measured state) for a pair inside it, so
// a robot inside the margin may stay or plan its way out but never plan deeper. enabled false is
// the recorded B5 (every plan executed).
struct SlackGate
{
  bool enabled = true;
  double horizon_s = 1.0;
  // While holding, a plan resumes tracking only when it passes the gate over this time (default: the
  // whole 2 s plan). Equal to horizon_s, a plan resumes as soon as its first horizon_s passes.
  double resume_horizon_s = 2.0;
  // ... and keeps passing on every cycle for this time (0: the first passing plan resumes).
  double resume_dwell_s = 0.2;
  double tolerance_m = 0.005;
};

// wholebody_mpc.yaml: horizon, dt_s, the weights, the posture preference's mode and comfort pose,
// the slack gate and the QP solver's tolerance and iteration budget (solver-agnostic numbers; the controller
// passes them to its backend).
struct MpcConfig
{
  int horizon;
  double dt;
  Weights weights;
  PostureMode posture_mode = PostureMode::Blend;
  std::string comfort_pose = "home";  // a name of kComfortPoses (posture_targets resolves it)
  std::vector<std::string> tight_poses = {"vertical_carry"};  // posture.tight_poses (the side picker's poses)
  double solver_eps;    // absolute and relative tolerance
  int solver_max_iter;  // iteration budget per cycle
  double fallback_s;    // failures in a row tolerated before braking, as time (ruling R27)
  SlackGate slack_gate;  // slack_gate, slack_gate_horizon_s, slack_gate_tolerance_m
  CollisionConfig collision;  // collision.base_margin_m, self_rows, self_pairs_per_stage, self_links
  BlockedConfig blocked;      // blocked: detect_s, fail_s, clear_s, progress_m (own_*: the node sets the breakaway)
  // Phase 4B-B switches (spec Section 4), each optional in the YAML with the frozen B5 as its default.
  SolverConfig solver;
  // Phase 4B-B option A (ADR 0014): each streamed arm trajectory starts this long before now, so the JTC commands the
  // plan that far ahead and the arm, which reaches a command about this much later, follows the plan on time
  // (arm_stream_lead_s; 0, start on receipt, in the frozen B5).
  double arm_stream_lead_s = 0.0;
  int qp_iterations = 1;              // 2: a second QP around the first solution, kept after a step check
  double iter2_budget = 0.5;          // of the period: a first solve that used more skips the second
  double iter2_cost_tolerance = 0.01;  // relative: the second iterate may not cost more than this above the first
  bool rti_split = false;             // the preparation/feedback split
  RtiDeviation rti_max_deviation;
};
MpcConfig load_mpc_config(const std::string & yaml_path);

// The side picker's choice (SidePicker below): which tight pose the posture blend pulls towards.
enum class PostureChoice { None, Comfort, Tight0, Tight1 };
// "none", "comfort", "left", "right" (Tight0/Tight1 in the configured order [vertical_carry, vertical_carry_right]).
const char * to_string(PostureChoice choice);

struct Inputs
{
  State x0;              // measured state
  Control last_command;  // the command sent last cycle (the stage-0 rate limits are relative to it)
  const Route * route;
  double goal_s;    // arc length of the goal on the route
  double goal_yaw;  // rad
  std::vector<PanelReference> references;  // empty: unscripted
  PostureTargets posture;
  double tightness;  // alpha in [0, 1]: 0 prefers the comfort pose, 1 the tight posture (blend only)
  // Which tight pose the blend pulls towards (the side picker): Tight0/Tight1 index posture.tights (posture.tight
  // when it has no such entry); Comfort or None keep the comfort pose (Phase 4B-A spec Section 3).
  PostureChoice posture_choice = PostureChoice::Tight0;
  // Route progress of the measured state in the previous cycle on this route: stage 0 is projected
  // within the stage window around it, so a route that passes near itself is followed on the
  // branch the robot is on (review R31). NaN (a new goal or a new route): the nearest point of the
  // whole route.
  double s0_hint = std::numeric_limits<double>::quiet_NaN();
};

// Control boxes (forward-only base) and the change allowed from one control to the next over dt
// (linear accel/decel, yaw symmetric with the smaller accelerating limit, arm joints by their
// acceleration limit).
Control lower_control(const Limits & l);
Control upper_control(const Limits & l);
Control lower_rate(const Limits & l, double dt);
Control upper_rate(const Limits & l, double dt);

constexpr double kTightComfortM = 0.30;  // comfort-pose clearance above which alpha is 0

// The side picker (Phase 4B-A spec Section 3): which tight pose the posture blend pulls towards. Below the
// release level kPickAlphaOff nothing is chosen; from kPickAlphaOn the poses are scored once at the first
// conflict stage k* (the first stage where the comfort pose's clearance is below kTightComfortM) by their
// minimum clearance over k*-1..k*+1; a tight pose is eligible only if it beats the comfort pose by more than
// kPickMarginM (else the comfort pose stays: a low beam), the larger eligible score wins by more than
// kPickMarginM, a tie goes to the side the arm pans towards (|pan| > kPickPanRad), else the route's turn
// side, else Tight0 (left). Left and right are latched until alpha falls below kPickAlphaOff; Comfort stays open
// to a tight pose while alpha stays up (a low beam, then a gate: no side is crossed; Task 8 review), but a switch
// from Comfort also needs the rigid comfort pose to collide at its worst stage (a gate narrower than the panel, not
// a low beam it clears) and the tight pose to beat it by kPickMarginM there (under a low beam a vertical pose
// scores better on the approach and hits the beam further on; Task 10 and its review).
// With both tight poses eligible a route probe decides past the horizon (Phase 4B-A Task 11, user decision
// 2026-10-19: v2_room_r4_nav tied in the window at the hallway's right turn, took the turn side and kept it
// through the left turn into R4). Its tight segment runs from the first sample where the comfort pose is below
// kTightComfortM to the last, bridging clear gaps of up to RouteProbe::bridge_m (the choice stays latched over
// them); samples where neither tight pose keeps the margin and the comfort pose does better than both (a beam) are
// left to the collision rows. Each pose's window score capped by its segment minimum decides by more than kPickMarginM; if those tie,
// the window scores; then the segment means; then the pan and turn tie-breaks. With one eligible pose the route
// is not probed (it cannot change the choice, also where that pose collides later on: a deliberate limit).
constexpr double kPickAlphaOn = 0.05, kPickAlphaOff = 0.02, kPickMarginM = 0.02, kPickPanRad = 0.1;
constexpr double kPickTurnDeadbandRad = 0.05;  // the nominal plan's heading change below this counts as straight
constexpr double kRouteProbeM = 6.0, kRouteProbeStepM = 0.1;  // the route probe: length ahead and sample spacing

// Clearances of the rigid poses held on the route (base on it, heading along it) every step_m from the current
// progress (CollisionModel::clearance_per_stage of those states, the obstacles where they are now): the comfort
// pose and each tight pose.
struct RouteProbe
{
  std::vector<double> comfort;
  std::vector<std::vector<double>> tights;
  double step_m = kRouteProbeStepM;
  double bridge_m = 0.0;   // clear gaps shorter than this stay in the segment (the horizon's travel)
  double margin_m = 0.05;  // the collision margin (samples no tight pose keeps it at are skipped)
};

class SidePicker
{
public:
  // `comfort` and each of `tight` per stage (CollisionModel::clearance_per_stage), probed only when
  // needs_probe(alpha) (empty otherwise); `shoulder_pan` the measured pan; `route_turn_sign` > 0 for a left
  // turn of the route over the horizon, < 0 right, 0 straight; `route_probe` computes the route probe of the same
  // cycle, called only when both tight poses are eligible (none, or null: the horizon window decides alone).
  PostureChoice update(double alpha, const std::vector<double> & comfort, const std::vector<std::vector<double>> & tight,
                       double shoulder_pan, double route_turn_sign,
                       const std::function<const RouteProbe *()> & route_probe = nullptr);
  // The scores of the last choosing cycle: comfort and each tight pose in the first-conflict window, then (when both
  // were eligible and the route's tight segment had counted samples) each tight pose's capped score and segment mean
  // (empty before a choice).
  bool needs_probe(double alpha) const
  {
    return (choice_ == PostureChoice::None || choice_ == PostureChoice::Comfort) && alpha >= kPickAlphaOn - 1e-12;
  }
  PostureChoice choice() const { return choice_; }
  const std::vector<double> & scores() const { return scores_; }

private:
  PostureChoice choice_ = PostureChoice::None;
  std::vector<double> scores_;
};
constexpr double kTightTauS = 1.0;       // time constant of the tightness low-pass

// alpha from the clearance the arm and panel would have in the comfort pose over the horizon: 0 above
// comfort_m, 1 at margin_m (the collision margin), linear in between, then a first-order
// low-pass with time constant tau_s (exact discretization), starting at 0.
class TightnessFilter
{
public:
  TightnessFilter(double comfort_m = kTightComfortM, double margin_m = CollisionConfig{}.margin_m,
                  double tau_s = kTightTauS);
  double update(double home_clearance_m, double dt);
  double value() const { return alpha_; }

private:
  double comfort_m_, margin_m_, tau_s_, alpha_ = 0.0;
};

class Formulation
{
public:
  // `collision` gives the margin and the per-stage row budget (rows beyond it are an error).
  Formulation(int horizon, double dt, Limits limits, Weights weights, CollisionConfig collision = {});

  // The QP in the deviations (dz, du) from `nominal` (N + 1 states, N controls). `rows` from
  // CollisionModel::rows over nominal.x; stage-0 rows are ignored. Every call gives the same
  // dimensions and the same finite/infinite bound pattern, so a solver set up with the first
  // problem can be reused. Throws std::invalid_argument for a nominal of the wrong length, a
  // null route or more rows in a stage than the budget.
  QpProblem build(const Inputs & in, const Plan & nominal, const std::vector<DistanceRow> & rows) const;

  // The cost build() linearizes, evaluated at `plan` (Phase 4B-B second QP iteration's step check): the sum of
  // 0.5 w r^2 over the same residuals (route, heading, panel reference, posture, progress, effort, smoothness), without
  // the slack and the optional clearance terms. The route projections, references and the target speed come from
  // `anchor` (the plan the QP was built around; nullptr: `plan` itself), as build() holds them fixed, while the
  // residuals are evaluated at `plan`. Throws std::invalid_argument for a null route or a plan of the wrong length.
  double objective(const Inputs & in, const Plan & plan, const Plan * anchor = nullptr) const;

  // Route progress of the measured state in.x0 as build() uses it for stage 0: the nearest route
  // point within the stage window around in.s0_hint, or of the whole route when the hint is NaN.
  // Throws std::invalid_argument for a null route.
  double stage0_progress(const Inputs & in) const;
  // Arc length searched around the previous stage's (or cycle's) progress: max(1 m, 3 v_max dt).
  double progress_window() const;

  // nominal + solution: the controls, projected onto the control boxes and (from stage 1 on) the
  // rate limits relative to the previous stage, which removes the solver's tolerance-sized
  // violations; then rolled out with the model from nominal.x[0] + dz_0.
  Plan apply(const Plan & nominal, const QpSolution & s) const;
  // The same, in place (no allocation for a plan of the horizon's size): the controls of `plan`
  // projected and rolled out from plan.x[0] + dz_0.
  void apply_in_place(Plan & plan, const QpSolution & s) const;
  // The plan one cycle later: controls shifted by one (the last repeated), rolled out from x0.
  Plan shift(const Plan & plan, const State & x0) const;
  // In place: controls shifted by `stages` (clamped to [0, N]; the last repeated), rolled out
  // from x0. Throws std::invalid_argument for a plan of the wrong length.
  void advance(Plan & plan, const State & x0, int stages) const;
  // Cold start: zero controls, every state x0.
  Plan hold(const State & x0) const;

  // Model rollout of `u` from x0.
  Plan rollout(const State & x0, const std::vector<Control> & u) const;

  int horizon() const { return horizon_; }
  double dt() const { return dt_; }
  const Limits & limits() const { return limits_; }
  const Weights & weights() const { return weights_; }
  int rows_per_stage() const { return row_budget(collision_); }

  // Index layout of the QP state z = (x, u_prev).
  static constexpr int kNz = kNx + kNu;

private:
  int horizon_;
  double dt_;
  Limits limits_;
  Weights weights_;
  CollisionConfig collision_;
};
}  // namespace mobile_manipulator_mpc
