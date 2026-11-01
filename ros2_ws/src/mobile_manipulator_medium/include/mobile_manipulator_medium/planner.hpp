#pragma once
// The medium-horizon planner (Phase 5A spec Section 3.5): multi-start SCP from seeds, the merge of seeds that end in the
// same solution, the choice of the cheapest feasible one, the homotopy hysteresis, the solve cap, and the plan as
// samples by arc length for B5 (MediumPlan). ROS-free; medium_horizon_node and the benches drive it.
#include <functional>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_medium/dense_qp.hpp"
#include "mobile_manipulator_medium/problem.hpp"
#include "mobile_manipulator_medium/scp.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mobile_manipulator_medium
{
struct PlanSample
{
  double s, x, y, yaw, speed;  // arc length along the planned path from its first sample; map pose; speed (m/s)
  Joints q;                    // the planned joints; the others at the comfort pose
};

struct SeedStats
{
  std::string name, homotopy;
  double merit = 0.0, slack = 0.0, solve_s = 0.0;
  double progress_m = 0.0;  // how far along the route the solution ends (MediumPlanner::progress)
  int iterations = 0;
  bool converged = false, feasible = false;
  bool merged = false;  // ended within kMergeTolerance of an earlier seed's solution
  bool resumed = false;  // went on from its last turn's iterate (the cap cut that turn short)
};

struct PlanOutput
{
  uint64_t route_id = 0, plan_id = 0;
  std::string status;  // ok, infeasible, stale_reused
  double planned_at = 0.0;  // the cycle time the samples were planned at (a stale_reused plan keeps its own)
  std::string seed, homotopy;
  double cost = 0.0, solve_s = 0.0, linearize_s = 0.0, qp_s = 0.0, merit_s = 0.0, splice_s = 0.0;
  LinearizeTiming linearize_parts;  // summed over the seeds
  bool full_multistart = false, capped = false;
  bool homotopy_switched = false;  // the swing side flipped (left <-> right) against the last published plan
  std::vector<PlanSample> samples;
  std::vector<SeedStats> seeds;
};

// The homotopy of a plan: "left" / "right" when shoulder_pan leaves +-kPanSideRad, "lowered" when the panel top drops
// more than kLoweredM below the comfort pose's, joined with "+"; otherwise "comfort".
constexpr double kPanSideRad = 0.5, kLoweredM = 0.03;
// Seeds whose solutions' control points differ by less than this (max-norm, m or rad) are one solution.
constexpr double kMergeTolerance = 0.02;

// A heading error beyond this stops the base to turn in place (the seeds' pacing).
constexpr double kTurnInPlaceRad = 0.5;

// The base along `route` as the skid-steer drives it (the seeds' base, Task 9 CP: a constant speed along the route's
// polyline asks for instant turns at its corners, and the SCP rarely recovers from that): from `start` (position, yaw)
// at forward speed `v_start` and yaw rate `w_start` (the measured ones, which the fit pins), the yaw follows the route's heading within omega_max and yaw_accel; the speed, at most
// `v_ref(s)` (at arc length s) and changed within accel and decel, falls with the heading error to a stop beyond kTurnInPlaceRad, brakes
// ahead of the next such corner and stops at `goal_s`. The position is the route's at the arc length reached (from
// `s0`), blended from the start position over the first second. One pose (x, y, yaw; the yaw continuous) per entry of
// `times` (s from the start, increasing). Until `hold_s` the speed reference is 0 (the base stops and may turn: a seed
// that reconfigures in place first).
std::vector<Eigen::Vector3d> paced_route(const mmm::Route & route, double s0, double goal_s, const mmm::State & start,
                                         double v_start, double w_start, const std::function<double(double)> & v_ref,
                                         const mmm::Limits & limits, const std::vector<double> & times,
                                         double hold_s = 0.0);

class MediumPlanner
{
public:
  // `seed_poses` by name (the qualified poses; every name of the config's seeds but "previous" must be one, or one with
  // planned joints offset, seed_pose, else std::invalid_argument); `panel_half_size` the payload box's half extents (the "panel" frame's axes).
  MediumPlanner(MediumProblem problem, std::unique_ptr<DenseQpSolver> solver, std::map<std::string, Joints> seed_poses,
                Eigen::Vector3d panel_half_size);

  // A route (copied) and goal; the next plan is a full multi-start. The same goal as before is a replan of the same
  // task (B5 announces every one): the published plan stays the incumbent. A new goal starts over.
  void set_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal, uint64_t route_id);
  bool has_route() const { return route_.has_value(); }
  void request_full_multistart() { full_requested_ = true; }
  void set_solve_cap_s(double seconds) { cap_s_ = seconds; }
  // SCP iterations per cycle over all its seeds (0: none), cutting the cycle as the cap does: a cap the same on any
  // host (the benches' reproducible stand-in for the cap under load).
  void set_iteration_budget(int iterations) { iteration_budget_ = iterations; }
  const MediumProblem & problem() const { return problem_; }
  // The solution the last plan published (none before the first plan or after a new route).
  const Trajectory * last_trajectory() const { return last_ ? &*last_ : nullptr; }

  // One cycle at `now` (seconds) from the measured state and its rate (d/dt of x, y, yaw and the six joints).
  // Throws std::logic_error without a route.
  const PlanOutput & plan(double now, const mmm::State & measured, const Eigen::Matrix<double, 9, 1> & rate,
                          const mmm::World & world);

  // A seed's arm pose: a qualified pose by name, or "pose+joint:offset[+joint:offset...]" (rad) with each joint one the
  // plan moves (the 6-joint mask's wrist seeds: the panel rolled or pitched; seeds only, nothing qualified).
  // Throws std::invalid_argument for an unknown pose or joint, a joint not planned or a malformed offset.
  Joints seed_pose(const std::string & name) const;

  std::string homotopy_label(const Trajectory & t) const;
  double panel_top(const mmm::State & x) const;

private:
  // The least-squares trajectory through the start state closest to `targets` (samples x coordinates) at the samples.
  Trajectory fit(const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate, const Eigen::MatrixXd & targets) const;
  // `hold_first`: the base holds while the joints reach the pose at half their speed limits, then drives (a seed from a
  // standstill in front of a tight place); otherwise the joints reach it over the first half of the horizon while the
  // base drives.
  Trajectory pose_seed(const Joints & pose, const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate,
                       const RouteTask & task, bool hold_first = false) const;
  // `t` (planned at `planned_at`) at now, held at its end beyond the horizon, through the measured start.
  Trajectory shifted(const Trajectory & t, double planned_at, double now, const mmm::State & start,
                     const Eigen::Matrix<double, 9, 1> & rate) const;
  std::vector<PlanSample> samples_of(const Trajectory & t) const;
  // A plan moves when its path covers at least `share` of min(the reach at the reference speed over the horizon, the
  // route left), or the goal is within 0.1 m (0.5: tools/medium_bench.py progressing).
  bool moving(const std::vector<PlanSample> & samples, const RouteTask & task, double share = 0.5) const;
  // A plan waits when it does not move and is not short for a turn in place (the route's heading more than
  // kTurnInPlaceRad from its yaw within the distance moving asks for, where paced_route stops to turn).
  bool waits(const std::vector<PlanSample> & samples, const RouteTask & task, double share = 0.5) const;
  // How far along the route (from task.s0) the trajectory ends.
  double progress(const Trajectory & t, const RouteTask & task) const;

  MediumProblem problem_;
  std::unique_ptr<DenseQpSolver> solver_;
  std::map<std::string, Joints> seed_poses_;
  Eigen::Vector3d panel_half_;
  double comfort_top_;
  Eigen::MatrixXd fit_;  // free_parameters x samples: the least-squares map of the sample values
  double cap_s_;
  int iteration_budget_ = 0;

  std::optional<mmm::Route> route_;
  mobile_manipulator_navigation::Pose2 goal_{};
  uint64_t route_id_ = 0, plan_id_ = 0;
  bool new_route_ = false, full_requested_ = false;
  double s0_hint_ = 0.0, last_full_ = -1e300;
  size_t rotation_ = 0;  // the next pose seed in turn (seeds_per_cycle)
  std::optional<Trajectory> last_;  // the published solution and when it was planned
  double last_now_ = 0.0;
  std::string incumbent_;
  bool waiting_ = false;  // the last published ok plan waits (waits)
  std::string last_side_;  // the side of the last published plan that had one ("left", "right")
  // The pose seeds the cap cut short, their last iterate and when: the next turn goes on from it (within kResumeMaxAgeS).
  std::map<std::string, std::pair<Trajectory, double>> resume_;
  PlanOutput out_;
};
}  // namespace mobile_manipulator_medium
