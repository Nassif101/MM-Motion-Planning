#pragma once
// The medium-horizon planner (Phase 5A spec Section 3.5): multi-start SCP from seeds, the merge of seeds that end in the
// same solution, the choice of the cheapest feasible one, the homotopy hysteresis, the solve cap, and the plan as
// samples by arc length for B5 (MediumPlan). ROS-free; medium_horizon_node and the benches drive it.
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
  int iterations = 0;
  bool converged = false, feasible = false;
  bool merged = false;  // ended within kMergeTolerance of an earlier seed's solution
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

class MediumPlanner
{
public:
  // `seed_poses` by name (the qualified poses; every name of the config's seeds but "previous" must be there, else
  // std::invalid_argument); `panel_half_size` the payload box's half extents (the "panel" frame's axes).
  MediumPlanner(MediumProblem problem, std::unique_ptr<DenseQpSolver> solver, std::map<std::string, Joints> seed_poses,
                Eigen::Vector3d panel_half_size);

  // A route (copied) and goal; the next plan is a full multi-start. The same goal as before is a replan of the same
  // task (B5 announces every one): the published plan stays the incumbent. A new goal starts over.
  void set_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal, uint64_t route_id);
  bool has_route() const { return route_.has_value(); }
  void request_full_multistart() { full_requested_ = true; }
  void set_solve_cap_s(double seconds) { cap_s_ = seconds; }
  const MediumProblem & problem() const { return problem_; }
  // The solution the last plan published (none before the first plan or after a new route).
  const Trajectory * last_trajectory() const { return last_ ? &*last_ : nullptr; }

  // One cycle at `now` (seconds) from the measured state and its rate (d/dt of x, y, yaw and the six joints).
  // Throws std::logic_error without a route.
  const PlanOutput & plan(double now, const mmm::State & measured, const Eigen::Matrix<double, 9, 1> & rate,
                          const mmm::World & world);

  std::string homotopy_label(const Trajectory & t) const;
  double panel_top(const mmm::State & x) const;

private:
  // The least-squares trajectory through the start state closest to `targets` (samples x coordinates) at the samples.
  Trajectory fit(const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate, const Eigen::MatrixXd & targets) const;
  Trajectory pose_seed(const Joints & pose, const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate,
                       const RouteTask & task) const;
  Trajectory shifted_previous(double now, const mmm::State & start, const Eigen::Matrix<double, 9, 1> & rate) const;
  std::vector<PlanSample> samples_of(const Trajectory & t) const;

  MediumProblem problem_;
  std::unique_ptr<DenseQpSolver> solver_;
  std::map<std::string, Joints> seed_poses_;
  Eigen::Vector3d panel_half_;
  double comfort_top_;
  Eigen::MatrixXd fit_;  // free_parameters x samples: the least-squares map of the sample values
  double cap_s_;

  std::optional<mmm::Route> route_;
  mobile_manipulator_navigation::Pose2 goal_{};
  uint64_t route_id_ = 0, plan_id_ = 0;
  bool new_route_ = false, full_requested_ = false;
  double s0_hint_ = 0.0, last_full_ = -1e300;
  std::optional<Trajectory> last_;  // the published solution and when it was planned
  double last_now_ = 0.0;
  std::string incumbent_;
  PlanOutput out_;
};
}  // namespace mobile_manipulator_medium
