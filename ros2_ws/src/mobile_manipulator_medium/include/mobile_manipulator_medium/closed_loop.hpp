#pragma once
// The closed-loop kinematic bench (Phase 5A spec Section 6.2, plan Task 12): B5's controller at its rate on the
// kinematic plant (ideal actuators with the base's static friction), the medium planner at its rate from the plant's
// state, each plan visible to B5 (MediumSwitch, reference_source medium) at its start time plus its measured compute
// time rounded up to the next B5 cycle. Without the medium layer the same loop is the frozen B5 on the plant.
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_medium/planner.hpp"
#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/references.hpp"

namespace mobile_manipulator_medium
{
struct ClosedLoopOptions
{
  double b5_rate_hz = 50.0;
  double timeout_s = 120.0;
  bool medium = true;  // false: B5 alone on the same plant
  // A plan's latency (s, rounded up to the next B5 cycle) instead of its measured compute time; NaN: measured. With
  // a planner cap above every cycle's compute the loop is then independent of the host's load (the gtests).
  double fixed_latency_s = std::numeric_limits<double>::quiet_NaN();
  // Called with every medium plan, its cycle time and the plant's state (the bench's --trace).
  std::function<void(double, const mobile_manipulator_mpc::State &, const PlanOutput &)> on_plan;
};

// The swing sides of a run's plans (homotopy labels): the first, the sequence of side changes and the switches after
// the first (a plan without a side, comfort or lowered, keeps the last side: right -> lowered -> left is a switch).
struct SideTracker
{
  std::string first, last;
  std::vector<std::string> sequence;
  int switches = 0;
  void observe(const std::string & homotopy);
};

struct ClosedLoopResult
{
  bool reached = false;
  std::string end_status;  // B5's last status (goal_reached, failed, ...) or "timeout"
  double time_s = 0.0;
  mmm::State final_state = mmm::State::Zero();
  int b5_cycles = 0, following_cycles = 0;
  std::map<std::string, int> fallbacks;  // B5 cycles by fallback reason (MediumSwitch)
  // B5 cycles that executed a plan (Ok) whose collision slack exceeded 1 mm, and the largest such slack
  int executed_slack_plans = 0;
  double max_executed_slack_m = 0.0;
  double min_clearance_m = std::numeric_limits<double>::infinity();  // true 3D, every primitive and box
  double max_torque_ratio = 0.0;  // largest |static torque| / limit over the plant's joints
  int medium_cycles = 0, medium_capped = 0, medium_infeasible = 0;
  std::vector<double> medium_solve_s;
  std::string first_side;  // the first swing side a plan chose ("" if none)
  int homotopy_switches_after_first = 0;  // SideTracker::switches
  std::vector<std::string> side_sequence;
  // B5 cycles that followed a medium plan published as infeasible (spec 3.5: B5 takes it, its own rows still hold), and
  // how many such plans it followed
  int followed_infeasible_cycles = 0, followed_infeasible_plans = 0;
  std::string fallback_summary() const;
};

// `b5` built for b5_config at o.b5_rate_hz (replay.hpp build_controller), `medium` the planner (its config's rate),
// `posture` B5's posture targets, `primitives` the robot's (true clearance). Starts the goal on both.
ClosedLoopResult run_closed_loop(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                 const mmm::PostureTargets & posture, const mmm::Route & route,
                                 const mobile_manipulator_navigation::Pose2 & goal, const mmm::State & start,
                                 const mmm::World & world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                 const ClosedLoopOptions & options);
}  // namespace mobile_manipulator_medium
