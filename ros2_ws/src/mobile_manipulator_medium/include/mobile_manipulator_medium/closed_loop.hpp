#pragma once
// The closed-loop kinematic bench (Phase 5A spec Section 6.2, plan Task 12): B5's controller at its rate on the
// kinematic plant (ideal actuators with the base's static friction), the medium planner at its rate from the plant's
// state, each plan visible to B5 (MediumSwitch, reference_source medium) at its start time plus its measured compute
// time rounded up to the next B5 cycle. Without the medium layer the same loop is the frozen B5 on the plant.
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_medium/planner.hpp"
#include "mobile_manipulator_mpc/closed_loop_plant.hpp"
#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/medium_switch.hpp"
#include "mobile_manipulator_mpc/statics.hpp"
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
  // Called after every B5 cycle's plant step with the cycle's time and the plant's new state (the MuJoCo plant's
  // equivalence and determinism checks, its run log).
  std::function<void(double, const mobile_manipulator_mpc::State &)> on_cycle;
  // Nav2's replans of the same goal (0: none): every this many seconds B5 takes a new route from the plant's position
  // along the rest of the route (update_goal) and announces it with the next route_id, as the live node does.
  double route_update_period_s = 0.0;
  // The first cycle's time: a mission's next goal continues the plant's clock (its commands are stamped with it); the
  // timeout counts from it.
  double start_time_s = 0.0;
  // Last command-boundary check. False (or an exception) replaces any proposed output with a
  // controlled base/arm hold; the held command still advances the plant and simulation clock.
  std::function<bool(double, const mmm::State &, const mmm::CycleOutput &)> command_guard;
};

// The swing sides of a run's ok plans (homotopy labels): the first, the sequence of side changes and the switches after
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
  int b5_cycles = 0, following_cycles = 0, route_updates = 0;
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
  // The plant's ground truth, when it has one (the MuJoCo plant): the smallest true clearance and the B5 cycles that
  // ended with a contact.
  double plant_min_clearance_m = std::numeric_limits<double>::infinity();
  int plant_contacts = 0;
  std::string fallback_summary() const;
};

// The loop one B5 cycle at a time (the MuJoCo plant's session, plan Task 8): the constructor starts the goal on both
// (as run_closed_loop), step() runs one cycle and returns false once the run has ended (goal reached, failed or timed
// out; the cycle that ends it is counted), result() is the run so far. run_closed_loop is this stepped to the end.
class ClosedLoopStepper
{
public:
  ClosedLoopStepper(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                    const mmm::PostureTargets & posture, const mmm::Route & route,
                    const mobile_manipulator_navigation::Pose2 & goal, mmm::ClosedLoopPlant & plant, mmm::World world,
                    const std::vector<mobile_manipulator_geometry::Primitive> & primitives, const ClosedLoopOptions & options);
  bool step();
  // Spec 2 Task 7: install an accepted route without recreating the plant/controller/executor, at the next cycle's
  // time(). Same-goal updates (the goal still running; route_id above the current one, else std::invalid_argument)
  // keep the goal's statistics, deadline and pending medium deliveries, as a route update does. A new goal rearms a
  // terminal executor and resets everything goal-local (result, deadline, medium deliveries and incumbent, switch,
  // side tracking); the plant, B5's command/tracking history and the clock continue. A finite absolute deadline
  // (the goal manager's: it includes time already spent planning) replaces timeout_s from now.
  void adopt_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal,
                   uint64_t route_id, bool new_goal,
                   double absolute_deadline = std::numeric_limits<double>::quiet_NaN());
  // One solver-free holding tick (planning, idle, or after the goal ended): the base brakes at the envelope's limits
  // from the command sent last, the arm's velocities ramp to zero (B5's braking hold), status Holding for the plant
  // (the realistic arm path sends its one braking hold). B5 records the sent command (record_external_command). The
  // goal's result is unchanged; time(), the plant and the last-cycle diagnostics advance. Returns true.
  bool step_holding();
  bool holding() const { return holding_; }  // the last tick sent the executor's hold (step_holding or the guard)
  uint64_t route_id() const { return route_id_; }
  bool done() const { return done_; }
  bool timed_out() const { return timed_out_; }  // ended by the timeout (result().end_status keeps B5's last status)
  double time() const { return o_.start_time_s + static_cast<double>(k_) * dt_; }  // the next cycle's time
  const ClosedLoopResult & result() const { return r_; }
  // The last cycle's B5 status and, with the medium layer, its fallback reason ("" while following a plan).
  mmm::CycleStatus last_status() const { return last_status_; }
  const std::string & last_fallback() const { return last_fallback_; }
  // The last B5 cycle's slack gate (CycleOutput's gate_slack, gate_violation, gate_unsafe): diagnostics only.
  struct Gate
  {
    double slack = 0.0, violation = 0.0;
    bool unsafe = false;
    bool dwell = false;  // holding with a passing plan inside the resume dwell (E1's decision trace)
    // E1 Task 9: the deepest gated pair's violation, its stage-1 prediction, the plan's linearization error (NaN
    // without B5's diagnostics or without a solve)
    double depth = 0.0, depth_stage1 = 0.0, lin_error = std::numeric_limits<double>::quiet_NaN();
    bool guard_tripped = false;  // E1 CP3: the measured-depth guard held
    double guard_depth = 0.0;
    // E2 Task 1 (B5's diagnostics only, empty / zero otherwise): the deepest pair and its distance gradient
    std::string pair;
    Eigen::Matrix<double, 9, 1> gradient = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> kink = Eigen::Matrix<double, 9, 1>::Zero();  // forward minus backward slope
    // E2 Task 4b, the nearest-pair probe (CycleOutput::probe_*)
    std::string probe_link, probe_obstacle;
    double probe_distance = 0.0;
    bool probe_in_stage1 = false, probe_in_rows = false;
    int probe_dropped = 0;
    // E2 Task 7: the tracking model the cycle planned with, g_inf, g_0, sideslip k (CycleOutput::tracking)
    Eigen::Vector3d tracking = Eigen::Vector3d(1.0, 1.0, 0.0);
  };
  const Gate & last_gate() const { return last_gate_; }
  // The command B5 sent the plant in the last stepped cycle (zero before the first): E2's diagnostics.
  const mmm::Control & last_command() const { return last_command_; }
  // E2 Task 9 (T7): the last B5 cycle's timing and the B5 cycles stepped so far
  const mmm::CycleTiming & last_b5_timing() const { return last_b5_timing_; }
  long b5_cycles_stepped() const { return b5_cycles_stepped_; }
  // The first control of B5's plan in that cycle (before the output stage: rate band, boxes, breakaway boost; zero
  // without a plan): E2's diagnostics.
  const mmm::Control & last_planned() const { return last_planned_; }
  // E3 Task 1: the last B5 cycle's row trace (CycleOutput::row_trace, row_trace_x, row_trace_u; empty when untraced)
  const std::vector<mmm::RowTrace> & last_row_trace() const { return last_row_trace_; }
  const std::vector<mmm::State> & last_row_trace_x() const { return last_row_trace_x_; }
  const std::vector<mmm::Control> & last_row_trace_u() const { return last_row_trace_u_; }
  const mmm::World & world() const { return world_; }

private:
  void finish();
  // The executor's hold for this tick from the command sent last (starts the arm's ramp on the first held tick).
  mmm::CycleOutput holding_output(const mmm::CycleStatus status, const mmm::TrackingModel & model);
  void clear_cycle_diagnostics();

  struct Delivery
  {
    double at;
    std::vector<mmm::MediumSample> samples;
    double splice_s, stamp;
    uint64_t plan_id, route_id;
    std::optional<bool> feasible = true;  // the status is not infeasible; nullopt for stale_reused (E1)
  };
  mmm::Controller & b5_;
  MediumPlanner & medium_;
  mmm::MpcConfig b5_config_;
  mmm::PostureTargets posture_;
  mobile_manipulator_navigation::Pose2 goal_;
  mmm::ClosedLoopPlant & plant_;
  mmm::World world_;
  ClosedLoopOptions o_;
  double dt_, medium_period_;
  Joints torque_limits_;
  mmm::CollisionModel clearance_;
  mmm::GeneratedStaticTorque statics_;
  mmm::MediumSwitch sw_;
  std::deque<Delivery> pending_;
  std::map<uint64_t, bool> infeasible_;  // plan_id -> published infeasible
  std::set<uint64_t> followed_infeasible_;
  SideTracker sides_;
  mmm::Route active_;  // B5's Nav2 route: the given one, then each replan's
  double active_goal_s_;
  uint64_t route_id_ = 1;
  double next_medium_, next_update_;
  double goal_started_at_ = 0.0, absolute_deadline_ = std::numeric_limits<double>::quiet_NaN();
  bool holding_ = false;
  Joints hold_v0_ = Joints::Zero();  // the arm velocities sent when the hold started
  double hold_duration_s_ = 0.0, hold_elapsed_s_ = 0.0;
  mmm::TrackingModel last_model_{};  // the last B5 cycle's tracking model (the realistic arm hold's base braking)
  long k_ = 0;
  bool done_ = false, timed_out_ = false;
  mmm::CycleStatus last_status_ = mmm::CycleStatus::Holding;
  std::string last_fallback_;
  Gate last_gate_;
  mmm::Control last_command_ = mmm::Control::Zero();
  mmm::CycleTiming last_b5_timing_;
  long b5_cycles_stepped_ = 0;
  mmm::Control last_planned_ = mmm::Control::Zero();
  std::vector<mmm::RowTrace> last_row_trace_;
  std::vector<mmm::State> last_row_trace_x_;
  std::vector<mmm::Control> last_row_trace_u_;
  ClosedLoopResult r_;
};

// `b5` built for b5_config at o.b5_rate_hz (replay.hpp build_controller), `medium` the planner (its config's rate),
// `posture` B5's posture targets, `primitives` the robot's (true clearance). Starts the goal on both.
ClosedLoopResult run_closed_loop(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                 const mmm::PostureTargets & posture, const mmm::Route & route,
                                 const mobile_manipulator_navigation::Pose2 & goal, const mmm::State & start,
                                 const mmm::World & world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                 const ClosedLoopOptions & options);

// The same on any plant (its state at the start is the run's start); the world gets the plant's movers every cycle.
ClosedLoopResult run_closed_loop(mmm::Controller & b5, MediumPlanner & medium, const mmm::MpcConfig & b5_config,
                                 const mmm::PostureTargets & posture, const mmm::Route & route,
                                 const mobile_manipulator_navigation::Pose2 & goal, mmm::ClosedLoopPlant & plant,
                                 mmm::World world, const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                                 const ClosedLoopOptions & options);
}  // namespace mobile_manipulator_medium
