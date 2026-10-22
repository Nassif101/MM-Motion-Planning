#pragma once
// The replay bench (Phase 4B-B spec Section 3): a recorded B5 run replayed open-loop through a controller built
// from any configuration. The robot follows the recorded states, so the bench measures computation and the plan
// at the recorded state, not closed-loop outcomes. No ROS here (replay_bag.hpp reads a recording from a bag).
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/scene.hpp"

namespace mobile_manipulator_mpc
{
// One recorded cycle. stamp: the controller's `now` (the cycle's header stamp); bag_time: the cycle's state time
// on the recording's clock (its sent command's receipt less state_to_command_s). x: the state the node gave the
// controller (the plan's stage 0). executed and arm_v: the previous command the node passed (executed base command,
// streamed reference velocity, unclamped). odom_*: the base's measured speeds (base at rest).
struct RecordedCycle
{
  double stamp = 0.0, bag_time = 0.0, state_to_command_s = 0.0;
  State x = State::Zero();
  std::string status, posture_choice;
  int iterations = 0;
  bool deadline_missed = false;
  Eigen::Vector2d executed = Eigen::Vector2d::Zero();
  Eigen::Matrix<double, 6, 1> arm_v = Eigen::Matrix<double, 6, 1>::Zero();
  double odom_v = 0.0, odom_omega = 0.0;
  std::vector<State> plan;  // the recorded plan (empty if none was recorded)
};

// A FollowPath goal as /plan recorded it (the route's points, the goal pose at its last point).
struct RecordedGoal
{
  double bag_time = 0.0;
  std::vector<Eigen::Vector2d> points;
  mobile_manipulator_navigation::Pose2 goal{};
};

// A /scenario/movers sample: positions in the scene's mover order (by name).
struct MoverSample
{
  double bag_time = 0.0, stamp = 0.0;
  std::vector<Eigen::Vector2d> positions;
};

struct Recording
{
  std::vector<RecordedCycle> cycles;
  std::vector<RecordedGoal> goals;
  std::vector<MoverSample> movers;
  double period_s = 0.0;  // the median spacing of the cycle stamps
};

struct ReplaySetup
{
  World world;                   // the known boxes (movers are added from the recording)
  std::vector<MoverSpec> movers;  // the scene's movers, by name
  PostureTargets posture;
  // The panel references of a route (scripted runs); empty: unscripted.
  std::function<std::vector<PanelReference>(const Route &)> references;
  Limits limits;
};

struct ReplayOptions
{
  double rate_hz = 0.0;          // 0: the recorded rate; else due times every 1/rate_hz take the next recorded cycle
  bool recorded_misses = false;  // note_failure on the recorded misses (fidelity) instead of the replay's own
  int clearance_stages = 10;     // the true clearance over the plan's stages 1..this
};

// Against the recording, at the recorded rate: the fraction of cycles with the recorded status; of the cycles
// recorded ok, those replayed ok with stage 1 within 1 mm (base) and 1 mrad (yaw, joints); of the cycles with
// recorded iterations, those within 10 % (at least 1).
struct Fidelity
{
  double status_match = 0.0, stage1_match = 0.0, iterations_match = 0.0;
  int compared = 0;  // cycles recorded ok
  // The stamp of the first cycle replayed with another status, or recorded ok with stage 1 beyond the tolerance (NaN:
  // none).
  double first_divergence_stamp = std::numeric_limits<double>::quiet_NaN();
};

// Per replayed cycle. true_clearance_m: the smallest 3D distance of the robot primitives to the obstacles over the
// executed plan's stages 1..clearance_stages (NaN when the cycle executed no plan); lin_error_m: CycleOutput's;
// progress_m: route progress from stage 0 to the plan's last stage; contour_m: the plan's mean |lateral| offset.
struct ReplayResult
{
  std::vector<double> stamp, total_s, solve_s, collision_s, prep_s, feedback_s, true_clearance_m, lin_error_m,
    progress_m, contour_m;
  // At the recorded rate: the largest |replayed - recorded| component of stage 1 (metres and radians mixed; NaN
  // where either has no plan).
  std::vector<double> stage1_error_m;
  std::vector<int> iterations;
  std::vector<std::string> status, posture_choice;
  int goal_starts = 0, goal_updates = 0;
  std::optional<Fidelity> fidelity;  // at the recorded rate only
};

// The controller as wholebody_mpc_node builds it from `config` at rate_hz (`diagnostics`: the bench's
// measurements on).
std::unique_ptr<Controller> build_controller(const MpcConfig & config, const Limits & limits, const std::string & urdf,
                                             const mobile_manipulator_navigation::Json & payload, double rate_hz,
                                             bool diagnostics);

// Replays `recording` cycle by cycle as the node ran it: goals start (first goal, after a gap of more than three
// recorded periods, or after a goal_reached / failed cycle; there the last goal restarts also without a new /plan,
// as the behaviour tree resends FollowPath) or update in place; the previous command from the
// recording; the external hold on blocked cycles; stale on stale_state cycles; note_failure on a miss.
ReplayResult replay(Controller & controller, const Recording & recording, const ReplaySetup & setup,
                    const ReplayOptions & options);
}  // namespace mobile_manipulator_mpc
