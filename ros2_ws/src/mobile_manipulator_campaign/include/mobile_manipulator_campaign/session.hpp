#pragma once
// A plant session (MuJoCo plant spec Section 6.1, plan Task 8): one scenario at a time on the MuJoCo plant with B5 and
// (optionally) the medium planner, stepped one B5 cycle at a time. It runs a recording's goals in order (a mission,
// the plant's state carried over) or a goal it is given (a straight route from the robot), emits frames (state,
// plans, scans, events) for the run log and the demo stream, and summarizes the run as medium_bench closed records do,
// plus the plant's fields. Seeds perturb the start pose (+-1 mm, +-1 mrad; CP1) and drive the plant's noise.
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mobile_manipulator_campaign/closed_loop_case.hpp"
#include "mobile_manipulator_campaign/goal_manager.hpp"
#include "mobile_manipulator_campaign/mujoco_plant.hpp"
#include "mobile_manipulator_medium/closed_loop.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"
#include "mobile_manipulator_routing/search.hpp"

namespace mobile_manipulator_campaign
{
struct RoutingInputs;
// A recording's goals as a mission: Nav2 publishes a /plan per replan, so consecutive plans to the same goal (within
// 1e-6 in x, y and yaw) are one goal with the run's first route; plans of fewer than two points are skipped.
std::vector<mobile_manipulator_mpc::RecordedGoal> mission_goals(const std::vector<mobile_manipulator_mpc::RecordedGoal> & plans);

// The working directory's git commit and whether the tree is dirty ({} outside a repository): what the executables put
// into SessionConfig::provenance.
nlohmann::json git_provenance();

struct SessionConfig
{
  std::string profile;  // the robot profile; empty: the plant's profile.yaml
  mobile_manipulator_plant::PlantMode mode = mobile_manipulator_plant::PlantMode::Ideal;
  bool medium = true;          // false: B5 alone
  std::string medium_config;   // empty: medium.yaml
  std::string b5_config;       // empty: the installed wholebody_mpc.yaml (E1: a variant, plant_campaign --b5-config)
  double start_nudge_rad = 0.0;  // added to the start yaw after the seed's perturbation (E1's twin runs; --start-nudge)
  // E2 Task 7: B5's tracking model, overriding the B5 config's: empty keeps the config's (and, since Task 11, a config
  // naming none gets the installed MuJoCo model with the turn-gain correction), "none" the identity, else a model file
  // (plant_campaign --tracking-model); its online correction: -1 the config's, 0 off, 1 on, 2 on for
  // the turn gain alone, the sideslip frozen (Task 10c) (plant_campaign --tracking-adaptation on|off|gain)
  std::string tracking_model;
  int tracking_adaptation = -1;
  bool diagnostics = false;  // E1 Task 9: B5's diagnostics on, State frames carry diag (plant_campaign --diagnostics)
  // E3 Task 1: B5's row trace once per 0.1 s, State frames carry rows, rows_x, rows_u (--row-trace); turns the
  // diagnostics on too (the trace's analysis reads diag_cmd and diag_track)
  bool row_trace = false;
  bool lidar = false;
  int scan_decimation = 1;     // every n-th scan into the frames
  double timeout_s = 0.0;      // per goal; 0: the scenario's timeout_s (as Unity's runner), 120 s without one
  int iteration_budget = 30;   // run_case's: reproducible plans, latency fixed at the profile's cap
  bool perturb_start = true;
  std::string cell;  // the summary's cell (a corpus cell's name); empty: the scenario
  double stall_s = 0.0;  // end a goal as stalled after this long without progress (0: off; plant_campaign: 45)
  bool profile_timing = false;  // the summary's timing: the plant step's and the scans' wall times (gate P)
  bool queue_frames = false;   // keep the frames for take_frames (the demo stream); the run log gets them regardless
  std::string log_path;        // empty: no run log
  nlohmann::json provenance = nlohmann::json::object();  // into the Hello as given (the executables: git_commit, git_dirty)
  // Explicit opt-in: the lattice planner supplies routes and a persistent executor keeps the plant clock running.
  bool generated_routes = false;
  bool auto_goal = true;  // submit the scenario's destination at reset
  std::string routing_map, routing_primitives;
  mobile_manipulator_routing::SearchConfiguration routing_search;
  mobile_manipulator_routing::WorkBudget routing_budget;
};

// E2 Task 9 (T7): one B5 cycle's timing sample for the summary: (QP solve, cycle) seconds. The cycle is
// CycleTiming::total_s, which already leaves the diagnostics out (Task 9 review I3).
std::pair<double, double> b5_timing_sample(const mobile_manipulator_mpc::CycleTiming & tm);

class Session
{
public:
  explicit Session(SessionConfig config);
  ~Session();
  // The V2 site's scenarios (scenarios_site_v2.yaml).
  static std::vector<std::string> list_scenarios();
  // A new run: the scenario's boxes and movers; with a bag, its recorded goals in order from the recording's first goal
  // state; without, the scenario's start and goal on a straight route. `start` overrides the base pose. A new run log.
  void reset(const std::string & scenario, const std::string & bag, std::optional<mobile_manipulator_plant::Pose2> start,
             uint64_t seed);
  // Replaces the goals: a straight route from the robot (click goals), or the given route.
  void set_goal(const mobile_manipulator_navigation::Pose2 & goal);
  void set_route(const mobile_manipulator_mpc::Route & route, const mobile_manipulator_navigation::Pose2 & goal);
  // Replaces the goals with a mission: each route and goal in order, the plant's state carried over.
  void set_mission(const std::vector<std::pair<mobile_manipulator_mpc::Route, mobile_manipulator_navigation::Pose2>> & goals);
  // One B5 cycle; false when no goal is active (all reached, failed, timed out, or none set).
  bool step_cycle();
  bool active() const;
  // Generated mode: one simulation tick, including while idle, planning, or after a terminal goal.
  bool advance();
  GoalAcknowledgement submit_goal(GoalSubmission goal);
  GoalAcknowledgement schedule_goal(GoalSubmission goal, double simulation_time);
  GoalAcknowledgement request_replan();
  const GoalManager & goal_manager() const;
  const SessionConfig & config() const { return config_; }
  double time() const;  // the plant's clock
  void move_body(const std::string & name, const Eigen::Vector3d & center, double yaw);
  std::vector<std::string> take_frames();
  std::vector<nlohmann::json> take_events();
  nlohmann::json summary() const;
  const mobile_manipulator_mpc::State & start_state() const { return start_; }
  MujocoPlant & plant() { return *plant_; }

private:
  struct Goal
  {
    mobile_manipulator_mpc::Route route;
    mobile_manipulator_navigation::Pose2 goal;
  };
  void start_next_goal();
  void finish_goal(bool stalled = false);
  mobile_manipulator_medium::ClosedLoopOptions loop_options();
  bool execute_cycle(bool holding);
  void synchronize_goals();
  void publish_goal_events();
  void advance_search();
  mobile_manipulator_routing::Pose2 routing_pose() const;
  void archive_goal(const GoalRecord & goal, GoalEventKind kind, const std::string & reason);
  void emit(mobile_manipulator_plant::FrameType type, const nlohmann::json & header,
            const std::vector<std::vector<float>> & blocks = {});
  void event(const std::string & name, nlohmann::json details = {});
  nlohmann::json mover_xy() const;
  static nlohmann::json reserved_bodies();  // the Hello's: name and size of each reserved body
  nlohmann::json body_poses() const;       // the State's: [name, x, y, yaw] of each placed reserved body
  nlohmann::json record(const mobile_manipulator_medium::ClosedLoopResult & r, const mobile_manipulator_navigation::Pose2 & goal,
                        double wall_s, bool timed_out) const;

  SessionConfig config_;
  std::string profile_path_, site_mjcf_, scenario_, bag_;
  uint64_t seed_ = 0;
  double timeout_s_ = 120.0;  // resolved at reset
  ClosedLoopCase case_;
  std::unique_ptr<MujocoPlant> plant_;
  std::unique_ptr<mobile_manipulator_mpc::Controller> b5_;
  std::unique_ptr<mobile_manipulator_medium::MediumPlanner> medium_;
  std::unique_ptr<mobile_manipulator_medium::ClosedLoopStepper> stepper_;
  mobile_manipulator_mpc::PostureTargets posture_;
  std::vector<Goal> goals_;
  size_t goal_index_ = 0;
  mobile_manipulator_mpc::State start_ = mobile_manipulator_mpc::State::Zero();
  Eigen::Vector3d perturbation_ = Eigen::Vector3d::Zero();
  std::string tracking_model_sha256_;  // of the tracking model file B5 predicted with ("": the identity)
  std::string b5_config_sha256_;  // of the B5 config file the run loaded
  std::string medium_config_sha256_;  // of an explicit medium config at reset ("" for the installed one)
  std::unique_ptr<mobile_manipulator_plant::RunLogWriter> log_;
  std::vector<std::string> frames_;
  std::vector<nlohmann::json> events_;
  std::vector<nlohmann::json> records_;
  double goal_wall_start_ = 0.0, wall_s_ = 0.0;
  long scans_ = 0;
  mobile_manipulator_mpc::State progress_ref_ = mobile_manipulator_mpc::State::Zero();  // stall detector
  double progress_t_ = 0.0;
  std::vector<double> step_wall_s_, scan_cast_s_;  // profile_timing: per cycle, per scan
  std::vector<double> b5_solve_s_, b5_cycle_s_;  // E2 Task 9 (T7): B5 per cycle, with profile_timing
  int last_contacts_ = 0;
  std::string last_fallback_;
  std::unique_ptr<RoutingInputs> routing_inputs_;
  std::unique_ptr<mobile_manipulator_routing::FootprintChecker> routing_checker_;
  std::unique_ptr<GoalManager> goal_manager_;
  // the run's Smac Lattice port, serving its searches in order (spec 2 Amendment 1, A1.2 item 4)
  std::shared_ptr<mobile_manipulator_routing::RoutePlanner> routing_planner_;
  std::unique_ptr<mobile_manipulator_routing::Search> search_;
  std::optional<mobile_manipulator_routing::MotionCheck> adoption_check_;
  mobile_manipulator_routing::Pose2 adoption_start_;
  std::vector<mobile_manipulator_routing::Pose2> adoption_poses_;
  std::uint64_t executing_goal_id_ = 0, legacy_client_request_ = 0;
};
}  // namespace mobile_manipulator_campaign
