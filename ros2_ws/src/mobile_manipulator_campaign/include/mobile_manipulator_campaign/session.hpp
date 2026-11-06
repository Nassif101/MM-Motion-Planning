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
#include <vector>

#include "mobile_manipulator_campaign/closed_loop_case.hpp"
#include "mobile_manipulator_campaign/mujoco_plant.hpp"
#include "mobile_manipulator_medium/closed_loop.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"

namespace mobile_manipulator_campaign
{
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
  bool lidar = false;
  int scan_decimation = 1;     // every n-th scan into the frames
  double timeout_s = 120.0;    // per goal
  int iteration_budget = 30;   // run_case's: reproducible plans, latency fixed at the profile's cap
  bool perturb_start = true;
  bool queue_frames = false;   // keep the frames for take_frames (the demo stream); the run log gets them regardless
  std::string log_path;        // empty: no run log
  nlohmann::json provenance = nlohmann::json::object();  // into the Hello as given (the executables: git_commit, git_dirty)
};

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
  bool active() const { return stepper_ != nullptr; }
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
  void finish_goal();
  void emit(mobile_manipulator_plant::FrameType type, const nlohmann::json & header,
            const std::vector<std::vector<float>> & blocks = {});
  void event(const std::string & name, nlohmann::json details = {});
  nlohmann::json mover_xy() const;
  nlohmann::json record(const mobile_manipulator_medium::ClosedLoopResult & r, const mobile_manipulator_navigation::Pose2 & goal,
                        double wall_s, bool timed_out) const;

  SessionConfig config_;
  std::string profile_path_, site_mjcf_, scenario_, bag_;
  uint64_t seed_ = 0;
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
  std::unique_ptr<mobile_manipulator_plant::RunLogWriter> log_;
  std::vector<std::string> frames_;
  std::vector<nlohmann::json> events_;
  std::vector<nlohmann::json> records_;
  double goal_wall_start_ = 0.0, wall_s_ = 0.0;
  long scans_ = 0;
  int last_contacts_ = 0;
  std::string last_fallback_;
};
}  // namespace mobile_manipulator_campaign
