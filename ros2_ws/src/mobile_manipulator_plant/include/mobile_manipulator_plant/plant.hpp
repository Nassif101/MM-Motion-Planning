#pragma once
// The MuJoCo plant (MuJoCo plant spec Sections 2.1 and 4): one step API for any driver. Ideal mode: the base and arm
// follow the command exactly (planar and arm joints set kinematically every 2 ms step), collisions detected and
// logged but not pushing; optional static friction from rest (the kinematic plant's breakaway). Realistic mode: commands
// arrive after the command delay; the base through Unity's limiter (BaseDrive) on force-limited planar velocity servos,
// the arm through the JTC sampler and servos with MuJoCo's feedforward (ArmServo); physics with contacts; the state is
// reported after the state delay.
#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mobile_manipulator_plant/arm_servo.hpp"
#include "mobile_manipulator_plant/base_drive.hpp"
#include "mobile_manipulator_plant/delay_queue.hpp"
#include "mobile_manipulator_plant/lidar.hpp"
#include "mobile_manipulator_plant/movers.hpp"
#include "mobile_manipulator_plant/robot_profile.hpp"
#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_plant
{
class Plant
{
public:
  Plant(RobotProfile profile, std::string site_mjcf, PlantMode mode);
  ~Plant();
  Plant(const Plant &) = delete;
  Plant & operator=(const Plant &) = delete;

  // A new model for `scenario`, the base at `start`, the arm at `q0` (clamped to the joint limits), t = 0.
  void reset(const ScenarioSetup & scenario, const Pose2 & start, const Vec6 & q0, uint64_t seed);

  // Advances `dt` (a multiple of the 2 ms physics step within 1e-9, else std::invalid_argument) holding `command`. The
  // ideal mode takes joint velocities; the realistic mode a stream or joint velocities (integrated into the reference).
  const PlantState & step(const PlantCommand & command, double dt);

  const PlantState & state() const { return state_; }
  GroundTruth ground_truth() const;
  std::vector<std::pair<std::string, Eigen::Vector2d>> mover_positions() const;
  struct MoverStatus
  {
    std::string name;
    Eigen::Vector2d position;
    double walked = 0.0, blocked_s = 0.0;
    bool triggered = false, finished = false;
  };
  std::vector<MoverStatus> mover_status() const;
  // The smallest distance between a mover's box and any robot geom (m); +inf for an unknown mover.
  double mover_robot_distance(const std::string & mover) const;
  // Moves a mocap body reserved to appear mid-run to `center` with `yaw`; a mover's motion owns its body
  // (std::invalid_argument).
  void move_body(const std::string & name, const Eigen::Vector3d & center, double yaw);
  void set_ideal_breakaway(bool on) { ideal_breakaway_ = on; }
  // Switches the lidar on, from now (mid-run: the first scan covers the next period) or from the next reset (off by
  // default: campaigns on recorded routes need none); `pattern_path` empty: the profile's. disable_lidar() switches it off.
  void enable_lidar(const std::string & pattern_path = "");
  void disable_lidar() { lidar_.reset(); }
  // The oldest completed scan not yet read, if any (scans queue between reads).
  std::optional<PlantScan> scan();
  bool base_at_rest() const;
  const mjModel * model() const { return m_; }
  const mjData * data() const { return d_; }
  const RobotProfile & profile() const { return profile_; }

private:
  void substep(const Eigen::Vector2d & u, const Vec6 & qd);
  void realistic_substep();
  void finish_substep();
  void update_movers(double t);
  void read_state();
  void measure();  // truth_ from the data now

  RobotProfile profile_;
  std::string site_mjcf_;
  PlantMode mode_;
  bool ideal_breakaway_;
  mjModel * m_ = nullptr;
  mjData * d_ = nullptr;
  PlantState state_;
  Eigen::Vector2d executed_ = Eigen::Vector2d::Zero();  // base (v, omega) executed last
  int qadr_base_[3] = {0, 0, 0}, vadr_base_[3] = {0, 0, 0}, qadr_arm_[6] = {0, 0, 0, 0, 0, 0}, vadr_arm_[6] = {0, 0, 0, 0, 0, 0};
  Vec6 q_lo_ = Vec6::Zero(), q_hi_ = Vec6::Zero();
  int panel_body_ = -1;
  std::vector<int> robot_geoms_, world_geoms_;  // world: site, scenario and mover geoms (not the ground)
  struct Mover
  {
    MoverMotion motion;
    int mocap = -1, geom = -1;
    double height = 0.0;
  };
  std::vector<Mover> movers_;
  long steps_ = 0;
  PlantState truth_;  // the state now; state_ is it after the state delay
  std::unique_ptr<BaseDrive> drive_;
  std::unique_ptr<ArmServo> servo_;
  std::unique_ptr<JtcSampler> jtc_;
  // commands split by destination: the base and the arm have their own delays
  struct BaseCommand
  {
    double stamp = 0.0, v = 0.0, omega = 0.0;
  };
  DelayQueue<BaseCommand> base_commands_{0.0};
  // the JTC runs on the ROS side: it samples a stream on receipt; its output reaches the drives after the arm command
  // delay (arm_targets_)
  DelayQueue<ArmReference> arm_commands_{0.0};
  DelayQueue<ArmTarget> arm_targets_{0.0};
  ArmTarget start_target_{Vec6::Zero(), Vec6::Zero(), Vec6::Zero()};
  DelayQueue<PlantState> base_states_{0.0}, arm_states_{0.0};  // odometry and joint states: separate delays
  Vec6 q_ref_ = Vec6::Zero();
  int drive_act_[3] = {-1, -1, -1};
  std::unique_ptr<Lidar> lidar_;
  uint64_t seed_ = 0;
};
}  // namespace mobile_manipulator_plant
