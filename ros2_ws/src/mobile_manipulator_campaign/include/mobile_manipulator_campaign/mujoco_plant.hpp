#pragma once
// The MuJoCo plant behind the closed loop's plant interface (MuJoCo plant plan Tasks 4 and 6). Ideal mode: B5's command
// as joint velocities, B5 sees the plant's state. Realistic mode (after configure_arm_path): the node's arm command path
// (ArmCommandPath: B5 sees the arm reference, the arm gets streams and holds, the JTC starting each on receipt, after the
// plant's command delay); B5 sees the plant's state after its state delay.
#include <string>

#include <memory>

#include "mobile_manipulator_campaign/arm_command_path.hpp"
#include "mobile_manipulator_mpc/closed_loop_plant.hpp"
#include "mobile_manipulator_plant/plant.hpp"

namespace mobile_manipulator_campaign
{
class MujocoPlant : public mobile_manipulator_mpc::ClosedLoopPlant
{
public:
  MujocoPlant(mobile_manipulator_plant::RobotProfile profile, std::string site_mjcf, mobile_manipulator_plant::PlantMode mode);
  // A new run: the scenario's boxes and movers, the base and arm at `start` (B5's state layout).
  void reset(const mobile_manipulator_plant::ScenarioSetup & scenario, const mobile_manipulator_mpc::State & start, uint64_t seed);
  // The realistic mode's arm command path from B5's limits, configuration and collision primitives (run_case does it).
  void configure_arm_path(const mobile_manipulator_mpc::Limits & limits, const mobile_manipulator_mpc::MpcConfig & config,
                          const std::vector<mobile_manipulator_geometry::Primitive> & primitives);
  bool realistic() const { return path_ != nullptr; }
  const mobile_manipulator_mpc::State & measured() const { return measured_; }
  int reanchors() const { return reanchors_; }

  const mobile_manipulator_mpc::State & state() const override { return x_; }
  const mobile_manipulator_mpc::Control & executed() const override { return executed_; }
  bool at_rest() const override { return plant_.base_at_rest(); }
  void step(const mobile_manipulator_mpc::Control & command, const mobile_manipulator_mpc::CycleOutput & cycle, double t,
            double dt) override;
  void update_world(mobile_manipulator_mpc::World & world, double t) override;
  std::optional<double> true_min_clearance() const override { return truth_.min_clearance_m; }
  int contacts() const override { return static_cast<int>(truth_.contacts.size()); }

  mobile_manipulator_plant::Plant & plant() { return plant_; }
  const mobile_manipulator_plant::GroundTruth & truth() const { return truth_; }

private:
  void refresh();

  mobile_manipulator_plant::Plant plant_;
  mobile_manipulator_plant::PlantMode mode_;
  mobile_manipulator_plant::ScenarioSetup scenario_;
  std::unique_ptr<ArmCommandPath> path_;
  mobile_manipulator_plant::ArmReference last_arm_;
  const mobile_manipulator_mpc::World * world_ = nullptr;
  int reanchors_ = 0;
  mobile_manipulator_mpc::State measured_ = mobile_manipulator_mpc::State::Zero();
  mobile_manipulator_mpc::State x_ = mobile_manipulator_mpc::State::Zero();
  mobile_manipulator_mpc::Control executed_ = mobile_manipulator_mpc::Control::Zero();
  mobile_manipulator_plant::GroundTruth truth_;
};
}  // namespace mobile_manipulator_campaign
