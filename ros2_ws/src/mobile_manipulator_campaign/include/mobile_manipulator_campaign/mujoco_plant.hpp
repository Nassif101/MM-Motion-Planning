#pragma once
// The MuJoCo plant behind the closed loop's plant interface (MuJoCo plant plan Task 4).
#include <string>

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
  mobile_manipulator_plant::ScenarioSetup scenario_;
  mobile_manipulator_mpc::State x_ = mobile_manipulator_mpc::State::Zero();
  mobile_manipulator_mpc::Control executed_ = mobile_manipulator_mpc::Control::Zero();
  mobile_manipulator_plant::GroundTruth truth_;
};
}  // namespace mobile_manipulator_campaign
