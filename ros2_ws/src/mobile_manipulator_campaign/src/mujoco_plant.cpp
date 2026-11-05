#include "mobile_manipulator_campaign/mujoco_plant.hpp"

#include "mobile_manipulator_campaign/adapters.hpp"

namespace mobile_manipulator_campaign
{
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

MujocoPlant::MujocoPlant(mmp::RobotProfile profile, std::string site_mjcf, mmp::PlantMode mode)
: plant_(std::move(profile), std::move(site_mjcf), mode)
{
}

void MujocoPlant::reset(const mmp::ScenarioSetup & scenario, const mmm::State & start, uint64_t seed)
{
  scenario_ = scenario;
  plant_.reset(scenario, {start(0), start(1), start(2)}, start.tail<6>(), seed);
  refresh();
}

void MujocoPlant::step(const mmm::Control & command, const mmm::CycleOutput & cycle, double t, double dt)
{
  plant_.step(to_plant_command(command, cycle, t), dt);
  refresh();
}

void MujocoPlant::update_world(mmm::World & world, double t)
{
  for (const auto & [name, position] : plant_.mover_positions()) {
    for (const auto & spec : scenario_.movers)
      if (spec.name == name) world.update_mover(name, position, spec.size, t);
  }
}

void MujocoPlant::refresh()
{
  x_ = to_state(plant_.state());
  executed_ = to_executed(plant_.state());
  truth_ = plant_.ground_truth();
}
}  // namespace mobile_manipulator_campaign
