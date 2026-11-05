#include "mobile_manipulator_campaign/mujoco_plant.hpp"

#include "mobile_manipulator_campaign/adapters.hpp"

#include <stdexcept>

namespace mobile_manipulator_campaign
{
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

MujocoPlant::MujocoPlant(mmp::RobotProfile profile, std::string site_mjcf, mmp::PlantMode mode)
: plant_(std::move(profile), std::move(site_mjcf), mode), mode_(mode)
{
}

void MujocoPlant::configure_arm_path(const mmm::Limits & limits, const mmm::MpcConfig & config,
                                     const std::vector<mobile_manipulator_geometry::Primitive> & primitives)
{
  if (mode_ == mmp::PlantMode::Realistic) path_ = std::make_unique<ArmCommandPath>(limits, config, primitives);
}

void MujocoPlant::reset(const mmp::ScenarioSetup & scenario, const mmm::State & start, uint64_t seed)
{
  scenario_ = scenario;
  plant_.reset(scenario, {start(0), start(1), start(2)}, start.tail<6>(), seed);
  if (path_) path_->reset();
  last_arm_ = mmp::ArmReference{};
  reanchors_ = 0;
  world_ = nullptr;
  refresh();
}

void MujocoPlant::step(const mmm::Control & command, const mmm::CycleOutput & cycle, double t, double dt)
{
  if (mode_ == mmp::PlantMode::Realistic && !path_)
    throw std::logic_error("MujocoPlant: the realistic mode needs configure_arm_path");
  if (!path_) {
    plant_.step(to_plant_command(command, cycle, t), dt);
  } else {
    mmp::PlantCommand c;
    c.stamp = t;
    c.v = command(0);
    c.omega = command(1);
    if (auto sent = path_->command(cycle, measured_, t, world_, command.head<2>())) {
      last_arm_ = *sent;
      last_arm_.start_time = t + plant_.profile().arm_command_delay_s;  // the JTC starts it on receipt
    }
    c.arm = last_arm_;
    plant_.step(c, dt);
  }
  refresh();
}

void MujocoPlant::update_world(mmm::World & world, double t)
{
  world_ = &world;  // valid while run_closed_loop runs (the only caller of step); reset clears it
  for (const auto & [name, position] : plant_.mover_positions()) {
    for (const auto & spec : scenario_.movers)
      if (spec.name == name) world.update_mover(name, position, spec.size, t);
  }
}

void MujocoPlant::refresh()
{
  measured_ = to_state(plant_.state());
  executed_ = to_executed(plant_.state());
  if (path_) {  // B5 sees the arm reference (the node's arm_reference), the arm's executed velocity is its velocity
    // sampled at the node's clock now (the plant's time), not at the delayed state's stamp
    x_ = path_->b5_state(measured_, plant_.data()->time);
    reanchors_ += path_->reanchored();
    executed_.tail<6>() = path_->reference_v();
  } else {
    x_ = measured_;
  }
  truth_ = plant_.ground_truth();
}
}  // namespace mobile_manipulator_campaign
