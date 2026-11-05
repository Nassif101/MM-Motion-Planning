#pragma once
// The plant a closed loop drives (MuJoCo plant plan Task 4): the kinematic stand-in or the MuJoCo plant behind one
// interface, so run_closed_loop runs B5 and the medium layer on either.
#include <optional>

#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/kinematic_plant.hpp"
#include "mobile_manipulator_mpc/world.hpp"

namespace mobile_manipulator_mpc
{
class ClosedLoopPlant
{
public:
  virtual ~ClosedLoopPlant() = default;
  // The state B5 measures, the base and arm command executed last, and whether the base is at rest.
  virtual const State & state() const = 0;
  virtual const Control & executed() const = 0;
  virtual bool at_rest() const = 0;
  // Applies B5's cycle output (its command now and its plan, for a plant that streams the arm) at time `t` for `dt`.
  virtual void step(const Control & command, const CycleOutput & cycle, double t, double dt) = 0;
  // Gives the world the plant's movers (World::update_mover) before B5's cycle at `t`; nothing by default.
  virtual void update_world(World & /*world*/, double /*t*/) {}
  // Ground truth after the last step, when the plant has it: the true minimum clearance (m) and the contacts.
  virtual std::optional<double> true_min_clearance() const { return std::nullopt; }
  virtual int contacts() const { return 0; }
};

// The kinematic plant (ideal actuators with the base's static friction) behind the interface.
class KinematicClosedLoop : public ClosedLoopPlant
{
public:
  KinematicClosedLoop(const State & start, const Breakaway & breakaway) : plant_{start, Control::Zero(), breakaway} {}
  const State & state() const override { return plant_.x; }
  const Control & executed() const override { return plant_.executed; }
  bool at_rest() const override { return plant_.at_rest(); }
  void step(const Control & command, const CycleOutput & /*cycle*/, double /*t*/, double dt) override { plant_.step(command, dt); }

private:
  KinematicPlant plant_;
};
}  // namespace mobile_manipulator_mpc
