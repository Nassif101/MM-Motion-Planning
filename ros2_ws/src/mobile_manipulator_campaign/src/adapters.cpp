#include "mobile_manipulator_campaign/adapters.hpp"

namespace mobile_manipulator_campaign
{
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

mmp::PlantCommand to_plant_command(const mmm::Control & command, const mmm::CycleOutput &, double t)
{
  mmp::PlantCommand c;
  c.stamp = t;
  c.v = command(0);
  c.omega = command(1);
  c.arm.joint_velocity = mmp::Vec6(command.tail<6>());
  return c;
}

mmm::State to_state(const mmp::PlantState & state)
{
  mmm::State x;
  x << state.base.x, state.base.y, state.base.yaw, state.q;
  return x;
}

mmm::Control to_executed(const mmp::PlantState & state)
{
  mmm::Control u;
  u << state.v, state.omega, state.qd;
  return u;
}
}  // namespace mobile_manipulator_campaign
