#pragma once
// A kinematic stand-in for the robot (the controller's closed-loop tests, the medium layer's closed-loop bench;
// Phase 5A Task 4, moved from test/closed_loop.hpp): the generated model with the base's static friction. From rest
// the base does not move while |v| < breakaway linear and |omega| < breakaway yaw (the arm still moves); it counts as
// at rest again once the executed speeds fall below the rest thresholds.
#include <cmath>

#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/model.hpp"

namespace mobile_manipulator_mpc
{
struct KinematicPlant
{
  State x;
  Control executed = Control::Zero();
  Breakaway breakaway{};  // set it: the value-initialized default has no static friction

  bool at_rest() const { return base_at_rest(executed(0), executed(1), breakaway); }

  void step(const Control & command, double dt)
  {
    const bool from_rest = at_rest();
    executed = command;
    if (from_rest && std::abs(command(0)) < breakaway.linear_mps && std::abs(command(1)) < breakaway.yaw_radps) {
      executed(0) = 0.0;
      executed(1) = 0.0;
    }
    x = discrete_step(x, executed, dt).x_next;
  }
};
}  // namespace mobile_manipulator_mpc
