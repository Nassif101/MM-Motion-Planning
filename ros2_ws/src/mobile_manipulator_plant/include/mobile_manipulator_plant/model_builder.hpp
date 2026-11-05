#pragma once
// Composes the plant's model at reset (MuJoCo plant spec Section 3.2): the generated site and robot models, attached
// through mjSpec, plus the scenario's boxes (static geoms scenario/<name>, group 3) and movers (mocap bodies
// mover/<name>, group 4). Physics options are the plant's: 2 ms, implicitfast.
#include <mujoco/mujoco.h>

#include <string>

#include "mobile_manipulator_plant/robot_profile.hpp"
#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_plant
{
constexpr double kPhysicsStepS = 0.002;

// A scenario box whose name is already a site box (B5's scenes list the site's boxes too) must equal it within 1e-6
// and is not added again; otherwise std::runtime_error. Returns the compiled model (free with mj_deleteModel) or throws
// std::runtime_error with MuJoCo's message.
mjModel * build_model(const RobotProfile & profile, const std::string & site_mjcf, const ScenarioSetup & scenario);
}  // namespace mobile_manipulator_plant
