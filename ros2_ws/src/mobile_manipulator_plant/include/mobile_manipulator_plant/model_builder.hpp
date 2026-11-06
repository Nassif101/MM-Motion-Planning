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
// `wheels`: the profile's wheels model (the realistic base on its wheels: ground friction mu on the ground and the
// wheels, the elliptic cone with the profile's impratio, the wheel motors' kv); else the planar model.
mjModel * build_model(const RobotProfile & profile, const std::string & site_mjcf, const ScenarioSetup & scenario,
                      bool wheels = false);

// Under the profile's gain rule: kp = I w^2, kd = 2 zeta I w per arm joint, I the joint-space inertia's diagonal
// (armature included) with the arm at the rule's pose, w = 2 pi bandwidth; written into `profile` and the model's
// servos. Without a rule: nothing.
void resolve_arm_gains(mjModel * m, RobotProfile & profile);
}  // namespace mobile_manipulator_plant
