#pragma once
// Open-loop replay of recorded commands through the realistic plant (MuJoCo plant plan Task 6, the calibration's
// engine): JSON in, JSON out, no ROS. Input:
//   {"start": {"base": [x, y, yaw], "q": [6]}, "duration_s": D, "sample_period_s": 0.02,
//    "base_commands": [[t, v, omega], ...],                       // as the base received them, t from the start
//    "arm_streams": [{"t": receipt, "start": jtc_start, "points": [{"t": .., "q": [6], "v": [6]}, ..]}, ..],
//    "params": {"command_delay_s": .., "kp": [6], ..}}            // optional profile overrides (apply_overrides)
// Output: {"t": [..], "base": [[x, y, yaw]], "twist": [[v, omega]], "q": [[6]], "qd": [[6]], "params": {..}}.
#include <nlohmann/json.hpp>
#include <string>

#include "mobile_manipulator_plant/robot_profile.hpp"

namespace mobile_manipulator_campaign
{
// Overrides profile fields by name: scalars command_delay_s, state_delay_s, base_breakaway, jtc_rate_hz,
// ff_accel_limit, gravity_ff, motion_ff; two-vectors base_lag_s, base_slip; six-vectors kp, kd, armature, damping,
// frictionloss; "name.i" sets one element (e.g. "kp.2"). Throws std::invalid_argument for an unknown name.
void apply_overrides(mobile_manipulator_plant::RobotProfile & profile, const nlohmann::json & params);

nlohmann::json replay(const nlohmann::json & input, mobile_manipulator_plant::RobotProfile profile, const std::string & site_mjcf);
}  // namespace mobile_manipulator_campaign
