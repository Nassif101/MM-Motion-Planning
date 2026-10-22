#pragma once
// A B5 recording (rosbag2) for the replay bench (Phase 4B-B spec Section 3.2): /wholebody_mpc/cycle and /plan
// paired by stamp, /plan goals, /cmd_vel_smoothed (sent) and /cmd_vel (executed) through ExecutedCommand,
// /arm_controller/joint_trajectory (the streamed reference), /odom and /scenario/movers, in bag order.
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <mobile_manipulator_interfaces/msg/mpc_cycle.hpp>

#include "mobile_manipulator_mpc/replay.hpp"

namespace mobile_manipulator_mpc
{
// An MpcCycle from its CDR bytes, also in the shorter layout recorded before Phase 4B-B (the fields it lacks read as
// zero and false): the bytes are zero-padded before deserializing.
mobile_manipulator_interfaces::msg::MpcCycle decode_cycle(const std::vector<uint8_t> & cdr);

// `joint_names` in the model's order. Throws std::runtime_error if the bag cannot be opened.
Recording read_recording(const std::string & bag_uri, const std::array<std::string, 6> & joint_names);
}  // namespace mobile_manipulator_mpc
