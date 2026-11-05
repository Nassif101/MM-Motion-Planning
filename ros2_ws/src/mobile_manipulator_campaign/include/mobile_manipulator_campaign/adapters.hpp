#pragma once
// Conversions between the framework's types (B5's state, control and cycle output) and the plant's.
#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_campaign
{
// B5's command at `t`: the base twist and the arm's joint velocities (the ideal mode's arm reference).
mobile_manipulator_plant::PlantCommand to_plant_command(const mobile_manipulator_mpc::Control & command,
                                                        const mobile_manipulator_mpc::CycleOutput & cycle, double t);
// The state B5 measures: base pose and the six joints.
mobile_manipulator_mpc::State to_state(const mobile_manipulator_plant::PlantState & state);
// The executed control: the base twist and the joint velocities.
mobile_manipulator_mpc::Control to_executed(const mobile_manipulator_plant::PlantState & state);
}  // namespace mobile_manipulator_campaign
