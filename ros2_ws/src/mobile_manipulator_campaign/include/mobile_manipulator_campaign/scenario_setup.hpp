#pragma once
// A scenario as the plant takes it, from the scene B5 loads (mmm::load_scene): its boxes (the site's included; the
// plant keeps the site's own and adds the rest) and its movers from the scenario's "movers" entries.
#include "mobile_manipulator_mpc/scene.hpp"
#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_campaign
{
mobile_manipulator_plant::ScenarioSetup scenario_setup(const mobile_manipulator_mpc::Scene & scene, const std::string & name);
}  // namespace mobile_manipulator_campaign
