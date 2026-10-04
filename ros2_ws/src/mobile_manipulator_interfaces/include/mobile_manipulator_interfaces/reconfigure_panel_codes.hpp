#pragma once
// Names of ReconfigurePanel result codes, as reports and run summaries spell them.
#include <cstdint>

#include "mobile_manipulator_interfaces/action/reconfigure_panel.hpp"

namespace mobile_manipulator_interfaces
{
inline const char * reconfigure_code_name(uint8_t code)
{
  using Result = action::ReconfigurePanel::Result;
  switch (code) {
    case Result::SUCCESS: return "SUCCESS";
    case Result::BASE_NOT_STOPPED: return "BASE_NOT_STOPPED";
    case Result::ARM_NOT_ACTIVE: return "ARM_NOT_ACTIVE";
    case Result::UNKNOWN_PROFILE: return "UNKNOWN_PROFILE";
    case Result::NO_IK: return "NO_IK";
    case Result::PLANNING_FAILED: return "PLANNING_FAILED";
    case Result::PROFILE_TOO_SMALL: return "PROFILE_TOO_SMALL";
    case Result::EXECUTION_FAILED: return "EXECUTION_FAILED";
    case Result::ARM_FAULT: return "ARM_FAULT";
    case Result::PROFILE_VIOLATED_AFTER_EXECUTION: return "PROFILE_VIOLATED_AFTER_EXECUTION";
    case Result::CANCELED: return "CANCELED";
    case Result::HULL_IN_COLLISION: return "HULL_IN_COLLISION";
    default: return "UNKNOWN";
  }
}
}  // namespace mobile_manipulator_interfaces
