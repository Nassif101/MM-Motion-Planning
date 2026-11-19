#pragma once
#include <filesystem>
#include "mobile_manipulator_routing/grid.hpp"
namespace mobile_manipulator_routing
{
GridSnapshot load_map(const std::filesystem::path & yaml_path, InflationParameters inflation = {});
}  // namespace mobile_manipulator_routing
