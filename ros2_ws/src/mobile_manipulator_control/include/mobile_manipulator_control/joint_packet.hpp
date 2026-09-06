#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace mobile_manipulator_control
{
// Validate the complete packet before mutating any state. Array order is never a contract.
inline std::vector<size_t> packet_mapping(
  const std::vector<std::string> & required, const std::vector<std::string> & names,
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  if (names.size() != required.size() || positions.size() != names.size() ||
      velocities.size() != names.size()) throw std::invalid_argument("Expected six complete joint states");
  std::unordered_map<std::string, size_t> lookup;
  for (size_t i = 0; i < names.size(); ++i) {
    if (!lookup.emplace(names[i], i).second || !std::isfinite(positions[i]) ||
        !std::isfinite(velocities[i])) throw std::invalid_argument("Duplicate name or non-finite joint value");
  }
  std::vector<size_t> result;
  for (const auto & name : required) {
    auto it = lookup.find(name);
    if (it == lookup.end()) throw std::invalid_argument("Missing joint / unknown replacement: " + name);
    result.push_back(it->second);
  }
  return result;
}
}
