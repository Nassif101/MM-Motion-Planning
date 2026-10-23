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

// The first joint whose commanded target is non-finite or outside its position or velocity limit, -1 when the whole
// command is valid. Accelerations are checked for finiteness only (the URDF has no acceleration limits).
inline int invalid_command_joint(
  const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & a,
  const std::vector<double> & lower, const std::vector<double> & upper, const std::vector<double> & vmax)
{
  for (size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || !std::isfinite(a[i]) || q[i] < lower[i] - 1e-6 ||
        q[i] > upper[i] + 1e-6 || std::abs(v[i]) > vmax[i] + 1e-6) return static_cast<int>(i);
  }
  return -1;
}
}
