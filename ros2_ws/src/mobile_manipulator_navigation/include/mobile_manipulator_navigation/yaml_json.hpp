#pragma once
// YAML -> JSON with PyYAML (YAML 1.1) scalar typing, so the C++ tools read the package
// configuration exactly as the Python tools did: plain scalars become null, bool, int or
// float where PyYAML would resolve them; quoted scalars stay strings; map order is kept.
#include <limits>
#include <regex>
#include <string>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace mobile_manipulator_navigation
{
using Json = nlohmann::ordered_json;

inline Json plain_scalar(const std::string & text)
{
  static const std::regex null_re("~|null|Null|NULL|");
  static const std::regex true_re("yes|Yes|YES|true|True|TRUE|on|On|ON");
  static const std::regex false_re("no|No|NO|false|False|FALSE|off|Off|OFF");
  static const std::regex int_re("[-+]?(0|[1-9][0-9_]*)");
  static const std::regex float_re(
    "[-+]?([0-9][0-9_]*)?\\.[0-9_]*([eE][-+][0-9]+)?|[-+]?\\.(inf|Inf|INF)|\\.(nan|NaN|NAN)");
  if (std::regex_match(text, null_re)) return nullptr;
  if (std::regex_match(text, true_re)) return true;
  if (std::regex_match(text, false_re)) return false;
  std::string digits;
  for (const char c : text) if (c != '_') digits += c;
  if (std::regex_match(text, int_re)) return std::stoll(digits);
  if (std::regex_match(text, float_re) && text != "." && text != "+." && text != "-.") {
    if (text.find("inf") != std::string::npos || text.find("Inf") != std::string::npos ||
        text.find("INF") != std::string::npos) {
      return text[0] == '-' ? -std::numeric_limits<double>::infinity()
                            : std::numeric_limits<double>::infinity();
    }
    if (text.find(".nan") != std::string::npos || text.find(".NaN") != std::string::npos ||
        text.find(".NAN") != std::string::npos) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::stod(digits);
  }
  return text;
}

inline Json to_json(const YAML::Node & node)
{
  switch (node.Type()) {
    case YAML::NodeType::Map: {
      Json out = Json::object();
      for (const auto & item : node) out[item.first.as<std::string>()] = to_json(item.second);
      return out;
    }
    case YAML::NodeType::Sequence: {
      Json out = Json::array();
      for (const auto & item : node) out.push_back(to_json(item));
      return out;
    }
    case YAML::NodeType::Scalar:
      return node.Tag() == "!" ? Json(node.Scalar()) : plain_scalar(node.Scalar());
    default:
      return nullptr;
  }
}

inline Json load_yaml_file(const std::string & path)
{
  return to_json(YAML::LoadFile(path));
}
}  // namespace mobile_manipulator_navigation
