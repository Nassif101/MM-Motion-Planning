#pragma once
// Minimal command-line parsing for the scenario tools: positional arguments and
// "--option value..." groups (values run until the next "--" token). Values may be
// negative numbers.
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace mobile_manipulator_navigation
{
class Args
{
public:
  // ROS arguments after --ros-args are not ours.
  Args(int argc, char ** argv)
  {
    std::string option;
    for (int i = 1; i < argc; ++i) {
      const std::string token = argv[i];
      if (token == "--ros-args") break;
      if (token.rfind("--", 0) == 0 && token.size() > 2) {
        option = token.substr(2);
        options_[option];
      } else if (option.empty()) {
        positional_.push_back(token);
      } else {
        options_[option].push_back(token);
      }
    }
  }

  bool has(const std::string & name) const { return options_.count(name) > 0; }
  const std::vector<std::string> & positional() const { return positional_; }

  const std::vector<std::string> & values(const std::string & name, size_t count = 0) const
  {
    const auto it = options_.find(name);
    if (it == options_.end()) throw std::invalid_argument("missing --" + name);
    if (count && it->second.size() != count) {
      throw std::invalid_argument("--" + name + " takes " + std::to_string(count) + " values");
    }
    return it->second;
  }

  std::string get(const std::string & name, const std::string & fallback) const
  {
    return has(name) ? values(name, 1)[0] : fallback;
  }

  std::string get(const std::string & name) const { return values(name, 1)[0]; }

  double number(const std::string & name, double fallback) const
  {
    return has(name) ? std::stod(get(name)) : fallback;
  }

  std::vector<double> numbers(const std::string & name, size_t count) const
  {
    std::vector<double> out;
    for (const auto & value : values(name, count)) out.push_back(std::stod(value));
    return out;
  }

private:
  std::vector<std::string> positional_;
  std::map<std::string, std::vector<std::string>> options_;
};
}  // namespace mobile_manipulator_navigation
