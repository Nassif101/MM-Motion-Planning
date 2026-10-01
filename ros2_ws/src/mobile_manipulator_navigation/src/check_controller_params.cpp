// Fail if a configured controller parameter is not declared by the running controller server.
//
// ROS 2 silently ignores parameter overrides that a plugin never declares, so a misspelled
// key in config/nav2_controllers.yaml would leave the stock default in place. Run with
// navigation.launch.py active and the same `controller` block it was launched with.
//
// Usage: check_controller_params CONTROLLER [--node /controller_server]
#include <algorithm>
#include <chrono>
#include <iostream>
#include <set>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mmn = mobile_manipulator_navigation;
using rcl_interfaces::srv::ListParameters;

namespace
{
void flatten(const mmn::Json & block, const std::string & prefix, std::set<std::string> & names)
{
  for (const auto & item : block.items()) {
    const std::string name = prefix + "." + item.key();
    if (item.value().is_object()) {
      flatten(item.value(), name, names);
    } else {
      names.insert(name);
    }
  }
}

int fail(const std::string & message)
{
  std::cerr << message << std::endl;
  rclcpp::shutdown();
  return 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  if (args.positional().size() != 1) return fail("usage: check_controller_params CONTROLLER [--node NODE]");
  const std::string controller = args.positional()[0];
  const std::string server = args.get("node", "/controller_server");

  const auto share = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const auto block = mmn::load_yaml_file(share + "/config/nav2_controllers.yaml").at("controllers").at(controller);
  std::set<std::string> configured;
  flatten(block, "FollowPath", configured);

  auto node = std::make_shared<rclcpp::Node>("check_controller_params");
  auto client = node->create_client<ListParameters>(server + "/list_parameters");
  if (!client->wait_for_service(std::chrono::seconds(10))) return fail(server + "/list_parameters is not available");
  auto request = std::make_shared<ListParameters::Request>();
  request->prefixes = {"FollowPath"};
  request->depth = 0;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return fail(server + " did not list its parameters");
  }
  const auto names = future.get()->result.names;
  const std::set<std::string> declared(names.begin(), names.end());
  std::vector<std::string> undeclared;
  std::set_difference(configured.begin(), configured.end(), declared.begin(), declared.end(),
                      std::back_inserter(undeclared));
  std::cout << mmn::Json{{"controller", controller}, {"configured", configured.size()},
                         {"undeclared", undeclared}}.dump() << std::endl;
  rclcpp::shutdown();
  return undeclared.empty() ? 0 : 1;
}
