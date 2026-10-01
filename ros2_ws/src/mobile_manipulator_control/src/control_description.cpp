// Publish the ros2_control-augmented model without introducing a second TF publisher.
//
// The controller manager reads /arm/robot_description (latched); robot_state_publisher
// keeps publishing the plain model and the TF tree.
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arm_control_description");
  const auto description = node->declare_parameter("robot_description", std::string());
  if (description.empty()) throw std::runtime_error("robot_description is required");
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "/arm/robot_description", rclcpp::QoS(1).transient_local().reliable());
  std_msgs::msg::String message;
  message.data = description;
  publisher->publish(message);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
