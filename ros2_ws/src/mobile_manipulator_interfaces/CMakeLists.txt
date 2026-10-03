cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/ReconfigurePanel.action
  DEPENDENCIES geometry_msgs action_msgs)

# Header-only helpers, beside the generated headers (same include directory).
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
