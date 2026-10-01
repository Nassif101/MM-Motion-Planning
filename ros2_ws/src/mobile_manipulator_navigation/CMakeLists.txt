cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mobile_manipulator_control REQUIRED)  # cli.hpp
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(lidar_robot_filter src/lidar_robot_filter.cpp)
target_compile_features(lidar_robot_filter PUBLIC cxx_std_17)
target_compile_options(lidar_robot_filter PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(lidar_robot_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(lidar_robot_filter Eigen3::Eigen urdf::urdf)

add_executable(livox_robot_filter src/livox_robot_filter_node.cpp)
target_compile_options(livox_robot_filter PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(livox_robot_filter lidar_robot_filter ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2::tf2 tf2_eigen::tf2_eigen tf2_ros::tf2_ros yaml-cpp)

add_executable(nav_telemetry src/nav_telemetry_node.cpp)
target_compile_features(nav_telemetry PRIVATE cxx_std_17)
target_compile_options(nav_telemetry PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(nav_telemetry PRIVATE include)
target_link_libraries(nav_telemetry rclcpp::rclcpp ${action_msgs_TARGETS} ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros
  nlohmann_json::nlohmann_json)

add_library(scenario_tools src/scenario_spec.cpp)
target_compile_features(scenario_tools PUBLIC cxx_std_17)
target_compile_options(scenario_tools PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(scenario_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(scenario_tools nlohmann_json::nlohmann_json yaml-cpp)

add_executable(scenario_spec src/scenario_spec_main.cpp)
target_compile_options(scenario_spec PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(scenario_spec scenario_tools ament_index_cpp::ament_index_cpp)

add_executable(check_cmd_vel_ownership src/check_cmd_vel_ownership.cpp)
target_compile_options(check_cmd_vel_ownership PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(check_cmd_vel_ownership PRIVATE include)
target_link_libraries(check_cmd_vel_ownership rclcpp::rclcpp)

add_executable(check_controller_params src/check_controller_params.cpp)
target_compile_options(check_controller_params PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(check_controller_params scenario_tools rclcpp::rclcpp ${rcl_interfaces_TARGETS}
  ament_index_cpp::ament_index_cpp)

add_executable(plan_scenario_task src/plan_scenario_task.cpp)
target_compile_options(plan_scenario_task PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(plan_scenario_task scenario_tools rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS} ${nav2_msgs_TARGETS} ${nav_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros)

add_executable(navigate_scenario_task src/navigate_scenario_task.cpp)
target_compile_options(navigate_scenario_task PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(navigate_scenario_task scenario_tools rclcpp::rclcpp rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp ${action_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros)

# Read-only probes for the perception, transport and odometry contracts.
add_executable(sensor_transport_probe src/sensor_transport_probe.cpp)
target_link_libraries(sensor_transport_probe scenario_tools rclcpp::rclcpp ${rosgraph_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${tf2_msgs_TARGETS})
add_executable(lidar_self_return_probe src/lidar_self_return_probe.cpp)
target_link_libraries(lidar_self_return_probe scenario_tools rclcpp::rclcpp ${sensor_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp tf2::tf2 tf2_eigen::tf2_eigen tf2_ros::tf2_ros Eigen3::Eigen)
add_executable(local_costmap_check src/local_costmap_check.cpp)
target_link_libraries(local_costmap_check scenario_tools rclcpp::rclcpp ${nav_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros)
add_executable(odom_consistency_check src/odom_consistency_check.cpp)
target_link_libraries(odom_consistency_check scenario_tools rclcpp::rclcpp ${nav_msgs_TARGETS} ${tf2_msgs_TARGETS})
add_executable(gate_planning_check src/gate_planning_check.cpp)
target_link_libraries(gate_planning_check scenario_tools rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS} ${nav2_msgs_TARGETS})
foreach(probe sensor_transport_probe lidar_self_return_probe local_costmap_check odom_consistency_check
        gate_planning_check)
  target_compile_options(${probe} PRIVATE -Wall -Wextra -Wpedantic -O2)
endforeach()
install(TARGETS sensor_transport_probe lidar_self_return_probe local_costmap_check odom_consistency_check
  gate_planning_check RUNTIME DESTINATION lib/${PROJECT_NAME})

foreach(tool scenario_spec check_cmd_vel_ownership check_controller_params plan_scenario_task
        navigate_scenario_task sensor_transport_probe lidar_self_return_probe local_costmap_check
        odom_consistency_check gate_planning_check)
  target_include_directories(${tool} PRIVATE ${mobile_manipulator_control_INCLUDE_DIRS})
endforeach()

# Global-costmap layer for persistent lidar obstacles (persistence_grid.hpp).
add_library(persistent_obstacle_layer SHARED src/persistent_obstacle_layer.cpp)
target_compile_features(persistent_obstacle_layer PUBLIC cxx_std_17)
target_compile_options(persistent_obstacle_layer PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(persistent_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(persistent_obstacle_layer nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::layers pluginlib::pluginlib rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2::tf2
  tf2_eigen::tf2_eigen tf2_ros::tf2_ros Eigen3::Eigen)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS persistent_obstacle_layer
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS lidar_robot_filter livox_robot_filter nav_telemetry scenario_tools scenario_spec
  check_cmd_vel_ownership check_controller_params plan_scenario_task navigate_scenario_task
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY behavior_trees config docs launch maps
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_robot_filter test/test_lidar_robot_filter.cpp)
  target_link_libraries(test_lidar_robot_filter lidar_robot_filter ament_index_cpp::ament_index_cpp yaml-cpp)
  ament_add_gtest(test_scenario_spec test/test_scenario_spec.cpp)
  target_link_libraries(test_scenario_spec scenario_tools)
  target_compile_definitions(test_scenario_spec PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_gtest(test_scenario_metrics test/test_scenario_metrics.cpp)
  target_include_directories(test_scenario_metrics PRIVATE include)
  ament_add_gtest(test_persistence_grid test/test_persistence_grid.cpp)
  target_include_directories(test_persistence_grid PRIVATE include)
  ament_add_gtest(test_telemetry test/test_telemetry.cpp)
  target_include_directories(test_telemetry PRIVATE include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    validate_unity_nav2_map
    test/test_unity_nav2_map.py
  )
  ament_add_pytest_test(
    validate_footprint_profiles
    test/test_footprint_profiles.py
  )
  ament_add_pytest_test(
    validate_operating_envelope
    test/test_operating_envelope.py
  )
  ament_add_pytest_test(
    validate_planner_config
    test/test_planner_config.py
  )
  ament_add_pytest_test(
    validate_local_costmap_config
    test/test_local_costmap_config.py
  )
  ament_add_pytest_test(
    validate_navigation_config
    test/test_navigation_config.py
  )
endif()

ament_package()
