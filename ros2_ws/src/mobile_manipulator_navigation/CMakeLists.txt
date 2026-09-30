cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

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

install(TARGETS lidar_robot_filter livox_robot_filter
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY behavior_trees config docs launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/check_cmd_vel_ownership.py scripts/check_controller_params.py scripts/local_costmap_check.py scripts/navigate_scenario_task.py scripts/nav_telemetry.py scripts/gate_planning_check.py scripts/lidar_self_return_probe.py scripts/odom_consistency_check.py scripts/plan_scenario_task.py scripts/scenario_spec.py scripts/sensor_transport_probe.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_robot_filter test/test_lidar_robot_filter.cpp)
  target_link_libraries(test_lidar_robot_filter lidar_robot_filter ament_index_cpp::ament_index_cpp yaml-cpp)
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
    validate_scenarios
    test/test_scenarios.py
  )
  ament_add_pytest_test(
    validate_local_costmap_config
    test/test_local_costmap_config.py
  )
  ament_add_pytest_test(
    validate_navigation_config
    test/test_navigation_config.py
  )
  ament_add_pytest_test(
    validate_telemetry
    test/test_telemetry.py
  )
endif()

ament_package()
