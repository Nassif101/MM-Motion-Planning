cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_routing_nav2_check LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mobile_manipulator_routing REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_map_server REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2_ros REQUIRED)
  set(NAVIGATION_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../mobile_manipulator_navigation")

  ament_add_gtest(test_costmap_equivalence test/test_costmap_equivalence.cpp TIMEOUT 300
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_costmap_equivalence mobile_manipulator_routing::routing
    nav2_costmap_2d::nav2_costmap_2d_core nav2_costmap_2d::layers
    nav2_util::nav2_util_core rclcpp::rclcpp tf2_ros::tf2_ros)
  ament_target_dependencies(test_costmap_equivalence nav2_map_server)
  target_compile_definitions(test_costmap_equivalence PRIVATE NAVIGATION_SOURCE="${NAVIGATION_SOURCE}")
endif()

ament_package()
