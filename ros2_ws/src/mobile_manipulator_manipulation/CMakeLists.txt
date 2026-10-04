cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_manipulation)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mobile_manipulator_navigation REQUIRED)
find_package(mobile_manipulator_geometry REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(mobile_manipulator_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

# Known MoveIt collision world: exported static boxes, scenario boxes, raised floor, panel.
add_library(scene_diff STATIC src/scene_diff.cpp)
target_compile_features(scene_diff PUBLIC cxx_std_17)
target_compile_options(scene_diff PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(scene_diff PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(scene_diff mobile_manipulator_geometry::geometry mobile_manipulator_navigation::scenario_tools
  ${moveit_msgs_TARGETS} ${geometry_msgs_TARGETS} ${shape_msgs_TARGETS} Eigen3::Eigen yaml-cpp)

# Pure ReconfigurePanel helpers: goal constraints, base-stopped window, freshness, metrics.
add_library(reconfigure_logic STATIC src/reconfigure_logic.cpp)
target_compile_features(reconfigure_logic PUBLIC cxx_std_17)
target_compile_options(reconfigure_logic PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(reconfigure_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(reconfigure_logic ${moveit_msgs_TARGETS} ${geometry_msgs_TARGETS}
  ${shape_msgs_TARGETS} ${trajectory_msgs_TARGETS} ${builtin_interfaces_TARGETS} Eigen3::Eigen)

# Occupied lidar-Octomap voxels inside known boxes (Octomap scene source evidence).
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
add_library(octomap_box STATIC src/octomap_box.cpp)
target_compile_features(octomap_box PUBLIC cxx_std_17)
target_compile_options(octomap_box PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(octomap_box PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_box scene_diff ${OCTOMAP_LIBRARIES})

add_executable(octomap_box_check src/octomap_box_check.cpp)
target_compile_options(octomap_box_check PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(octomap_box_check octomap_box rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  nlohmann_json::nlohmann_json ${moveit_msgs_TARGETS} ${octomap_msgs_TARGETS})

add_executable(planning_scene_loader src/planning_scene_loader.cpp)
target_compile_options(planning_scene_loader PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(planning_scene_loader scene_diff rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

add_executable(reconfigure_panel_server src/reconfigure_panel_server.cpp)
target_compile_options(reconfigure_panel_server PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(reconfigure_panel_server reconfigure_logic mobile_manipulator_geometry::geometry mobile_manipulator_navigation::scenario_tools
  rclcpp::rclcpp rclcpp_action::rclcpp_action ament_index_cpp::ament_index_cpp
  moveit_core::moveit_planning_scene moveit_core::moveit_robot_model moveit_core::moveit_robot_state
  srdfdom::srdfdom urdf::urdf ${mobile_manipulator_interfaces_TARGETS} ${control_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${moveit_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} yaml-cpp
  tf2::tf2 tf2_ros::tf2_ros)

add_executable(reconfigure_panel src/reconfigure_panel_cli.cpp)
target_compile_options(reconfigure_panel PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(reconfigure_panel rclcpp::rclcpp rclcpp_action::rclcpp_action tf2::tf2
  nlohmann_json::nlohmann_json ${mobile_manipulator_interfaces_TARGETS})

add_executable(transition_validity_check src/transition_validity_check.cpp)
target_compile_options(transition_validity_check PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(transition_validity_check rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  nlohmann_json::nlohmann_json ${moveit_msgs_TARGETS})

install(TARGETS planning_scene_loader reconfigure_panel_server reconfigure_panel transition_validity_check
  octomap_box_check
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scene_diff test/test_scene_diff.cpp)
  target_link_libraries(test_scene_diff scene_diff ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_octomap_box test/test_octomap_box.cpp)
  target_link_libraries(test_octomap_box octomap_box)
  ament_add_gtest(test_reconfigure_logic test/test_reconfigure_logic.cpp)
  target_link_libraries(test_reconfigure_logic reconfigure_logic ${mobile_manipulator_interfaces_TARGETS})
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_reconfigure_mock.py TIMEOUT 900)
  add_launch_test(test/test_reconfigure_octomap_mock.py TIMEOUT 600)
  add_launch_test(test/test_reconfigure_mock_dynamic.py TIMEOUT 600)
endif()

ament_package()
