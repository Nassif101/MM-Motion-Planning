cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_manipulation)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mobile_manipulator_navigation REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Known MoveIt collision world: exported static boxes, scenario boxes, raised floor, panel.
add_library(scene_diff STATIC src/scene_diff.cpp)
target_compile_features(scene_diff PUBLIC cxx_std_17)
target_compile_options(scene_diff PRIVATE -Wall -Wextra -Wpedantic -O2)
target_include_directories(scene_diff PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(scene_diff mobile_manipulator_navigation::footprint_projection
  ${moveit_msgs_TARGETS} ${geometry_msgs_TARGETS} ${shape_msgs_TARGETS} Eigen3::Eigen yaml-cpp)

add_executable(planning_scene_loader src/planning_scene_loader.cpp)
target_compile_options(planning_scene_loader PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(planning_scene_loader scene_diff rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

install(TARGETS planning_scene_loader RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scene_diff test/test_scene_diff.cpp)
  target_link_libraries(test_scene_diff scene_diff)
endif()

ament_package()
