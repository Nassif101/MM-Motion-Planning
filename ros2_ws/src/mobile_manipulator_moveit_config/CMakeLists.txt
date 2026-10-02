cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(moveit_setup_srdf_plugins REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

# Headless Setup Assistant collision sampling for the SRDF disable_collisions block.
add_executable(generate_collision_matrix src/generate_collision_matrix.cpp)
target_compile_features(generate_collision_matrix PRIVATE cxx_std_17)
target_compile_options(generate_collision_matrix PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(generate_collision_matrix moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model moveit_setup_srdf_plugins::moveit_setup_srdf_plugins
  srdfdom::srdfdom urdf::urdf)
install(TARGETS generate_collision_matrix RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(validate_moveit_config test/test_moveit_config.py)
endif()

ament_package()
