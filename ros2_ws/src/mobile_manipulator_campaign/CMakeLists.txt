cmake_minimum_required(VERSION 3.22)
project(mobile_manipulator_campaign CXX)

# The glue between the ROS-free MuJoCo plant and the framework (MuJoCo plant spec Section 2.1).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(mobile_manipulator_plant REQUIRED)
find_package(mobile_manipulator_mpc REQUIRED)
find_package(mobile_manipulator_medium REQUIRED)
find_package(mobile_manipulator_geometry REQUIRED)
find_package(mobile_manipulator_navigation REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_profile_consistency test/test_profile_consistency.cpp)
  target_link_libraries(test_profile_consistency mobile_manipulator_plant::plant mobile_manipulator_medium::medium
                        ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_distance_agreement test/test_distance_agreement.cpp TIMEOUT 600)
  target_link_libraries(test_distance_agreement mobile_manipulator_plant::plant mobile_manipulator_mpc::mpc
                        ament_index_cpp::ament_index_cpp)
endif()

ament_package()
