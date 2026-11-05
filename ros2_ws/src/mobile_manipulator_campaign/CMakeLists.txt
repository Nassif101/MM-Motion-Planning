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

add_library(campaign STATIC src/adapters.cpp src/scenario_setup.cpp src/mujoco_plant.cpp src/closed_loop_case.cpp)
target_include_directories(campaign PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(campaign PUBLIC mobile_manipulator_plant::plant mobile_manipulator_medium::bench_setup
  mobile_manipulator_mpc::replay_bag)
set_target_properties(campaign PROPERTIES POSITION_INDEPENDENT_CODE ON)
install(TARGETS campaign EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_profile_consistency test/test_profile_consistency.cpp)
  target_link_libraries(test_profile_consistency mobile_manipulator_plant::plant mobile_manipulator_medium::medium
                        ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_distance_agreement test/test_distance_agreement.cpp TIMEOUT 600)
  target_link_libraries(test_distance_agreement mobile_manipulator_plant::plant mobile_manipulator_mpc::mpc
                        ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_scenario_setup test/test_scenario_setup.cpp)
  target_link_libraries(test_scenario_setup campaign)
  # E1 runs the corpus cells closed loop twice each (minutes)
  get_filename_component(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
  ament_add_gtest(test_equivalence test/test_equivalence.cpp TIMEOUT 3600)
  target_link_libraries(test_equivalence campaign)
  target_compile_definitions(test_equivalence PRIVATE REPO_ROOT="${REPO_ROOT}"
    CORPUS="docs/experiments/mujoco-plant/corpus_campaign_a.json")
  set_tests_properties(test_equivalence PROPERTIES RUN_SERIAL TRUE)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mobile_manipulator_plant mobile_manipulator_medium mobile_manipulator_mpc)
ament_package()
