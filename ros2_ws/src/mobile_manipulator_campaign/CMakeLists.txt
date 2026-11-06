cmake_minimum_required(VERSION 3.22)
project(mobile_manipulator_campaign CXX)

# The glue between the ROS-free MuJoCo plant and the framework (MuJoCo plant spec Section 2.1).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# The plant's speed is part of what it is for: never build it unoptimized by accident (the workspace's cb passes
# RelWithDebInfo; a bare colcon build passes none).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(mobile_manipulator_plant REQUIRED)
find_package(mobile_manipulator_mpc REQUIRED)
find_package(mobile_manipulator_medium REQUIRED)
find_package(mobile_manipulator_geometry REQUIRED)
find_package(mobile_manipulator_navigation REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)  # SHA-256 of the profile and models in the run log's Hello

# The glue: adapters, the MuJoCo plant behind the closed loop's interface, the node's arm command path, scenarios. No ROS
# graph; it links B5's arm_stream (its stream points are B5's own code, beside the JTC sampler's message types).
add_library(campaign STATIC src/adapters.cpp src/scenario_setup.cpp src/mujoco_plant.cpp src/arm_command_path.cpp src/replay.cpp)
target_include_directories(campaign PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(campaign PUBLIC mobile_manipulator_plant::plant mobile_manipulator_mpc::mpc mobile_manipulator_mpc::arm_stream)
set_target_properties(campaign PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Cells from recorded runs (rosbag2): the only part that reads ROS bags.
add_library(campaign_cases STATIC src/closed_loop_case.cpp src/session.cpp src/stream_server.cpp src/demo_server.cpp)
target_link_libraries(campaign_cases PUBLIC campaign mobile_manipulator_medium::bench_setup mobile_manipulator_mpc::replay_bag OpenSSL::Crypto)
set_target_properties(campaign_cases PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_executable(plant_replay src/plant_replay.cpp)
target_link_libraries(plant_replay campaign ament_index_cpp::ament_index_cpp)
add_executable(plant_campaign src/plant_campaign.cpp)
target_link_libraries(plant_campaign campaign_cases campaign ament_index_cpp::ament_index_cpp)
add_executable(plant_demo_server src/plant_demo_server.cpp)
target_link_libraries(plant_demo_server campaign_cases campaign ament_index_cpp::ament_index_cpp)
install(TARGETS plant_replay plant_campaign plant_demo_server DESTINATION lib/${PROJECT_NAME})
install(TARGETS campaign campaign_cases EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_profile_consistency test/test_profile_consistency.cpp)
  target_link_libraries(test_profile_consistency mobile_manipulator_plant::plant mobile_manipulator_medium::medium
                        ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_distance_agreement test/test_distance_agreement.cpp TIMEOUT 600)
  target_link_libraries(test_distance_agreement mobile_manipulator_plant::plant mobile_manipulator_mpc::mpc
                        ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_arm_command_path test/test_arm_command_path.cpp)
  target_link_libraries(test_arm_command_path campaign_cases ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_replay test/test_replay.cpp TIMEOUT 300)
  target_link_libraries(test_replay campaign ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_scenario_setup test/test_scenario_setup.cpp)
  target_link_libraries(test_scenario_setup campaign ament_index_cpp::ament_index_cpp)
  # E1 runs the corpus cells closed loop twice each (minutes)
  get_filename_component(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
  ament_add_gtest(test_session test/test_session.cpp TIMEOUT 900)
  target_link_libraries(test_session campaign_cases campaign ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_demo_server test/test_demo_server.cpp TIMEOUT 600)
  target_link_libraries(test_demo_server campaign_cases campaign ament_index_cpp::ament_index_cpp)
  target_compile_definitions(test_demo_server PRIVATE REPO_ROOT="${REPO_ROOT}")
  ament_add_gtest(test_equivalence test/test_equivalence.cpp TIMEOUT 3600)
  target_link_libraries(test_equivalence campaign_cases)
  target_compile_definitions(test_equivalence PRIVATE REPO_ROOT="${REPO_ROOT}"
    CORPUS="docs/experiments/mujoco-plant/corpus_campaign_a.json")
  set_tests_properties(test_equivalence PROPERTIES RUN_SERIAL TRUE)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mobile_manipulator_plant mobile_manipulator_medium mobile_manipulator_mpc OpenSSL)
ament_package()
