cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_medium CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hpipm_vendor REQUIRED)  # dense QP backend (BLASFEO + HPIPM)
find_package(osqp REQUIRED)          # 0.6.x from osqp_vendor
find_package(daqp_vendor REQUIRED)   # dense QP backend (DAQP)
find_package(mobile_manipulator_mpc REQUIRED)  # B5's model, collision, route, limits and static torque

# The medium layer without ROS (Phase 5A spec Sections 3 and 6): basis, problem, QP backends, SCP, planner.
add_library(medium STATIC src/bezier.cpp src/dense_qp_factory.cpp src/dense_qp_hpipm.cpp src/dense_qp_osqp.cpp
  src/dense_qp_daqp.cpp src/medium_config.cpp src/problem.cpp src/scp.cpp src/planner.cpp src/closed_loop.cpp)
target_compile_features(medium PUBLIC cxx_std_17)
target_compile_options(medium PRIVATE -Wall -Wextra -Wpedantic -O2)
set_target_properties(medium PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(medium PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(medium PUBLIC Eigen3::Eigen mobile_manipulator_mpc::mpc
  PRIVATE hpipm_vendor::hpipm osqp::osqp daqp_vendor::daqp)

# The open-loop bench (plan Task 9): the planner on recorded B5 states.
find_package(ament_index_cpp REQUIRED)
add_executable(medium_bench src/medium_bench.cpp)
target_compile_options(medium_bench PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(medium_bench medium mobile_manipulator_mpc::replay_bag ament_index_cpp::ament_index_cpp)
install(TARGETS medium_bench DESTINATION lib/${PROJECT_NAME})

# The medium-horizon node (plan Task 11).
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mobile_manipulator_interfaces REQUIRED)
add_executable(medium_horizon_node src/medium_horizon_node.cpp)
target_compile_options(medium_horizon_node PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(medium_horizon_node medium mobile_manipulator_mpc::ros_state ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp tf2_ros::tf2_ros ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${control_msgs_TARGETS}
  ${visualization_msgs_TARGETS} ${mobile_manipulator_interfaces_TARGETS})
install(TARGETS medium_horizon_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS medium EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 hpipm_vendor osqp daqp_vendor mobile_manipulator_mpc)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bezier test/test_bezier.cpp)
  target_link_libraries(test_bezier medium)
  ament_add_gtest(test_dense_qp test/test_dense_qp.cpp TIMEOUT 300)
  target_link_libraries(test_dense_qp medium)
  ament_add_gtest(test_problem test/test_problem.cpp TIMEOUT 300)
  target_link_libraries(test_problem medium ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_planner test/test_planner.cpp TIMEOUT 1200)
  target_link_libraries(test_planner medium ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp TIMEOUT 1200)
  target_link_libraries(test_closed_loop medium ament_index_cpp::ament_index_cpp)
  # the medium plans' latency and cap run on the wall clock: a loaded host (tests in parallel) changes the loop
  set_tests_properties(test_closed_loop PROPERTIES RUN_SERIAL TRUE)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_medium_horizon_node.py TIMEOUT 120)
endif()

ament_package()
