cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_mpc C CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mobile_manipulator_geometry REQUIRED)
find_package(mobile_manipulator_navigation REQUIRED)  # scenario/mission parsing for the route references
find_package(mobile_manipulator_description REQUIRED)  # URDF for the generated model
find_package(mobile_manipulator_control REQUIRED)      # qualified_payload.json
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(fcl REQUIRED)  # 0.7, as MoveIt: exact box and cylinder distances (GJK)
find_package(urdf REQUIRED)  # joint position limits (load_limits)
find_package(osqp REQUIRED)  # 0.6.x from osqp_vendor: the stage-wise QP backend

# Whole-body kinematic model, generated with CasADi from the installed robot description and
# payload at build time (the generated C is not committed); the runtime is C++ only.
get_filename_component(MM_DESCRIPTION_SHARE "${mobile_manipulator_description_DIR}/.." ABSOLUTE)
get_filename_component(MM_CONTROL_SHARE "${mobile_manipulator_control_DIR}/.." ABSOLUTE)
set(MODEL_URDF "${MM_DESCRIPTION_SHARE}/urdf/mobile_manipulator.urdf")
set(MODEL_PAYLOAD "${MM_CONTROL_SHARE}/config/qualified_payload.json")
set(MODEL_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(MODEL_OUTPUTS "${MODEL_DIR}/wholebody_model.c" "${MODEL_DIR}/wholebody_model.h"
  "${MODEL_DIR}/wholebody_model_links.h")
add_custom_command(
  OUTPUT ${MODEL_OUTPUTS}
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_model.py"
    --urdf "${MODEL_URDF}" --payload "${MODEL_PAYLOAD}" --out "${MODEL_DIR}"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_model.py" "${MODEL_URDF}" "${MODEL_PAYLOAD}"
  COMMENT "Generating the whole-body model with CasADi"
  VERBATIM)

add_library(wholebody_model_c STATIC "${MODEL_DIR}/wholebody_model.c")
target_compile_options(wholebody_model_c PRIVATE -O2 -w)  # generated code
set_target_properties(wholebody_model_c PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mpc STATIC src/model.cpp src/route.cpp src/references.cpp src/world.cpp src/collision.cpp src/osqp_solver.cpp
  src/formulation.cpp src/controller.cpp)
target_compile_features(mpc PUBLIC cxx_std_17)
target_compile_options(mpc PRIVATE -Wall -Wextra -Wpedantic -O2)
set_target_properties(mpc PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mpc
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE "${MODEL_DIR}")
target_link_libraries(mpc PUBLIC Eigen3::Eigen mobile_manipulator_geometry::geometry
  mobile_manipulator_navigation::scenario_tools
  PRIVATE wholebody_model_c fcl osqp::osqp urdf::urdf)

# The B5 FollowPath server (Section 2 of the Phase 4 spec).
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mobile_manipulator_interfaces REQUIRED)
add_executable(wholebody_mpc_node src/wholebody_mpc_node.cpp)
target_compile_options(wholebody_mpc_node PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(wholebody_mpc_node mpc rclcpp::rclcpp rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp tf2::tf2 tf2_ros::tf2_ros ${geometry_msgs_TARGETS} ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${trajectory_msgs_TARGETS} ${mobile_manipulator_interfaces_TARGETS})
install(TARGETS wholebody_mpc_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS mpc wholebody_model_c EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 fcl osqp mobile_manipulator_geometry mobile_manipulator_navigation)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_model test/test_model.cpp)
  target_link_libraries(test_model mpc ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_route test/test_route.cpp)
  target_link_libraries(test_route mpc)
  ament_add_gtest(test_references test/test_references.cpp)
  target_link_libraries(test_references mpc ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_world test/test_world.cpp)
  target_link_libraries(test_world mpc)
  ament_add_gtest(test_collision test/test_collision.cpp)
  target_link_libraries(test_collision mpc ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_osqp_solver test/test_osqp_solver.cpp)
  target_link_libraries(test_osqp_solver mpc)
  ament_add_gtest(test_formulation test/test_formulation.cpp TIMEOUT 600)
  target_link_libraries(test_formulation mpc ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_controller test/test_controller.cpp TIMEOUT 600)
  target_link_libraries(test_controller mpc ament_index_cpp::ament_index_cpp)
  add_launch_test(test/test_wholebody_mpc_node.py TIMEOUT 600)
  # The 1 ms timing check only fails in an optimized build (the project default is RelWithDebInfo).
  target_compile_definitions(test_collision PRIVATE
    MPC_OPTIMIZED_BUILD=$<IF:$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>,1,0>)
endif()

ament_package()
