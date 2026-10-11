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

add_library(mpc STATIC src/model.cpp src/route.cpp src/references.cpp src/world.cpp src/collision.cpp)
target_compile_features(mpc PUBLIC cxx_std_17)
target_compile_options(mpc PRIVATE -Wall -Wextra -Wpedantic -O2)
set_target_properties(mpc PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mpc
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE "${MODEL_DIR}")
target_link_libraries(mpc PUBLIC Eigen3::Eigen mobile_manipulator_geometry::geometry
  mobile_manipulator_navigation::scenario_tools
  PRIVATE wholebody_model_c fcl)

install(TARGETS mpc wholebody_model_c EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 fcl mobile_manipulator_geometry mobile_manipulator_navigation)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
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
  # The 1 ms timing check only fails in an optimized build (the project default is RelWithDebInfo).
  target_compile_definitions(test_collision PRIVATE
    MPC_OPTIMIZED_BUILD=$<IF:$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>,1,0>)
endif()

ament_package()
