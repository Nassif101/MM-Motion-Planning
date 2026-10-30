cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_medium CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# The medium layer without ROS (Phase 5A spec Sections 3 and 6): basis, problem, QP backends, SCP, planner.
add_library(medium STATIC src/bezier.cpp)
target_compile_features(medium PUBLIC cxx_std_17)
target_compile_options(medium PRIVATE -Wall -Wextra -Wpedantic -O2)
set_target_properties(medium PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(medium PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(medium PUBLIC Eigen3::Eigen)

install(TARGETS medium EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bezier test/test_bezier.cpp)
  target_link_libraries(test_bezier medium)
endif()

ament_package()
