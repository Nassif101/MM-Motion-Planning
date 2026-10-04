cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_geometry)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(nlohmann_json REQUIRED)

# Robot collision model, ground projection and footprint polygons; no ROS graph, so the
# navigation, manipulation and (later) MPC code share one geometry.
add_library(geometry STATIC src/robot_model.cpp src/footprint_projection.cpp src/polygon_ops.cpp
  src/footprint_model.cpp)
target_compile_features(geometry PUBLIC cxx_std_17)
target_compile_options(geometry PRIVATE -Wall -Wextra -Wpedantic -O2)
set_target_properties(geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(geometry PUBLIC Eigen3::Eigen urdf::urdf nlohmann_json::nlohmann_json)

install(TARGETS geometry EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 urdf nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_projection test/test_projection.cpp)
  target_link_libraries(test_projection geometry ament_index_cpp::ament_index_cpp)
  ament_add_gtest(test_polygon_ops test/test_polygon_ops.cpp)
  target_link_libraries(test_polygon_ops geometry)
endif()

ament_package()
