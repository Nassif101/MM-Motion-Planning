cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_routing VERSION 0.1.0 LANGUAGES CXX)

option(ROUTING_WITH_AMENT "Export the portable target into an ament workspace" OFF)
option(ROUTING_BUILD_TESTS "Build routing behavioral tests" ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
add_library(routing src/grid.cpp src/map_io.cpp src/primitives.cpp src/footprint.cpp src/search.cpp src/heuristic.cpp)
add_library(mobile_manipulator_routing::routing ALIAS routing)
target_compile_features(routing PUBLIC cxx_std_17)
target_compile_options(routing PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>)
target_include_directories(routing PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(routing PRIVATE yaml-cpp nlohmann_json::nlohmann_json)
set_target_properties(routing PROPERTIES POSITION_INDEPENDENT_CODE ON)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/data/diff_5cm.json" ROUTING_PRIMITIVE_SHA256)
if(NOT ROUTING_PRIMITIVE_SHA256 STREQUAL "105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9")
  message(FATAL_ERROR "The provisioned Nav2 motion data does not match its recorded SHA-256")
endif()
add_executable(route_check src/route_check.cpp)
target_link_libraries(route_check PRIVATE routing nlohmann_json::nlohmann_json)

include(GNUInstallDirs)
set(ROUTING_CMAKE_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
install(TARGETS routing EXPORT RoutingTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS route_check RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY data config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
# Optional discovery metadata for the existing workspace adapter. The core does not link ROS.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker" "")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker"
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ament_index/resource_index/packages RENAME ${PROJECT_NAME})
install(EXPORT RoutingTargets NAMESPACE mobile_manipulator_routing:: DESTINATION ${ROUTING_CMAKE_INSTALL_DIR})
include(CMakePackageConfigHelpers)
set(ROUTING_DATA_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/data)
set(ROUTING_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config)
configure_package_config_file(cmake/RoutingConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mobile_manipulator_routingConfig.cmake
  INSTALL_DESTINATION ${ROUTING_CMAKE_INSTALL_DIR}
  PATH_VARS ROUTING_DATA_INSTALL_DIR ROUTING_CONFIG_INSTALL_DIR)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/mobile_manipulator_routingConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/mobile_manipulator_routingConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/mobile_manipulator_routingConfigVersion.cmake
  DESTINATION ${ROUTING_CMAKE_INSTALL_DIR})

if(ROUTING_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  foreach(name grid map_io primitives footprint search)
    add_executable(test_${name} test/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE routing GTest::gtest_main nlohmann_json::nlohmann_json)
    target_compile_definitions(test_${name} PRIVATE ROUTING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME routing_${name} COMMAND test_${name})
  endforeach()
endif()

if(ROUTING_WITH_AMENT)
  find_package(ament_cmake REQUIRED)
  ament_export_targets(RoutingTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(yaml-cpp nlohmann_json)
  ament_package()
endif()
