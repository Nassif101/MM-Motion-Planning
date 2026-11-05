cmake_minimum_required(VERSION 3.22)
project(mobile_manipulator_plant CXX)

# The ROS-free MuJoCo campaign plant (MuJoCo plant spec): depends on mujoco_vendor, Eigen, yaml-cpp, nlohmann_json and
# zlib only; no ROS graph, no messages, no other mobile_manipulator package.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# The plant's speed is part of what it is for: never build it unoptimized by accident (the workspace's cb passes
# RelWithDebInfo; a bare colcon build passes none).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(nlohmann_json REQUIRED)

add_library(plant STATIC src/robot_profile.cpp src/movers.cpp src/model_builder.cpp src/plant.cpp src/lidar_noise.cpp
  src/lidar.cpp src/static_bvh.cpp)
target_include_directories(plant PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(plant PUBLIC mujoco::mujoco Eigen3::Eigen yaml-cpp nlohmann_json::nlohmann_json)
set_target_properties(plant PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The Livox Mid-360 scan pattern (MIT, Livox) as UnitySensors ships it, at the commit the Unity project pins
# (motion-planning-sim/Packages/packages-lock.json): fetched at configure time, checked, converted (spec Section 5.1).
set(SCAN_PATTERN_COMMIT 91698e3593abdb04baac022a670cc52fee027238)
set(SCAN_PATTERN_URL https://raw.githubusercontent.com/Field-Robotics-Japan/UnitySensors/${SCAN_PATTERN_COMMIT}/Packages/UnitySensors/Runtime/RawData/LivoxScanPattern)
set(SCAN_PATTERN_CSV ${CMAKE_CURRENT_BINARY_DIR}/mid360.csv)
set(SCAN_PATTERN_LICENSE ${CMAKE_CURRENT_BINARY_DIR}/LICENSE.livox_scan_pattern)
# Unity's package cache holds the same files: the fallback for an offline build (hash-checked as well).
file(GLOB SCAN_PATTERN_UNITY_CACHE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion-planning-sim/Library/PackageCache/com.frj.unity-sensors@*/Runtime/RawData/LivoxScanPattern)
foreach(item "mid360.csv;${SCAN_PATTERN_CSV};ed5f7fb5dcfa6a753ff9a58a687d90fb72af64026443fe655e7cb84e341c26a7"
             "LICENSE;${SCAN_PATTERN_LICENSE};96cc49491aed23ba9035db9c074efd6a8fb19f930979ee0d8bd926b886dfb336")
  list(GET item 0 name)
  list(GET item 1 path)
  list(GET item 2 hash)
  # with EXPECTED_HASH an existing file with the right hash is kept, a wrong or partial one is fetched again
  file(DOWNLOAD ${SCAN_PATTERN_URL}/${name} ${path} EXPECTED_HASH SHA256=${hash} STATUS status)
  list(GET status 0 code)
  if(NOT code EQUAL 0)
    file(REMOVE ${path})
    foreach(dir ${SCAN_PATTERN_UNITY_CACHE})
      if(EXISTS ${dir}/${name})
        file(SHA256 ${dir}/${name} cached)
        if(cached STREQUAL hash)
          configure_file(${dir}/${name} ${path} COPYONLY)
          set(code 0)
          message(STATUS "mobile_manipulator_plant: the scan pattern's ${name} from Unity's package cache (offline)")
        endif()
      endif()
    endforeach()
  endif()
  if(NOT code EQUAL 0)
    message(FATAL_ERROR "mobile_manipulator_plant: fetching the scan pattern's ${name} failed: ${status}")
  endif()
endforeach()
add_executable(make_scan_pattern src/make_scan_pattern.cpp)
set(SCAN_PATTERN_BIN ${CMAKE_CURRENT_BINARY_DIR}/lidar/mid360.bin)
add_custom_command(OUTPUT ${SCAN_PATTERN_BIN}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/lidar
  COMMAND make_scan_pattern ${SCAN_PATTERN_CSV} ${SCAN_PATTERN_BIN}
  DEPENDS make_scan_pattern ${SCAN_PATTERN_CSV})
add_custom_target(scan_pattern ALL DEPENDS ${SCAN_PATTERN_BIN})
install(FILES ${SCAN_PATTERN_BIN} ${SCAN_PATTERN_LICENSE} DESTINATION share/${PROJECT_NAME}/lidar)

# The lidar's cost benchmark (plan Task 5 Step 4).
add_executable(lidar_bench bench/lidar_bench.cpp)
target_link_libraries(lidar_bench plant)
install(TARGETS lidar_bench DESTINATION lib/${PROJECT_NAME})

install(TARGETS plant EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models robots DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mujoco_vendor Eigen3 yaml-cpp nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The site export is read from the navigation package's source tree (a test input only: no package dependency).
  set(BOXES_YAML ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_manipulator_navigation/maps/construction_site_v2.boxes.yaml)
  ament_add_gtest(test_lidar test/test_lidar.cpp TIMEOUT 300)
  target_link_libraries(test_lidar plant)
  add_dependencies(test_lidar scan_pattern)
  target_compile_definitions(test_lidar PRIVATE PACKAGES_SRC="${CMAKE_CURRENT_SOURCE_DIR}/.."
    PROFILE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/robots/mobile_manipulator/profile.yaml"
    SITE_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/site_v2.xml" PATTERN_BIN="${SCAN_PATTERN_BIN}")
  ament_add_gtest(test_static_bvh test/test_static_bvh.cpp)
  target_link_libraries(test_static_bvh plant)
  ament_add_gtest(test_lidar_noise test/test_lidar_noise.cpp)
  target_link_libraries(test_lidar_noise plant)
  ament_add_gtest(test_movers test/test_movers.cpp)
  target_link_libraries(test_movers plant)
  ament_add_gtest(test_plant_ideal test/test_plant_ideal.cpp)
  target_link_libraries(test_plant_ideal plant)
  target_compile_definitions(test_plant_ideal PRIVATE PACKAGES_SRC="${CMAKE_CURRENT_SOURCE_DIR}/.."
    PROFILE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/robots/mobile_manipulator/profile.yaml"
    SITE_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/site_v2.xml")
  ament_add_gtest(test_robot_profile test/test_robot_profile.cpp)
  target_link_libraries(test_robot_profile plant)
  target_compile_definitions(test_robot_profile PRIVATE PACKAGES_SRC="${CMAKE_CURRENT_SOURCE_DIR}/.."
    PROFILE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/robots/mobile_manipulator/profile.yaml")
  file(SHA256 ${BOXES_YAML} BOXES_SHA256)
  ament_add_gtest(test_site_model test/test_site_model.cpp)
  target_link_libraries(test_site_model mujoco::mujoco yaml-cpp)
  # the installed model (colcon installs before testing): the install rule is part of what is tested
  target_compile_definitions(test_site_model PRIVATE
    SITE_MODEL="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/site_v2.xml" BOXES_YAML="${BOXES_YAML}"
    BOXES_SHA256="${BOXES_SHA256}")
endif()

ament_package()
