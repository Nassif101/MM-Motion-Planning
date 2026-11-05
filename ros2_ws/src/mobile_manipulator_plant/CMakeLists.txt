cmake_minimum_required(VERSION 3.22)
project(mobile_manipulator_plant CXX)

# The ROS-free MuJoCo campaign plant (MuJoCo plant spec): depends on mujoco_vendor, Eigen, yaml-cpp, nlohmann_json and
# zlib only; no ROS graph, no messages, no other mobile_manipulator package.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(nlohmann_json REQUIRED)

add_library(plant STATIC src/robot_profile.cpp src/movers.cpp src/model_builder.cpp src/plant.cpp)
target_include_directories(plant PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(plant PUBLIC mujoco::mujoco Eigen3::Eigen yaml-cpp nlohmann_json::nlohmann_json)
set_target_properties(plant PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS plant EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models robots DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mujoco_vendor Eigen3 yaml-cpp nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The site export is read from the navigation package's source tree (a test input only: no package dependency).
  set(BOXES_YAML ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_manipulator_navigation/maps/construction_site_v2.boxes.yaml)
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
