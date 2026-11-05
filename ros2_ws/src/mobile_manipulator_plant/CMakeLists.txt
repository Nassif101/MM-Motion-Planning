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

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The site export is read from the navigation package's source tree (a test input only: no package dependency).
  set(BOXES_YAML ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_manipulator_navigation/maps/construction_site_v2.boxes.yaml)
  file(SHA256 ${BOXES_YAML} BOXES_SHA256)
  ament_add_gtest(test_site_model test/test_site_model.cpp)
  target_link_libraries(test_site_model mujoco::mujoco yaml-cpp)
  # the installed model (colcon installs before testing): the install rule is part of what is tested
  target_compile_definitions(test_site_model PRIVATE
    SITE_MODEL="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/site_v2.xml" BOXES_YAML="${BOXES_YAML}"
    BOXES_SHA256="${BOXES_SHA256}")
endif()

ament_package()
