cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    validate_unity_nav2_map
    test/test_unity_nav2_map.py
  )
endif()

ament_package()
