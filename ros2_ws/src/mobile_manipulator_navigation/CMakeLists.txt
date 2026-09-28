cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gate_planning_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    validate_unity_nav2_map
    test/test_unity_nav2_map.py
  )
  ament_add_pytest_test(
    validate_footprint_profiles
    test/test_footprint_profiles.py
  )
  ament_add_pytest_test(
    validate_operating_envelope
    test/test_operating_envelope.py
  )
  ament_add_pytest_test(
    validate_planner_config
    test/test_planner_config.py
  )
endif()

ament_package()
