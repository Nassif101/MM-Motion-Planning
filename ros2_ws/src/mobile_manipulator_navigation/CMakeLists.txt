cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY behavior_trees config docs launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/check_cmd_vel_ownership.py scripts/livox_robot_filter.py scripts/local_costmap_check.py scripts/navigate_scenario_task.py scripts/gate_planning_check.py scripts/lidar_self_return_probe.py scripts/odom_consistency_check.py scripts/plan_scenario_task.py scripts/scenario_spec.py scripts/sensor_transport_probe.py DESTINATION lib/${PROJECT_NAME})

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
  ament_add_pytest_test(
    validate_scenarios
    test/test_scenarios.py
  )
  ament_add_pytest_test(
    validate_lidar_robot_filter
    test/test_lidar_robot_filter.py
  )
  ament_add_pytest_test(
    validate_local_costmap_config
    test/test_local_costmap_config.py
  )
  ament_add_pytest_test(
    validate_navigation_config
    test/test_navigation_config.py
  )
endif()

ament_package()
