cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    validate_mobile_manipulator_description
    test/validate_description.py
  )
endif()

ament_package()
