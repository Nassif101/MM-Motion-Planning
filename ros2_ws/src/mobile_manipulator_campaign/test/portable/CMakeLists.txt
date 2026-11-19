cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_goal_manager_portable LANGUAGES CXX)

find_package(mobile_manipulator_routing REQUIRED)
find_package(GTest REQUIRED)

add_library(portable_goal_manager STATIC ../../src/goal_manager.cpp)
target_compile_features(portable_goal_manager PUBLIC cxx_std_17)
target_compile_options(portable_goal_manager PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>)
target_include_directories(portable_goal_manager PUBLIC ../../include)
target_link_libraries(portable_goal_manager PUBLIC mobile_manipulator_routing::routing)

enable_testing()
add_executable(test_goal_manager ../test_goal_manager.cpp)
target_link_libraries(test_goal_manager PRIVATE portable_goal_manager GTest::gtest_main)
add_test(NAME portable_goal_manager COMMAND test_goal_manager)
