cmake_minimum_required(VERSION 3.16)
project(parkinglot_dataset)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(PROJ REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)

## C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## Executables
add_executable(${PROJECT_NAME}_ground_truth src/ground_truth.cpp)
ament_target_dependencies(${PROJECT_NAME}_ground_truth
  PROJ
  rclcpp rosbag2 rosbag2_cpp
  tf2 tf2_ros tf2_eigen
  sensor_msgs nav_msgs
  steam
  vtr_common vtr_logging
)

add_executable(${PROJECT_NAME}_velocity_cost src/velocity_cost.cpp)
ament_target_dependencies(${PROJECT_NAME}_velocity_cost
  PROJ
  rclcpp rosbag2 rosbag2_cpp
  tf2 tf2_ros tf2_eigen
  sensor_msgs nav_msgs
  steam
  vtr_common vtr_logging
)

install(
  TARGETS
    ${PROJECT_NAME}_ground_truth
    ${PROJECT_NAME}_velocity_cost
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
