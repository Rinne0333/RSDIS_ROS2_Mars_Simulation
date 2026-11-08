cmake_minimum_required(VERSION 3.8)
project(x500_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(offboard_scan src/offboard_scan.cpp)
ament_target_dependencies(offboard_scan rclcpp px4_msgs geometry_msgs nav_msgs)
target_include_directories(offboard_scan PRIVATE include)

add_executable(rover_investigator src/rover_investigator.cpp)
ament_target_dependencies(rover_investigator rclcpp geometry_msgs nav_msgs std_msgs)
target_include_directories(rover_investigator PRIVATE include)

install(TARGETS offboard_scan rover_investigator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
