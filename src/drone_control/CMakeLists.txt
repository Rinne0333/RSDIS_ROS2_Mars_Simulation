cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ingenuity_teleop control.cpp)
target_compile_features(ingenuity_teleop PUBLIC cxx_std_17)
ament_target_dependencies(ingenuity_teleop geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(ingenuity_flight_controller control.cpp)
target_compile_features(ingenuity_flight_controller PUBLIC cxx_std_17)
target_compile_definitions(ingenuity_flight_controller PRIVATE INGENUITY_FLIGHT_CONTROLLER)
ament_target_dependencies(ingenuity_flight_controller geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(ingenuity_area_scan area_scan.cpp)
target_compile_features(ingenuity_area_scan PUBLIC cxx_std_17)
ament_target_dependencies(ingenuity_area_scan geometry_msgs nav_msgs rclcpp)

install(TARGETS ingenuity_teleop
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ingenuity_flight_controller
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ingenuity_area_scan
  DESTINATION lib/${PROJECT_NAME})

ament_package()