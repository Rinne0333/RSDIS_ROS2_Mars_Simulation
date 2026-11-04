cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ingenuity_teleop control.cpp)
target_compile_features(ingenuity_teleop PUBLIC cxx_std_17)
ament_target_dependencies(ingenuity_teleop nav_msgs rclcpp std_msgs)

install(TARGETS ingenuity_teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()