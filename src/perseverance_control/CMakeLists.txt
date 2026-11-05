cmake_minimum_required(VERSION 3.8)
project(perseverance_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(perseverance_node src/perseverance_node.cpp)
target_compile_features(perseverance_node PRIVATE cxx_std_17)
target_include_directories(perseverance_node PRIVATE include)
ament_target_dependencies(perseverance_node rclcpp geometry_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(perseverance_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS perseverance_node DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
