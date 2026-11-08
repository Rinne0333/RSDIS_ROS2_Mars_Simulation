cmake_minimum_required(VERSION 3.8)
project(ingenuity_navigation)

# A lógica matemática é C++17 e não precisa de ROS 2.
# Apenas navigation.cpp e navigation_node.cpp utilizam rclcpp.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(ingenuity_controllers STATIC
  src/altitude_controller.cpp
  src/attitude_controller.cpp
  src/motor_mixer.cpp
  src/position_controller.cpp
  src/mission_manager.cpp
  src/yaw_controller.cpp
)
target_compile_features(ingenuity_controllers PUBLIC cxx_std_17)
target_include_directories(ingenuity_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(navigation_node
  src/navigation_node.cpp
  src/navigation.cpp
)
target_compile_features(navigation_node PUBLIC cxx_std_17)
target_link_libraries(navigation_node ingenuity_controllers)
target_include_directories(navigation_node PRIVATE include)
ament_target_dependencies(navigation_node
  rclcpp nav_msgs std_msgs geometry_msgs
)

# ROS 2 procura executáveis em lib/<package_name>.
install(TARGETS navigation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Testes matemáticos executáveis sem Gazebo (colcon test).
if(BUILD_TESTING)
  enable_testing()
  add_executable(controller_smoke_test test/controller_smoke_test.cpp)
  target_compile_features(controller_smoke_test PUBLIC cxx_std_17)
  target_link_libraries(controller_smoke_test ingenuity_controllers)
  add_test(NAME controller_smoke_test COMMAND controller_smoke_test)
endif()

ament_package()
