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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(offboard_scan src/offboard_scan.cpp)
ament_target_dependencies(offboard_scan rclcpp px4_msgs geometry_msgs nav_msgs)
target_include_directories(offboard_scan PRIVATE include)

add_executable(lander_detector src/lander_detector.cpp)
ament_target_dependencies(lander_detector rclcpp sensor_msgs geometry_msgs cv_bridge)
target_include_directories(lander_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lander_detector ${OpenCV_LIBS})

add_executable(poi_locator src/poi_locator.cpp)
ament_target_dependencies(poi_locator rclcpp px4_msgs geometry_msgs)
target_include_directories(poi_locator PRIVATE include)

install(TARGETS offboard_scan lander_detector poi_locator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
