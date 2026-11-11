cmake_minimum_required(VERSION 3.16)
project(robot_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(robot_bridge_node
  src/robot_bridge_node.cpp
  src/robot_link.cpp
  communication/TCPserver.cpp
  communication/LidarParser.cpp
  communication/MotionParser.cpp
  communication/StatusParser.cpp
)
target_include_directories(robot_bridge_node PRIVATE include communication)
target_link_libraries(robot_bridge_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

install(TARGETS robot_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
