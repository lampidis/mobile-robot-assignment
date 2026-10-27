cmake_minimum_required(VERSION 3.8)
project(turtle_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_nav_interfaces REQUIRED)

add_executable(navigation_action_server src/navigation_action_server.cpp)
ament_target_dependencies(navigation_action_server rclcpp rclcpp_action turtle_nav_interfaces)
set_target_properties(navigation_action_server PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

add_executable(navigation_controller src/navigation_controller.cpp)
ament_target_dependencies(navigation_controller
  rclcpp rclcpp_action geometry_msgs turtlesim turtle_nav_interfaces)
set_target_properties(navigation_controller PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

install(TARGETS
  navigation_action_server
  navigation_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
