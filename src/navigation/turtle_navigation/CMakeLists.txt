cmake_minimum_required(VERSION 3.8)
project(turtle_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtle_nav_interfaces REQUIRED)

add_executable(navigation_action_server src/navigation_action_server.cpp)
ament_target_dependencies(navigation_action_server rclcpp turtle_nav_interfaces)

add_executable(navigation_controller src/navigation_controller.cpp)
ament_target_dependencies(navigation_controller rclcpp turtle_nav_interfaces)

install(TARGETS
  navigation_action_server
  navigation_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
