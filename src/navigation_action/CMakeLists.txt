cmake_minimum_required(VERSION 3.8)
project(navigation_action)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
)

add_library(navigation_action SHARED
  src/navigation_server.cpp
  src/navigation_ui_client.cpp
  src/navigation_gazebo_bridge.cpp
)

ament_target_dependencies(navigation_action
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
)

rclcpp_components_register_nodes(navigation_action
  "rt2_navigation::NavigationServer"
  "rt2_navigation::NavigationUIClient"
)

install(TARGETS navigation_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()