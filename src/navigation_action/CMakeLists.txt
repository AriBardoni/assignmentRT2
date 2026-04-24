cmake_minimum_required(VERSION 3.8)
project(navigation_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
  DEPENDENCIES geometry_msgs
)

# === Component 1: Gazebo Broadcaster (TF2) ===
add_library(gazebo_broadcaster_component SHARED
  src/navigation_gazebo_broadcaster.cpp
)
ament_target_dependencies(gazebo_broadcaster_component
  rclcpp
  rclcpp_components
  tf2_ros
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(gazebo_broadcaster_component
  PLUGIN "navigation_action::GazeboBroadcaster"
  EXECUTABLE gazebo_broadcaster_exe
)

# === Component 2: Action Server ===
add_library(navigation_server_component SHARED
  src/navigation_action_server.cpp
)
ament_target_dependencies(navigation_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
rclcpp_components_register_node(navigation_server_component
  PLUGIN "navigation_action::NavigationServer"
  EXECUTABLE navigation_server_exe
)

# === Executable: UI Client (non componente, per interazione console) ===
add_executable(navigation_ui_client
  src/navigation_ui_client.cpp
)
ament_target_dependencies(navigation_ui_client
  rclcpp
  rclcpp_action
)

# Install libraries and executables
install(TARGETS
  gazebo_broadcaster_component
  navigation_server_component
  navigation_ui_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()