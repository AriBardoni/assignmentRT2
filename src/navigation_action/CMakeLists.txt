cmake_minimum_required(VERSION 3.8)
project(navigation_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# Dopo rosidl_generate_interfaces, aggiungi:
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
)

# === Broadcaster ===
add_library(robot_broadcaster_component SHARED
  src/robot_broadcaster.cpp
)
ament_target_dependencies(robot_broadcaster_component
  rclcpp
  rclcpp_components
  tf2_ros
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(robot_broadcaster_component
  PLUGIN "navigation_action::RobotBroadcaster"
  EXECUTABLE robot_broadcaster_exe
)

# === Server ===
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
# Aggiunge la directory con gli header generati
target_include_directories(navigation_server_component PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)
rclcpp_components_register_node(navigation_server_component
  PLUGIN "navigation_action::NavigationServer"
  EXECUTABLE navigation_server_exe
)

# === Client ===
add_executable(navigation_ui_client
  src/navigation_ui_client.cpp
)
ament_target_dependencies(navigation_ui_client
  rclcpp
  rclcpp_action
)
# Aggiunge la directory con gli header generati
target_include_directories(navigation_ui_client PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Install
install(TARGETS
  robot_broadcaster_component
  navigation_server_component
  navigation_ui_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()