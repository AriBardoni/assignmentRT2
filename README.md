Research Track 2 – Assignment 2

The project implements a ROS2 Action-based navigation system for a mobile robot in Gazebo simulation.

The project relies on ROS2 Actions, Components, TF2 transforms, publishers and subscribers to allow the robot to autonomously reach a desired target position.

The system is composed of two main parts:
*Navigation Action Server – receives navigation goals and controls the robot motion.
*Navigation UI Client – allows the user to send goals, cancel them, and monitor feedback.

The Navigation Action Server:

The server receives a target pose composed of:
*x coordinate
*y coordinate
*theta orientation
The node performs autonomous navigation using a finite state controller.

The node:

*receives goals through a ROS2 Action interface
*publishes velocity commands to the robot using the /cmd_vel topic
*uses TF2 transforms to estimate robot pose from /odom
*publishes feedback during navigation
*supports goal cancellation
*returns final result when the goal is reached

The navigation behaviour is divided into states:
*State 0 – rotate toward target
*State 1 – move straight toward target
*State 2 – adjust final orientation
*State 3 – goal completed

Robot movement is controlled using geometry_msgs/Twist messages, where:
*linear.x controls forward motion
*angular.z controls rotation

The Navigation UI Client:
This node provides a simple terminal interface for the user.

The user can:
*send a new navigation goal
*cancel the current goal
*exit the client
During execution, the client receives and displays feedback from the server.

Action Definition:
The project defines a custom ROS2 Action:
Navigate.action

Goal:
*float64 x
*float64 y
*float64 theta

Result:
*bool success
*string message

Feedback:
*float64 remaining_distance
*float64 remaining_angle
*int32 state

Components:
The Navigation Server is implemented as a ROS2 Component Node and loaded dynamically inside a component container.

This allows:
*modular execution
*runtime loading
*better resource management

To Run the Code:

Build the workspace:
colcon build

Source ROS2:
source /opt/ros/jazzy/setup.bash

Source workspace:
source install/setup.bash

Launch Gazebo simulation:
ros2 launch bme_gazebo_sensors spwn_robot_ex.launch.py

Run component container:
ros2 run rclcpp_components component_container_mt

Load Navigation Server component:
ros2 component load /ComponentManager navigation_action navigation_action::NavigationServer

Run Action Client:
ros2 run navigation_action navigation_ui_client

To send a goal from terminal:
ros2 action send_goal /navigate navigation_action/action/Navigate "{x: 2.0, y: 2.0, theta: 0.0}"

To monitor robot velocity:
ros2 topic echo /cmd_vel

To monitor odometry:
ros2 topic echo /odom

To cancel a goal:
use the client command:
cancel

Example client usage:
send
2 2 0
