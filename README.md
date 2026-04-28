Research Track 2 – Assignment 1

The project implements a ROS2 Action-based navigation system for a mobile robot in Gazebo simulation.

The project relies on ROS2 Actions, Components, TF2 transforms, publishers and subscribers to allow the robot to autonomously reach a desired target position.

-System Architecture

The system is composed of two main parts:
- Navigation Action Server – receives navigation goals and controls the robot motion.
- Navigation UI Client – allows the user to send goals, cancel them, and monitor feedback.

-Navigation Action Server

The server receives a target pose composed of:
- x coordinate
- y coordinate
- theta orientation

The node performs autonomous navigation using a finite state controller.

The node:
- receives goals through a ROS2 Action interface
- publishes velocity commands to the robot using the /cmd_vel topic
- subscribes to /odom to estimate robot position and orientation
- publishes feedback during navigation
- supports goal cancellation
- returns final result when the goal is reached

The navigation behaviour is divided into states:
- State 0 – rotate toward target
- State 1 – move straight toward target
- State 2 – adjust final orientation
- State 3 – goal completed

Robot movement is controlled using geometry_msgs/Twist messages, where:
- linear.x controls forward motion
- angular.z controls rotation

-Navigation UI Client

This node provides a simple terminal interface for the user.

The user can:
- send a new navigation goal
- cancel the current goal
- exit the client

During execution, the client receives and displays feedback from the server.

-Action Definition

The project defines a custom ROS2 Action: Navigate.action

Goal:
- float64 x
- float64 y
- float64 theta

Result:
- bool success
- string message

Feedback:
- float64 remaining_distance
- float64 remaining_angle
- int32 state

-Components

The Navigation Server is implemented as a ROS2 Component Node and loaded dynamically inside a component container.

This allows:
- modular execution
- runtime loading
- better resource management

-How to Run
TERMINAL 1: SIMULATION
cd ~/desktop/assignmentRT2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select navigation_action
source install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py

TERMINAL 2 - CONTAINER WITH COMPONENTS:
cd ~/desktop/assignmentRT2
source install/setup.bash
ros2 launch navigation_action start_components_launch.py

TERMINAL 3 - CLIENT:
cd ~/desktop/assignmentRT2
source install/setup.bash
ros2 run navigation_action navigation_ui_client

To send a goal, write send in the client terminal and insert x,y and theta values:
send 
2 2 0 
