import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',   # QUESTO È OBBLIGATORIO SU JAZZY
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',

        composable_node_descriptions=[

            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::RobotBroadcaster',
                name='robot_broadcaster'
            ),

            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::NavigationServer',
                name='navigation_server'
            ),
        ],
    )

    return launch.LaunchDescription([container])