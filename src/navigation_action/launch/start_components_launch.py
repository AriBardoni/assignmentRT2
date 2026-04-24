import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='navigation_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            # Broadcaster TF2
            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::RobotBroadcaster',
                name='robot_broadcaster',
            ),
            # Action Server
            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::NavigationServer',
                name='navigation_server',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # NOTA: il client NON è più qui - va eseguito separatamente!
        ],
    )
    return launch.LaunchDescription([container])