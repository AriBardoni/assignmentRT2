import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',  # <-- OBBLIGATORIO anche se vuoto
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::NavigationServer',
                name='navigation_server',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )
    return launch.LaunchDescription([container])