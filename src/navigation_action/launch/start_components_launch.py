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
            # Broadcaster TF2 (pubblica odom -> robot)
            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::GazeboBroadcaster',  # ATTENZIONE: controlla il nome esatto
                name='gazebo_broadcaster',
            ),
            # Action Server
            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::NavigationActionServer',  # controlla nome
                name='navigation_server',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Action Client (UI)
            ComposableNode(
                package='navigation_action',
                plugin='navigation_action::NavigationUIClient',  # controlla nome
                name='ui_client',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )
    return launch.LaunchDescription([container])