import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with path_buffer component."""
    container = ComposableNodeContainer(
            name='path_buffer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='path_buffer',
                    plugin='path_buffer::PathBuffer',
                    name='path_buffer'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])