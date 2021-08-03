import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
            name='estimate_shape_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='estimate_shape',
                    plugin='estimate_shape::EstimateShape',
                    name='estimate_shape')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
