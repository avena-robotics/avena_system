import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='generate_path_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='generate_path',
                    plugin='generate_path::GeneratePath',
                    name='generate_path'
                ),
            ],
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
        )
    ])
