import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='setup_bullet_scene_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='bullet_server',
                    plugin='bullet_server::SetupScene',
                    parameters=[{'log_level': 'info'}],
                ),
            ],
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
        )
    ])
