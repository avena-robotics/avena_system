import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='bullet_server_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='bullet_server',
                    plugin='bullet_server::BulletServer',
                    name='bullet_server'
                ),
            ],
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
        )
    ])
