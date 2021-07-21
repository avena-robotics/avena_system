import launch
import sys
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    container = ComposableNodeContainer(
        name='generate_trajectory_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='generate_trajectory',
                plugin='generate_trajectory::GenerateTrajectory',
                name='generate_trajectory'
                )
        ],
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
    )

    return launch.LaunchDescription([container])
