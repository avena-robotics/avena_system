import launch
import sys
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    container = ComposableNodeContainer(
        name='motion_planning_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='motion_planning',
                plugin='motion_planning::MotionPlanning',
                name='motion_planning',
                parameters=[{'log_level': 'debug'}],
            ),
        ],
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
    )

    return launch.LaunchDescription([container])
