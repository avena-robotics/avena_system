from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='visualization_tools',
            executable='go_to_pose_command_node',
            output='both',
            parameters=[{'log_level': 'info'}],
        ),
    ])
