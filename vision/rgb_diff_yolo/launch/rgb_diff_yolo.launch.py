from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rgb_diff_yolo',
            executable='rgb_diff_yolo',
        )
    ])
