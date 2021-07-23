from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='security_rgb',
            namespace='',
            executable='security_rgb'
            # prefix = ['xterm -e gdb -ex run --args']
        )])
