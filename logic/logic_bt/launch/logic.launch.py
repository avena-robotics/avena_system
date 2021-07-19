from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logic_bt',
            namespace='',
            executable='logic_bt'
            # prefix = ['xterm -e gdb -ex run --args']
        )])

