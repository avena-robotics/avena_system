from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

# from subprocess import call
# call(["python3", "/root/ros2_ws/src/avena_ros2/logic_bt/test/scripts/simple_action_server.py"])

def generate_launch_description():

    parameters_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'parameters_server'), 'launch', 'parameters_server.launch.py'))
    )

    rgb_diff = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'rgb_diff'), 'launch', 'rgb_diff_server.launch.py'))
    )

    security_rgb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'security_rgb'), 'launch', 'security_rgb.launch.py'))
    )

    return LaunchDescription([parameters_server, rgb_diff, security_rgb])
