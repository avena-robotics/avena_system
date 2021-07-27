from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import os


def generate_launch_description():

    parameters_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'parameters_server'), 'launch', 'parameters_server.launch.py'))
    )
    data_streams = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'avena_bringup'), 'launch', 'data_streams.launch.py'))
    )
    system_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'system_monitor'), 'launch', 'system_monitor.launch.py'))
    )
    security_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'security_controller'), 'launch', 'security_controller_server.launch.py'))
    )

    return LaunchDescription([parameters_server, data_streams, system_monitor, security_controller,])
