import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    working_side = 'left'  # "left" or "right"
    robot = 'franka' if working_side == 'right' else 'avena'     

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
                'avena_bringup'), 'launch', 'robot.launch.py')),
            launch_arguments={
                'robot_xacro_file': f'{robot}_arm.urdf.xacro',
                'xacro_arguments': f'side:={working_side}'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
                'avena_bringup'), 'launch', 'calibration.launch.py')),
            launch_arguments={
                'working_side': working_side,
                'robot': robot}.items()
        ),
    ])
