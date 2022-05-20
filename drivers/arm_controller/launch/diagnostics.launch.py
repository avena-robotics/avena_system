import os
import subprocess

import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    config_path = os.path.join(os.path.split(
        os.path.abspath(os.pardir))[0], "config")

    pkg_share = FindPackageShare('arm_controller').find('arm_controller')
    config = os.path.join(pkg_share, 'config')
    params = os.path.join(config, 'diagnostics.yaml')
    tq_arg = DeclareLaunchArgument('tq', default_value='12.')
    serial_arg = DeclareLaunchArgument('serial_number', default_value='NA')
    note_arg = DeclareLaunchArgument('note', default_value='NA')

    return [tq_arg, serial_arg, note_arg,
            Node(
                package='arm_controller',
                executable='hw_interface',
                output='both',
                parameters=[params]
                # prefix=['valgrind --leak-check=yes --track-origins=yes']
            ),
            Node(
                package='arm_controller',
                executable='diagnostics',
                output='both',
                parameters=[params, {'config_path': config, 'tq': LaunchConfiguration(
                    'tq'), 'serial_number': LaunchConfiguration('serial_number'), 'note': LaunchConfiguration('note')}]
            )
            ]


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(
            function=launch_setup
        )
    ])
