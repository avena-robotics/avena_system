import os
import subprocess

import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    config_path=os.path.join(os.path.split(os.path.abspath(os.pardir))[0],"config")

    pkg_share = FindPackageShare('arm_controller').find('arm_controller')
    config = os.path.join(pkg_share,'config')
    params = os.path.join(config,'diagnostics.yaml')

    return [
        Node(
            package='arm_controller',
            executable='hw_interface',
            output='both',
            parameters=[{'loop_frequency':500.}]
            # prefix=['valgrind --leak-check=yes --track-origins=yes']
        ),
        Node(
            package='arm_controller',
            executable='diagnostics',
            output='both',
            parameters=[params, {'config_path':config}]
        )
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(
            function=launch_setup
        )
    ])
