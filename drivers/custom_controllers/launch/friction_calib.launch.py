import os
import subprocess

import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    

    pkg_share = FindPackageShare('custom_controllers').find('custom_controllers')
    config = os.path.join(pkg_share,'config')
    params = os.path.join(config,'simple_controller_friction.yaml')

    return [
        Node(
            package='custom_controllers',
            executable='friction_calibration',
            output='both',
            parameters=[params, {'config_path':config}]
        )
        # ,
        # Node(
        #     package='candriver',
        #     executable='can_node',
        #     output='both',
        #     parameters=[params]
        # ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(
            function=launch_setup
        )
    ])
