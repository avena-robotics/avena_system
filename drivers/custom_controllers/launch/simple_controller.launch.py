import os
import subprocess

import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    config_path=os.path.join(os.path.split(os.path.abspath(os.pardir))[0],"config")

    pkg_share = FindPackageShare('custom_controllers').find('custom_controllers')
    config = os.path.join(pkg_share,'config')
    params = os.path.join(config,'simple_controller.yaml')

    return [
        Node(
            package='custom_controllers',
            executable='simple_controller',
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