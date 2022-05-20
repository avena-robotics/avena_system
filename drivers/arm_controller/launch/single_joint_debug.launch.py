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
    params = os.path.join(config,'single_joint_debug.yaml')
    amp_arg = DeclareLaunchArgument('sin_amp', default_value = '1.57')
    time_arg = DeclareLaunchArgument('trajectory_period', default_value = '12.')
    num_arg = DeclareLaunchArgument('cycles', default_value = '8')

    

    return [amp_arg,time_arg,num_arg,
        Node(
            package='arm_controller',
            executable='hw_interface',
            output='both',
            parameters=[params]
            # prefix=['valgrind --leak-check=yes --track-origins=yes']
        ),
        Node(
            package='arm_controller',
            executable='single_joint_debug',
            output='both',
            parameters=[params, {'config_path':config,'sin_amp':LaunchConfiguration('sin_amp'),'trajectory_period':LaunchConfiguration('trajectory_period'),'cycles':LaunchConfiguration('cycles')}]
        )
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(
            function=launch_setup
        )
    ])
