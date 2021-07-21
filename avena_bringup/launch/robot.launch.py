import os
import subprocess

import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_xacro_file = LaunchConfiguration('robot_xacro_file').perform(context)
    xacro_arguments = LaunchConfiguration('xacro_arguments').perform(context)
    
    # Load robot description
    pkg_share = FindPackageShare('avena_bringup').find('avena_bringup')
    xacro_file = os.path.join(pkg_share, 'urdf', robot_xacro_file)
    if len(xacro_arguments) > 0:
        p = subprocess.Popen(['xacro', xacro_file, xacro_arguments], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    else:
        p = subprocess.Popen(['xacro', xacro_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    robot_desc, _ = p.communicate()
    params = {'robot_description': robot_desc.decode('utf-8')}

    return [
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     output='both',
        #     parameters=[{'source_list': ['arm_joint_states', 'gripper_joint_states'],
        #                  'rate': 10,
        #                  'publish_default_positions': False}]
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[params]
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='robot_xacro_file',
            description='Xacro file name of robot to load.',
        ),
        DeclareLaunchArgument(
            name='xacro_arguments',
            default_value='',
            description='Xacro file arguments which are used when parsing robot description files.',
        ),
        OpaqueFunction(
            function=launch_setup
        )
    ])
