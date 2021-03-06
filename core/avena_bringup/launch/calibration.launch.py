import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ###############################
    # Parsing inputs parameters
    working_side = LaunchConfiguration('working_side').perform(context)
    robot = 'franka' if working_side == 'right' else 'avena'     
    world_to_link_0_pos = {}
    world_to_link_0_pos['x'] = '0.3'
    world_to_link_0_pos['y'] = '-0.53' if working_side == 'right' else '0.53'
    world_to_link_0_pos['z'] = '0'
    ###############################

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='both',
            arguments=[world_to_link_0_pos['x'], world_to_link_0_pos['y'], world_to_link_0_pos['z'], '0', '0', '0', '1', 'world', 'robot_base_link']
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
        #         'camera_extrinsics_calibration'), 'launch', 'calibrate.launch.py')),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
        #         'avena_bringup'), 'launch', 'load_table.launch.py'))
        # ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='working_side',
            default_value='right',
            description='Side on which currently used robot is on the table. Options: "left" or "right"'
        ),
        OpaqueFunction(
            function=launch_setup
        )
    ])
