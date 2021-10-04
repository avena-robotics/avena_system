import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    ###############################
    # Parsing inputs parameters
    working_side = 'right'
    world_to_link_0_pos = {}
    world_to_link_0_pos['x'] = '0.3'
    world_to_link_0_pos['y'] = '-0.53'
    world_to_link_0_pos['z'] = '0'
    ###############################

    rviz_config = os.path.join(
        get_package_share_directory('avena_bringup'),
        'rviz',
        'franka_probe.rviz')
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='both',
            arguments=[world_to_link_0_pos['x'], world_to_link_0_pos['y'], world_to_link_0_pos['z'], '0', '0', '0', '1', 'world', 'robot_base_link']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
                'avena_bringup'), 'launch', 'robot.launch.py')),
            launch_arguments={
                'robot_xacro_file': f'franka_with_probe.urdf.xacro',
                'xacro_arguments': f'side:={working_side}'}.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='both',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
                'avena_bringup'), 'launch', 'load_table.launch.py'))
        ),
    ])
