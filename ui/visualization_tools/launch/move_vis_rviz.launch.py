import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('visualization_tools'),
        'rviz',
        'movement_visualization.rviz')

    return LaunchDescription([
        Node(
            package='visualization_tools',
            executable='movement_visualization_node',
            output='both',
            parameters=[{'log_level': 'info'}],
        ),
        Node(
            package='visualization_tools',
            executable='scene_visualization_node',
            output='both',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='both',
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
        #         'avena_bringup'), 'launch', 'load_table.launch.py'))
        # ),
    ])
