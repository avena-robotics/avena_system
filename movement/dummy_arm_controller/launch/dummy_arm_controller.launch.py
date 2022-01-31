import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration


def generate_launch_description():
    # Main module which saves generated trajectories
    dummy_arm_controller = Node(
        package="dummy_arm_controller",
        executable="dummy_arm_controller_node",
        output="screen",
    )

    return LaunchDescription([
        dummy_arm_controller,
    ])
