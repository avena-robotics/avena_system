import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    base_path_arg = DeclareLaunchArgument(
        name='base_path', 
        default_value=TextSubstitution(text='/home/avena/Documents'),
        description="Path to directory where generated trajectories will be saved.",
    )

    rviz_config = os.path.join(get_package_share_directory("generate_trajectory_from_waypoints"), "launch", "play_generated_trajectories.rviz")
    # Avena MoveIt setup
    avena_moveit_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'avena_arm_moveit_setup.launch.py')),
        launch_arguments={'rviz_config': rviz_config}.items(),
    )

    # Dummy arm controller
    dummy_arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('dummy_arm_controller'), 'launch', 'dummy_arm_controller.launch.py')),
    )

    play_generated_trajectories_node = Node(
        package="generate_trajectory_from_waypoints",
        executable="play_generated_trajectories",
        parameters=[{
            'base_path': LaunchConfiguration('base_path'),
        }],
        output="screen",
    )

    return LaunchDescription([
        base_path_arg,

        avena_moveit_setup,
        dummy_arm_controller,
        play_generated_trajectories_node,
    ])
