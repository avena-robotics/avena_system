import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration


def generate_launch_description():
    # Declate launch arguments
    base_path_arg = DeclareLaunchArgument(
        name='base_path', 
        default_value=TextSubstitution(text='/home/avena/Documents'),
        description="Path to directory where generated trajectories will be saved.",
    )
    run_joint_state_pub_arg = DeclareLaunchArgument(
        name='run_joint_state_pub', 
        default_value='True',
        description="Whether module should run dummy joint state publisher or not.",
        choices=['True', 'False'],
    )

    # Avena MoveIt setup
    avena_moveit_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'avena_arm_moveit_setup.launch.py')),
    )

    # Main module which saves generated trajectories
    save_trajectory = Node(
        package="save_trajectory",
        executable="save_trajectory_node",
        parameters=[{
            'base_path': LaunchConfiguration('base_path'),
            'run_joint_state_pub': LaunchConfiguration('run_joint_state_pub'),
        }],
        output="screen",
    )

    return LaunchDescription([
        base_path_arg,
        run_joint_state_pub_arg,
        avena_moveit_setup,
        save_trajectory,
    ])
