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
    # Warehouse parameters
    warehouse_port_arg = DeclareLaunchArgument(
        name='warehouse_port', 
        default_value='33829',
        description="Port of database.",
    )
    warehouse_host_arg = DeclareLaunchArgument(
        name='warehouse_host', 
        default_value='localhost',
        description="Host of database",
    )
    warehouse_plugin_arg = DeclareLaunchArgument(
        name='warehouse_plugin', 
        default_value='warehouse_ros_mongo::MongoDatabaseConnection',
        description="Plugin of warehouse",
    )
    base_path_arg = DeclareLaunchArgument(
        name='base_path', 
        default_value=TextSubstitution(text='/home/avena/Documents'),
        description="Path to directory where generated trajectories will be saved.",
    )

    # initialized_env = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('generate_trajectory_from_waypoints'), 
    #                                                  'launch', 
    #                                                  'save_waypoints.launch.py')),
    #     launch_arguments={"warehouse_port": LaunchConfiguration('warehouse_port'),
    #                       "warehouse_host": LaunchConfiguration('warehouse_host'),
    #                       "warehouse_plugin": LaunchConfiguration('warehouse_plugin')}.items(),
    # )

    # # Main module which saves generated trajectories
    # save_trajectory = Node(
    #     package="save_trajectory",
    #     executable="save_trajectory_node",
    #     parameters=[{
    #         'base_path': LaunchConfiguration('base_path'),
    #     }],
    #     output="screen",
    # )

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("avena_bringup"),
            "urdf",
            "avena_arm_demo.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "avena_moveit_config", os.path.join(
            "config", 
            "avena.srdf",
        )
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    
    kinematics_yaml = load_yaml(
        "avena_moveit_config", os.path.join(
            "config", 
            "kinematics.yaml",
        )
    )

    # Main module which saves generated trajectories
    save_trajectory = Node(
        package="save_trajectory",
        executable="save_trajectory_node",
        parameters=[{
            'base_path': LaunchConfiguration('base_path'),
        }],
        output="screen",
    )

    generate_trajectory_node = Node(
        package="generate_trajectory_from_waypoints",
        executable="generate_trajectory_from_waypoints",
        parameters=[
            {"warehouse_port": LaunchConfiguration('warehouse_port')},
            {"warehouse_host": LaunchConfiguration('warehouse_host')},
            {"warehouse_plugin": LaunchConfiguration('warehouse_plugin')},
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output="screen",
    )

    return LaunchDescription([
        warehouse_port_arg,
        warehouse_host_arg,
        warehouse_plugin_arg,
        base_path_arg,

        save_trajectory,
        generate_trajectory_node,
    ])
