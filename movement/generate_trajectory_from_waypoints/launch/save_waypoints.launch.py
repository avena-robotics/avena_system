import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration


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

    # Avena MoveIt setup
    avena_moveit_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'avena_arm_moveit_setup.launch.py')),
    )

    # Dummy arm controller
    dummy_arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('dummy_arm_controller'), 'launch', 'dummy_arm_controller.launch.py')),
    )

    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": LaunchConfiguration('warehouse_port')},
            {"warehouse_host": LaunchConfiguration('warehouse_host')},
            {"warehouse_plugin": LaunchConfiguration('warehouse_plugin')},
        ],
        output="screen",
    )

    return LaunchDescription([
        warehouse_port_arg,
        warehouse_host_arg,
        warehouse_plugin_arg,

        avena_moveit_setup,
        dummy_arm_controller,
        mongodb_server_node,
    ])
