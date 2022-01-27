import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
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
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization ",
                                # "default_planner_request_adapters/FixWorkspaceBounds "
                                # "default_planner_request_adapters/FixStartStateBounds "
                                # "default_planner_request_adapters/FixStartStateCollision "
                                # "default_planner_request_adapters/FixStartStatePathConstraints",
            # "start_state_max_bounds_error": 0.1,
            "resample_dt": 0.01,
        }
    }
    
    ompl_planning_yaml = load_yaml(
        "avena_moveit_config", os.path.join(
            "config", 
            "ompl_planning.yaml",
        )
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # # RViz
    # rviz_full_config = os.path.join(get_package_share_directory("avena_moveit_config"), "launch", "avena_moveit_config_demo.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_full_config],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         ompl_planning_pipeline_config,
    #         kinematics_yaml,
    #     ],
    # )

    save_trajectory = Node(
        package="save_trajectory",
        executable="save_trajectory_node",
        parameters=[{
            'base_path': LaunchConfiguration('base_path'),
            'run_joint_state_pub': LaunchConfiguration('run_joint_state_pub'),
        }],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='base_path', 
                default_value=TextSubstitution(text='/home/avena/Documents'),
                description="Path to directory where generated trajectories will be saved.",
            ),
            DeclareLaunchArgument(
                name='run_joint_state_pub', 
                default_value='True',
                description="Whether module should run dummy joint state publisher or not.",
                choices=['True', 'False'],
            ),
            # rviz_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'avena_arm_moveit_setup.launch.py')),
            ),
            save_trajectory,
        ]
    )
