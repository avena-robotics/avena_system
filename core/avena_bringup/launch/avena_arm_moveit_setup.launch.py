import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro


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
    # run_rviz = DeclareLaunchArgument(
    #     name="rviz", 
    #     default_value="True", 
    #     description="Whether to run RViz visualization or not.",
    #     choices=['True', 'False'],
    # )

    # planning_context
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
        },
        # "default_planning_pipeline": "avena_arm",
    }
    
    ompl_planning_yaml = load_yaml(
        "avena_moveit_config", os.path.join(
            "config", 
            "ompl_planning.yaml",
        )
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "avena_moveit_config", os.path.join(
            "config", 
            "avena_controllers.yaml",
        )
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "avena_base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz
    rviz_full_config = os.path.join(get_package_share_directory("avena_moveit_config"), "launch", "avena_moveit_config_demo.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='rviz', 
                default_value='True', 
                description='Whether to run RViz visualization or not.',
                choices=['True', 'False'],
            ),
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
        ]
    )
