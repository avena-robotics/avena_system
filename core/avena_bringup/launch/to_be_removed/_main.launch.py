import os
import launch
import launch_ros
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    extra_arguments = {"use_intra_process_comms": True}
    log_levels_params = {
        'get_cameras_data': {'log_level': 'info'},
    }

    get_cameras_data = launch_ros.actions.ComposableNodeContainer(
        name='data_streams_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='get_cameras_data',
                plugin='get_cameras_data::GetCamerasData',
                name='get_cameras_data',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['get_cameras_data']],
            ),
        ],
        output='screen',
    )

    parameters_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'parameters_server'), 'launch', 'parameters_server.launch.py'))
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'avena_bringup'), 'launch', 'avena.launch.py'))
    )
    fast_tracking_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'avena_bringup'), 'launch', 'fast_tracking.launch.py'))
    )

    scene_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'scene_publisher'), 'launch', 'scene_publisher.launch.py'))
    )

    containers_to_run = [parameters_server, robot_state_publisher, get_cameras_data, fast_tracking_pipeline, scene_publisher]
    return launch.LaunchDescription(containers_to_run)
