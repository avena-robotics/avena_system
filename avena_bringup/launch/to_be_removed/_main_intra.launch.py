from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():

    parameters_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'parameters_server'), 'launch', 'parameters_server.launch.py'))
    )
    data_streams = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'avena_bringup'), 'launch', 'data_streams.launch.py'))
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'avena_bringup'), 'launch', 'avena.launch.py'))
    )
    get_occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'get_occupancy_grid'), 'launch', 'get_occupancy_grid.launch.py'))
    )
    item_select = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'item_select'), 'launch', 'item_select.launch.py'))
    )
    gripper_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'gripper_controller'), 'launch', 'gripper_controller.launch.py'))
    )
    arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'arm_controller'), 'launch', 'arm_controller.launch.py'))
    )
    cli_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'manager_prepare_pick_and_place_data'), 'launch', 'manager_prepare_pick_and_place_data_server.launch.py'))
    )
    path_buffer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'path_buffer'), 'launch', 'path_buffer.launch.py'))
    )
    generate_trajectory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'generate_trajectory'), 'launch', 'generate_trajectory.launch.py'))
    )

    # Generate objects pipeline (using intra process communication)
    extra_arguments = {"use_intra_process_comms": True}
    log_levels_params = {
        'scene_publisher':   {'log_level': 'debug'},
        'detect':            {'log_level': 'debug'},
        'filter_detections': {'log_level': 'debug'},
        'select_new_masks':  {'log_level': 'debug'},
        'compose_items':     {'log_level': 'debug'},
        'estimate_shape':    {'log_level': 'debug'},
        'merge_items':       {'log_level': 'debug'},
        'octomap_filter':    {'log_level': 'debug'},
    }

    generate_objects_pipeline = ComposableNodeContainer(
        name='generate_objects_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='scene_publisher',
                plugin='scene_publisher::ScenePublisher',
                name='scene_publisher',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['scene_publisher']],
            ),
            # ComposableNode(
            #     package='detect',
            #     plugin='detect::DetectActionServer',
            #     name='detect',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['detect']],
            # ),
            # ComposableNode(
            #     package='filter_detections',
            #     plugin='FilterDetection',
            #     name='filter_detections',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['filter_detections']],
            # ),
            # ComposableNode(
            #     package='select_new_masks',
            #     plugin='select_new_masks::SelectNewMasks',
            #     name='select_new_masks',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['select_new_masks']],
            # ),
            # ComposableNode(
            #     package='compose_items',
            #     plugin='compose_items::ComposeItems',
            #     name='compose_items',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['compose_items']],
            # ),
            # ComposableNode(
            #     package='estimate_shape',
            #     plugin='estimate_shape::EstimateShape',
            #     name='estimate_shape',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['estimate_shape']],
            # ),
            # ComposableNode(
            #     package='merge_items',
            #     plugin='merge_items::MergeItems',
            #     name='merge_items',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['merge_items']],
            # ),
            # ComposableNode(
            #     package='octomap_filter',
            #     plugin='octomap_filter::OctomapFilter',
            #     name='octomap_filter',
            #     extra_arguments=[extra_arguments],
            #     parameters=[log_levels_params['octomap_filter']],
            # ),
        ],
    )

    return LaunchDescription([parameters_server, data_streams, robot_state_publisher, generate_objects_pipeline])
