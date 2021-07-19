from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    extra_arguments = {"use_intra_process_comms": True}
    log_levels_params = {
        'fast_detect': {'log_level': 'info'},
        'detections_tracker': {'log_level': 'debug'},
        # 'detections_tracker_history': {'log_level': 'debug'},
    }

    fast_tracking_pipeline = ComposableNodeContainer(
        name='fast_tracking_pipeline',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='detect',
                plugin='detect::FastDetect',
                name='fast_detect',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['fast_detect']],
            ),
            ComposableNode(
                package='detections_tracker',
                plugin='detections_tracker::DetectionsTracker',
                name='detections_tracker',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['detections_tracker']],
            ),
        ],
    )

    return LaunchDescription([fast_tracking_pipeline])
