"""Launch a talker and a listener in a component container."""
import os
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with multiple components."""

    extra_arguments = {'use_intra_process_comms': True}
    log_levels_params = {
        'get_cameras_data_bag':  {'log_level': 'debug'},
        'ptcld_transformer':     {'log_level': 'debug'},
        'robot_self_filter':     {'log_level': 'debug'},
        'change_detect':         {'log_level': 'debug'},
    }

    # get_cameras_data_bag_container = ComposableNodeContainer(
    #     name='get_cameras_data_bag_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='get_cameras_data_bag',
    #             plugin='get_cameras_data_bag::GetCamerasDataBag',
    #             name='get_cameras_data_bag',
    #             extra_arguments=[extra_arguments],
    #             parameters=[log_levels_params['get_cameras_data_bag']],
    #         ),
    #     ],
    #     output='screen',
    # )

    container = ComposableNodeContainer(
        name='data_streams_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ptcld_transformer',
                plugin='ptcld_transformer::PtcldTransformer',
                name='ptcld_transformer',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['ptcld_transformer']],
            ),
            ComposableNode(
                package='robot_self_filter',
                plugin='robot_self_filter::RobotSelfFilter',
                name='robot_self_filter',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['robot_self_filter']],
            ),
            ComposableNode(
                package='change_detect',
                plugin='change_detect::ChangeDetect',
                name='change_detect',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['change_detect']],
            ),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])


    # return launch.LaunchDescription([container, get_cameras_data_bag_container])
