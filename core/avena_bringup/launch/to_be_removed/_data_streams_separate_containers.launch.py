"""Launch a talker and a listener in a component container."""
import os
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with multiple components."""
    LOG_LEVEL = 'debug'

    get_cameras_data_container = ComposableNodeContainer(
            name='get_cameras_data_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package='get_cameras_data', plugin='get_cameras_data::GetCamerasData', name='get_cameras_data'),
            ],
            output='screen',
    )

    ptcld_transformer_container = ComposableNodeContainer(
            name='ptcld_transformer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package='ptcld_transformer', plugin='ptcld_transformer::PtcldTransformer', name='ptcld_transformer'),
            ],
            output='screen',
    )

    robot_self_filter_container = ComposableNodeContainer(
            name='robot_self_filter_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package='robot_self_filter', plugin='robot_self_filter::RobotSelfFilter', name='robot_self_filter'),
            ],
            output='screen',
    )

    change_detect_container = ComposableNodeContainer(
            name='change_detect_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package='change_detect', plugin='change_detect::ChangeDetect', name='change_detect'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([get_cameras_data_container, ptcld_transformer_container, robot_self_filter_container, change_detect_container])
