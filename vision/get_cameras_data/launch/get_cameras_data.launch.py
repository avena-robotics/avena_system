import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='get_cameras_data_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='get_cameras_data',
                plugin='get_cameras_data::GetCamerasData',
                name='get_cameras_data')
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
