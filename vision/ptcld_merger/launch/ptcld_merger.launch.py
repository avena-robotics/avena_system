from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='ptcld_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ptcld_merger',
                plugin='ptcld_merger::PtcldMerger',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])