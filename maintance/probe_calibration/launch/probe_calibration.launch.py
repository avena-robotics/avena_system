import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='probe_calibration_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='probe_calibration',
                    plugin='probe_calibration::ProbeCalibration',
                    name='probe_calibration',
                ),
            ],
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
        )
    ])
