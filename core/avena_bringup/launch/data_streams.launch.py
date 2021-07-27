import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    simulation_mode = LaunchConfiguration('simulation_mode').perform(context)
    extra_arguments = {'use_intra_process_comms': True}
    log_levels_params = {
        'get_cameras_data':  {'log_level': 'info'},
        'ptcld_transformer': {'log_level': 'info'},
        'robot_self_filter': {'log_level': 'info'},
        'change_detect':     {'log_level': 'info'},
    }

    components = [            
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
    ]

    if simulation_mode.lower() == 'true':
        components.append(            
            ComposableNode(
                package='get_cameras_data',
                plugin='get_cameras_data::GetCamerasData',
                name='get_cameras_data',
                extra_arguments=[extra_arguments],
                parameters=[log_levels_params['get_cameras_data']],
            )
        )

    data_streams_container = ComposableNodeContainer(
        name='data_streams_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=components,
        output='screen',
    )

    return [data_streams_container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='simulation_mode',
            default_value='false',
            description='Whether data stream pipeline should run with addition module used in simulation or not',
        ),
        OpaqueFunction(
            function=launch_setup
        )
    ])
