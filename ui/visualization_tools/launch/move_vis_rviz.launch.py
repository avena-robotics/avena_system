import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def get_table_urdf() -> str:
    # Load robot description
    xacro_table = os.path.join(
        get_package_share_directory('avena_bringup'), 
        'worlds', 
        'avena_table.urdf.xacro')

    # Load table dimensions from parameters server config file
    areas_parameters = os.path.join(
        get_package_share_directory('parameters_server'),
        'config', 'parameters', 'globals', 'areas.yaml')

    with open(areas_parameters, 'r') as f:
        params = yaml.safe_load(f)
        table_xacro_args = params['table_area']
    
    try:
        xacro_args = 'x_min:={} x_max:={} y_min:={} y_max:={}'.format(
            table_xacro_args['min']['x'], table_xacro_args['max']['x'], 
            table_xacro_args['min']['y'], table_xacro_args['max']['y'])
    except:
        print('Error in parameters server YAML file')
        exit(1)

    output = subprocess.run(['xacro', xacro_table, *xacro_args.split()], capture_output=True)
    urdf_content = output.stdout.decode()
    error = output.stderr.decode()
    if output.returncode:
        raise Exception('Error occured while processing XACRO. Error code: {}. Error message: {}'.format(output.returncode, error))
    return urdf_content


def generate_launch_description():
    table_urdf = get_table_urdf()

    rviz_config = os.path.join(
        get_package_share_directory('visualization_tools'),
        'rviz',
        'movement_visualization.rviz')

    return LaunchDescription([
        Node(
            package='visualization_tools',
            executable='movement_visualization_node',
            output='both',
            parameters=[{'log_level': 'info'}],
        ),
        Node(
            package='visualization_tools',
            executable='scene_visualization_node',
            output='both',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='both',
        ),

        # Load table URDF as robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            name='table_state_publisher',
            parameters=[{'robot_description': table_urdf}],
            remappings=[
                ('robot_description', 'table_description'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='both',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'table_base_link']
        ),
    ])
