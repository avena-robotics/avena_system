import os
import yaml
import json
import glob

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def load_parameters(parameters_config_path: str):
    file_name = parameters_config_path.split('/')[-1]
    param_key = file_name.split('.')[-2]
    parameters = {}
    with open(parameters_config_path, 'r') as f:
        parameters_yaml = yaml.safe_load(f)
        parameters[param_key] = json.dumps(parameters_yaml)
    return parameters


def load_labels_parameters():
    label_file_list = glob.glob(os.path.join(get_package_share_directory('parameters_server'), 'config', 'labels', '*.yaml'))
    labels_configurations = []
    for label in label_file_list:
        with open(label, 'r') as f:
            label_config = yaml.safe_load(f)
            label_config['label'] = label.split('/')[-1].split('.')[-2]
            labels_configurations.append(label_config)
    return {'labels': json.dumps(labels_configurations)}


def launch_setup(context, *args, **kwargs):
    working_side = LaunchConfiguration('working_side').perform(context)

    parameters = load_labels_parameters()
    for root, dirs, files in os.walk(os.path.join(get_package_share_directory('parameters_server'), 'config', 'parameters')):
        if not dirs:
            for file_name in files:
                parameters_config_path = os.path.join(root, file_name)
                parameters.update(load_parameters(parameters_config_path))
    
    # Update 'working_side' parameter
    robot = json.loads(parameters['robot'])
    robot['working_side'] = working_side
    parameters['robot'] = json.dumps(robot)

    parameters_server = ComposableNodeContainer(
        name='parameters_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='parameters_server', 
                            plugin='parameters_server::ParametersServer', 
                            name='parameters_server', 
                            parameters=[parameters],
            ),
        ],
        output='screen',
    )
    return [parameters_server]

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='working_side',
            default_value='right',
            description='Side on which robot is working. Choose between "left" or "right"'
        ),
        OpaqueFunction(
            function=launch_setup
        )
    ])
