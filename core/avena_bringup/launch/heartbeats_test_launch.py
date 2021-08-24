from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

# from subprocess import call
# call(["python3", "/root/ros2_ws/src/avena_ros2/logic_bt/test/scripts/simple_action_server.py"])

def generate_launch_description():

    parameters_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'parameters_server'), 'launch', 'parameters_server.launch.py'))
    )

    # rgb_diff = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
    #         'rgb_diff'), 'launch', 'rgb_diff_server.launch.py'))
    # )

    # security_rgb = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
    #         'security_rgb'), 'launch', 'security_rgb.launch.py'))
    # )
    
    rgbd_sync = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'rgbd_sync'), 'launch', 'rgbd_sync.launch.py'))
    )
    
    octomap_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'octomap_generator'), 'launch', 'octomap_generator.launch.py'))
    )    
    
    data_store = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'data_store'), 'launch', 'data_store.launch.py'))
    )
    
    detect_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'detect_server'), 'launch', 'detect_server.launch.py'))
    )  
    
    get_cameras_data = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'get_cameras_data'), 'launch', 'get_cameras_data.launch.py'))
    )
    compose_items = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'compose_items'), 'launch', 'compose_items_server.launch.py'))
    )
    estimate_shape = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(
            'estimate_shape'), 'launch', 'estimate_shape.launch.py'))
    )

    return LaunchDescription([parameters_server, rgbd_sync, data_store, get_cameras_data,compose_items,estimate_shape, detect_server])
