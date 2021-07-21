
import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    arguments_file_path = os.path.join(get_package_share_directory('camera_extrinsics_calibration'), 'config')
    print(arguments_file_path)
    camera_1_config_path = os.path.join(arguments_file_path, "camera1.yaml")
    camera_2_config_path = os.path.join(arguments_file_path, "camera2.yaml")
    
    camera_1_config = None
    camera_2_config = None


    try:
        with open(camera_1_config_path) as file:
            camera_1_config = yaml.load(file, Loader=yaml.FullLoader)
    except IOError: 
        print("There is no configuration data for camera 1")

    try:
        with open(camera_2_config_path) as file:
            camera_2_config = yaml.load(file, Loader=yaml.FullLoader)
    except IOError: 
        print("There is no configuration datafor camera 2")
        

    if camera_1_config is None or camera_2_config is None:
        return launch.LaunchDescription()


    cam1 = launch_ros.actions.Node(
                        package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output = "both",
                       arguments = [str(camera_1_config['position'][0]), str(camera_1_config['position'][1]), str(camera_1_config['position'][2]), str(camera_1_config['orientation'][0]), str(camera_1_config['orientation'][1]), str(camera_1_config['orientation'][2]), str(camera_1_config['orientation'][3]), camera_1_config['parent'], camera_1_config['child']]
                       )

    cam2 = launch_ros.actions.Node(
                        package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output = "both",
                       arguments = [str(camera_2_config['position'][0]), str(camera_2_config['position'][1]), str(camera_2_config['position'][2]), str(camera_2_config['orientation'][0]), str(camera_2_config['orientation'][1]), str(camera_2_config['orientation'][2]), str(camera_2_config['orientation'][3]), camera_2_config['parent'], camera_2_config['child']]
                       )


    return launch.LaunchDescription([cam1,cam2])
