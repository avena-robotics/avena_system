from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import os

def generate_launch_description():
   container = IncludeLaunchDescription (
                     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros2mysql'), 'launch', 'ros2mysql.launch.py')),
                     launch_arguments={'world': 'PATH','verbose': 'true'}.items()
                  )
   validate_estimate_shape = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('validate_estimate_shape'), 'launch', 'validate_estimate_shape.launch.py')),
        launch_arguments={'world': 'PATH','verbose': 'true'}.items()
   )
   return LaunchDescription([container, validate_estimate_shape])
