from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import os


def generate_launch_description():


   main_kinect = IncludeLaunchDescription (
                     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'main_kinect.launch.py'))
                  )
   ros2mysql = IncludeLaunchDescription (
                     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'debug.launch.py')),
                     launch_arguments={'world': 'PATH','verbose': 'true'}.items()
                  )
   return LaunchDescription([main_kinect, ros2mysql])
