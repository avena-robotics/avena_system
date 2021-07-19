from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import os


def generate_launch_description():


   get_real_cameras_data = IncludeLaunchDescription (
                     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('get_real_cameras_data'), 'launch', 'get_real_cameras_data.launch.py'))
                  )
   main = IncludeLaunchDescription (
                     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('avena_bringup'), 'launch', 'main.launch.py'))
                  )

   return LaunchDescription([get_real_cameras_data, main])
