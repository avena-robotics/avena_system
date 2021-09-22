# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


arguments_file_path = os.path.join(get_package_share_directory('realsense2_camera'), 'cameras_settings/d455.json')


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera_2', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': '_053422251478', 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': '', 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': '', 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': '', 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud',            'default': 'true', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method',             'default': '', 'description': '[copy|linear_interpolation]'},                           
                           {'name': 'json_file_path',               'default': arguments_file_path, 'description': 'allows advanced configuration'},                           
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},                           
                           {'name': 'depth_width',                  'default': '1280', 'description': 'depth image width'},                           
                           {'name': 'depth_height',                 'default': '720', 'description': 'depth image height'},                           
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_width',                    'default': '1280', 'description': 'color image width'},                           
                           {'name': 'rgb_height',                   'default': '720', 'description': 'color image height'},                           
                           {'name': 'enable_rgb',                   'default': 'true', 'description': 'enable color stream'},
                           {'name': 'infra_width',                  'default': '-1', 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': '-1', 'description': 'infra width'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width',                'default': '-1', 'description': 'fisheye width'},
                           {'name': 'fisheye_height',               'default': '-1', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'false', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'false', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width',             'default': '-1', 'description': 'depth image width'},                           
                           {'name': 'confidence_height',            'default': '-1', 'description': 'depth image height'},                           
                           {'name': 'enable_confidence',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps',                  'default': '-1.', 'description': ''},                           
                           {'name': 'depth_fps',                    'default': '30.', 'description': ''},                           
                           {'name': 'confidence_fps',               'default': '-1.', 'description': ''},                           
                           {'name': 'infra_fps',                    'default': '-1.', 'description': ''},                           
                           {'name': 'color_fps',                    'default': '30.', 'description': ''},                           
                           {'name': 'gyro_fps',                     'default': '-1.', 'description': ''},                           
                           {'name': 'accel_fps',                    'default': '-1.', 'description': ''},    
                           {'name': 'color_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'confidence_qos',               'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'depth_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'fisheye_qos',                  'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'infra_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'enable_gyro',                  'default': 'false', 'description': ''},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': ''},                           
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_COLOR', 'description': 'testure stream for pointcloud'},                           
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'testure stream index for pointcloud'},                           
                           {'name': 'enable_sync',                  'default': 'true', 'description': ''},                           
                           {'name': 'align_depth',                  'default': 'true', 'description': ''},
                           {'name': 'remove_shadows',               'default': 'false', 'description': 'Removing shadows from pointcloud '},
                        #    {'name': 'rgb_camera.auto_exposure_priority',          'default': 'false', 'description': ''},
                        #    {'name': 'rgb_camera.enable_auto_exposure',          'default': 'false', 'description': ''},
                        #    {'name': 'rgb_camera.exposure',          'default': '330', 'description': 'dont change it to 333 !!!'},
                        #    {'name': 'rgb_camera.gain',          'default': '0', 'description': ''},
                        #    {'name': 'rgb_camera.white_balance',          'default': '4600.0', 'description': 'lights on '},
                           {'name': 'filters',                      'default': 'pointcloud,temporal', 'description': ''},

                        #    {'name': 'filters',                      'default': 'pointcloud,spatial,temporal', 'description': ''},
                        #    {'name': 'filters',                      'default': 'spatial', 'description': ''},  
                        #    {'name': 'spatial.filter_magnitude', 'default': '1', 'description': ' INT filter iterations '}, 
                        #    {'name': 'spatial.filter_smooth_alpha', 'default': '0.5', 'description': 'DOUBLE Alfa factor of exp moving average (0.25-1) '},
                        #    {'name': 'spatial.filter_smooth_delta', 'default': '7', 'description': 'INT   Description: Edge-preserving Threshold (1-50 )'},
                        #    {'name': 'spatial.holes_fill', 'default': '4', 'description': 'Size of the radius which should be used for filling holes'},
                           {'name': 'temporal.filter_smooth_alpha', 'default': '0.3', 'description': ''},
                           {'name': 'temporal.filter_smooth_delta', 'default': '20', 'description': 'treshold for edge preserving filter'},
                        #    {'name': 'temporal.holes_fill', 'default': '1', 'description': ' valid in 8/8 '},                                
                           {'name': 'clip_distance',                'default': '-2.', 'description': ''},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},                           
                           {'name': 'initial_reset',                'default': 'false', 'description': ''},                           
                           {'name': 'allow_no_texture_points',      'default': 'true', 'description': ''},                           
                           {'name': 'ordered_pc',                   'default': 'true', 'description': ''},
                           {'name': 'base_frame_id',                'default': 'camera_2/camera_base', 'description': ''},
                           {'name': 'calib_odom_file',              'default': '', 'description': ''},                           
                           {'name': 'topic_odom_in',                'default': '', 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': '', 'description': 'A realsense bagfile to run from as a device'},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' == ''"])),
            package='realsense2_camera', 
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters = [set_configurable_parameters(configurable_parameters)],
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/camera_2/aligned_depth_to_rgb/image_raw', '/camera_2/depth_to_rgb/image_raw'),  # depths
                ('/camera_2/aligned_depth_to_rgb/camera_info', '/camera_2/depth_to_rgb/camera_info'),  # camera info
                ('/camera_2/depth/color/points', '/camera_2/points2'),  # Points
            ]
        ),
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' != ''"])),
            package='realsense2_camera', 
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters = [set_configurable_parameters(configurable_parameters)
                          ,{LaunchConfiguration("config_file")}
                          ],
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/camera_2/aligned_depth_to_rgb/image_raw', '/camera_2/depth_to_rgb/image_raw'),  # depths
                ('/camera_2/aligned_depth_to_rgb/camera_info', '/camera_2/depth_to_rgb/camera_info'),  # camera info
                ('/camera_2/depth/color/points', '/camera_2/points2'),  # Points
            ]
        ),
    ])
