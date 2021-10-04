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


# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera.launch.py camera_name1:=D400 config_file1:='d435i.yaml' device_type2:=l5. device_type1:=d4..

"""Launch realsense2_camera node."""
import copy
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch_1st
import rs_launch_2nd
import rs_launch_3rd
import rs_launch_4th


def launch_setup(*args, **kwargs):
    print("-----------------")
    print(args)
    print(kwargs)
    print("-----------------")


def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params
    

def generate_launch_description():
    params1 = duplicate_params(rs_launch_1st.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch_2nd.configurable_parameters, '2')
    params3 = duplicate_params(rs_launch_3rd.configurable_parameters, '3')
    params4 = duplicate_params(rs_launch_4th.configurable_parameters, '4')
    return LaunchDescription(
        rs_launch_1st.declare_configurable_parameters(params1) + 
        rs_launch_2nd.declare_configurable_parameters(params2) +
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch_1st.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
        ),
           IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch_2nd.py']),
            launch_arguments=set_configurable_parameters(params2).items(),
        ),
        # rs_launch_1st.declare_configurable_parameters(params3) + 
        # rs_launch_2nd.declare_configurable_parameters(params4) +
        # [
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch_3rd.py']),
        #     launch_arguments=set_configurable_parameters(params3).items(),
        # ),
        #    IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch_4th.py']),
        #     launch_arguments=set_configurable_parameters(params4).items(),
        # ),



        ])