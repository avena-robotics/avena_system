# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from custom_interfaces.msg import Heartbeat
from custom_interfaces.msg import Watchdog
from custom_interfaces.msg import ModuleCommand
from custom_interfaces.msg import Statistics


from abc import ABC

class PyWatchdogInterface(ABC):
    def __init__(self):
        self.status=0

    @abstractmethod
    def initNode(self):
        pass
    
    @abstractmethod
    def shutDownNode(self):
        pass

class PyWatchdog:
    def __init__(self, node, gui_interface, system_monitor_name):
        self._node = node
        self._gui_interface = gui_interface
        self._system_monitor_name = system_monitor_name
        self._working = True
        self._pub_heartbeat_duration = 0.1
        self._pub_statistic_duration = 0.1
        self._system_check_duration = 0.2
        self._action_started = False
        self._action_period = 0
        self._node_name = self._node.get_name()
        
        self._sub_watchdog = self._node.create_subscription(
            Watchdog,
            'system_monitor/watchdog',
            self._watchdogCallback,
            10
        )
        self._sub_command = self._node.create_subscription(
            ModuleCommand,
            'system_monitor/command',
            self._commandCallback,
            10
        )
        self._pub_heartbeat = self._node.create_publisher(
            Heartbeat,
            'system_monitor/heartbeat',
            10
        )
        self._pub_statistic = self._node.create_publisher(
            Statistics,
            'system_monitor/statistics'
        )

        self._pub_statistic_timer = self._node.create_wall_timer(
            self._pub_statistic_duration,
            self._brodcastStatistics
        )

        self._system_check_timer = self._node.create_wall_timer(
            self._system_check_duration,
            self._systemCheckTimerCallback
        )

        self._heartbeat_timer = self._node.create_wall_timer(
            self._pub_heartbeat_duration,
            self._brodcastHeartbeat
        )

        pass

    def _brodcastStatistics(self):
        
        pass

    def _brodcastHeartbeat(self):
        msg = Heartbeat()
        msg.header.frame_id = "world"
        msg.header.stamp = self._node.now()
        msg.module_name = self._node_name
        msg.status = self._gui_interface.status
        self._pub_heartbeat.publish(msg)
        pass
    
    def _systemCheckTimerCallback(self):
        pass

    def _watchdogCallback(self, msg):
        pass

    def _commandCallback(self, command):
        pass

