
# Copyright 2020
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
#
# Authors: Iñaki Lorente Albert Arlà

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ageve_utils.general.yaml import getNamespace
from ageve_utils.general.yaml import getConfig
import yaml

#=====================================
#        LAUNCH CODE: lidar
#=====================================
def generate_launch_description():
    name_space = getNamespace('agl_bringup')
    config = getConfig('agl_bringup')
    cfg = open(config, 'r')
    data = yaml.safe_load(cfg)
    return LaunchDescription([
        # RPLIDAR
         Node(
            name='rplidar_composition',
            namespace=name_space,
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'topic_name': data[name_space]['Lidar']['ros__parameters']['topic_name'],
                'channel_type': data[name_space]['Lidar']['ros__parameters']['channel_type'],
                'tcp_ip': data[name_space]['Lidar']['ros__parameters']['tcp_ip'],
                'tcp_port': data[name_space]['Lidar']['ros__parameters']['tcp_port'],
                'serial_port': data[name_space]['Lidar']['ros__parameters']['serial_port'], #ttyUSBx
                'serial_baudrate': data[name_space]['Lidar']['ros__parameters']['serial_baudrate'],  # A3
                'frame_id': data[name_space]['Lidar']['ros__parameters']['frame_id'],
                'inverted': data[name_space]['Lidar']['ros__parameters']['inverted'],
                'angle_compensate': data[name_space]['Lidar']['ros__parameters']['angle_compensate'], # Esto deberia estar a true, pero a true peta
                'scan_mode': data[name_space]['Lidar']['ros__parameters']['scan_mode']
                #'angle_min': data[name_space]['Lidar']['ros__parameters']['angle_min'], #Por defecto 0º
                #'angle_max': data[name_space]['Lidar']['ros__parameters']['angle_max'] #Por defecto 360º
            }]),
    ])