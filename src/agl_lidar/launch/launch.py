
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
from ament_index_python.packages import get_package_share_directory

#=====================================
#        LAUNCH CODE: lidar
#=====================================
def generate_launch_description():

    config_file = os.path.join(get_package_share_directory('agl_lidar'), "config", "params.yaml")

    # Node to launch
    node = Node(
            package='rplidar_ros',
        #     namespace='agl',
            name='Lidar',
            executable='rplidar_node',
            arguments=['--ros-args','--log-level','info'],
            parameters=[config_file],
            output="screen",
            emulate_tty=True
            )
            
    ld = LaunchDescription()
    ld.add_action(node)

    return ld