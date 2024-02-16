"""
*  Copyright 2023 @ MOVVO ROBOTICS
*  ---------------------------------------------------------
*  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
*  Contact: support.idi@movvo.eu
*
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition

from socket import gethostname

NAMESPACE=gethostname().replace("-", "_")
PACKAGE_NAME = 'ast_camera'

def generate_launch_description():
    # Get config file
    config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")

    # Launch arguments
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='False', 
        choices=['True', 'False'],
        description='Parameter use_sim_time')
    
    # Launch arguments
    sim_time = LaunchConfiguration('use_sim_time')
    
    # remaps
    remaps = [
        ('/tf','tf'),
        ('/tf_static','tf_static')
    ]
    
    node = Node(
            package='realsense2_camera',
            namespace=NAMESPACE,
            name="Camera",
            executable='realsense2_camera_node',
            parameters=[config_file],
            output="screen",
            remappings=remaps,    
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info']
        )
    
    return LaunchDescription([
        GroupAction(
            actions=[
                sim_time_arg,
                SetParameter("use_sim_time", sim_time),
                node
            ]
        )
    ])
