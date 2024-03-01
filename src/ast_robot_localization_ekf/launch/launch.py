"""
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from socket import gethostname

NAMESPACE=gethostname().replace("-", "_")
PACKAGE_NAME = 'ast_robot_localization_ekf'

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
        ('/tf_static','tf_static'),
        ("odometry/filtered","EKF/Odometry")
    ]
    
    node = Node(
        package='robot_localization',
        # namespace=NAMESPACE,
        executable='ekf_node',
        name='EKF_Node',
        parameters=[config_file],
        output="screen",
        remappings=remaps,    
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    # Nodes to launch
    return LaunchDescription([
        GroupAction(
            actions=[
                sim_time_arg,
                SetParameter("use_sim_time", sim_time),
                node
            ]
        )
    ])

