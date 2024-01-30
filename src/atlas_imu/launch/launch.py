"""
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
   Contact: support.idi@ageve.net
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

PACKAGE_NAME = 'atlas_imu'

def generate_launch_description():
    # Configs files
    config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")

    # Launch arguments
    valgrind_arg = DeclareLaunchArgument(
        name='valgrind', 
        default_value='False', 
        choices=['True', 'False'],
        description='Debug with valgrind')

    # Launch configration
    valgrind = LaunchConfiguration('valgrind')

    remaps=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Nodes to launch
    node = Node(
        package=PACKAGE_NAME,
        executable='atlas_imu',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[config_file],
        remappings=remaps,
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['not ', valgrind]))
    )
    
    node_with_valgrind = Node(
        package=PACKAGE_NAME,
        executable='atlas_imu',
        arguments=['--ros-args', '--log-level', 'info'],
        prefix=["valgrind --leak-check=yes -s"],
        parameters=[config_file],
        remappings=remaps,
        output="screen",
        emulate_tty=True,
        condition=IfCondition(valgrind)
    )

    # EKFs
    local_ekf = Node(
        package='robot_localization',
        name='ekf_local',
        executable='ekf_node',
        remappings=[
            ('odometry/filtered', '~/odometry/filtered'), 
            ('/tf', 'tf'), 
            ('/tf_static', 'tf_static'),
            ('/diagnostics', 'diagnostics'),
            ('cmd_vel', 'Control/cmd_vel')
        ],
        parameters=[local_ekf_file],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['not ', sim_time]))
    )
    
    global_ekf = Node(
        package='robot_localization',
        name='ekf_global',
        executable='ekf_node',
        remappings=[
            ('odometry/filtered', '~/odometry/filtered'), 
            ('set_pose', 'initialpose'), 
            ('/tf', 'tf'), 
            ('/tf_static', 'tf_static')
        ],
        parameters=[global_ekf_file],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['not ', sim_time]))
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace(
                    os.getenv('ATLAS_NAMESPACE') if 'ATLAS_NAMESPACE' in os.environ.keys() else ""
                ),
                valgrind_arg,
                node,
                node_with_valgrind,
                local_ekf,
                global_ekfs
            ]
        )
    ])
