# """
#    Copyright 2021 @ AGEVE
#    ---------------------------------------------------------
#    Authors: Iñaki Lorente, Albert Arlà Romero
#    Contact: support.idi@ageve.net
# """

# import os
# from ament_index_python.packages import get_package_share_directory
# from ageve_utils.launch.gazebo import QuickGazebo
# from ageve_utils.launch.start import QuickClass
# from ageve_utils.general.yaml import bringupParams, getNamespace
# from ageve_utils.general.system import getPkgName

# # World file
# world = os.path.join(get_package_share_directory('me00_description'), 
#                      'world/ageve_me00', 
#                      'ageve_me00')
                    
# # Launch
# def generate_launch_description():
#    Package = getPkgName(__file__)
#    packages_list = bringupParams(Package)
#    namespace = getNamespace(Package)
#    launcher = QuickClass(Package, namespace)
#    return launcher.launch_simulation(packages_list,  world_name=world)  

"""
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
   Contact: support.idi@ageve.net
"""

import os
import sys
import yaml
import rclpy
from typing import Tuple, Dict

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = 'agl_bringup'

def BringupParams(config_file):
    logger = rclpy.logging.get_logger("agl_bringup")
    cfg = open(config_file, 'r')
    data = yaml.safe_load(cfg)
    packages_list = []
    nodes_list = list()
    try:
        # Get all packages to execute
        for x in data['Start']['packages_list']:
            package = list(x.keys())[0]
            packages_list.append(package)
            
            # Get all nodes of each package
            for node in x[package]:
               nodes_list.append(node)

    except Exception as e:
        logger.error(f"Parametross de {config_file} incorrectos. ¿Existe Start?")
        print(f"DESCRIPCIÓN ERROR --> {e} on BringupParams, line {sys.exc_info()[-1].tb_lineno}")
    
    return packages_list, nodes_list

def generate_launch_description():

    world = os.path.join(get_package_share_directory('agl_description'), 
                        'world', 
                        'agl_world.sdf')

    namespace = os.getenv('ATLAS_NAMESPACE') if 'ATLAS_NAMESPACE' in os.environ.keys() else ""
    ns_instruction = f'__ns:=/{namespace}'
    # Configs files
    start = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "start_simulation.yaml")

    # Launch files to launch
    packages_to_execute, nodes_list = BringupParams(start)

    packages_actions = []
    for package in packages_to_execute:
      launch_dir = get_package_share_directory(package)

        
      packages_actions.append(
      IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'launch.py')),
      launch_arguments={
      'use_sim_time': 'True'
      }.items()
      )
      )
    
    return LaunchDescription([
        GroupAction(packages_actions),

        ExecuteProcess(
            cmd=[
                f'gazebo', 
                '--verbose',
                world,
                '-s',
                f'libgazebo_ros_init.so',
                '--ros-args --remap',
                f'{ns_instruction if namespace else ""}'
            ],
            output='screen')
    ])