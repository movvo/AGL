# """
#    Copyright 2021 @ AGEVE
#    ---------------------------------------------------------
#    Authors: Albert Arlà Romero, Iñaki Lorente
#    Contact: support.idi@ageve.net
# """

# from ageve_utils.launch.start import QuickClass
# from ageve_utils.general.yaml import bringupParams, getNamespace
# from ageve_utils.general.system import getPkgName

# def generate_launch_description():

#    Package = getPkgName(__file__)
#    packages_list = bringupParams(Package)
#    namespace = getNamespace(Package)
#    launcher = QuickClass(Package, namespace)
#    launcher.launch_packages(['agl_bringup'],'lidar_launch.py')

#    return launcher.launch_packages(packages_list) 

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
from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = 'agl_bringup'

def BringupParams(config_file: str):
    logger = rclpy.logging.get_logger(PACKAGE_NAME)
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

      #   Filter By Black List
        nodes_list = [node for node in nodes_list if node not in data['Start']['black_list']]

    except Exception as e:
        logger.error(f"Parametros {config_file} incorrectos. ¿Existe Start?")
        print(f"DESCRIPCIÓN ERROR --> {e} on BringupParams, line {sys.exc_info()[-1].tb_lineno}")
    
    return packages_list, nodes_list

def generate_launch_description():
    # Configs files
    start = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "start.yaml")

    # Launch files to launch
    packages_to_execute, nodes_list = BringupParams(start)

    packages_actions = []
    for package in packages_to_execute:
        launch_dir = get_package_share_directory(package)

            packages_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'start_launch.py'))
                )
            )
    
    return LaunchDescription(packages_actions)