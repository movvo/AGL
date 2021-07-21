"""
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero
   Contact: support.idi@ageve.net
"""

import os
from ament_index_python.packages import get_package_share_directory
from ageve_utils.launch.rviz import QuickRviz
from ageve_utils.general.yaml import bringupParams, getNamespace
from ageve_utils.general.system import getPkgName
                    
# Launch
def generate_launch_description():
    Package = getPkgName(__file__)
    get_namespace = getNamespace(Package)
    rviz = QuickRviz(namespace=get_namespace)
    return rviz.generate_launch_description()  