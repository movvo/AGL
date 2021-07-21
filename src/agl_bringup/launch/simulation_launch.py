"""
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente, Albert Arlà Romero
   Contact: support.idi@ageve.net
"""

import os
from ament_index_python.packages import get_package_share_directory
from ageve_utils.launch.gazebo import QuickGazebo
from ageve_utils.launch.start import QuickClass
from ageve_utils.general.yaml import bringupParams, getNamespace
from ageve_utils.general.system import getPkgName

# World file
world = os.path.join(get_package_share_directory('me00_description'), 
                     'world/ageve_me00', 
                     'ageve_me00')
                    
# Launch
def generate_launch_description():
   Package = getPkgName(__file__)
   packages_list = bringupParams(Package)
   namespace = getNamespace(Package)
   launcher = QuickClass(Package, namespace)
   return launcher.launch_simulation(packages_list,  world_name=world)  