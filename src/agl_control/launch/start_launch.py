"""
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero, Iñaki Lorente
   Contact: support.idi@ageve.net
"""

from launch import LaunchDescription
from ageve_utils.launch.start import QuickClass
from ageve_utils.general.yaml import getNamespace
from ageve_utils.general.system import getPkgName

def generate_launch_description():

   Package = getPkgName(__file__)
   namespace = getNamespace(Package) # En caso de usar sin bringup utilizar getNamespace(Package, standalone=True)
   launcher = QuickClass(Package, namespace)
   nodes_dic = launcher.set_dictionary(package=Package, ros_name='Control')
   
   return launcher.launch_nodes(nodes_dic)