"""
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero, Iñaki Lorente
   Contact: support.idi@ageve.net
"""

from ageve_utils.launch.start import QuickClass
from ageve_utils.general.yaml import bringupParams, getNamespace
from ageve_utils.general.system import getPkgName

def generate_launch_description():

   Package = getPkgName(__file__)
   packages_list = bringupParams(Package)
   namespace = getNamespace(Package)
   launcher = QuickClass(Package, namespace)

   return launcher.launch_packages(packages_list)