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
    nodes_dic = launcher.set_dictionary(package='tf2_ros',
                                        ros_name='tf2_map_odom',
                                        executable='static_transform_publisher',
                                        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']) # [x y z yaw pitch roll frame_id child_frame_id ]

    nodes_dic = launcher.set_dictionary(package='tf2_ros',
                                        ros_name='tf2_odom_basefootprint',
                                        executable='static_transform_publisher',
                                        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']) # [x y z yaw pitch roll frame_id child_frame_id ]


    nodes_dic = launcher.set_dictionary(package='robot_state_publisher',
                                        ros_name='Description',
                                        executable='robot_state_publisher',
                                        topics=['robot_description', 'joint_states'],
                                        arguments=[launcher.getConfig(pkg=Package, 
                                                                      folder='urdf',
                                                                      file_name='me00.urdf')])

    return launcher.launch_nodes(nodes_dic)


