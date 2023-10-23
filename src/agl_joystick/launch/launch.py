import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from socket import gethostname

NAMESPACE = 'agl'
PACKAGE_NAME = 'agl_joystick'

def generate_launch_description():
    # Get config file
    joy_config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "joy_params.yaml")
    joystick_config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "joystick_params.yaml")

    # Nodes to launch
    return LaunchDescription([
        Node(
            package=PACKAGE_NAME,
            namespace=NAMESPACE,
            name='Joystick',
            executable='agl_joystick',
            arguments=['--ros-args','--log-level','info'],
            parameters=[joystick_config_file],
            output="screen",
            emulate_tty=True
        ), 
        Node(
            package='joy',
            namespace=NAMESPACE,
            name='Joy',
            executable='joy_node',
            arguments=['--ros-args','--log-level','info'],
            parameters=[joy_config_file],
            output="screen"
        )
    ])