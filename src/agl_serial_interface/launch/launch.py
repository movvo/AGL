import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from socket import gethostname

#if you run the launch file using ros2 launch after changing the parameter values, you need to rerun colcon build

NAMESPACE = 'agl'
PACKAGE_NAME = 'agl_serial_interface'

def generate_launch_description():
    # Get config file
    config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")

    return LaunchDescription([
        Node(
            package=PACKAGE_NAME,
            namespace=NAMESPACE,
            executable='serial_server',
            name='serial_server_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file]
        ),
    ])