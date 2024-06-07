import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from socket import gethostname
#if you run the launch file using ros2 launch after changing the parameter values, you need to rerun colcon build

PACKAGE_NAME = 'agl_odometry'

def generate_launch_description():

    config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")

    return LaunchDescription([
        Node(
            package='agl_odometry',
            executable='odometry',
            name='odometry_node',
            output='screen',
            emulate_tty=True,
            
            parameters=[config_file],
        ),
    ])